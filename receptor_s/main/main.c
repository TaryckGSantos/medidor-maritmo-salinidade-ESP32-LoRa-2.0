#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "esp_http_client.h"
#include "esp_crt_bundle.h" // bundle de CAs para TLS (HTTPS)

#include <inttypes.h>

#include "lora.h" // driver da SX127x (LoRa)

#define TAG "RX_TS"

// Ajuste do Wi-fi
#define WIFI_SSID "Melk"
#define WIFI_PASS "GMUH2021*"
#define THINGSPEAK_WRITE_KEY "R427PWWEE3FCJVPY"

#define RESTART_EVERY_S   70     // reinicia o ESP periodicamente (hard watchdog simplificado)
#define INACTIVITY_S      0      // se >0: reinicia se ficar sem RX por esse tempo (segundos)

static volatile uint32_t s_last_ok_rx_ms = 0; // marca do último RX válido (ms desde boot)

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry = 0;

// Handler de eventos do Wi-Fi/IP
static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect(); // ao iniciar STA, tenta conectar
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        // política simples de retry
        if (s_retry < 5) {
            esp_wifi_connect();
            s_retry++;
            ESP_LOGW(TAG, "retry Wi-Fi");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        // obteve IP -> conectado
        s_retry = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Inicializa Wi-Fi em modo estação e espera conectar (ou falhar)
static void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // registra o mesmo handler para eventos Wi-Fi e IP
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
        &wifi_event_handler, NULL, NULL));

    // configura credenciais
    wifi_config_t wifi_config = { 0 };
    snprintf((char*)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), WIFI_SSID);
    snprintf((char*)wifi_config.sta.password, sizeof(wifi_config.sta.password), WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // bloqueia até conectar (ou falhar)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connected");
    } else {
        ESP_LOGW(TAG, "Wi-Fi failed");
    }
}

// Envia dados ao ThingSpeak usando HTTP GET sobre HTTPS (TLS)
static esp_err_t http_send_thingspeak(float tds, float voltage) {
    char url[256];
    // fields: field1=tds (ppm arredondado), field2=voltage
    snprintf(url, sizeof(url),
        "https://api.thingspeak.com/update?api_key=%s&field1=%.0f&field2=%.2f",
        THINGSPEAK_WRITE_KEY, tds, voltage);

    // GET síncrono
    esp_http_client_config_t cfg = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach, // usa bundle interno de CAs
        .timeout_ms = 7000,
    };
    esp_http_client_handle_t h = esp_http_client_init(&cfg);
    if (!h) return ESP_FAIL;

    esp_err_t err = esp_http_client_perform(h);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ThingSpeak status: %d", esp_http_client_get_status_code(h));
    } else {
        ESP_LOGE(TAG, "HTTP error: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(h);
    return err;
}

// Faz o parse do payload ASCII esperado: "TD,<ppm>,<volt>"
static bool parse_payload(const char *s, float *out_tds, float *out_v) {
    if (strncmp(s, "TD,", 3) != 0) return false; // prefixo obrigatório
    const char *p = s + 3;
    char *end = NULL;
    float ppm = strtof(p, &end); // lê <ppm>
    if (!end || *end != ',') return false; // exige vírgula
    float volt = strtof(end + 1, NULL); // lê <volt>
    if (out_tds) *out_tds = ppm;
    if (out_v)   *out_v   = volt;
    return true;
}

// Task principal de recepção LoRa e publicação
static void task_rx(void *arg) {
    ESP_LOGI(TAG, "RX start");

    uint8_t buf[255];

    lora_receive(); // coloca o rádio em RX contínuo

    while (1) {
        if (lora_received()) { // checa IRQ/flag de pacote recebido
            int rxLen = lora_receive_packet(buf, sizeof(buf)); // lê FIFO
            if (rxLen > 0 && rxLen < (int)sizeof(buf)) {
                buf[rxLen] = 0; // termina string
                float ppm=0, v=0;
                if (parse_payload((char*)buf, &ppm, &v)) {
                    ESP_LOGI(TAG, "LoRa ok: ppm=%.0f v=%.2f", ppm, v);

                    s_last_ok_rx_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

                    // publica imediatamente no ThingSpeak (se Wi-Fi está conectado)
                    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
                    if (bits & WIFI_CONNECTED_BIT) {
                        if (http_send_thingspeak(ppm, v) == ESP_OK) {
// opcional: reiniciar após publicar para "garantir" próximo ciclo
#if 1   // 0 para desativar o reboot após publicar
                            ESP_LOGW(TAG, "Publicado com sucesso. Reiniciando...");
                            esp_restart();
#endif
                        }
                    } else {
                        ESP_LOGW(TAG, "Sem Wi-Fi; não enviou.");
                    }
                } else {
                    ESP_LOGW(TAG, "Ignorado payload: %s", (char*)buf);
                }

                // Algumas libs saem de RX após ler FIFO; re-arma RX contínuo
                lora_receive();
            } else {
                // leitura inválida: re-arma RX assim mesmo
                lora_receive();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // pequeno descanso para CPU/RTOS
    }
}

// Task que reinicia o ESP32 a cada RESTART_EVERY_S (hard watchdog simples)
static void task_periodic_restart(void *arg) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(RESTART_EVERY_S * 1000));
        ESP_LOGW(TAG, "Reiniciando (periodic %ds)...", RESTART_EVERY_S);
        esp_restart();
    }
}

// reinicia se ficar sem RX válido por INACTIVITY_S
static void task_inactivity_restart(void *arg) {
    for (;;) {
        if (INACTIVITY_S > 0) {
            uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            if (s_last_ok_rx_ms != 0 && (now - s_last_ok_rx_ms) > (INACTIVITY_S * 1000U)) {
                uint32_t delta_ms = (now - s_last_ok_rx_ms);
                ESP_LOGW(TAG, "Sem RX há %" PRIu32 " ms, reiniciando...", delta_ms);
                esp_restart();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void) {
    // NVS é pré-requisito para Wi-Fi
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();

    // cria tasks de reinício antes do LoRa (para sempre haver um "guardião")
    xTaskCreate(task_periodic_restart, "RST", 2048, NULL, 3, NULL);
    xTaskCreate(task_inactivity_restart, "RST2", 2048, NULL, 3, NULL); // opcional

    // Inicializa rádio LoRa
    if (lora_init() == 0) {
        ESP_LOGE(TAG, "SX127x not found");
        vTaskDelay(pdMS_TO_TICKS(1000));   
        esp_restart();                     // se falhar, reinicia tudo
    }

#if CONFIG_915MHZ
    lora_set_frequency(915e6); // frequência via menuconfig
#elif CONFIG_OTHER
    long frequency = CONFIG_OTHER_FREQUENCY * 1000000;
    lora_set_frequency(frequency);
#endif
    lora_enable_crc(); // exige CRC nos pacotes

    // parâmetros PHY (devem bater com o TX)
    lora_set_coding_rate(1);
    lora_set_bandwidth(7);
    lora_set_spreading_factor(9);
    // lora_set_sync_word(0x12); // usar igual nos dois lados se ativar

    xTaskCreate(task_rx, "RX", 4096, NULL, 5, NULL);
}
