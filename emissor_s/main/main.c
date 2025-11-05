#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"

#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"

#include "lora.h"

#define TAG "TX_TDS_SLEEP"

#define VREF            3.3f // tensão de referência usada p/ converter o ADC
#define SAMPLES         32 // nº de amostras p/ média
#define TEMPERATURE_C   25.0f // temperatura usada p/ compensação TDS
#define TDS_GPIO        32 
#define ADC_UNIT_ID     ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_4 // GPIO32 no ADC1
#define SLEEP_SECONDS   30 // tempo de deep sleep entre ciclos
#define SLEEP_US        ((uint64_t)SLEEP_SECONDS * 1000000ULL)

#define TX_BURST_WINDOW_MS   5000  // janela total do "burst" (~5 s transmitindo)
#define TX_BURST_GAP_MS       500  // intervalo entre reenvios dentro do burst

static adc_oneshot_unit_handle_t s_adc;

// Inicializa ADC1 em modo oneshot no canal do GPIO32 com 12 bits e 11 dB de atenuação
static void adc_init(void) {
    adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = ADC_UNIT_ID};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));
    adc_oneshot_chan_cfg_t ch_cfg = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_11 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHANNEL, &ch_cfg));
}

// Libera o handle do ADC (boa prática antes de dormir)
static void adc_deinit(void) {
    if (s_adc) {
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
    }
}

// Lê SAMPLES amostras, faz média, converte p/ tensão e calcula TDS (ppm)
// Retorna TDS; se out_voltage != NULL, devolve a tensão lida (antes da compensação)
static float read_tds_ppm(float *out_voltage) {
    int sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        int raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc, ADC_CHANNEL, &raw));
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(2)); // pequeno intervalo p/ reduzir ruído/acoplamento
    }
    float raw_avg = (float)sum / (float)SAMPLES;
    float voltage = raw_avg * (VREF / 4095.0f); // 12 bits -> 0..4095

    // Compensação simples de temperatura (coef. típico 2%/°C)
    float comp_coeff   = 1.0f + 0.02f * (TEMPERATURE_C - 25.0f);
    float comp_voltage = voltage / comp_coeff;

    // Polinômio comum em exemplos de TDS (E-201-C/Gravity p.ex.), escalado por 0.5
    float tds = (133.42f*comp_voltage*comp_voltage*comp_voltage
              - 255.86f*comp_voltage*comp_voltage
              + 857.39f*comp_voltage) * 0.5f;

    if (tds < 0) tds = 0;
    if (out_voltage) *out_voltage = voltage;
    return tds;
}

// Entra em deep-sleep pelo tempo configurado
static void go_to_sleep(void) {
    ESP_LOGI(TAG, "Dormindo por %d seg...", SLEEP_SECONDS);

    esp_deep_sleep_disable_rom_logging(); // opcional: reduz logs da ROM ao acordar

    // Habilita wake-up por tempo
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(SLEEP_US));

    // fecha ADC para economizar
    adc_deinit();

    // consumo típico ~5–20 µA (placa-dependente)
    esp_deep_sleep_start();
}

// Reenvia o mesmo pacote várias vezes por ~TX_BURST_WINDOW_MS (para confiabilidade)
static void lora_send_burst(const uint8_t *buf, int len) {
    uint32_t start = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    while (((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - start) < TX_BURST_WINDOW_MS) {
        lora_send_packet(buf, len); // envio não-bloqueante (depende da lib)
        vTaskDelay(pdMS_TO_TICKS(TX_BURST_GAP_MS));
    }
}

void app_main(void) {
    // Motivo do wake-up (primeiro boot, timer, etc.)
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        ESP_LOGI(TAG, "Acordei pelo TIMER");
    } else if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
        ESP_LOGI(TAG, "Boot frio (primeira inicialização)");
    } else {
        ESP_LOGI(TAG, "Acordei por outra causa: %d", (int)cause);
    }

    // Init LoRa
    if (lora_init() == 0) { // precisa detectar o SX127x
        ESP_LOGE(TAG, "SX127x não encontrado");
        go_to_sleep(); // falhou? dorme e tenta denovo no próximo ciclo
    }

// Frequência (garanta que bate com o receptor)
#if CONFIG_915MHZ
    lora_set_frequency(915e6);
#elif CONFIG_OTHER
    long frequency = CONFIG_OTHER_FREQUENCY * 1000000;
    lora_set_frequency(frequency);
#endif

    lora_enable_crc(); // habilita CRC no payload (melhor integridade)

    // PHY: precisa bater com o receptor
    int cr = 1, bw = 7, sf = 9;
#if CONFIG_ADVANCED
    cr = CONFIG_CODING_RATE;
    bw = CONFIG_BANDWIDTH;
    sf = CONFIG_SF_RATE;
#endif
    lora_set_coding_rate(cr);
    lora_set_bandwidth(bw);
    lora_set_spreading_factor(sf);
    // (Opcional) lora_set_sync_word(0x12);

    // Init ADC
    adc_init();

    // Medir e enviar (single-shot)
    float v = 0.0f;
    float tds = read_tds_ppm(&v);

    // Monta payload ASCII no formato esperado pelo receptor: "TD,<ppm>,<volt>"
    uint8_t buf[64];
    int len = snprintf((char*)buf, sizeof(buf), "TD,%.0f,%.2f", tds, v);
    if (len < 0) len = 0;

    if (len > 0) {
        lora_send_burst(buf, len); // envia várias vezes na janela de burst
        ESP_LOGI(TAG, "LoRa sent: %s", (char*)buf);
        int lost = lora_packet_lost(); // se a lib suportar estatística
        if (lost) ESP_LOGW(TAG, "packets lost: %d", lost);
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // pequena folga p/ terminar TX e logs

    // Volta a dormir
    go_to_sleep();
}
