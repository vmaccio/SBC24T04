#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_timer.h"

// Bibliotecas para ADC y GPIO
#include "driver/adc.h"
#include "driver/gpio.h"

#define GPIO_INPUT_PIN GPIO_NUM_33
#define PEAK_THRESHOLD 4000
static const char *TAG = "ECG";

static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            break;
        case HTTP_EVENT_ON_CONNECTED:
            break;
        case HTTP_EVENT_HEADER_SENT:
            break;
        case HTTP_EVENT_ON_HEADER:
            break;
        case HTTP_EVENT_ON_DATA:
            break;
        case HTTP_EVENT_ON_FINISH:
            break;
        case HTTP_EVENT_DISCONNECTED:
            break;
        case HTTP_EVENT_REDIRECT:
            break;
        default:
            break;
    }
    return ESP_OK;
}

void ecg_task(void *pvParameter) {
    int ecg_value = 0;
    int bpm = 0;

    int last_peak_time = 0;
    int current_time = 0;
    int interval = 0;

    esp_http_client_config_t config = {
        .url = "http://104.196.24.70/api/v1/QmfwwWShllCHJ6T6yHiy/telemetry",
        .event_handler = http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    while (1) {
        // Leer el valor del ADC
        int anterior = ecg_value;
        ecg_value = adc1_get_raw(ADC1_CHANNEL_5); // ADC1_CHANNEL_5 corresponde a GPIO_NUM_33
        ESP_LOGI(TAG, "ECG Value: %d", ecg_value);

        // Detectar picos y calcular BPM
        if (ecg_value > PEAK_THRESHOLD && anterior != 4095) {
            current_time = esp_timer_get_time() / 1000; // Tiempo actual en ms
            if (last_peak_time > 0) {
                interval = current_time - last_peak_time; // Intervalo entre picos
                if (interval > 0) {
                    bpm = 60000 / interval; // Calcular BPM
                    ESP_LOGI(TAG, "Detected BPM: %d", bpm);

                    // Enviar BPM a ThingsBoard
                    char post_data[50];
                    snprintf(post_data, sizeof(post_data), "{\"bpm\": %d}", bpm);

                    esp_http_client_set_method(client, HTTP_METHOD_POST);
                    esp_http_client_set_header(client, "Content-Type", "application/json");
                    esp_http_client_set_post_field(client, post_data, strlen(post_data));

                    esp_err_t err = esp_http_client_perform(client);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
                    } else {
                        ESP_LOGI(TAG, "Heart rate sent successfully");
                    }
                }
            }

            last_peak_time = current_time; // Actualizar tiempo del último pico
        }

        // Pausa entre lecturas
        vTaskDelay(pdMS_TO_TICKS(10)); // 10 ms de espera
    }

    esp_http_client_cleanup(client);
}

void app_main(void) {
    // Inicializar NVS, red y otros componentes
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", (uint32_t)esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    adc1_config_width(ADC_WIDTH_BIT_12); // Configurar ADC a 12 bits de resolución
    adc1_config_channel_atten(GPIO_INPUT_PIN, ADC_ATTEN_DB_11);

    // Iniciar la tarea ECG
    xTaskCreate(ecg_task, "ecg_task", 4096, NULL, 5, NULL);
}
