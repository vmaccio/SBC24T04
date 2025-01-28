#include "protocol_examples_common.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#include "esp_http_client.h"
#include <stddef.h>
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_netif_net_stack.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#if IP_NAPT
#include "lwip/lwip_napt.h"
#endif
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_system.h"

#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <stdint.h>
#include "driver/gpio.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sd_test_io.h"
#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#endif

#define EXAMPLE_MAX_CHAR_SIZE    64

static const char *TAG = "example";	

#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  10

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

#define GPIO_INPUT_PIN GPIO_NUM_33
#define PEAK_THRESHOLD 4000
#define GPIO_VOICECOMMAND_BUTTON GPIO_NUM_35

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            //.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            //.sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            //.sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

// Create Wav Struct
// https://docs.fileformat.com/audio/wav/
struct wav_header
{
  char riff[4];           /* "RIFF"                                  */
  int32_t flength;        /* file length in bytes                    */
  char wave[4];           /* "WAVE"                                  */
  char fmt[4];            /* "fmt "                                  */
  int32_t chunk_size;     /* size of FMT chunk in bytes (usually 16) */
  int16_t format_tag;     /* 1=PCM, 257=Mu-Law, 258=A-Law, 259=ADPCM */
  int16_t num_chans;      /* 1=mono, 2=stereo                        */
  int32_t srate;          /* Sampling rate in samples per second     */
  int32_t bytes_per_sec;  /* bytes per second = srate*bytes_per_samp */
  int16_t bytes_per_samp; /* 2=16-bit mono, 4=16-bit stereo          */
  int16_t bits_per_samp;  /* Number of bits per sample               */
  char data[4];           /* "data"                                  */
  int32_t dlength;        /* data length in bytes (filelength - 44)  */
};
// Populate Wav Struct
struct wav_header wavh;
const float MIDDLE_C = 256.00;
const int sample_rate = 8000;
const int duration_seconds = 5;
//const int buffer_size = sample_rate * duration_seconds;
#define BUFFER_SIZE (8000 * 5)  // sample_rate * duration_seconds
short int buffer[BUFFER_SIZE] = {};


const int header_length = sizeof(struct wav_header);

#define MOUNT_POINT "/sdcard"

#ifdef CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
const char* names[] = {"CLK ", "MOSI", "MISO", "CS  "};
const int pins[] = {CONFIG_EXAMPLE_PIN_CLK,
                    CONFIG_EXAMPLE_PIN_MOSI,
                    CONFIG_EXAMPLE_PIN_MISO,
                    CONFIG_EXAMPLE_PIN_CS};

const int pin_count = sizeof(pins)/sizeof(pins[0]);
#if CONFIG_EXAMPLE_ENABLE_ADC_FEATURE
const int adc_channels[] = {CONFIG_EXAMPLE_ADC_PIN_CLK,
                            CONFIG_EXAMPLE_ADC_PIN_MOSI,
                            CONFIG_EXAMPLE_ADC_PIN_MISO,
                            CONFIG_EXAMPLE_ADC_PIN_CS};
#endif //CONFIG_EXAMPLE_ENABLE_ADC_FEATURE

pin_configuration_t config = {
    .names = names,
    .pins = pins,
#if CONFIG_EXAMPLE_ENABLE_ADC_FEATURE
    .adc_channels = adc_channels,
#endif
};
#endif //CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS

// Pin assignments can be set in menuconfig, see "SD SPI Example Configuration" menu.
// You can also change the pin assignments here by changing the following 4 lines.
#define PIN_NUM_MISO  2
#define PIN_NUM_MOSI  15
#define PIN_NUM_CLK   14
#define PIN_NUM_CS    13

// Configuración SPI
#define PMOD_MIC3_SPI_HOST SPI3_HOST
#define PMOD_MIC3_CS_PIN 5  // Define aquí el GPIO para el Chip Select (CS)
#define PMOD_MIC3_SCLK_PIN 18  // Pin del reloj SPI (SCLK) rojo
#define PMOD_MIC3_MISO_PIN 19  // Pin de datos (MISO)

#define GPIO_BUZZER 25
#define GPIO_RX 3
#define GPIO_TX 1
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_SPEED      LEDC_HIGH_SPEED_MODE
#define LEDC_RESOLUTION LEDC_TIMER_13_BIT
#define RX_BUF_SIZE     128
static bool buzzer_on = false;

static esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t s_example_append_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for appending");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File appended");

    return ESP_OK;
}

static esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    return ESP_OK;
}

spi_device_handle_t spi;
// Configuración del dispositivo SPI
static void configure_spi(void) {
    esp_err_t ret;

    // Configurar el bus SPI
    spi_bus_config_t buscfg = {
        .miso_io_num = PMOD_MIC3_MISO_PIN,
        .mosi_io_num = -1,
        .sclk_io_num = PMOD_MIC3_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };
    ret = spi_bus_initialize(PMOD_MIC3_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Configurar el dispositivo SPI
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode = 0,                         // SPI modo 0
        .spics_io_num = PMOD_MIC3_CS_PIN,  // GPIO para CS
        .queue_size = 1,                   // Tamaño de la cola
    };
    ret = spi_bus_add_device(PMOD_MIC3_SPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Micro configurado correctamente");
}

// Leer datos desde el Pmod MIC3
int read_mic_data() {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    uint8_t data[2]; // Pmod MIC3 entrega 2 bytes de datos
	short int mic_value;
    t.length = 16; // 16 bits
    t.rx_buffer = data;

    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret == ESP_OK) {
        mic_value = (data[0] << 8) | data[1];
    } else {
		mic_value = 0;
        ESP_LOGE(TAG, "Error en la lectura del MIC");
    }
    return mic_value;
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
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

#define CHUNK_SIZE 512  // Tamaño del fragmento en bytes

void http_post_file() {
    FILE *file = fopen(MOUNT_POINT"/fwrite.wav", "rb");
    if (!file) {
        ESP_LOGE(TAG, "No se pudo abrir el archivo: %s", MOUNT_POINT"/fwrite.wav");
        return;
    }

    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    rewind(file);

    char *file_buffer = malloc(file_size);
    if (!file_buffer) {
        ESP_LOGE(TAG, "No se pudo asignar memoria para el archivo");
        fclose(file);
        return;
    }

    fread(file_buffer, 1, file_size, file);
    fclose(file);

    esp_http_client_config_t config = {
        .url = "http://192.168.66.179:8000",
        .event_handler = http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "audio/wav");
    esp_http_client_set_post_field(client, file_buffer, file_size);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Archivo enviado exitosamente, Código HTTP: %d",
                 esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "Error al enviar archivo: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    free(file_buffer);
}

	int ecg_value = 0;
    int bpm = 0;
    int last_peak_time = 0;
    int current_time = 0;
    int interval = 0;
    esp_http_client_handle_t client2;
void ecg_task(void) {
    //while (1) {
        // Leer el valor del ADC
        int anterior = ecg_value;
        ecg_value = adc1_get_raw(ADC1_CHANNEL_5); // ADC1_CHANNEL_5 corresponde a GPIO_NUM_33
        ESP_LOGI(TAG, "ECG VALUE: %d", ecg_value);

        // Detectar picos y calcular BPM
        if (ecg_value > PEAK_THRESHOLD && anterior != 4095) {
            current_time = esp_timer_get_time() / 1000; // Tiempo actual en ms
            if (last_peak_time > 0) {
                interval = current_time - last_peak_time; // Intervalo entre picos
                if (interval > 0) {
                    bpm = 60000 / interval; // Calcular BPM
                    ESP_LOGI(TAG, "Detected BPM: %d", bpm * 2);

                    // Enviar BPM a ThingsBoard
                    char post_data[50];
                    snprintf(post_data, sizeof(post_data), "{\"bpm\": %d}", bpm * 2);
					
                    esp_http_client_set_method(client2, HTTP_METHOD_POST);
                    esp_http_client_set_header(client2, "Content-Type", "application/json");
                    esp_http_client_set_post_field(client2, post_data, strlen(post_data));

                    esp_err_t err = esp_http_client_perform(client2);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
                    } else {
                        ESP_LOGI(TAG, "Heart rate sent successfully");
                    }
                }
            //}

            last_peak_time = current_time; // Actualizar tiempo del último pico
        }

        // Pausa entre lecturas
        vTaskDelay(pdMS_TO_TICKS(10)); // 10 ms de espera
    }

    esp_http_client_cleanup(client2);
}

void record_and_send(){
	esp_err_t ret;
	strncpy(wavh.riff, "RIFF", 4);
  	strncpy(wavh.wave, "WAVE", 4);
	strncpy(wavh.fmt, "fmt ", 4);
	strncpy(wavh.data, "data", 4);
	wavh.chunk_size = 16;
	wavh.format_tag = 1;
	wavh.num_chans = 1;
	wavh.srate = sample_rate;
	wavh.bits_per_samp = 16;
	wavh.bytes_per_sec = wavh.srate * wavh.bits_per_samp / 8 * wavh.num_chans;
	wavh.bytes_per_samp = wavh.bits_per_samp / 8 * wavh.num_chans;

	ESP_LOGI(TAG, "Inicio Grabacion");
  	for (int i = 0; i < BUFFER_SIZE; i++) {
    	//buffer[i] = (short int)((cos((2 * M_PI * MIDDLE_C * i) / sample_rate) * 1000));
    	//buffer[i] = (short int)((cos((2 * M_PI * read_mic_data() * i) / sample_rate) * 1000));
    	buffer[i] = read_mic_data();
    	vTaskDelay(9/portTICK_PERIOD_MS);
    	//ESP_LOGI(TAG, "Lectura de MIC: %d", read_mic_data());
  	}
  	wavh.dlength = BUFFER_SIZE * wavh.bytes_per_samp;
  	wavh.flength = wavh.dlength + header_length;
  	ESP_LOGI(TAG, "Fin Grabacion");
  	const char *file_hello = MOUNT_POINT"/fwrite.wav";
    int temp;
    char data[EXAMPLE_MAX_CHAR_SIZE];
  	
  	FILE *fp = fopen(file_hello, "w");
	if (fp == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
    }
  	fwrite(&wavh, 1, header_length, fp);
  	fwrite(buffer, 2, BUFFER_SIZE, fp);
  	ESP_LOGI(TAG, "File written");
  	fclose(fp);
  	
  	    // Format FATFS
#ifdef CONFIG_EXAMPLE_FORMAT_SD_CARD
    ret = esp_vfs_fat_sdcard_format(mount_point, card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to format FATFS (%s)", esp_err_to_name(ret));
        return;
    }

    if (stat(file_foo, &st) == 0) {
        ESP_LOGI(TAG, "file still exists");
        return;
    } else {
        ESP_LOGI(TAG, "file doesn't exist, formatting done");
    }
#endif // CONFIG_EXAMPLE_FORMAT_SD_CARD

	    //Open file for reading
    const char *file_test = MOUNT_POINT"/fwrite.wav";
    ret = s_example_read_file(file_test);
    if (ret != ESP_OK) {
        return;
    }
    http_post_file();
}


void configure_buzzer() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_SPEED,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_RESOLUTION,
        .freq_hz          = 1000,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = GPIO_BUZZER,
        .speed_mode     = LEDC_SPEED,
        .channel        = LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void buzzer_on_with_frequency(int freq_hz) {
    if (!buzzer_on) {
        ledc_set_freq(LEDC_SPEED, LEDC_TIMER, freq_hz);
        ledc_set_duty(LEDC_SPEED, LEDC_CHANNEL, 4096);
        ledc_update_duty(LEDC_SPEED, LEDC_CHANNEL);
        buzzer_on = true;
    }
}

void buzzer_off() {
    if (buzzer_on) {
        ledc_set_duty(LEDC_SPEED, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_SPEED, LEDC_CHANNEL);
        buzzer_on = false;
    }
}

void configure_uart() {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_TX, GPIO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 0, NULL, 0));
}

void process_uart_data(const uint8_t *data, int length) {
    for (int i = 0; i < length - 4; ++i) {
        if (data[i] == 0xEA && data[i + 4] == 0x0D) {
            char distance_str[4] = {data[i + 2], data[i + 3], '\0'};
            int distance_inch = atoi(distance_str);

            if (distance_inch <= 15) {
                ESP_LOGI(TAG, "Distancia válida: %d pulgadas", distance_inch);
                buzzer_on_with_frequency(1000);
            } else {
                ESP_LOGW(TAG, "Distancia no válida: %d pulgadas", distance_inch);
                buzzer_off();
            }
            return;
        }
    }
}

void app_main(void)
{
    esp_err_t ret;
    gpio_set_direction(GPIO_VOICECOMMAND_BUTTON, GPIO_MODE_INPUT);

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 5000;

    // For SoCs where the SD power can be supplied both via an internal or external (e.g. on-board LDO) power supply.
    // When using specific IO pins (which can be used for ultra high-speed SDMMC) to connect to the SD card
    // and the internal LDO power supply, we need to initialize the power supply first.
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_IO_ID,
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
        return;
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;
#endif

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }
    
    //Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

	ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
	
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
#ifdef CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
            check_sd_card_pins(&config, pin_count);
#endif
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
    adc1_config_width(ADC_WIDTH_BIT_12); // Configurar ADC a 12 bits de resolución
    adc1_config_channel_atten(GPIO_NUM_33, ADC_ATTEN_DB_11);
  	// Playing a C Note
  	configure_spi();
  	
  	
  	//wavh.dlength = BUFFER_SIZE * wavh.bytes_per_samp;
  	//wavh.flength = wavh.dlength + header_length;

   

    
    configure_buzzer();
    configure_uart();
    //http_post_file();
    uint8_t *dataUART = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    if (dataUART == NULL) {
        ESP_LOGE(TAG, "Error al asignar memoria");
        return;
    }
    int rxBytes;
    esp_http_client_config_t config2 = {
        .url = "http://104.196.24.70/api/v1/QmfwwWShllCHJ6T6yHiy/telemetry",
        .event_handler = http_event_handler,
    };
    esp_http_client_handle_t client2 = esp_http_client_init(&config2);
    while (1) {
		ESP_LOGE(TAG, "Ciclo de bucle principal--------------------------------------------------------------------------------");
		if(gpio_get_level(GPIO_VOICECOMMAND_BUTTON) == 1){
			record_and_send();
		}
		//ecg_task();
		rxBytes = uart_read_bytes(UART_NUM_1, dataUART, RX_BUF_SIZE, pdMS_TO_TICKS(1000));
		if (rxBytes > 0) {
            process_uart_data(dataUART, rxBytes);
        }
		vTaskDelay(pdMS_TO_TICKS(10));
	}
    
	free(dataUART);
    // All done, unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");

    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);

    // Deinitialize the power control driver if it was used
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    ret = sd_pwr_ctrl_del_on_chip_ldo(pwr_ctrl_handle);
    if (ret != ESP_OK) { 	
        ESP_LOGE(TAG, "Failed to delete the on-chip LDO power control driver");
        return;
    }
#endif
}