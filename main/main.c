/* I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "mpu6050.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "driver/i2c.h"
#include "driver/uart.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/api.h"

#define LED_GPIO GPIO_NUM_2

#ifndef CONFIG_ESP_MAX_STA_CONN
#define CONFIG_ESP_MAX_STA_CONN 4     // or 8
#endif

#define EXAMPLE_ESP_WIFI_SSID      "Orange-148190"
#define EXAMPLE_ESP_WIFI_PASS      "RMFGAEX9HUJC9ZAC"
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

/* The examples use WiFi configuration that you can set via project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define BUF_SIZE (1024)

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static const char *TAG = "main";

static int s_retry_num = 0;

static SemaphoreHandle_t mpu6050_mutex;
static MPU6050_t mpu6050_dev;

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
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    /* Setting a password implies station will connect to all security modes including WEP/WPA.
        * However these modes are deprecated and not advisable to be used. Incase your Access point
        * doesn't support WPA2, these mode can be enabled by commenting below line */

    if (strlen((char *)wifi_config.sta.password)) {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
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

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

// static void echo_task(void *arg)
// {
//     // Configure parameters of an UART driver,
//     // communication pins and install the driver
//     uart_config_t uart_config = {
//         .baud_rate = 9600,
//         .data_bits = UART_DATA_8_BITS,
//         .parity    = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
//     };
//     uart_param_config(UART_NUM_0, &uart_config);
//     uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

//     // Configure a temporary buffer for the incoming data
//     uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

//     while (1) {
//         // Read data from the UART
//         int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS);
//         // Write data back to the UART
//         //uart_write_bytes(UART_NUM_0, (const char *) data, len);
//         vTaskDelay(300 / portTICK_PERIOD_MS);
//     }
// }

void httpd_task(void *pvParameters)
{
    struct netconn *nc = netconn_new(NETCONN_TCP);
    if (!nc) {
        printf("Failed to allocate socket\n");
        vTaskDelete(NULL);
    }

    netconn_bind(nc, IP_ADDR_ANY, 80);
    netconn_listen(nc);

    char buf[1024];

    while (1) {

        struct netconn *client = NULL;
        struct netbuf *nb = NULL;

        err_t err = netconn_accept(nc, &client);
        if (err != ERR_OK || client == NULL) {
            continue;
        }

        err = netconn_recv(client, &nb);
        if (err == ERR_OK && nb != NULL) {

            void *data;
            u16_t len;
            netbuf_data(nb, &data, &len);

            char *req = (char *)data;

            printf("Received:\n%.*s\n", len, req);

            /* ---------------------------
               ROUTE: /toggle
            ----------------------------*/
            if (strncmp(req, "GET /toggle", 11) == 0) {

                gpio_set_level(LED_GPIO, !gpio_get_level(LED_GPIO));

                const char *resp =
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: text/plain\r\n"
                    "Connection: close\r\n\r\n"
                    "OK";

                netconn_write(client, resp, strlen(resp), NETCONN_COPY);
            }

            /* ---------------------------
               ROUTE: /data
            ----------------------------*/
            else if (strncmp(req, "GET /data", 9) == 0) {

                xSemaphoreTake(mpu6050_mutex, portMAX_DELAY);

                float local_temp = mpu6050_dev.temperature;
                int16_t accelX = (int16_t)((mpu6050_dev.sensor_data[0] << 8) | mpu6050_dev.sensor_data[1]);
                int16_t accelY = (int16_t)((mpu6050_dev.sensor_data[2] << 8) | mpu6050_dev.sensor_data[3]);
                int16_t accelZ = (int16_t)((mpu6050_dev.sensor_data[4] << 8) | mpu6050_dev.sensor_data[5]);

                xSemaphoreGive(mpu6050_mutex);

                int whole = (int)local_temp;
                int frac = abs((int)(local_temp * 100) % 100);

                snprintf(buf, sizeof(buf),
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: text/plain\r\n"
                    "Connection: close\r\n\r\n"
                    "%d.%02d,%d,%d,%d",
                    whole, frac,
                    accelX, accelY, accelZ
                );

                netconn_write(client, buf, strlen(buf), NETCONN_COPY);
            }

            /* ---------------------------
               ROUTE: /
            ----------------------------*/
            else {

                snprintf(buf, sizeof(buf),
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: text/html\r\n"
                    "Connection: close\r\n\r\n"

                    "<html><body>"
                    "<h2>ESP8266 Dashboard</h2>"

                    "<p>Temperature: <span id='temp'>--</span> C</p>"
                    "<p>Accel X: <span id='ax'>--</span></p>"
                    "<p>Accel Y: <span id='ay'>--</span></p>"
                    "<p>Accel Z: <span id='az'>--</span></p>"

                    "<button onclick=\"fetch('/toggle')\">Toggle LED</button>"

                    "<script>"
                    "setInterval(() => {"
                    " fetch('/data')"
                    "  .then(r => r.text())"
                    "  .then(data => {"
                    "    let values = data.split(',');"
                    "    document.getElementById('temp').innerText = values[0];"
                    "    document.getElementById('ax').innerText = values[1];"
                    "    document.getElementById('ay').innerText = values[2];"
                    "    document.getElementById('az').innerText = values[3];"
                    "  });"
                    "}, 100);"
                    "</script>"

                    "</body></html>"
                );

                netconn_write(client, buf, strlen(buf), NETCONN_COPY);
            }
        }

        if (nb) {
            netbuf_delete(nb);
        }

        netconn_close(client);
        netconn_delete(client);
    }
}

static void i2c_task_example(void *arg)
{
    xSemaphoreTake(mpu6050_mutex, portMAX_DELAY);
    memset(&mpu6050_dev, 0, sizeof(MPU6050_t));
    mpu6050_i2c_init(&mpu6050_dev, I2C_EXAMPLE_MASTER_NUM);
    xSemaphoreGive(mpu6050_mutex);

    uint8_t who_am_i, i;
    static uint32_t error_count = 0;
    int ret;

    while (1) {
        who_am_i = 0;

        xSemaphoreTake(mpu6050_mutex, portMAX_DELAY);
        mpu6050_i2c_read(&mpu6050_dev, WHO_AM_I, &who_am_i, 1);

        if (0x68 != who_am_i) {
            error_count++;
        }

        memset(mpu6050_dev.sensor_data, 0, 14);
        ret = mpu6050_i2c_read(&mpu6050_dev, ACCEL_XOUT_H, mpu6050_dev.sensor_data, 14);
        mpu6050_dev.temperature = (float)(36.53 + ((int16_t)((mpu6050_dev.sensor_data[6] << 8) | mpu6050_dev.sensor_data[7]) / 340.0));
        xSemaphoreGive(mpu6050_mutex);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "*******************\n");
            ESP_LOGI(TAG, "WHO_AM_I: 0x%02x\n", who_am_i);
            ESP_LOGI(TAG, "TEMP: %d.%d\n", (uint16_t)mpu6050_dev.temperature, (uint16_t)(mpu6050_dev.temperature * 100) % 100);
            for (i = 0; i < 7; i++) {
                ESP_LOGI(TAG, "sensor_data[%d]: %d\n", i, (int16_t)((mpu6050_dev.sensor_data[i * 2] << 8) | mpu6050_dev.sensor_data[i * 2 + 1]));
            }

            ESP_LOGI(TAG, "error_count: %d\n", error_count);
        } else {
            ESP_LOGE(TAG, "No ack, sensor not connected...skip...\n");
        }

        gpio_set_level(LED_GPIO, 0); // LED ON (active-low)
        vTaskDelay(100 / portTICK_PERIOD_MS);

        gpio_set_level(LED_GPIO, 1); // LED OFF
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    mpu6050_mutex = xSemaphoreCreateMutex();

    xTaskCreate(i2c_task_example, "i2c_task_example", 4096, NULL, 10, NULL);
    vTaskDelay(20/portTICK_PERIOD_MS);
    xTaskCreate(httpd_task, "httpd", 8192, NULL, 5, NULL);
    //xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);

    while(1)
    {
        printf("Hello from main...\n");
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}
