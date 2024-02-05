
#include <freertos/FreeRTOS.h>
#include <esp_system.h>
#include <esp_event.h>
#include <driver/gpio.h>
#include <driver/twai.h>
#include <esp_log.h>

#define RX_GPIO_PIN (GPIO_NUM_5)
#define TX_GPIO_PIN (GPIO_NUM_16)
#define PING_DELAY (2000 / portTICK_PERIOD_MS)

static const char *TAG = "main";
static bool driver_installed = false;

void twai_alert_task(void *taskArgs) 
{
    ESP_LOGI(TAG, "starting twai alert task...");
    uint32_t alerts_triggered;
    twai_status_info_t status_info;
    twai_message_t message;
    message.identifier = 0x7E8;
    message.extd = 0;
    message.data_length_code = 5;
    message.data[0] = 0x04;
    message.data[1] = 0x41;
    message.data[2] = 0x0C;
    message.data[3] = 0x00;
    message.data[4] = 0x35;

    if (!driver_installed)
    {
        ESP_LOGE(TAG, "driver not installed, ending twai task");
        vTaskDelete(NULL);
    } 

    for (;;)
    {
        // blocking wait indefinitely for alerts
        twai_read_alerts(&alerts_triggered, PING_DELAY);
        twai_get_status_info(&status_info);

        // handle alerts
        if (alerts_triggered & TWAI_ALERT_ERR_PASS)
        {
            ESP_LOGI(TAG, "twai controller has become error passive");
        }
        else if (alerts_triggered & TWAI_ALERT_BUS_ERROR) 
        {
            ESP_LOGI(TAG, "twai controller found %lu errors (bit, stuff, crc, form, ack)", status_info.bus_error_count);
        }
        else if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) 
        {
            ESP_LOGI(TAG, "twai controller overrun (%lu buffered, %lu missed, %lu overrun)", status_info.msgs_to_rx, status_info.rx_missed_count, status_info.rx_overrun_count);
        }
        else if (alerts_triggered & TWAI_ALERT_RX_DATA) 
        {
            // handle all pending messages
            twai_message_t message;
            while (twai_receive(&message, 0) == ESP_OK)
            {
                ESP_LOGI(TAG, "twai recieved: [%lu]#", message.identifier);
                for (int i = 0; i < message.data_length_code; i++) 
                {
                    printf("%02x ", message.data[i]);
                }
                printf("\n");
            }
        }
        else 
        {
            // lets send something!
            if (twai_transmit(&message, portMAX_DELAY) == ESP_OK)
            {
                ESP_LOGI(TAG, "twai controller sending message");
            }
            else 
            {
                ESP_LOGI(TAG, "twai controller failed send message");
            } 
        }
    }
}

extern "C" void app_main()
{
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    // configure twai
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_PIN, RX_GPIO_PIN, TWAI_MODE_NORMAL);
    general_config.tx_queue_len = 100;
    general_config.rx_queue_len = 100;
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_LOGI(TAG, "installing twai driver...");
    if (twai_driver_install(&general_config, &timing_config, &filter_config) == ESP_OK)
    {
        ESP_LOGI(TAG, "installed twai driver");

        if (twai_start() == ESP_OK)
        {
            ESP_LOGI(TAG, "started twai driver");

            // Reconfigure the alerts to detect the error of frames received, Bus-Off error and RX queue full error
            /*
            TWAI_ALERT_RX_DATA        0x00000004    Alert(4)    : A frame has been received and added to the RX queue
            TWAI_ALERT_ERR_PASS       0x00001000    Alert(4096) : TWAI controller has become error passive
            TWAI_ALERT_BUS_ERROR      0x00000200    Alert(512)  : A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus
            TWAI_ALERT_RX_QUEUE_FULL  0x00000800    Alert(2048) : The RX queue is full causing a frame to be lost
            */
            uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
            // uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL;

            if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
            {
                ESP_LOGI(TAG, "configured twai alerts");
                driver_installed = true;
            }
        }
    }

    // start tasks
    xTaskCreate(
        twai_alert_task,
        "TWAI task",
        16384,
        NULL,
        1,
        NULL
    );

    // xTaskCreate(
    //     twai_transmit_task,
    //     "TWAI RX task",
    //     16384,
    //     NULL,
    //     1,
    //     NULL
    // );

    for (;;)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}