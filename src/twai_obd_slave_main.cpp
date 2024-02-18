#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include <string.h>
#include "esp_random.h"

/* -------------------------------------------------------------------------- */
/*                      Definitions and static variables                      */
/* -------------------------------------------------------------------------- */
#ifndef MIN
#define MIN(a, b) ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#define RX_TASK_PRIO 8
#define TX_TASK_PRIO 9
#define CTRL_TASK_PRIO 10
#define OBD_TASK_PRIO 7
#define TX_GPIO_NUM GPIO_NUM_5
#define RX_GPIO_NUM GPIO_NUM_16
#define CTRL_TAG "ctrl_task"
#define MAIN_TAG "fake vmcu"
#define RX_TAG "rx_task"
#define TX_TAG "tx_task"

#define ID_MASTER_REQ_DTA 0x7DF
#define OBD_SVC_DTA 0x01
#define OBD_DEV_RPM 0x0C
#define OBD_DEV_SPD 0x0D
#define OBD_SVC_INF 0x09
#define OBD_INF_VIN 0x02
#define ID_SLAVE_RESP_DTA 0x7E8

#define INFO_UPDATE_PERIOD (pdMS_TO_TICKS(300));

typedef enum
{
    RX_RECV_REQ,
    TX_SEND_SNGL,
    TX_SEND_FRST,
    RX_RECV_FLOW,
    TX_SEND_CONS,
    IDLE,
} ctrl_task_action_t;

static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    TX_GPIO_NUM,
    RX_GPIO_NUM,
    TWAI_MODE_NORMAL);

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

static SemaphoreHandle_t twai_task_sem;
static SemaphoreHandle_t obd_info_mut;

static uint16_t rpm;
static uint8_t speed;

#define MSB_BYTE(A) ((A >> 8) & 0xFF)
#define LSB_BYTE(A) ((A) & 0xFF)
#define MSB_NIBBLE(A) ((A >> 4) & 0x0F)
#define LSB_NIBBLE(A) ((A) & 0x0F)

// KMHC75LD0MU250580
static const uint8_t vin[] = {
    0x01, 0x4B, 0x4D, 0x48, 0x43, 
    0x37, 0x35, 0x4C, 0x44, 0x30,
    0x4D, 0x55, 0x32, 0x35, 0x30,
    0x35, 0x38, 0x30};

/* -------------------------------------------------------------------------- */
/*                             Tasks and Functions                            */
/* -------------------------------------------------------------------------- */

static void twai_control_task(void *arg)
{
    xSemaphoreTake(twai_task_sem, portMAX_DELAY);
    twai_message_t inc_msg; // incoming and outgoing relative to control task
    twai_message_t out_msg;
    uint16_t rem_dta; // len remaining data in dta
    uint16_t dta_len;  // current location of data in dta to be transmitted
    uint8_t clear_to_send = 0; // remaining #of frames clear-to-send
    uint8_t frame_len = 0; // number of data bytes in this frame
    uint8_t cons_delay = 0; // delay between consecutive frames
    uint8_t counter = 1; // mod 0x0F counter for consective frames
    uint8_t dta[4096]; // maximum length of data by CAN-TP spec
    ctrl_task_action_t state;

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(CTRL_TAG, "TWAI driver started");

    for (;;)
    {
        // reset finite state machine for the next request
        ESP_LOGI(CTRL_TAG, "receive request");
        state = RX_RECV_REQ;
        rem_dta = 0;
        dta_len = 0;

        while (state != IDLE)
        {
            switch (state)
            {
            case RX_RECV_REQ:
                // listen for the next time the obd diagnostic tool asks for something
                twai_receive(&inc_msg, portMAX_DELAY);
                ESP_LOGI(
                    CTRL_TAG, 
                    "identified request %02x %02x", 
                    inc_msg.data[1], 
                    inc_msg.data[2]
                );

                dta[0] = (0x01 << 6) | inc_msg.data[1];
                dta[1] = inc_msg.data[2];

                // response based on service and device
                switch (inc_msg.data[1])
                {
                case OBD_SVC_DTA:
                    switch (inc_msg.data[2])
                    {
                    case OBD_DEV_RPM:
                        // 0x01 0x0C
                        xSemaphoreTake(obd_info_mut, portMAX_DELAY);
                        dta[2] = (uint8_t) (rpm >> 8);
                        dta[3] = (uint8_t) rpm;
                        xSemaphoreGive(obd_info_mut);
                        rem_dta = 4;
                        break;
                    case OBD_DEV_SPD:
                        // 0x01 0x0D
                        xSemaphoreTake(obd_info_mut, portMAX_DELAY);
                        dta[2] = speed;
                        xSemaphoreGive(obd_info_mut);
                        rem_dta = 3;
                        break;
                    default:
                        // unsupported device
                        ESP_LOGE(CTRL_TAG, "identfied unsupported device!");
                        break;
                    }
                    break;
                case OBD_SVC_INF:
                    switch (inc_msg.data[2])
                    {
                    case OBD_INF_VIN:
                        // 0x09 0x02
                        memcpy(&dta[2], vin, 18);
                        rem_dta = 20;
                        break;
                    default:
                        // unsupported info
                        ESP_LOGE(CTRL_TAG, "identified unsupported info!");
                        break;
                    }
                    break;
                default:
                    // unsupported service
                    ESP_LOGE(CTRL_TAG, "identified unsupported service!");
                    break;
                }

                if (rem_dta > 7)
                {
                    state = TX_SEND_FRST;
                }
                else 
                {
                    state = TX_SEND_SNGL;
                }
                break;
            case TX_SEND_SNGL:
                ESP_LOGI(
                    CTRL_TAG, 
                    "transmit single frame (%d bytes remain)",
                    rem_dta
                    );
                out_msg.identifier = ID_SLAVE_RESP_DTA;
                out_msg.data_length_code = 8;
                frame_len = rem_dta;
                out_msg.data[0] = frame_len; // 0x0L
                memcpy(&out_msg.data[1], &dta[dta_len], frame_len);
                dta_len += frame_len;
                rem_dta -= frame_len;

                twai_transmit(&out_msg, portMAX_DELAY);
                state = IDLE;
                break;
            case TX_SEND_FRST:
                ESP_LOGI(
                    CTRL_TAG, 
                    "transmit first frame (%d bytes remain)", 
                    rem_dta
                    );
                out_msg.identifier = ID_SLAVE_RESP_DTA;
                out_msg.data_length_code = 8;
                frame_len = 6;
                out_msg.data[0] = (0x01 << 4) | LSB_NIBBLE(MSB_BYTE(rem_dta));
                out_msg.data[1] = LSB_BYTE(rem_dta);
                memcpy(&out_msg.data[2], &dta[dta_len], frame_len);
                dta_len += frame_len;
                rem_dta -= frame_len;
                counter = 1;

                twai_transmit(&out_msg, portMAX_DELAY);
                state = RX_RECV_FLOW;
                break;
            case RX_RECV_FLOW:
                ESP_LOGI(CTRL_TAG, "receive clear-to-send");
                twai_receive(&inc_msg, portMAX_DELAY);
                clear_to_send = inc_msg.data[1];
                cons_delay = inc_msg.data[2];
                
                ESP_LOGI(
                    CTRL_TAG,
                    "identified BS: %02x; STmin: %02x",
                    clear_to_send,
                    cons_delay      
                );

                state = TX_SEND_CONS; 
                break;
            case TX_SEND_CONS:
                vTaskDelay(pdMS_TO_TICKS(cons_delay));
                ESP_LOGI(
                    CTRL_TAG,
                    "transmit consecutive (%02x; %d bytes remain)", 
                    clear_to_send,
                    rem_dta
                );

                out_msg.identifier = ID_SLAVE_RESP_DTA;
                out_msg.data_length_code = 8;
                frame_len = MIN(7, rem_dta);
                out_msg.data[0] = 0x20 + counter;
                memcpy(&out_msg.data[1], &dta[dta_len], frame_len);
                twai_transmit(&out_msg, portMAX_DELAY);
                dta_len += frame_len;
                rem_dta -= frame_len;
                counter = (counter + 1) % 0x0F;

                if (rem_dta)
                {
                    if (clear_to_send == 1)
                    {
                        state = RX_RECV_FLOW;
                    }
                    else
                    {
                        state = TX_SEND_CONS;
                    }

                    if (clear_to_send)
                        clear_to_send--;
                }
                else
                {
                    state = IDLE;
                }
                break;
            case IDLE:
            default:
                state = IDLE;
                break;
            }
        }

        ESP_LOGI(CTRL_TAG, "finish transaction");
    }
}

static void obd_info_task(void *arg)
{
    TickType_t x_last_wake_time;
    const TickType_t x_period = INFO_UPDATE_PERIOD;
    x_last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&x_last_wake_time, x_period);
        uint32_t rand = esp_random();

        xSemaphoreTake(obd_info_mut, portMAX_DELAY);
        speed = (uint8_t) (rand & 0xFF);
        rpm = (uint16_t) ((rand >> 8) & 0xFFFF);
        xSemaphoreGive(obd_info_mut);
    }
}

/* -------------------------------------------------------------------------- */
/*                              Application main                              */
/* -------------------------------------------------------------------------- */

extern "C" void app_main(void)
{
    // short bootup delay to get debug serial connected
    for (int i = 3; i > 0; i--)
    {
        ESP_LOGI(MAIN_TAG, "starting in %d", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // create semaphores and tasks
    obd_info_mut = xSemaphoreCreateMutex();
    twai_task_sem = xSemaphoreCreateBinary();

    ESP_LOGI(MAIN_TAG, "starting tasks");

    xTaskCreatePinnedToCore(
        twai_control_task,
        "TWAI_ctrl",
        16384,
        NULL,
        CTRL_TASK_PRIO,
        NULL,
        tskNO_AFFINITY);

    xTaskCreatePinnedToCore(
        obd_info_task,
        "OBD_rng",
        16384,
        NULL,
        OBD_TASK_PRIO,
        NULL,
        tskNO_AFFINITY);

    // check for drivers correctly installed
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(CTRL_TAG, "TWAI driver started");

    // start control task
    xSemaphoreGive(twai_task_sem);

    // tasks running, return :)
    return;
}