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
    TX_SEND,
    TX_IDLE,
} tx_task_action_t;

typedef enum
{
    RX_RECV,
    RX_IDLE,
} rx_task_action_t;

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

static QueueHandle_t tx_task_queue;
static QueueHandle_t rx_task_queue;
static QueueHandle_t tx_msg_queue;
static QueueHandle_t rx_msg_queue;

static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t obd_info_mut;

static uint16_t rpm;
static uint8_t speed;
// KMHC75LD0MU250580
static const uint8_t vin[] = {
    0x4B, 0x4D, 0x48, 0x43, 0x37, 
    0x35, 0x4C, 0x44, 0x30, 0x4D, 
    0x55, 0x32, 0x35, 0x30, 0x35, 
    0x38, 0x30};

/* -------------------------------------------------------------------------- */
/*                             Tasks and Functions                            */
/* -------------------------------------------------------------------------- */

static void twai_receive_task(void *arg)
{
    rx_task_action_t rx_action;
    twai_message_t msg;

    for (;;)
    {
        xQueueReceive(rx_task_queue, &rx_action, portMAX_DELAY);
        switch (rx_action)
        {
        case RX_RECV:
            ESP_LOGI(RX_TAG, "start receive");
            twai_receive(&msg, portMAX_DELAY);
            xQueueSend(rx_msg_queue, &msg, portMAX_DELAY);
            xSemaphoreGive(ctrl_task_sem);
            break;
        default:
            // idle
            break;
        }
    }
}

static void twai_transmit_task(void *arg)
{
    tx_task_action_t tx_action;
    twai_message_t msg;

    for (;;)
    {
        xQueueReceive(tx_task_queue, &tx_action, portMAX_DELAY);
        switch (tx_action)
        {
        case TX_SEND:
            ESP_LOGI(TX_TAG, "start send");
            xQueueReceive(tx_msg_queue, &msg, portMAX_DELAY);
            twai_transmit(&msg, portMAX_DELAY);
            xSemaphoreGive(ctrl_task_sem);
            break;
        default:
            // idle
            break;
        }
    }
}

static void twai_control_task(void *arg)
{
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    tx_task_action_t tx_action = TX_SEND;
    rx_task_action_t rx_action = RX_RECV;
    twai_message_t inc_msg; // incoming and outgoing relative to control task
    twai_message_t out_msg;
    uint16_t rem_len;
    uint8_t dta[4096]; // maximum length of data by CAN-TP spec
    ctrl_task_action_t state;

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(CTRL_TAG, "TWAI driver started");

    for (;;)
    {
        // reset finite state machine for the next request
        ESP_LOGI(CTRL_TAG, "waiting for next request from master");
        state = RX_RECV_REQ;
        rem_len = 0;

        while (state != IDLE)
        {
            switch (state)
            {
            case RX_RECV_REQ:
                // listen for the next time the obd diagnostic tool asks for something
                xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
                xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
                xQueueReceive(rx_msg_queue, &inc_msg, portMAX_DELAY);
                ESP_LOGI(
                    CTRL_TAG, 
                    "got request %02x %02x from master", 
                    inc_msg.data[1], 
                    inc_msg.data[2]
                );

                // response based on service and device
                switch (inc_msg.data[1])
                {
                case OBD_SVC_DTA:
                    switch (inc_msg.data[2])
                    {
                    case OBD_DEV_RPM:
                        // 0x01 0x0C
                        xSemaphoreTake(obd_info_mut, portMAX_DELAY);
                        dta[0] = (uint8_t) (rpm >> 8);
                        dta[1] = (uint8_t) rpm;
                        xSemaphoreGive(obd_info_mut);
                        rem_len = 2;
                        break;
                    case OBD_DEV_SPD:
                        // 0x01 0x0D
                        xSemaphoreTake(obd_info_mut, portMAX_DELAY);
                        dta[0] = speed;
                        xSemaphoreGive(obd_info_mut);
                        rem_len = 1;
                        break;
                    default:
                        // unsupported device
                        ESP_LOGE(CTRL_TAG, "unsupported device!");
                        break;
                    }
                    break;
                case OBD_SVC_INF:
                    switch (inc_msg.data[2])
                    {
                    case OBD_INF_VIN:
                        // 0x09 0x02
                        memcpy(dta, vin, 17);
                        rem_len = 17;
                        break;
                    default:
                        // unsupported info
                        ESP_LOGE(CTRL_TAG, "unsupported info!");
                        break;
                    }
                    break;
                default:
                    // unsupported service
                    ESP_LOGE(CTRL_TAG, "unsupported service!");
                    break;
                }

                if (rem_len > 7)
                {
                    state = TX_SEND_FRST;
                }
                else 
                {
                    state = TX_SEND_SNGL;
                }
                break;
            case TX_SEND_SNGL:
                out_msg.identifier = ID_SLAVE_RESP_DTA;
                out_msg.data_length_code = 8;
                out_msg.data[0] = rem_len; // 0x0L
                memcpy(&out_msg.data[1], dta, rem_len);

                xQueueSend(tx_msg_queue, &out_msg, portMAX_DELAY);
                xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
                xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
                state = IDLE;
                break;
            case TX_SEND_FRST:
            case RX_RECV_FLOW:
            case TX_SEND_CONS:
            case IDLE:
            default:
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
    rx_task_queue = xQueueCreate(1, sizeof(rx_task_action_t));
    rx_msg_queue = xQueueCreate(1, sizeof(twai_message_t));
    tx_task_queue = xQueueCreate(1, sizeof(tx_task_action_t));
    tx_msg_queue = xQueueCreate(1, sizeof(twai_message_t));
    obd_info_mut = xSemaphoreCreateMutex();
    ctrl_task_sem = xSemaphoreCreateBinary();

    ESP_LOGI(MAIN_TAG, "starting tasks");

    xTaskCreatePinnedToCore(
        twai_receive_task,
        "TWAI_rx",
        16384,
        NULL,
        RX_TASK_PRIO,
        NULL,
        tskNO_AFFINITY);

    xTaskCreatePinnedToCore(
        twai_transmit_task,
        "TWAI_tx",
        16384,
        NULL,
        TX_TASK_PRIO,
        NULL,
        tskNO_AFFINITY);

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
    xSemaphoreGive(ctrl_task_sem);

    // tasks running, return :)
    return;
}