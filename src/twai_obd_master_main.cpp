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
#define TX_GPIO_NUM GPIO_NUM_5
#define RX_GPIO_NUM GPIO_NUM_16
#define CTRL_TAG "ctrl_task"
#define MAIN_TAG "fake obd device"
#define TX_TAG "tx_task"
#define RX_TAG "rx_task"

#define ID_MASTER_REQ_DTA 0x7DF
#define OBD_SVC_DTA 0x01
#define OBD_DEV_RPM 0x0C
#define OBD_DEV_SPD 0x0D
#define OBD_SVC_INF 0x09
#define OBD_INF_VIN 0x02
#define ID_SLAVE_RESP_DTA 0x7E8

#define REQ_PERIOD (pdMS_TO_TICKS(1000))
#define OBD_FRAME_SINGLE (0x00)
#define OBD_FRAME_FIRST (0x01)
#define OBD_FRAME_CONS (0x02)
#define OBD_FRAME_FLOW (0x03)
#define OBD_CONSEC_DELAY (0x0A);

#define MSB_NIBBLE(A) ((A >> 4) & 0x0F)
#define LSB_NIBBLE(A) ((A) & 0x0F)

typedef enum
{
    TX_SEND_REQ,
    RX_RECV_SLAVE_SNGL_FRST,
    TX_SEND_FLOW,
    RX_RECV_SLAVE_CONS,
    IDLE,
} ctrl_task_action_t;

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

/* -------------------------------------------------------------------------- */
/*                             Tasks and functions                            */
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

static void twai_ctrl_task(void *arg)
{
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    tx_task_action_t tx_action = TX_SEND;
    rx_task_action_t rx_action = RX_RECV;
    twai_message_t inc_msg;
    twai_message_t out_msg;
    uint8_t frame_len;
    uint8_t rem_frames;
    uint8_t dta_len = 0;
    uint8_t dta[4096];
    uint8_t frame_type;
    char buf[256];
    ctrl_task_action_t state;

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(CTRL_TAG, "obd probe started");

    TickType_t x_last_wake_time;
    const TickType_t x_period = REQ_PERIOD;
    x_last_wake_time = xTaskGetTickCount();

    ESP_LOGI(CTRL_TAG, "starting request loop");

    for (;;)
    {
        // wait until next cycle
        ESP_LOGI(CTRL_TAG, "making a new request");

        // reset finite state machine to make a request
        state = TX_SEND_REQ;
        dta_len = 0;

        // start request finite state machine
        while (state != IDLE)
        {
            switch (state)
            {
            case TX_SEND_REQ:
                // send a single frame
                out_msg.identifier = ID_MASTER_REQ_DTA;
                out_msg.data_length_code = 8;
                out_msg.data[0] = 0x02;
                out_msg.data[1] = OBD_SVC_DTA;
                out_msg.data[2] = OBD_DEV_RPM;
                ESP_LOGI(CTRL_TAG, "sending request %02x %02x", out_msg.data[1], out_msg.data[2]);
                xQueueSend(tx_msg_queue, &out_msg, portMAX_DELAY);
                xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
                xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
                state = RX_RECV_SLAVE_SNGL_FRST;
                break;
            case RX_RECV_SLAVE_SNGL_FRST:
                // get first frame
                ESP_LOGI(CTRL_TAG, "receiving frame from VMCU");
                xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
                xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
                xQueueReceive(rx_msg_queue, &inc_msg, portMAX_DELAY);
                frame_type = MSB_NIBBLE(inc_msg.data[0]);
                ESP_LOGI(CTRL_TAG, "first byte %02x", inc_msg.data[0]);
                if (frame_type == OBD_FRAME_SINGLE)
                {
                    ESP_LOGI(CTRL_TAG, "got single frame");
                    // all data is in this frame
                    frame_len = LSB_NIBBLE(inc_msg.data[0]);
                    memcpy(&dta[dta_len], &inc_msg.data[1], frame_len);
                    dta_len += frame_len;

                    state = IDLE;
                }
                else if (frame_type == OBD_FRAME_FIRST)
                {
                    // unimplemented
                    ESP_LOGI(CTRL_TAG, "got first frame");
                    state = IDLE;
                }
                else
                {
                    // ??? confusion ???
                    ESP_LOGI(CTRL_TAG, "got unknown frame %d", frame_type);
                    state = IDLE;
                }
                break;
            case TX_SEND_FLOW:
            case RX_RECV_SLAVE_CONS:
                // unimplemented
                break;
            case IDLE:
                break;
            default:
                // broken
                break;
            }
        }

        // report data
        ESP_LOGI(CTRL_TAG, "Received data: ");
        for (int i = 0; i < dta_len; i += 8)
        {
            char *ptr = buf;
            for (int j = 0; j < MIN(8, dta_len - i); j++)
            {
                ptr += sprintf(ptr, "%02x ", dta[i + j]);
            }
            ESP_LOGI(CTRL_TAG, "%s", buf);
        }

        vTaskDelayUntil(&x_last_wake_time, x_period);
    }
}

/* -------------------------------------------------------------------------- */
/*                              Application main                              */
/* -------------------------------------------------------------------------- */

extern "C" void app_main(void)
{
    // short bootup delay to get debug serial connected and slave online
    for (int i = 5; i > 0; i--)
    {
        ESP_LOGI(MAIN_TAG, "starting in %d", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // check for drivers correctly installed
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(MAIN_TAG, "TWAI driver started");

    // create semaphores and tasks
    rx_task_queue = xQueueCreate(1, sizeof(rx_task_action_t));
    rx_msg_queue = xQueueCreate(1, sizeof(twai_message_t));
    tx_task_queue = xQueueCreate(1, sizeof(tx_task_action_t));
    tx_msg_queue = xQueueCreate(1, sizeof(twai_message_t));
    ctrl_task_sem = xSemaphoreCreateBinary();

    ESP_LOGI(MAIN_TAG, "starting tasks");

    xTaskCreatePinnedToCore(
        twai_ctrl_task,
        "TWAI_ctrl",
        65536,
        NULL,
        CTRL_TASK_PRIO,
        NULL,
        tskNO_AFFINITY);

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

    // start control task
    xSemaphoreGive(ctrl_task_sem);

    // tasks running, return
    return;
}