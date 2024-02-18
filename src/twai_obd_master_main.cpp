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

#define OBD_TASK_PRIO 8
#define CTRL_TASK_PRIO 10
#define TX_GPIO_NUM GPIO_NUM_5
#define RX_GPIO_NUM GPIO_NUM_16
#define CTRL_TAG "twai_task"
#define MAIN_TAG "fake obd device"
#define VIN_TAG "vin_task"
#define RPM_TAG "rpm_task"
#define SPEED_TAG "speed_task"

#define ID_MASTER_REQ_DTA 0x7DF
#define ID_SLAVE_RESP_DTA 0x7E8
#define OBD_SVC_DTA 0x01
#define OBD_DEV_RPM 0x0C
#define OBD_DEV_SPD 0x0D
#define OBD_SVC_INF 0x09
#define OBD_INF_VIN 0x02

#define VIN_PERIOD (pdMS_TO_TICKS(10000))
#define SPEED_PERIOD (pdMS_TO_TICKS(1000));
#define RPM_PERIOD (pdMS_TO_TICKS(2000));
#define OBD_FRAME_SINGLE (0x00)
#define OBD_FRAME_FIRST (0x01)
#define OBD_FRAME_CONS (0x02)
#define OBD_FRAME_FLOW (0x03)
#define OBD_CONSEC_DELAY (0x0A)
#define OBD_CONSEC_COUNT (0x00)

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

typedef struct 
{
    // response from VMCU
    uint8_t *dta;
    uint16_t *dta_len;
    uint16_t max_len;

    // transmission to VMCU
    uint8_t service; // service 0x01 - 0x09
    uint8_t s_id; // service ID depends on availability of services

    // task control
    SemaphoreHandle_t *sem;
} obd_transaction_t;

static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    TX_GPIO_NUM,
    RX_GPIO_NUM,
    TWAI_MODE_NORMAL);

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static SemaphoreHandle_t twai_task_sem;
static SemaphoreHandle_t vin_task_sem;
static SemaphoreHandle_t rpm_task_sem;
static SemaphoreHandle_t speed_task_sem;

static QueueHandle_t obd_trans_queue;

/* -------------------------------------------------------------------------- */
/*                             Tasks and functions                            */
/* -------------------------------------------------------------------------- */

void esp_log_buffer(const char *tag, uint8_t *dta, uint16_t dta_len)
{
    char buf[32]; // buffer for debug printing
    for (int i = 0; i < dta_len; i += 8)
    {
        char *ptr = buf;
        for (int j = 0; j < MIN(8, dta_len - i); j++)
        {
            ptr += sprintf(ptr, "%02x ", dta[i + j]);
        }
        ESP_LOGI(tag, "%s", buf);
    }
}

static void vin_task(void *arg)
{
    xSemaphoreTake(vin_task_sem, portMAX_DELAY);
    ESP_LOGI(VIN_TAG, "vin task started");
    obd_transaction_t out_trans;
    uint8_t dta[256];
    uint16_t dta_len;


    TickType_t x_last_wake_time;
    const TickType_t x_period = VIN_PERIOD;
    x_last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&x_last_wake_time, x_period);

        // generate transaction
        out_trans.dta = dta;
        out_trans.dta_len = &dta_len;
        out_trans.max_len = 256;
        out_trans.service = OBD_SVC_INF;
        out_trans.s_id = OBD_INF_VIN;
        out_trans.sem = &vin_task_sem;
        
        // hand request to twai task
        xQueueSend(obd_trans_queue, &out_trans, portMAX_DELAY);
        xSemaphoreTake(vin_task_sem, portMAX_DELAY);

        // log output (process in real task)
        ESP_LOGI(VIN_TAG, "Received data: ");
        esp_log_buffer(VIN_TAG, dta, dta_len);
    }
}

static void speed_task(void *arg)
{
    xSemaphoreTake(speed_task_sem, portMAX_DELAY);
    ESP_LOGI(VIN_TAG, "speed task started");
    obd_transaction_t out_trans;
    uint8_t dta[256];
    uint16_t dta_len;


    TickType_t x_last_wake_time;
    const TickType_t x_period = SPEED_PERIOD;
    x_last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&x_last_wake_time, x_period);

        // generate transaction
        out_trans.dta = dta;
        out_trans.dta_len = &dta_len;
        out_trans.max_len = 256;
        out_trans.service = OBD_SVC_DTA;
        out_trans.s_id = OBD_DEV_SPD;
        out_trans.sem = &speed_task_sem;
        
        // hand request to twai task
        xQueueSend(obd_trans_queue, &out_trans, portMAX_DELAY);
        xSemaphoreTake(speed_task_sem, portMAX_DELAY);

        // log output (process in real task)
        ESP_LOGI(SPEED_TAG, "Received data: ");
        esp_log_buffer(SPEED_TAG, dta, dta_len);
    }
}

static void rpm_task(void *arg)
{
    xSemaphoreTake(rpm_task_sem, portMAX_DELAY);
    ESP_LOGI(RPM_TAG, "rpm task started");
    obd_transaction_t out_trans;
    uint8_t dta[256];
    uint16_t dta_len;

    TickType_t x_last_wake_time;
    const TickType_t x_period = RPM_PERIOD;
    x_last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&x_last_wake_time, x_period);

        // generate transaction
        out_trans.dta = dta;
        out_trans.dta_len = &dta_len;
        out_trans.max_len = 256;
        out_trans.service = OBD_SVC_DTA;
        out_trans.s_id = OBD_DEV_RPM;
        out_trans.sem = &rpm_task_sem;
        
        // hand request to twai task
        xQueueSend(obd_trans_queue, &out_trans, portMAX_DELAY);
        xSemaphoreTake(rpm_task_sem, portMAX_DELAY);

        // log output (process in real task)
        ESP_LOGI(RPM_TAG, "Received data: ");
        esp_log_buffer(RPM_TAG, dta, dta_len);
    }
}

static void twai_ctrl_task(void *arg)
{
    xSemaphoreTake(twai_task_sem, portMAX_DELAY);
    twai_message_t inc_msg;
    twai_message_t out_msg;
    uint8_t frame_len;
    uint8_t clear_to_send; // remaining frames before another clear-to-send is necessary
    uint8_t rem_dta;       // remaining data length, used for multiple frame transaction
    uint8_t frame_type;    // storage for frame type
    ctrl_task_action_t state;
    obd_transaction_t t;

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(CTRL_TAG, "twai task started");

    for (;;)
    {
        // wait for another task to ask for an obd service
        xQueueReceive(obd_trans_queue, &t, portMAX_DELAY);

        // reset finite state machine to make a request
        state = TX_SEND_REQ;
        *t.dta_len = 0;

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
                out_msg.data[1] = t.service;
                out_msg.data[2] = t.s_id;                
                for (int i = 3; i < 8; i++)
                {
                    out_msg.data[i] = 0xAA;
                }
                ESP_LOGI(
                    CTRL_TAG,
                    "transmit request %02x %02x",
                    out_msg.data[1],
                    out_msg.data[2]);
                twai_transmit(&out_msg, portMAX_DELAY);
                esp_log_buffer(CTRL_TAG, out_msg.data, 8);
                state = RX_RECV_SLAVE_SNGL_FRST;
                break;
            case RX_RECV_SLAVE_SNGL_FRST:
                // get first frame
                ESP_LOGI(
                    CTRL_TAG, 
                    "receive single/first frame");
                twai_receive(&inc_msg, portMAX_DELAY);
                esp_log_buffer(CTRL_TAG, inc_msg.data, 8);
                frame_type = MSB_NIBBLE(inc_msg.data[0]);
                if (frame_type == OBD_FRAME_SINGLE)
                {
                    ESP_LOGI(CTRL_TAG, "identified single frame");
                    // all data is in this frame
                    frame_len = LSB_NIBBLE(inc_msg.data[0]);
                    memcpy(&t.dta[*t.dta_len], &inc_msg.data[1], frame_len);
                    rem_dta = 0;
                    *t.dta_len += frame_len;

                    state = IDLE;
                }
                else if (frame_type == OBD_FRAME_FIRST)
                {
                    // first 6 bytes of data is in this frame
                    frame_len = 6;
                    memcpy(&t.dta[*t.dta_len], &inc_msg.data[2], frame_len);
                    rem_dta = LSB_NIBBLE(inc_msg.data[0]) | inc_msg.data[1];
                    ESP_LOGI(
                        CTRL_TAG,
                        "identified first frame (%d bytes remain)",
                        rem_dta);
                    rem_dta -= frame_len;
                    *t.dta_len += frame_len;

                    state = TX_SEND_FLOW;
                }
                else
                {
                    // ??? confusion ???
                    ESP_LOGI(
                        CTRL_TAG,
                        "identified unknown frame %d",
                        frame_type);
                    state = IDLE;
                }
                break;
            case TX_SEND_FLOW:
                out_msg.identifier = ID_MASTER_REQ_DTA;
                out_msg.data_length_code = 8;
                out_msg.data[0] = 0x30;
                out_msg.data[1] = OBD_CONSEC_COUNT;
                out_msg.data[2] = OBD_CONSEC_DELAY;
                for (int i = 3; i < 8; i++)
                {
                    out_msg.data[i] = 0xAA;
                }
                clear_to_send = OBD_CONSEC_COUNT;
                ESP_LOGI(
                    CTRL_TAG,
                    "transmit clear-to-send BS: %02x; STmin: %02x",
                    out_msg.data[1],
                    out_msg.data[2]);
                twai_transmit(&out_msg, portMAX_DELAY);
                esp_log_buffer(CTRL_TAG, out_msg.data, 8);

                state = RX_RECV_SLAVE_CONS;
                break;
            case RX_RECV_SLAVE_CONS:
                ESP_LOGI(
                    CTRL_TAG,
                    "receive consecutive (%02x / %02x; %d bytes remain)",
                    clear_to_send,
                    OBD_CONSEC_COUNT,
                    rem_dta);

                twai_receive(&inc_msg, portMAX_DELAY);
                esp_log_buffer(CTRL_TAG, inc_msg.data, 8);
                frame_len = MIN(7, rem_dta);
                memcpy(&t.dta[*t.dta_len], &inc_msg.data[1], frame_len);
                *t.dta_len += frame_len;
                rem_dta -= frame_len;

                if (rem_dta)
                {
                    if (clear_to_send == 1)
                    {
                        state = TX_SEND_FLOW;
                    }
                    else
                    {
                        state = RX_RECV_SLAVE_CONS;
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

        // notify task that data is written to buffer supplied in request
        xSemaphoreGive(*t.sem);
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

    // inter-process communication
    twai_task_sem = xSemaphoreCreateBinary();
    vin_task_sem = xSemaphoreCreateBinary();
    speed_task_sem = xSemaphoreCreateBinary();
    rpm_task_sem = xSemaphoreCreateBinary();

    obd_trans_queue = xQueueCreate(5, sizeof(obd_transaction_t));

    ESP_LOGI(MAIN_TAG, "starting tasks");

    // create tasks
    xTaskCreatePinnedToCore(
        twai_ctrl_task,
        "twai_task",
        65536,
        NULL,
        CTRL_TASK_PRIO,
        NULL,
        tskNO_AFFINITY);
    
    xTaskCreatePinnedToCore(
        vin_task,
        "vin_task",
        16384,
        NULL,
        OBD_TASK_PRIO,
        NULL,
        tskNO_AFFINITY
    );

    // xTaskCreatePinnedToCore(
    //     speed_task,
    //     "speed_task",
    //     16384,
    //     NULL,
    //     OBD_TASK_PRIO,
    //     NULL,
    //     tskNO_AFFINITY
    // );

    // xTaskCreatePinnedToCore(
    //     rpm_task,
    //     "rpm_task",
    //     16384,
    //     NULL,
    //     OBD_TASK_PRIO,
    //     NULL,
    //     tskNO_AFFINITY
    // );

    // start control task
    xSemaphoreGive(twai_task_sem);
    xSemaphoreGive(vin_task_sem);
    xSemaphoreGive(speed_task_sem);
    xSemaphoreGive(rpm_task_sem);

    // tasks running, return
    return;
}