#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <driver/gpio.h>

#include "motor_control.h"
#include "ultrasonic_sensor.h"
#include "line_follow.h"
#include "bluetooth_com.h"
#include "ase_typedefs.h"
#include "helpers.h"

static const char *TAG = "orchestrator";

#define LOG_OVER_BLUETOOTH

static QueueHandle_t bt_tosend_h;

#ifdef LOG_OVER_BLUETOOTH
static int dual_vprintf(const char *fmt, va_list ap)
{
    static bt_com_msg_t bt_msg;
    bt_msg.len = vsnprintf((char *)bt_msg.data, BT_MSG_BUF_SIZE_BYTES, fmt, ap);
    if (bt_msg.len > 0)
    {
        xQueueSend(bt_tosend_h, &bt_msg, pdMS_TO_TICKS(0));
    }

    return vprintf(fmt, ap);
}
#endif

void app_main()
{
    QueueHandle_t sonar_queue_h = xQueueCreate(5, sizeof(ultrasonic_measurement_t));
    QueueHandle_t motors_control_queue_h = xQueueCreate(10, sizeof(motors_control_msg_t));
    sonar_motors_q_ok_or_abort(sonar_queue_h, motors_control_queue_h, TAG);
    static QueueHandle_t bt_rcv_h;
    bt_rcv_h = xQueueCreate(5, sizeof(bt_com_msg_t));
    bt_tosend_h = xQueueCreate(35, sizeof(bt_com_msg_t));

#ifdef LOG_OVER_BLUETOOTH
    // Line below enables sending app logs over bluetooth
    esp_log_set_vprintf(dual_vprintf);
    // Disable BT_HCI logs as they are truly useless
    esp_log_level_set("BT_HCI", ESP_LOG_NONE);
#endif
    // Create motor control task
    xTaskCreate(&motor_control_task, "motor_control_task", 4096, (void *)motors_control_queue_h, 15, NULL);
    // Create ultrasonic sensor scanner task
    xTaskCreate(&ultrasonic_sensor_task, "ultrasonic_sensor_task", 4096, (void *)sonar_queue_h, 10, NULL);

    ultrasonic_measurement_t sonar_notif = {
        .angle = 0,
        .distance = 0.0f};

    motors_control_msg_t motors_control = {
        .speed_cmd = {
            .left = 0.85f,
            .right = 0.85f}};

    line_follower_task_context_t lf_ctx = {
        .mot_cmd_q_handle = motors_control_queue_h,
        .mot_ctrl_msg = &motors_control,
        .main_task_h = xTaskGetCurrentTaskHandle()};

    bt_com_task_ctx_t bt_ctx = {
        .q_rcv_h = bt_rcv_h,
        .q_tosend_h = bt_tosend_h};

    static TaskHandle_t lf_task_h;
    xTaskCreate(&line_follower_task, "line_follow_task", 4096, (void *)&lf_ctx, 16, &lf_task_h);

    xTaskCreatePinnedToCore(&bluetooth_com_task, "bluetooth_com_task", 16384, (void *)&bt_ctx, 3, NULL, 0);


    mission_state_t mission_state = MISSION_STATE_IDLE;
    uint32_t any_bottom_ir_active = 0;
    static bt_com_msg_t bt_msg_rcv;
    for (;;)
    {
        if (xQueueReceive(sonar_queue_h, &sonar_notif, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            static uint16_t sonar_print_ctr = 0;
            if (sonar_print_ctr++ == 5)
            {
                ESP_LOGI(TAG, "Ultrasonic measurement at angle %d deg: %fcm",
                         sonar_notif.angle,
                         sonar_notif.distance);
                sonar_print_ctr = 0;
            }

            if (sonar_notif.distance > 10 && sonar_notif.distance < 30)
            {
                // ESP_LOGI(TAG, "Enabling motors!");
                // SET_BIT(motors_control.cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
                // xQueueSend(motors_control_queue_h, &motors_control, pdMS_TO_TICKS(0));
                // mission_state = MISSION_STATE_FOLLOW_LINE;
                // xTaskNotify(lf_task_h, LF_STATE_ACTIVE, eSetValueWithOverwrite);
            }
            else if (sonar_notif.distance < 8)
            {
                mission_state = MISSION_STATE_STOP;
            }
        }

        if (xQueueReceive(bt_rcv_h, &bt_msg_rcv, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            ESP_LOGI(TAG, "Bt msg received@");
            ESP_LOG_BUFFER_HEX(TAG, bt_msg_rcv.data, bt_msg_rcv.len);

            if (bt_msg_rcv.len == 1)
            {
                uint8_t m = bt_msg_rcv.data[0];
                ESP_LOGI(TAG, "Received byte: " BYTE_TO_BINARY_PATTERN,
                         BYTE_TO_BINARY(m));

                if (m == 1)
                {
                    // Enable motors
                    ESP_LOGI(TAG, "Enabling motors!");
                    SET_BIT(motors_control.cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
                    xQueueSend(motors_control_queue_h, &motors_control, pdMS_TO_TICKS(0));
                }
                else if (m == 2)
                {
                    // Disable motors
                    ESP_LOGI(TAG, "Disabling motors!");
                    SET_BIT(motors_control.cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
                    xQueueSend(motors_control_queue_h, &motors_control, pdMS_TO_TICKS(0));
                }
                else if (m == 4)
                {
                    // Enable line-following mode
                    mission_state = MISSION_STATE_FOLLOW_LINE;
                }
                else if (m == 3)
                {
                    // Enable STOP mode
                    mission_state = MISSION_STATE_STOP;
                }
            }

            // Echo message back as example
            bt_com_msg_t bt_msg = {
                .len = bt_msg_rcv.len};
            memcpy(bt_msg.data, bt_msg_rcv.data, bt_msg_rcv.len);
            if (xQueueSend(bt_tosend_h, &bt_msg, pdMS_TO_TICKS(0)) != pdTRUE)
            {
                ESP_LOGE(TAG, "Failed putting message into to send BT queue!");
            }
        }

        if (xTaskNotifyWait(0x00, ULONG_MAX, &any_bottom_ir_active, pdTICKS_TO_MS(0)) == pdTRUE)
        {
            ESP_LOGI(TAG, "Bottom IR shows activity [notif from lf received]");
            xTaskNotify(lf_task_h, LF_STATE_ACTIVE, eSetValueWithOverwrite);
        }

        switch (mission_state)
        {
        case MISSION_STATE_IDLE:
            // ESP_LOGI(TAG, "Idling!");
            break;

        case MISSION_STATE_FOLLOW_LINE:
            // ESP_LOGI(TAG, "MISSION_STATE_FOLLOW_LINE");
            break;

        case MISSION_STATE_STOP:
            xTaskNotify(lf_task_h, LF_STATE_INACTIVE, eSetValueWithOverwrite);
            ESP_LOGI(TAG, "MISSION_STATE_STOP - Disabling motors!");
            CLEAR_BIT(motors_control.cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
            send_mot_spd(motors_control_queue_h, &motors_control, 0.0, 0.0, pdMS_TO_TICKS(1000));
            mission_state = MISSION_STATE_IDLE;
            break;

        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}