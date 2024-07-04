#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <driver/gpio.h>

#include "ase_config.h"
#include "ase_typedefs.h"
#include "bluetooth_com.h"
#include "helpers.h"
#include "line_follow.h"
#include "meta_detection.h"
#include "motor_control.h"
#include "obstacle_avoidance.h"
#include "state_transitions.h"

//////////////////////// HELPER MACROS ////////////////////////////////////////////////////////
#define MOTORS_CMD(enable, left, right, d)                                     \
    do                                                                         \
    {                                                                          \
        if (enable)                                                            \
            SET_BIT(motors_control.cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);    \
        else                                                                   \
            CLEAR_BIT(motors_control.cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);  \
        send_mot_spd(motors_control_queue_h, &motors_control, left, right, d); \
    } while (0)
////////////////////////////////////////////////////////////////////////////////////////////////

static QueueHandle_t bt_tosend_h;

#ifdef LOG_OVER_BLUETOOTH
static int dual_vprintf(const char *fmt, va_list ap)
{
    static bt_com_msg_t bt_msg;
    bt_msg.len = vsnprintf((char *)bt_msg.data, BT_MSG_BUF_SIZE_BYTES, fmt, ap);
    if (bt_msg.len > 0)
    {
        if (xQueueSend(bt_tosend_h, &bt_msg, 0) != pdTRUE)
        {
            // ESP_LOGE(MAIN_TASK_LOG_TAG, "Can't put msg to bluetooth LOG queue!");
        }
    }

    return vprintf(fmt, ap);
}
#endif

void app_main()
{
    const TaskHandle_t current_task_h = xTaskGetCurrentTaskHandle();

    QueueHandle_t motors_control_queue_h = xQueueCreate(10, sizeof(motors_control_msg_t));
    if (motors_control_queue_h == NULL)
    {
        ESP_LOGE(MAIN_TASK_LOG_TAG, "Failed to create motors_control_queue_h!");
        abort();
    }

    static QueueHandle_t bt_rcv_h;
    bt_rcv_h    = xQueueCreate(5, sizeof(bt_com_msg_t));
    bt_tosend_h = xQueueCreate(35, sizeof(bt_com_msg_t));

#ifdef LOG_OVER_BLUETOOTH
    // Line below enables sending app logs over bluetooth
    esp_log_set_vprintf(dual_vprintf);
    // Disable BT_HCI logs as they are truly useless
    esp_log_level_set("BT_HCI", ESP_LOG_NONE);
    esp_log_level_set(SPP_TAG, ESP_LOG_NONE);
    esp_log_level_set(META_DETECTION_LOG_TAG, ESP_LOG_NONE);
    esp_log_level_set(SONAR_SERVO_LOG_TAG, ESP_LOG_NONE);
#endif

    motors_control_msg_t motors_control = {
        .speed_cmd = {
            .left  = 0.85f,
            .right = 0.85f}};

    line_follower_task_context_t lf_ctx = {
        .mot_cmd_q_handle = motors_control_queue_h,
        .mot_ctrl_msg     = &motors_control,
        .main_task_h      = current_task_h};

    bt_com_task_ctx_t bt_ctx = {
        .q_rcv_h    = bt_rcv_h,
        .q_tosend_h = bt_tosend_h};

    obstacle_avoidance_ctx_t avoidance_ctx = {
        .mot_cmd_q_handle = motors_control_queue_h,
        .mot_ctrl_msg     = &motors_control,
        .main_task_h      = current_task_h};

    static TaskHandle_t lf_task_h;
    static TaskHandle_t avoidance_task_h;

    ////////////////////////////////// TASKS CREATION //////////////////////////////////
    xTaskCreate(&motor_control_task, "motor_ctrl", 4096, (void *)motors_control_queue_h, 15, NULL);
    xTaskCreate(&line_follower_task, "line_follow", 4096, (void *)&lf_ctx, 17, &lf_task_h);
    xTaskCreate(&obstacle_avoidance_task, "avoidance", 4096, (void *)&avoidance_ctx, 17, &avoidance_task_h);
    xTaskCreatePinnedToCore(&bluetooth_com_task, "bt_com", 16384, (void *)&bt_ctx, 3, NULL, 0);
    // xTaskCreatePinnedToCore(&meta_detection_task, "meta-detect", 3048,
    // (void *)current_task_h, 11, NULL, 0);
    /////////////////////////////////////////////////////////////////////////////////////

    uint32_t            any_bottom_ir_active = 0;
    static bt_com_msg_t bt_msg_rcv;
    mission_state_t     current_state      = MISSION_STATE_IDLE;
    mission_state_t     new_state          = MISSION_STATE_IDLE;
    TickType_t          ticks_when_quitted = 0;
    for (;;)
    {
        if (xQueueReceive(bt_rcv_h, &bt_msg_rcv, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            ESP_LOGI(MAIN_TASK_LOG_TAG, "Bt msg received (len=%d)", bt_msg_rcv.len);
            // ESP_LOG_BUFFER_HEX(MAIN_TASK_LOG_TAG, bt_msg_rcv.data, bt_msg_rcv.len);

            uint8_t m = bt_msg_rcv.data[0];
            ESP_LOGI(MAIN_TASK_LOG_TAG, "1st byte received: " BYTE_TO_BINARY_PATTERN,
                     BYTE_TO_BINARY(m));
            if (m == 4)
            {
                // Enable line-following mode
                MOTORS_CMD(true, 0.8, 0.8, pdMS_TO_TICKS(0));
                new_state = MISSION_STATE_FOLLOW_LINE;
            }
            else if (m == 3)
            {
                // Enable STOP mode;
                new_state = MISSION_STATE_STOP;
                MOTORS_CMD(false, 0.0, 0.0, pdMS_TO_TICKS(5000));
            }
        }

        if (xTaskNotifyWaitIndexed(MAIN_BOTTOM_IR_ACTIVITY_NOTIF_IDX,
                                   0x00, ULONG_MAX, &any_bottom_ir_active, pdTICKS_TO_MS(0)) == pdTRUE)
        {
            // Allow changing mode with small frequency to avoid jittering
            // upon obstacle detection, as robot may still be on the line.
            if (current_state != MISSION_STATE_FOLLOW_LINE &&
                (xTaskGetTickCount() - ticks_when_quitted) > pdMS_TO_TICKS(3000))
            {
                ESP_LOGI(MAIN_TASK_LOG_TAG, "Bottom IR shows activity - entering line follower mode");
                new_state = MISSION_STATE_FOLLOW_LINE;
            }
        }

        static uint32_t tmp = 0;
        if (xTaskNotifyWaitIndexed(MAIN_META_DETECTION_NOTIF_IDX,
                                   0x00, ULONG_MAX, &tmp, pdTICKS_TO_MS(0)) == pdTRUE)
        {
            ESP_LOGI(MAIN_TASK_LOG_TAG, "Meta detected!");
            new_state = MISSION_STATE_STOP;
        }

        if (xTaskNotifyWaitIndexed(MAIN_OBSTACLE_AHEAD_NOTIF_IDX,
                                   0x00, ULONG_MAX, &tmp, pdTICKS_TO_MS(0)) == pdTRUE)
        {
            if (tmp == 1)
                new_state = MISSION_STATE_AVOID_OBSTACLE;
            else if (tmp == 2) {
                // Reverse direction of rotation in line follower for one next time
                
            }
        }

        // State switching
        if (current_state == new_state)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        switch (current_state)
        {
        case MISSION_STATE_IDLE:
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Quitting mission mode - IDLE");
            break;

        case MISSION_STATE_FOLLOW_LINE:
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Quitting mission mode - LF");
            xTaskNotify(lf_task_h, LF_STATE_INACTIVE, eSetValueWithOverwrite);
            ticks_when_quitted = xTaskGetTickCount();
            break;

        case MISSION_STATE_AVOID_OBSTACLE:
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Quitting mission mode - AVOIDANCE");
            xTaskNotify(avoidance_task_h, AVOIDANCE_STATE_INACTIVE, eSetValueWithOverwrite);
            break;

        case MISSION_STATE_STOP:
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Quitting mission mode - STOP");
        }

        switch (new_state)
        {
        case MISSION_STATE_IDLE:
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Entering mission mode - IDLE");
            break;

        case MISSION_STATE_FOLLOW_LINE:
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Entering mission mode - LF");
            xTaskNotify(lf_task_h, LF_STATE_ACTIVE, eSetValueWithOverwrite);
            break;

        case MISSION_STATE_AVOID_OBSTACLE:
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Entering mission mode - AVOIDANCE");
            xTaskNotify(avoidance_task_h, AVOIDANCE_STATE_ACTIVE, eSetValueWithOverwrite);
            break;

        case MISSION_STATE_STOP:
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Entering mission mode - STOP");
            MOTORS_CMD(false, 0.0, 0.0, pdMS_TO_TICKS(5000));
        }

        current_state = new_state;

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
//*/