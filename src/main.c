#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <driver/gpio.h>
#include <inttypes.h>
#include <sys/time.h>

#include "ase_config.h"
#include "ase_typedefs.h"
#include "helpers.h"
#include "meta_detection.h"
#include "motor_control.h"
#include "task_notif_indexes.h"
#include "wander.h"

#ifdef COMPILE_BLUETOOTH
#include "bluetooth_com.h"
#endif

#define SEND_BT_MISSION_STATE_MSG(STATE_STR)                                \
    do                                                                      \
    {                                                                       \
        static bt_com_msg_t bt_send_msg;                                    \
        memset(bt_send_msg.data, 0, BT_MSG_BUF_SIZE_BYTES);                 \
        bt_send_msg.data[0]           = 1;                                  \
        const char *mission_state_str = STATE_STR;                          \
        memcpy(bt_send_msg.data + 1, mission_state_str, sizeof(STATE_STR)); \
        bt_send_msg.len = sizeof(STATE_STR) + 1;                            \
        bluetooth_send(&bt_send_msg);                                       \
    } while (0);

void app_main()
{
    const TaskHandle_t current_task_h = xTaskGetCurrentTaskHandle();

#ifdef COMPILE_BLUETOOTH
    start_bluetooth_task();
    bt_com_msg_t bt_msg_rcv;
#endif

    xTaskCreate(&mc_motor_control_task, "motor_ctrl", 4096, NULL, 15, NULL);
    xTaskCreate(&meta_detection_task, "meta-detect", 3048, (void *)current_task_h, 11, NULL);

    wander_ctx_t wander_ctx = {
        .main_task_h = current_task_h,
    };
    static TaskHandle_t wander_task_h;
    xTaskCreate(&wander_task, "wander", 4096, (void *)&wander_ctx, 17, &wander_task_h);

    mission_state_t current_state = MISSION_STATE_UNKNOWN;
    mission_state_t new_state     = MISSION_STATE_IDLE;

    int64_t timestamp_mission_start = 0;
    for (;;)
    {
        // static TickType_t last_sent_ticks = 0;
        // if (xTaskGetTickCount() - last_sent_ticks > pdMS_TO_TICKS(1000))
        // {
        //     static int state_counter = 0;
        //     switch (state_counter++)
        //     {
        //     case 0:
        //         SEND_BT_MISSION_STATE_MSG("IDLE");
        //         break;

        //     case 1:
        //         SEND_BT_MISSION_STATE_MSG("STOP");
        //         break;

        //     case 2:
        //         SEND_BT_MISSION_STATE_MSG("WANDER");
        //         break;

        //     case 3:
        //         SEND_BT_MISSION_STATE_MSG("CELEBRATE");
        //         break;

        //     default:
        //         state_counter = 0;
        //         break;
        //     }
        //     last_sent_ticks = xTaskGetTickCount();
        // }

#ifdef COMPILE_BLUETOOTH
        if (bluetooth_wait_for_msg(&bt_msg_rcv, 0) == pdTRUE)
        {
            ESP_LOGI(MAIN_TASK_LOG_TAG, "Bt msg received (len=%d)", bt_msg_rcv.len);
            // ESP_LOG_BUFFER_HEX(MAIN_TASK_LOG_TAG, bt_msg_rcv.data, bt_msg_rcv.len);
            uint8_t m = bt_msg_rcv.data[0];
            ESP_LOGI(MAIN_TASK_LOG_TAG, "1st byte received: " BYTE_TO_BINARY_PATTERN,
                     BYTE_TO_BINARY(m));

#ifdef SUPPORT_MACIEJ_APP
            if (bt_msg_rcv.len == 1)
            {
                // Command
                if (m == 0x01)
                {
                    // Start
                    new_state = MISSION_STATE_WANDER;
                }
                else if (m == 0xFF)
                {
                    // Stop
                    new_state = MISSION_STATE_STOP;
                }
            }
#endif
#ifndef SUPPORT_MACIEJ_APP
            if (m == 4)
            {
                // Enable line-following mode
                mc_set_duty(0.8, 0.8);
                // time_mission_start = xx_time_get_time();
            }
            else if (m == 3)
            {
                // Enable STOP mode;
                new_state = MISSION_STATE_STOP;
                mc_disable_pwm();
            }
            else if (m == 2)
            {
                // Start wandering around & avoiding obstacles
                new_state = MISSION_STATE_WANDER;
                // mc_disable_pwm();
                // mc_set_duty(1.0, 1.0);
            }
            else if (m == 1)
            {
                new_state = MISSION_STATE_IDLE;
            }
#endif
        }
#endif
        // static uint32_t tmp = 0;
        // if (xTaskNotifyWaitIndexed(MAIN_META_DETECTION_NOTIF_IDX,
        //                            0x00, ULONG_MAX, &tmp, pdTICKS_TO_MS(0)) == pdTRUE)
        // {
        //     ESP_LOGI(MAIN_TASK_LOG_TAG, "Meta detected [low uncertainty]!");
        //     int64_t tofride = xx_time_get_time() - time_mission_start;
        //     ESP_LOGI(MAIN_TASK_LOG_TAG, "Time of ride so far: %lld ms (%.3f s)", tofride, (double)tofride / 1000.0);
        //     if (!celebrated_once && tofride > (60 * 1000))
        //     {
        //         new_state       = MISSION_STATE_CELEBRATE;
        //         celebrated_once = 1;
        //     }
        // }

        // State switching
        if (current_state == new_state)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // This code describes what happens when state is left
        switch (current_state)
        {
        case MISSION_STATE_IDLE:
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Quitting mission mode - IDLE");
            break;

        case MISSION_STATE_WANDER:
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Quitting mission mode - WANDER");
            xTaskNotify(wander_task_h, WANDER_STATE_INACTIVE, eSetValueWithOverwrite);
            break;

        case MISSION_STATE_STOP:
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Quitting mission mode - STOP");

        default:
            break;
        }

        // This code describes what happens when new state is entered
        switch (new_state)
        {
        case MISSION_STATE_IDLE:
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Entering mission mode - IDLE");
            SEND_BT_MISSION_STATE_MSG("IDLE");
            break;

        case MISSION_STATE_WANDER:
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Entering mission mode - WANDER");
            SEND_BT_MISSION_STATE_MSG("WANDER");
            xTaskNotify(wander_task_h, WANDER_STATE_ACTIVE, eSetValueWithOverwrite);
            timestamp_mission_start = get_sys_timestamp();
            break;

        case MISSION_STATE_CELEBRATE:
        { // Send duration of mission
            static bt_com_msg_t bt_msg_time_completion;
            memset(bt_msg_time_completion.data, 0, BT_MSG_BUF_SIZE_BYTES);
            bt_msg_time_completion.len = snprintf((char *)bt_msg_time_completion.data, BT_MSG_BUF_SIZE_BYTES, "<%" PRId64 ">",
                                                  (get_sys_timestamp() - timestamp_mission_start));
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Celebration!! Meta found!!");
            SEND_BT_MISSION_STATE_MSG("CELEBRATE");
            static size_t i = 0;
            for (i = 0; i < 7; i++)
            {
                mc_set_duty(-0.8, 0.8);
                vTaskDelay(pdMS_TO_TICKS(200));
                mc_set_duty(0.8, -0.8);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            mc_disable_pwm();
            new_state = MISSION_STATE_STOP;
        }
        break;

        case MISSION_STATE_STOP:
        {
            ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Entering mission mode - STOP");
            mc_disable_pwm();
            SEND_BT_MISSION_STATE_MSG("STOP");
            static bt_com_msg_t bt_msg_time_completion;
            memset(bt_msg_time_completion.data, 0, BT_MSG_BUF_SIZE_BYTES);
            bt_msg_time_completion.len = snprintf((char *)bt_msg_time_completion.data, BT_MSG_BUF_SIZE_BYTES, "<%" PRId64 ">",
                                                  (get_sys_timestamp() - timestamp_mission_start));
            // bt_msg_time_completion.data[bt_msg_time_completion.len - 1] = 0x0; // remove terminating null character
            bluetooth_send(&bt_msg_time_completion);
        }
        default:
            break;
        }

        current_state = new_state;

        vTaskDelay(pdMS_TO_TICKS(80));
    }
}
//*/