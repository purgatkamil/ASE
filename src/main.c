#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <driver/gpio.h>

#include "motor_control.h"
#include "ultrasonic_sensor.h"
#include "line_follow.h"
#include "ase_typedefs.h"
#include "helpers.h"

static const char *TAG = "orchestrator";

void app_main()
{
    QueueHandle_t sonar_queue_h = xQueueCreate(5, sizeof(ultrasonic_measurement_t));
    QueueHandle_t motors_control_queue_h = xQueueCreate(20, sizeof(motors_control_msg_t));
    sonar_motors_q_ok_or_abort(sonar_queue_h, motors_control_queue_h, TAG);

    // Create motor control task
    xTaskCreate(&motor_control_task, "motor_control_task", 2048, (void *)motors_control_queue_h, 15, NULL);
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

    static TaskHandle_t lf_task_h;
    xTaskCreate(&line_follower_task, "line_follow_task", 2048, (void *)&lf_ctx, 16, &lf_task_h);

    mission_state_t mission_state = MISSION_STATE_IDLE;
    uint32_t any_bottom_ir_active = 0;
    for (;;)
    {
        if (xQueueReceive(sonar_queue_h, &sonar_notif, pdMS_TO_TICKS(500)) == pdTRUE)
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
                ESP_LOGI(TAG, "Enabling motors!");
                SET_BIT(motors_control.cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
                xQueueSend(motors_control_queue_h, &motors_control, pdMS_TO_TICKS(0));
                mission_state = MISSION_STATE_FOLLOW_LINE;
                // xTaskNotify(lf_task_h, LF_STATE_ACTIVE, eSetValueWithOverwrite);
            }
            else if (sonar_notif.distance < 8)
            {
                mission_state = MISSION_STATE_STOP;
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

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
