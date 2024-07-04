#include "obstacle_avoidance.h"

void obstacle_avoidance_task(void *pvParameters)
{
    obstacle_avoidance_ctx_t *oa_ctx = (obstacle_avoidance_ctx_t *)pvParameters;
    motors_control_msg_t     *mc     = oa_ctx->mot_ctrl_msg;
    QueueHandle_t             mc_q_h = oa_ctx->mot_cmd_q_handle;

    QueueHandle_t sonar_queue_h = xQueueCreate(5, sizeof(ultrasonic_measurement_t));

    if (sonar_queue_h == NULL)
    {
        ESP_LOGE(OBSTACLE_AVOIDANCE_LOG_TAG, "Failed to create sonar queue!");
        abort();
    }

    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = 1,
        .pull_down_en = 0,
        .pin_bit_mask = (1ULL << IR_TOP_LEFT_GPIO) |
                        (1ULL << IR_TOP_RIGHT_GPIO) |
                        (1ULL << IR_TOP_FRONT_GPIO)};

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ultrasonic_measurement_t sonar_notif = {
        .angle    = 0,
        .distance = 0.0f};

    xTaskCreate(&ultrasonic_sensor_task, "sonar", 4096, (void *)sonar_queue_h, 10, NULL);
    uint8_t  ir_f                   = 0;
    uint8_t  ir_l                   = 0;
    uint8_t  ir_r                   = 0;
    uint32_t orchestrator_notif_val = 0;
    uint8_t  active                 = 0;
    uint8_t  DIR                    = 0;
    for (;;)
    {
        if (xTaskNotifyWait(0x00, ULONG_MAX, &orchestrator_notif_val, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Notify received: %lu", orchestrator_notif_val);
            active = orchestrator_notif_val == AVOIDANCE_STATE_ACTIVE;
        }

        ir_l = 1 - gpio_get_level(IR_TOP_LEFT_GPIO);
        ir_f = 1 - gpio_get_level(IR_TOP_FRONT_GPIO);
        ir_r = 1 - gpio_get_level(IR_TOP_RIGHT_GPIO);

        if (xQueueReceive(sonar_queue_h, &sonar_notif, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            if (sonar_notif.distance < 15 && sonar_notif.angle == 0)
            {
                ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Sonar reading: %.4f cm", sonar_notif.distance);
                if (ir_f == 1)
                    xTaskNotifyIndexed(oa_ctx->main_task_h, MAIN_OBSTACLE_AHEAD_NOTIF_IDX, 1, eNoAction);
            }
        }

        static bool oneshot = true;
        if (active && oneshot)
        {

            oneshot = true;
            int8_t sign = false ? -1 : 1;
            // false -> right, true -> left

            switch (DIR)
            {
            case 0:

                // Generating motor commands
                // static int ctr = 0;
                // ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "I am active! %d", ctr++);

                send_mot_spd(mc_q_h, mc, -0.9, -0.8, pdMS_TO_TICKS(0));
                vTaskDelay(pdMS_TO_TICKS(150));
                send_mot_spd(mc_q_h, mc, 0.0, -0.0, pdMS_TO_TICKS(0));
                vTaskDelay(pdMS_TO_TICKS(100));

                // Avoding to the right
                // First, turn right.
                ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Turning");
                send_mot_spd(mc_q_h, mc, sign * 1.0, -sign * 1.0, pdMS_TO_TICKS(0));
                vTaskDelay(pdMS_TO_TICKS(sign < 0 ? 430 + 90 : 430 + 40));
                send_mot_spd(mc_q_h, mc, 0.9, 0.8, pdMS_TO_TICKS(0));
                vTaskDelay(pdMS_TO_TICKS(250));
                send_mot_spd(mc_q_h, mc, 0.0, 0.0, pdMS_TO_TICKS(0));
                DIR++;
                break;

            case 1:
                ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Straight");
                send_mot_spd(mc_q_h, mc, 0.9, 0.9, pdMS_TO_TICKS(0));
                vTaskDelay(pdMS_TO_TICKS(600));
                send_mot_spd(mc_q_h, mc, 0.0, 0.0, pdMS_TO_TICKS(0));
                DIR++;
                break;

            case 2:
                ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Turning back");
                send_mot_spd(mc_q_h, mc, -sign * 1.0, +sign * 1.0, pdMS_TO_TICKS(0));
                vTaskDelay(pdMS_TO_TICKS(-sign < 0 ? 430 + 90 : 430 + 40));
                send_mot_spd(mc_q_h, mc, 0.0, 0.0, pdMS_TO_TICKS(0));
                DIR++;
                break;

            case 3:
                ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Straight");
                send_mot_spd(mc_q_h, mc, 0.9, 0.9, pdMS_TO_TICKS(0));
                vTaskDelay(pdMS_TO_TICKS(1400));
                send_mot_spd(mc_q_h, mc, 0.0, 0.0, pdMS_TO_TICKS(0));
                DIR++;
                break;

                 case 4:
                ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Turning back");
                send_mot_spd(mc_q_h, mc, -sign * 1.0, +sign * 1.0, pdMS_TO_TICKS(0));
                vTaskDelay(pdMS_TO_TICKS(-sign < 0 ? 430 + 0 : 430 + (-30)));
                send_mot_spd(mc_q_h, mc, 0.0, 0.0, pdMS_TO_TICKS(0));
                DIR++;
                break;

                 case 5:
                ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Straight");
                send_mot_spd(mc_q_h, mc, 0.7, 0.6, pdMS_TO_TICKS(0));
                DIR++;
                break;

            default:
                active = 0;
                DIR    = 0;
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(40));
    }
}