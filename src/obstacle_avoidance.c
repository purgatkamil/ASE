#include "obstacle_avoidance.h"

static void inline setServo(int16_t angle, TaskHandle_t sonar_task_h)
{
    xTaskNotifyIndexed(sonar_task_h, SONAR_SET_SERVO_ANGLE_NOTIF_IDX, (uint32_t)angle, eSetValueWithOverwrite);
    xTaskNotifyWaitIndexed(MAIN_SERVO_ANGLE_READY, 0x00, 0x00, NULL, portMAX_DELAY);
}

static float inline getDistance(QueueHandle_t q, uint32_t wait_ms)
{
    static ultrasonic_measurement_t measurement;
    xQueueReceive(q, &measurement, pdMS_TO_TICKS(wait_ms));
    return measurement.distance;
}

void obstacle_avoidance_task(void *pvParameters)
{
    obstacle_avoidance_ctx_t *oa_ctx = (obstacle_avoidance_ctx_t *)pvParameters;
    motors_control_msg_t     *mc     = oa_ctx->mot_ctrl_msg;
    QueueHandle_t             mc_q_h = oa_ctx->mot_cmd_q_handle;

    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = 1,
        .pull_down_en = 0,
        .pin_bit_mask = (1ULL << IR_TOP_LEFT_GPIO) |
                        (1ULL << IR_TOP_RIGHT_GPIO) |
                        (1ULL << IR_TOP_FRONT_GPIO)};

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    QueueHandle_t sonar_queue_h = xQueueCreate(1, sizeof(ultrasonic_measurement_t));
    if (sonar_queue_h == NULL)
    {
        ESP_LOGE(OBSTACLE_AVOIDANCE_LOG_TAG, "Failed to create sonar queue!");
        abort();
    }
    sonar_task_ctx_t sonar_task_ctx = {
        .masurements_queue_h            = sonar_queue_h,
        .servo_angle_ready_notif_task_h = xTaskGetCurrentTaskHandle()};
    TaskHandle_t sonar_task_h;
    xTaskCreate(&ultrasonic_sensor_task, "sonar", 4096, (void *)&sonar_task_ctx, 10, sonar_task_h);

    int16_t servo_angle         = 50;
    float   ultrasonic_distance = 0.f;

    uint8_t  ir_f                   = 0;
    uint8_t  ir_l                   = 0;
    uint8_t  ir_r                   = 0;
    uint32_t orchestrator_notif_val = 0;
    uint8_t  active                 = 0;
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

        if (active)
        {
            setServo(servo_angle, sonar_task_h);
            ultrasonic_distance = getDistance(sonar_queue_h, 1000);

            // Mr Kamil please code your logic here :)
            // Move motors using command below
            // This one to enable motors -> SET_BIT(mc->cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
            // This one to stop the motors -> CLEAR_BIT(mc->cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
            // But they will take effect ONLY after calling the one below!!!
            // send_mot_spd(mc_q_h, mc, 0.0, 0.0, pdMS_TO_TICKS(0));
        }

        vTaskDelay(pdMS_TO_TICKS(40));
    }
}