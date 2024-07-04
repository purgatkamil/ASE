#include "obstacle_avoidance.h"

void obstacle_avoidance_task(void *pvParameters)
{
    obstacle_avoidance_ctx_t *oa_ctx        = (obstacle_avoidance_ctx_t *)pvParameters;
    QueueHandle_t             sonar_queue_h = xQueueCreate(5, sizeof(ultrasonic_measurement_t));

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
    for (;;)
    {
        if (xTaskNotifyWait(0x00, ULONG_MAX, &orchestrator_notif_val, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            active = orchestrator_notif_val == AVOIDANCE_STATE_ACTIVE;
        }

        if (xQueueReceive(sonar_queue_h, &sonar_notif, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            if (sonar_notif.distance < 10)
            {
                ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Sonar reading: %f", sonar_notif.distance);
                xTaskNotifyIndexed(oa_ctx->main_task_h, MAIN_OBSTACLE_AHEAD_NOTIF_IDX, 0, eNoAction);
            }
        }

        ir_l = gpio_get_level(IR_TOP_LEFT_GPIO);
        ir_f = gpio_get_level(IR_TOP_FRONT_GPIO);
        ir_r = gpio_get_level(IR_TOP_RIGHT_GPIO);

        if (active)
        {
            // Generating motor commands
            static int ctr = 0;
            ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "I am active! %d", ctr++);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}