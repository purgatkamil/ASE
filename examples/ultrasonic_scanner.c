#include "freertos/FreeRTOS.h"

#include "helpers.h"
#include "ultrasonic_sensor.h"

#include "ase_config.h"
#include "task_notif_indexes.h"

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

void app_main()
{
    QueueHandle_t sonar_queue_h = xQueueCreate(1, sizeof(ultrasonic_measurement_t));

    if (sonar_queue_h == NULL)
    {
        ESP_LOGE(OBSTACLE_AVOIDANCE_LOG_TAG, "Failed to create sonar queue!");
        abort();
    }

    sonar_task_ctx_t sonar_task_ctx = {
        .masurements_queue_h = sonar_queue_h,
        .servo_angle_ready_notif_task_h = xTaskGetCurrentTaskHandle()
    };

    TaskHandle_t sonar_task_h;
    xTaskCreate(&ultrasonic_sensor_task, "sonar", 4096, (void *)&sonar_task_ctx, 10, &sonar_task_ctx);

    int16_t angle = 50;
    float distance = 0.f;
    for (;;)
    {
        setServo(angle, sonar_task_h);
        distance = getDistance(sonar_queue_h, 1000);
    }
}