#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "motor_control.h"
#include "ultrasonic_sensor.h"
#include "ase_typedefs.h"

static const char *TAG = "orchestrator";

void app_main()
{
    // Main task's code for controlling robot's behaviour
    QueueHandle_t sonar_queue_h = xQueueCreate(5, sizeof(ultrasonic_measurement_t));

    if (sonar_queue_h == NULL)
    {
        ESP_LOGE(TAG, "Failed to create sonar queue!");
        abort();
    }

    // Create motor control task
    xTaskCreate(&motor_control_task, "motor_control_task", 2048, NULL, 15, NULL);

    xTaskCreate(&ultrasonic_sensor_task, "ultrasonic_sensor_task", 4096, (void *)sonar_queue_h, 10, NULL);

    ultrasonic_measurement_t sonar_notif = {
        .angle = 0,
        .distance = 0.0f};

    for (;;)
    {
        if (xQueueReceive(sonar_queue_h, &sonar_notif, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            ESP_LOGI(TAG, "Ultrasonic measurement at angle %d deg: %fcm",
                     sonar_notif.angle,
                     sonar_notif.distance);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
