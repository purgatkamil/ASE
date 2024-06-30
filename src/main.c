#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "motor_control.h"
#include "ultrasonic_sensor.h"

void app_main()
{
    // Create motor control task
    // xTaskCreate(&motor_control_task, "motor_control_task", 2048, NULL, 15, NULL);

    xTaskCreate(&ultrasonic_sensor_task, "ultrasonic_sensor_task", 2048, NULL, 10, NULL);

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
