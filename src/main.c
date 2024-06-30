#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor_control.h"

void app_main()
{
    // Create motor control task
    xTaskCreate(&motor_control_task, "motor_control_task", 2048, NULL, 5, NULL);   


    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    } 
}
