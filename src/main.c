#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "motor_control.h"
#include "ultrasonic_sensor.h"
#include "ase_typedefs.h"

#include <driver/gpio.h>

static const char *TAG = "orchestrator";

#define IR_SENSOR_BOTTOM_LEFT_GPIO GPIO_NUM_23
#define IR_SENSOR_BOTTOM_RIGHT_GPIO GPIO_NUM_22
#define IR_SENSOR_BOTTOM_CENTER_GPIO GPIO_NUM_35

static void init_ir_gpio()
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << IR_SENSOR_BOTTOM_LEFT_GPIO) |
                        (1ULL << IR_SENSOR_BOTTOM_CENTER_GPIO) |
                        (1ULL << IR_SENSOR_BOTTOM_RIGHT_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 1};

    gpio_config(&io_conf);
}

void app_main()
{
    QueueHandle_t sonar_queue_h = xQueueCreate(5, sizeof(ultrasonic_measurement_t));
    QueueHandle_t motors_control_queue_h = xQueueCreate(20, sizeof(motors_control_msg_t));

    if (sonar_queue_h == NULL || motors_control_queue_h == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue (sonar_q_fail = %d, motors_control_q_fail = %d)",
                 sonar_queue_h == NULL,
                 motors_control_queue_h == NULL);
        abort();
    }

    init_ir_gpio();

    // Create motor control task
    xTaskCreate(&motor_control_task, "motor_control_task", 2048, (void *)motors_control_queue_h, 15, NULL);

    // xTaskCreate(&ultrasonic_sensor_task, "ultrasonic_sensor_task", 4096, (void *)sonar_queue_h, 10, NULL);

    ultrasonic_measurement_t sonar_notif = {
        .angle = 0,
        .distance = 0.0f};

    motors_control_msg_t motors_control = {
        .speed_cmd = {
            .left = 0.85f,
            .right = 0.0f}};

    for (;;)
    {
        if (xQueueReceive(sonar_queue_h, &sonar_notif, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            ESP_LOGI(TAG, "Ultrasonic measurement at angle %d deg: %fcm",
                     sonar_notif.angle,
                     sonar_notif.distance);
        }

        int l = gpio_get_level(IR_SENSOR_BOTTOM_LEFT_GPIO),
            c = gpio_get_level(IR_SENSOR_BOTTOM_CENTER_GPIO),
            r = gpio_get_level(IR_SENSOR_BOTTOM_RIGHT_GPIO);

        if (!c)
        {
            SET_BIT(motors_control.cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
            xQueueSend(motors_control_queue_h, &motors_control, pdMS_TO_TICKS(0));
        }
        if (!l)
        {
            CLEAR_BIT(motors_control.cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
            xQueueSend(motors_control_queue_h, &motors_control, pdMS_TO_TICKS(0));
        }
        if (!r)
        {
            motors_control.speed_cmd.left *= -1;
            xQueueSend(motors_control_queue_h, &motors_control, pdMS_TO_TICKS(0));
            ESP_LOGI(TAG, "Left motor control sig [-1; 1]: %f", motors_control.speed_cmd.left);
            vTaskDelay(pdMS_TO_TICKS(1500));
        }

        // ESP_LOGI(TAG, "left: %d\tcenter: %d\tright: %d", l, c, r);

        vTaskDelay(pdMS_TO_TICKS(10));
        // vTaskDelay(pdMS_TO_TICKS(500));
    }
}
