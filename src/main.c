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

static TaskHandle_t main_task_h;

static void IRAM_ATTR ir_bottom_isr_handler(void *arg)
{
    ir_isr_arg_t *isr_arg = (ir_isr_arg_t *)arg;
    int edge = gpio_get_level(isr_arg->gpio_num);

    uint32_t mask = (edge << (isr_arg->gpio_id)) | ((isr_arg->gpio_id) << 16);
    xTaskNotifyFromISR(main_task_h, mask, eSetValueWithOverwrite, NULL);
}

static inline void init_ir_gpio()
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

static inline void setup_ir_gpio_isr()
{
    static ir_isr_arg_t isr_args[3] = {
        {.gpio_id = IR_ISR_GPIO_BOT_LEFT_ID,
         .gpio_num = IR_SENSOR_BOTTOM_LEFT_GPIO},
        {.gpio_id = IR_ISR_GPIO_BOT_CENTER_ID,
         .gpio_num = IR_SENSOR_BOTTOM_CENTER_GPIO},
        {.gpio_id = IR_ISR_GPIO_BOT_RIGHT_ID,
         .gpio_num = IR_SENSOR_BOTTOM_RIGHT_GPIO},
    };

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    ESP_ERROR_CHECK(gpio_isr_handler_add(
        IR_SENSOR_BOTTOM_LEFT_GPIO, ir_bottom_isr_handler, (void *)&isr_args[0]));
    ESP_ERROR_CHECK(gpio_isr_handler_add(
        IR_SENSOR_BOTTOM_CENTER_GPIO, ir_bottom_isr_handler, (void *)&isr_args[1]));
    ESP_ERROR_CHECK(gpio_isr_handler_add(
        IR_SENSOR_BOTTOM_RIGHT_GPIO, ir_bottom_isr_handler, (void *)&isr_args[2]));
}

void app_main()
{
    main_task_h = xTaskGetCurrentTaskHandle();

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
    setup_ir_gpio_isr();

    // Create motor control task
    xTaskCreate(&motor_control_task, "motor_control_task", 2048, (void *)motors_control_queue_h, 15, NULL);
    xTaskCreate(&ultrasonic_sensor_task, "ultrasonic_sensor_task", 4096, (void *)sonar_queue_h, 10, NULL);

    ultrasonic_measurement_t sonar_notif = {
        .angle = 0,
        .distance = 0.0f};

    // motors_control_msg_t motors_control = {
    //     .speed_cmd = {
    //         .left = 0.85f,
    //         .right = 0.0f}};
    motors_control_msg_t motors_control = {
        .speed_cmd = {
            .left = 0.85f,
            .right = 0.85f}};

    TickType_t xlast_wake_time = xTaskGetTickCount();
    uint32_t ir_bot_notif_carrier = 0;
    for (;;)
    {
        if (xQueueReceive(sonar_queue_h, &sonar_notif, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            ESP_LOGI(TAG, "Ultrasonic measurement at angle %d deg: %fcm",
                     sonar_notif.angle,
                     sonar_notif.distance);

            if (abs(sonar_notif.angle) < 50)
            {
                if (sonar_notif.distance > 15 && sonar_notif.distance < 30)
                {
                    ESP_LOGI(TAG, "Enabling motors!");
                    SET_BIT(motors_control.cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
                    xQueueSend(motors_control_queue_h, &motors_control, pdMS_TO_TICKS(0));
                }
                else if (sonar_notif.distance < 15 && sonar_notif.distance > 1)
                {
                    ESP_LOGI(TAG, "Disabling motors!");
                    CLEAR_BIT(motors_control.cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
                    xQueueSend(motors_control_queue_h, &motors_control, pdMS_TO_TICKS(0));
                }
            }
        }

        if (xTaskNotifyWait(0x00, ULONG_MAX, &ir_bot_notif_carrier, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            // ESP_LOGI(TAG, "Ir state var: " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " ",
            //          BYTE_TO_BINARY(ir_bot_notif_carrier >> 24), BYTE_TO_BINARY(ir_bot_notif_carrier >> 16), BYTE_TO_BINARY(ir_bot_notif_carrier >> 8), BYTE_TO_BINARY(ir_bot_notif_carrier));

            uint8_t ir_gpio_id = ir_bot_notif_carrier >> 16;
            uint8_t ir_gpio_state = ir_bot_notif_carrier & 0xFF;

            ESP_LOGI(TAG, "GPIO_ID: %d, GPIO_STATE: %d", ir_gpio_id, ir_gpio_state);

            double left_speed_delta = 0, right_speed_delta = 0;

            switch (ir_gpio_id)
            {
            case IR_ISR_GPIO_BOT_LEFT_ID:
                left_speed_delta -= 0.1; // ir_gpio_state == 0 ? 0.1 : -0.1;
                break;

            case IR_ISR_GPIO_BOT_CENTER_ID:
                if (ir_gpio_state != 0)
                {
                    CLEAR_BIT(motors_control.cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
                    xQueueSend(motors_control_queue_h, &motors_control, pdMS_TO_TICKS(0));
                }
                break;

            case IR_ISR_GPIO_BOT_RIGHT_ID:
                right_speed_delta -= 0.1;
                break;

            default:
                ESP_LOGE(TAG, "Unknown IR ISR GPIO ID fired!");
                break;
            }

            motors_control.speed_cmd.left = 0.85f + left_speed_delta;
            motors_control.speed_cmd.right = 0.85f + right_speed_delta;
            ESP_LOGI(TAG, "Setting motor control: %f, %f",
                     motors_control.speed_cmd.left,
                     motors_control.speed_cmd.right);
            xQueueSend(motors_control_queue_h, &motors_control, pdMS_TO_TICKS(0));
        }

        // Frequency of control loop = 100 Hz (1/0.01s)
        vTaskDelayUntil(&xlast_wake_time, pdMS_TO_TICKS(10));
    }
}
