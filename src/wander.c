#include "wander.h"

static inline void init_ir_gpio()
{
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_LOW_LEVEL,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << IR_BOTTOM_LEFT_GPIO) |
                        (1ULL << IR_BOTTOM_CENTER_GPIO) |
                        (1ULL << IR_BOTTOM_RIGHT_GPIO) |
                        (1ULL << IR_TOP_LEFT_GPIO) |
                        (1ULL << IR_TOP_RIGHT_GPIO) |
                        (1ULL << IR_TOP_FRONT_GPIO),
        .pull_down_en = 0,
        .pull_up_en   = 1,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

#define WANDER_SPEED() mc_set_duty(0.9, 0.9)

void drift(int side)
{
    for (int i = 0; i < 5; i++)
    {
        mc_set_duty(0.0, -side * 0.9);
        vTaskDelay(pdMS_TO_TICKS(180));
        mc_set_duty(0.0, 0.0);
        vTaskDelay(pdMS_TO_TICKS(100));

        mc_set_duty(0.9, 0.9);
        vTaskDelay(pdMS_TO_TICKS(180));
        mc_set_duty(0.0, 0.0);
        vTaskDelay(pdMS_TO_TICKS(100));

        mc_set_duty(side * 0.9, 0.0);
        vTaskDelay(pdMS_TO_TICKS(180));

        mc_set_duty(-0.9, -0.9);
        vTaskDelay(pdMS_TO_TICKS(170));
        mc_set_duty(0.0, 0.0);

        mc_set_duty(0.0, side * 0.9);
        vTaskDelay(pdMS_TO_TICKS(200));

        mc_set_duty(0.0, 0.0);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

void wander_task(void *pvParameters)
{
    // wander_ctx_t *oa_ctx = (wander_ctx_t *)pvParameters;
    // wander_task_h = xTaskGetCurrentTaskHandle();

    TaskHandle_t scan_move_task_h;

    init_ir_gpio();

    sonar_task_ctx_t sonar_task_ctx;
    xTaskCreate(&ultrasonic_sensor_task, "sonar", 4096, (void *)&sonar_task_ctx, 10, NULL);
    xTaskCreate(&scan_move_task, "scan_move", 4096, NULL, 10, &scan_move_task_h);

    uint8_t  ir_f                   = 0;
    uint8_t  ir_l                   = 0;
    uint8_t  ir_r                   = 0;
    uint8_t  bot_ir_c               = 0;
    uint8_t  bot_ir_l               = 0;
    uint8_t  bot_ir_r               = 0;
    uint32_t orchestrator_notif_val = 0;
    uint8_t  active                 = 0;
    for (;;)
    {
        if (xTaskNotifyWait(0, ULONG_MAX, &orchestrator_notif_val,
                            active ? 0 : portMAX_DELAY) == pdTRUE)
        {
            // ESP_LOGI(WANDER_LOG_TAG, "Notify received: %lu", orchestrator_notif_val);
            active = orchestrator_notif_val == WANDER_STATE_ACTIVE;
            if (active)
            {
                // This code executes when active is true thus when this
                // mode has been activated and will not execute again without
                // another state change request (activate/deactivate)

                // Start moving right away
                // WANDER_SPEED();

                WANDER_SPEED();
            }

            if (!active)
            {
                xTaskNotify(scan_move_task_h, 0, eSetValueWithOverwrite);
            }
        }

        if (!active)
        {
            continue;
        }

        // Reversing logic of IR sensors, as at GPIO level 0 they are active
        // thanks to subtracting one from actual measurement if they are active
        // the value that is stored is 1, otherwise 0.

        // Obstacle detection ir sensors (on the top)
        ir_l = 1 - gpio_get_level(IR_TOP_LEFT_GPIO);
        ir_f = 1 - gpio_get_level(IR_TOP_FRONT_GPIO);
        ir_r = 1 - gpio_get_level(IR_TOP_RIGHT_GPIO);

        // ESP_LOGI(WANDER_LOG_TAG, "LEFT: %d, RIGHT: %d", bottom_ir_l, bottom_ir_r);

        if (ir_f)
        {
            // If any of the top sensors detects something
            // start scan move manouver after reversing for a moment
            mc_set_duty(-1.0, -1.0);
            vTaskDelay(pdMS_TO_TICKS(300));
            mc_set_duty(0.0, 0.0);
            static const uint32_t timeout = 60 * 1000;
            scan_move_start_and_wait(timeout);
            WANDER_SPEED();
        }

        if (ir_r)
        {
            drift(1);
            WANDER_SPEED();
        }

        if (ir_l)
        {
            drift(-1);
            WANDER_SPEED();
        }

        // Line breaching ir sensors (at the bottom)
        bot_ir_l = (1 - gpio_get_level(IR_BOTTOM_LEFT_GPIO));
        bot_ir_c = (1 - gpio_get_level(IR_BOTTOM_CENTER_GPIO));
        bot_ir_r = (1 - gpio_get_level(IR_BOTTOM_RIGHT_GPIO));

        if (bot_ir_l || bot_ir_c || bot_ir_r)
        {
            mc_set_duty(-0.925, -0.925);
            vTaskDelay(pdMS_TO_TICKS(700));

            mc_set_duty(0.0, 0.0);
            vTaskDelay(pdMS_TO_TICKS(100));

            if (bot_ir_l)
            {
                mc_set_duty(0.95, -0.95);
            }
            else if (bot_ir_r || bot_ir_c)
            {
                mc_set_duty(-0.95, 0.95);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            WANDER_SPEED();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}