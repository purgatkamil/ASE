#include "line_follow.h"

static inline void init_ir_gpio()
{
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << IR_SENSOR_BOTTOM_LEFT_GPIO) |
                        (1ULL << IR_SENSOR_BOTTOM_CENTER_GPIO) |
                        (1ULL << IR_SENSOR_BOTTOM_RIGHT_GPIO),
        .pull_down_en = 0,
        .pull_up_en   = 1};

    gpio_config(&io_conf);
}

void line_follower_task(void *pvParameters)
{
    TickType_t                    xlast_wake_time = xTaskGetTickCount();
    TickType_t                    last_turn_time  = 0;
    line_follower_task_context_t *lf_ctx          = (line_follower_task_context_t *)pvParameters;

    init_ir_gpio();

    line_follower_dir_t movement_dir           = LINE_FOLLOWER_DIR_STRAIGHT;
    uint32_t            ir_l_history           = 0;
    uint32_t            ir_c_history           = 0;
    uint32_t            ir_r_history           = 0;
    uint32_t            orchestrator_notif_val = 0;
    uint8_t             active                 = 0;
    int8_t              turnig_dir_interp      = 1;
    for (;;)
    {
        if (xTaskNotifyWait(0x00, ULONG_MAX, &orchestrator_notif_val, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            active = (orchestrator_notif_val & 0x01) == LF_STATE_ACTIVE;

            // Change to enum later, please.
            if ((orchestrator_notif_val & 0x02) == 0x02)
            {
                // Change interpretation of turning direction for one time
                turnig_dir_interp = -1;
            }
        }

        // Reversing logic of IR sensors, as at GPIO level 0 they are active
        // thanks to subtracting one from actual measurement if they are active
        // the value that is stored is 1, otherwise 0.
        ir_l_history = ir_l_history << 1 | (1 - gpio_get_level(IR_SENSOR_BOTTOM_LEFT_GPIO));
        ir_c_history = ir_c_history << 1 | (1 - gpio_get_level(IR_SENSOR_BOTTOM_CENTER_GPIO));
        ir_r_history = ir_r_history << 1 | (1 - gpio_get_level(IR_SENSOR_BOTTOM_RIGHT_GPIO));

        // Check if any sensor shows some activity, if yes, notify back
        // the main task about that so it can direct actions based on this information.
        if (N_BITS_ONES_N_ZEROS(ir_l_history, 0x00, 5, 0) ||
            N_BITS_ONES_N_ZEROS(ir_c_history, 0x00, 5, 0) ||
            N_BITS_ONES_N_ZEROS(ir_r_history, 0x00, 5, 0))
        {
            xTaskNotify(lf_ctx->main_task_h, 1ul, eSetValueWithOverwrite);
        }

        if (active)
        {
            // If recently center was active for some time and the most recent one are not so much
            // then we we can assume center actually went out of line
            // Forward checking max 100 ms
            // Also, after successfull turn detection, block further turning for at least couple seconds
            // in order to eliminate robot false classifications right after picking up orthogonal line.
            if (movement_dir == LINE_FOLLOWER_DIR_STRAIGHT &&
                pdTICKS_TO_MS(xTaskGetTickCount() - last_turn_time) > TURNING_DEAD_TIME_MS)
            {
                static uint8_t _turning;

                if (turnig_dir_interp != 1)
                {
                    if (ir_l_history & 0x01)
                    {
                        movement_dir = LINE_FOLLOWER_DIR_LEFT;
                        _turning     = 1;
                    }
                    else if (ir_r_history & 0x01)
                    {
                        movement_dir = LINE_FOLLOWER_DIR_RIGHT;
                        _turning     = 1;
                    }
                }
                else if (N_BITS_ONES_N_ZEROS(ir_c_history, 0xFF, 0, 8)) // center IR crossed the line
                {
                    // ESP_LOGI(LINE_FOLLOWER_LOG_TAG, "[center] IR assumed to have crossed the line");
                    _turning = 0;
                    // if (N_BITS_ONES_N_ZEROS(ir_l_history, 0b1111111, 3, 3))
                    // {
                    //     ESP_LOGI(LINE_FOLLOWER_LOG_TAG, "[center, left] IR assumed to have crossed the line - turning LEFT");
                    //     movement_dir = LINE_FOLLOWER_DIR_LEFT;
                    //     _turning     = 1;
                    // }
                    if (N_BITS_ONES_N_ZEROS(ir_r_history, 0b1111111, 3, 5))
                    {
                        ESP_LOGI(LINE_FOLLOWER_LOG_TAG, "[center, right] IR assumed to have crossed the line - turning RIGHT");
                        movement_dir = LINE_FOLLOWER_DIR_RIGHT;
                        _turning     = 1;
                    }
                }

                if (_turning)
                {
                    ir_c_history = ir_l_history = ir_r_history = 0;
                    last_turn_time                             = xTaskGetTickCount();
                }
            }

        }
        // Frequency of control loop = 50 Hz (1/0.02s)
        vTaskDelayUntil(&xlast_wake_time, pdMS_TO_TICKS(20));
    }
}