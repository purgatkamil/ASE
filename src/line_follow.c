#include "line_follow.h"

static void line_follow(uint8_t left_sensor, uint8_t center_sensor, uint8_t right_sensor,
                        double *left_motor, double *right_motor)
{
    *left_motor  = 0.0;
    *right_motor = 0.0;

    double base_speed = 0.6; // Base speed for straight movement
    double turn_speed = 0.5; // Additional speed for turning

    if (center_sensor && !left_sensor && !right_sensor)
    {
        // Go straight
        *left_motor  = base_speed;
        *right_motor = base_speed;
    }
    else if (left_sensor && !center_sensor && !right_sensor)
    {
        // Turn left
        *left_motor  = base_speed - turn_speed;
        *right_motor = base_speed + turn_speed;
    }
    else if (right_sensor && !center_sensor && !left_sensor)
    {
        // Turn right
        *left_motor  = base_speed + turn_speed;
        *right_motor = base_speed - turn_speed;
    }
    else if (left_sensor && center_sensor && !right_sensor)
    {
        // Slight left adjustment
        *left_motor  = base_speed - turn_speed / 2;
        *right_motor = base_speed + turn_speed / 2;
    }
    else if (right_sensor && center_sensor && !left_sensor)
    {
        // Slight right adjustment
        *left_motor  = base_speed + turn_speed / 2;
        *right_motor = base_speed - turn_speed / 2;
    }
    else if (left_sensor && right_sensor)
    {
        // ?
    }
    else
    {
        // Go straight if line is between sensors
        *right_motor = base_speed;
        *left_motor  = base_speed;
    }

    // Due to imperfections in the motors in order to allow robot going relatively straight
    // the control signals have to be adjusted by hand based on experiments
    *right_motor -= 0.132;
    *left_motor += 0.132;

    if (*left_motor > MOTOR_MAX)
        *left_motor = MOTOR_MAX;
    if (*left_motor < MOTOR_MIN)
        *left_motor = MOTOR_MIN;
    if (*right_motor > MOTOR_MAX)
        *right_motor = MOTOR_MAX;
    if (*right_motor < MOTOR_MIN)
        *right_motor = MOTOR_MIN;

    // Handle starting threshold
    if (*left_motor > 0 && *left_motor < MOTOR_START_THRESHOLD)
        *left_motor = MOTOR_START_THRESHOLD;
    if (*left_motor < 0 && *left_motor > -MOTOR_START_THRESHOLD)
        *left_motor = -MOTOR_START_THRESHOLD;
    if (*right_motor > 0 && *right_motor < MOTOR_START_THRESHOLD)
        *right_motor = MOTOR_START_THRESHOLD;
    if (*right_motor < 0 && *right_motor > -MOTOR_START_THRESHOLD)
        *right_motor = -MOTOR_START_THRESHOLD;
}

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
    motors_control_msg_t         *mc              = lf_ctx->mot_ctrl_msg;

    init_ir_gpio();

    line_follower_dir_t movement_dir           = LINE_FOLLOWER_DIR_STRAIGHT;
    uint32_t            ir_l_history           = 0;
    uint32_t            ir_c_history           = 0;
    uint32_t            ir_r_history           = 0;
    uint32_t            orchestrator_notif_val = 0;
    uint8_t             active                 = 0;
    for (;;)
    {
        if (xTaskNotifyWait(0x00, ULONG_MAX, &orchestrator_notif_val, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            active = orchestrator_notif_val == LF_STATE_ACTIVE;
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
                if (N_BITS_ONES_N_ZEROS(ir_c_history, 0xFF, 6, 8)) // center IR crossed the line
                {
                    // ESP_LOGI(LINE_FOLLOWER_LOG_TAG, "[center] IR assumed to have crossed the line");
                    static uint8_t _turning;
                    _turning = 0;
                    if (N_BITS_ONES_N_ZEROS(ir_l_history, 0b1111111, 3, 3))
                    {
                        ESP_LOGI(LINE_FOLLOWER_LOG_TAG, "[center, left] IR assumed to have crossed the line - turning LEFT");
                        movement_dir = LINE_FOLLOWER_DIR_LEFT;
                        _turning     = 1;
                    }
                    else if (N_BITS_ONES_N_ZEROS(ir_r_history, 0b1111111, 3, 3))
                    {
                        ESP_LOGI(LINE_FOLLOWER_LOG_TAG, "[center, right] IR assumed to have crossed the line - turning RIGHT");
                        movement_dir = LINE_FOLLOWER_DIR_RIGHT;
                        _turning     = 1;
                    }

                    if (_turning)
                    {
                        ir_c_history = ir_l_history = ir_r_history = 0;
                        last_turn_time                             = xTaskGetTickCount();
                    }
                }
            }

            switch (movement_dir)
            {
            case LINE_FOLLOWER_DIR_STRAIGHT:
                line_follow(ir_l_history & 0x01, ir_c_history & 0x01, ir_r_history & 0x01,
                            &mc->speed_cmd.left, &mc->speed_cmd.right);
                xQueueSend(lf_ctx->mot_cmd_q_handle, mc, pdMS_TO_TICKS(0));
                break;

            case LINE_FOLLOWER_DIR_LEFT:
            case LINE_FOLLOWER_DIR_RIGHT:
                int8_t sign = movement_dir == LINE_FOLLOWER_DIR_LEFT ? -1 : 1;
                ESP_LOGI(LINE_FOLLOWER_LOG_TAG, "Turning [dir=%s]",
                         sign < 0 ? "LEFT" : "RIGHT");
                send_mot_spd(lf_ctx->mot_cmd_q_handle, mc, sign * 1.0, -sign * 1.0, pdMS_TO_TICKS(0));
                vTaskDelay(pdMS_TO_TICKS(sign < 0 ? 430 + 80 : 430 + 10));
                send_mot_spd(lf_ctx->mot_cmd_q_handle, mc, 0.9, 0.9, pdMS_TO_TICKS(0));
                vTaskDelay(pdMS_TO_TICKS(100));
                movement_dir = LINE_FOLLOWER_DIR_STRAIGHT;
                break;
            }
        }
        // Frequency of control loop = 50 Hz (1/0.02s)
        vTaskDelayUntil(&xlast_wake_time, pdMS_TO_TICKS(20));
    }
}