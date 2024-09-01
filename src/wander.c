#include "wander.h"

#define MIN_DISTANCE_TOLERANCE_PERCENT 0.1f

// Number of times robot did not move with at least one movement before
static uint8_t align_count        = 0;
static uint8_t no_move_scan_count = 0;
static bool    moved_once         = 0;
static int16_t scan_range_one_way = 50;

static inline void reset_scan_params()
{
    align_count        = 0;
    moved_once         = 0;
    no_move_scan_count = 0;
    scan_range_one_way = 50;
}

static int8_t scan()
{
    ultrasonic_measurement_t sonar_meas;
    float                    distance    = 0.0;
    int16_t                  servo_angle = -scan_range_one_way;

    float   min_distance     = SONAR_MAX_DISTANCE_CM;
    int16_t closest_angle    = 0;
    bool    distance_read_ok = 0;

    while (servo_angle <= scan_range_one_way)
    {
        sonar_set_servo(servo_angle);
        vTaskDelay(pdMS_TO_TICKS(100));

        sonar_meas = sonar_get_measurement(300, &distance_read_ok);
        distance   = sonar_meas.distance;

        if (!distance_read_ok)
        {
            ESP_LOGW(WANDER_LOG_TAG,
                     "Distance read not ok! (angle[deg]=%d)",
                     servo_angle);
            continue;
        }

        if (distance <= (min_distance - 1.5f))
        {
            min_distance  = distance;
            closest_angle = servo_angle;
        }

        servo_angle += 5;
    }

    ESP_LOGI(WANDER_LOG_TAG,
             "Closest angle (deg): %d, min_distance: %f",
             closest_angle,
             min_distance);

    sonar_set_servo(closest_angle);
    return closest_angle;
}

static int8_t set_at_angle_to_obstacle(int8_t angle, bool *completed)
{
    int turn = scan();
    if (moved_once && (turn >= (15 + angle) || turn <= -(15 + angle)))
    {
        align_count++;
    }
    else if (turn <= (15 + angle) && turn >= -(15 + angle))
    {
        no_move_scan_count++;
    }
    if (no_move_scan_count > 3)
    {
        ESP_LOGI(WANDER_LOG_TAG, "Can continue mission!");
        *completed = true;
        return 0;
    }
    if (moved_once && align_count >= 3)
    {
        ESP_LOGI(WANDER_LOG_TAG, "Found optimal position, carry on!");
        *completed = true;
        turn       = 0;
        return 0;
    }
    if (turn < (angle + 2))
    {
        return 1;
    }
    else if (turn > -(angle + 2))
    {
        return -1;
    }

    return 0;
}

static inline void init_ir_gpio()
{
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << IR_BOTTOM_LEFT_GPIO) |
                        (1ULL << IR_BOTTOM_CENTER_GPIO) |
                        (1ULL << IR_BOTTOM_RIGHT_GPIO),
        .pull_down_en = 0,
        .pull_up_en   = 1};

    gpio_config(&io_conf);
}

void wander_task(void *pvParameters)
{
    // wander_ctx_t *oa_ctx = (wander_ctx_t *)pvParameters;

    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = 1,
        .pull_down_en = 0,
        .pin_bit_mask = (1ULL << IR_TOP_LEFT_GPIO) |
                        (1ULL << IR_TOP_RIGHT_GPIO) |
                        (1ULL << IR_TOP_FRONT_GPIO)};

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    sonar_task_ctx_t sonar_task_ctx;
    xTaskCreate(&ultrasonic_sensor_task, "sonar", 4096, (void *)&sonar_task_ctx, 10, NULL);

    // uint8_t  ir_f                = 0;
    // uint8_t  ir_l                = 0;
    // uint8_t  ir_r                = 0;
    // uint8_t  bottom_ir_c         = 0;
    uint8_t  bottom_ir_l            = 0;
    uint8_t  bottom_ir_r            = 0;
    uint32_t orchestrator_notif_val = 0;
    uint8_t  active                 = 0;

    bool    completed_setting_at_angle = false;
    uint8_t setting_at_angle_ctr       = 0;
    uint8_t target_angle_to_obstacle   = 0;
    for (;;)
    {
        if (xTaskNotifyWait(0, ULONG_MAX, &orchestrator_notif_val, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            // ESP_LOGI(WANDER_LOG_TAG, "Notify received: %lu", orchestrator_notif_val);
            active = orchestrator_notif_val == WANDER_STATE_ACTIVE;
            if (active)
            {
                // This code executes when active is true thus when this
                // mode has been activated and will not execute again without
                // another state change request (activate/deactivate)
                reset_scan_params();
                completed_setting_at_angle = false;
                setting_at_angle_ctr       = 0;
                target_angle_to_obstacle   = 0;
            }
        }

        // ir_l = 1 - gpio_get_level(IR_TOP_LEFT_GPIO);
        // ir_f = 1 - gpio_get_level(IR_TOP_FRONT_GPIO);
        // ir_r = 1 - gpio_get_level(IR_TOP_RIGHT_GPIO);

        // Line breaching
        // Reversing logic of IR sensors, as at GPIO level 0 they are active
        // thanks to subtracting one from actual measurement if they are active
        // the value that is stored is 1, otherwise 0.
        bottom_ir_l = (1 - gpio_get_level(IR_BOTTOM_LEFT_GPIO));
        // bottom_ir_ = (1 - gpio_get_level(IR_BOTTOM_CENTER_GPIO));
        bottom_ir_r = (1 - gpio_get_level(IR_BOTTOM_RIGHT_GPIO));

        // ESP_LOGI(WANDER_LOG_TAG, "LEFT: %d, RIGHT: %d", bottom_ir_l, bottom_ir_r);

        if (active && (bottom_ir_l || bottom_ir_r))
        {
            mc_set_duty(-1.0, -1.0);
            vTaskDelay(pdMS_TO_TICKS(200));

            if (bottom_ir_l)
            {
                mc_set_duty(-1.0, 1.0);
            }
            else if (bottom_ir_r)
            {
                mc_set_duty(1.0, -1.0);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            mc_set_duty(0.0, 0.0);
        }

        if (active && setting_at_angle_ctr == 0 && completed_setting_at_angle)
        {
            completed_setting_at_angle = 0;
            setting_at_angle_ctr       = 1;
            reset_scan_params();
            target_angle_to_obstacle = 70;
            scan_range_one_way       = 90;
            ESP_LOGI(WANDER_LOG_TAG, "Completed setting at angle (0)");
        }

        if (active && setting_at_angle_ctr == 1 && completed_setting_at_angle)
        {
            // completed_setting_at_angle = 0;
            setting_at_angle_ctr = 2;
            ESP_LOGI(WANDER_LOG_TAG, "Completed setting at angle (1)");
        }

        if (active && setting_at_angle_ctr == 2)
        {
            mc_set_duty(1.0, 1.0);
            completed_setting_at_angle = true;
        }

        if (active && !completed_setting_at_angle)
        {
            static int8_t facing_movement_dir;
            facing_movement_dir = set_at_angle_to_obstacle(target_angle_to_obstacle,
                                                           &completed_setting_at_angle);

            // Movement needed if movement dir is not zero
            if (facing_movement_dir != 0)
            {
                mc_disable_pwm();
                if (facing_movement_dir < 0)
                {
                    mc_set_duty(-1.0, 1.0);
                }
                else if (facing_movement_dir > 0)
                {
                    mc_set_duty(1.0, -1.0);
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                moved_once = 1;
                mc_set_duty(0.0, 0.0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(700));
    }
}