#include "obstacle_avoidance.h"

static QueueHandle_t sonar_queue_h;
static TaskHandle_t  sonar_task_h;

static void inline setServo(int16_t angle, TaskHandle_t sonar_task_h)
{
    xTaskNotifyIndexed(sonar_task_h, SONAR_SET_SERVO_ANGLE_NOTIF_IDX, (uint32_t)angle, eSetValueWithOverwrite);
    xTaskNotifyWaitIndexed(MAIN_SERVO_ANGLE_READY, 0x00, 0x00, NULL, portMAX_DELAY);
}

static float inline getDistance(QueueHandle_t q, uint32_t wait_ms, bool *success)
{
    static ultrasonic_measurement_t measurement;
    bool                            read_ok = xQueueReceive(q, &measurement, pdMS_TO_TICKS(wait_ms));
    if (success != NULL)
    {
        *success = read_ok;
    }
    return measurement.distance;
}

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
    float   distance    = 0.0;
    int16_t servo_angle = -scan_range_one_way;

    float   min_distance     = SONAR_MAX_DISTANCE_CM;
    int16_t closest_angle    = 0;
    bool    distance_read_ok = 0;

    while (servo_angle <= scan_range_one_way)
    {
        setServo(servo_angle, sonar_task_h);
        vTaskDelay(pdMS_TO_TICKS(100));
        distance = getDistance(sonar_queue_h, 300, &distance_read_ok);

        if (!distance_read_ok)
        {
            ESP_LOGW(OBSTACLE_AVOIDANCE_LOG_TAG,
                     "Distance read not ok! (angle[deg]=%d)",
                     servo_angle);
            servo_angle += 5;
            continue;
        }

        // if (distance < (min_distance * (1.f - MIN_DISTANCE_TOLERANCE_PERCENT)))
        if (distance <= (min_distance - 1.5f))
        {
            min_distance  = distance;
            closest_angle = servo_angle;
        }

        servo_angle += 5;
    }

    ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG,
             "Closest angle (deg): %d, min_distance: %f",
             closest_angle,
             min_distance);

    setServo(closest_angle, sonar_task_h);
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
        ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Can continue mission!");
        *completed = true;
        return 0;
    }
    if (moved_once && align_count >= 3)
    {
        ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Found optimal position, carry on!");
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

void obstacle_avoidance_task(void *pvParameters)
{
    obstacle_avoidance_ctx_t *oa_ctx = (obstacle_avoidance_ctx_t *)pvParameters;
    motors_control_msg_t     *mc     = oa_ctx->mot_ctrl_msg;
    QueueHandle_t             mc_q_h = oa_ctx->mot_cmd_q_handle;

    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = 1,
        .pull_down_en = 0,
        .pin_bit_mask = (1ULL << IR_TOP_LEFT_GPIO) |
                        (1ULL << IR_TOP_RIGHT_GPIO) |
                        (1ULL << IR_TOP_FRONT_GPIO)};

    if (sonar_queue_h != NULL || sonar_task_h != NULL)
    {
        esp_system_abort("Sonar queue or sonar task handle is not NULL before initialization! Aborting.");
    }

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    sonar_queue_h = xQueueCreate(1, sizeof(ultrasonic_measurement_t));
    if (sonar_queue_h == NULL)
    {
        ESP_LOGE(OBSTACLE_AVOIDANCE_LOG_TAG, "Failed to create sonar queue!");
        abort();
    }
    sonar_task_ctx_t sonar_task_ctx = {
        .masurements_queue_h            = sonar_queue_h,
        .servo_angle_ready_notif_task_h = xTaskGetCurrentTaskHandle()};
    xTaskCreate(&ultrasonic_sensor_task, "sonar", 4096, (void *)&sonar_task_ctx, 10, &sonar_task_h);

    // uint8_t  ir_f                   = 0;
    // uint8_t  ir_l                   = 0;
    // uint8_t  ir_r                   = 0;
    uint32_t orchestrator_notif_val = 0;
    uint8_t  active                 = 0;

    bool    completed_setting_at_angle = false;
    uint8_t setting_at_angle_ctr       = 0;
    uint8_t target_angle_to_obstacle   = 0;
    for (;;)
    {
        if (xTaskNotifyWait(0x00, ULONG_MAX, &orchestrator_notif_val, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Notify received: %lu", orchestrator_notif_val);
            active = orchestrator_notif_val == AVOIDANCE_STATE_ACTIVE;
            if (active)
            {
                reset_scan_params();
                completed_setting_at_angle = false;
                setting_at_angle_ctr       = 0;
                target_angle_to_obstacle   = 0;
            }
        }

        // ir_l = 1 - gpio_get_level(IR_TOP_LEFT_GPIO);
        // ir_f = 1 - gpio_get_level(IR_TOP_FRONT_GPIO);
        // ir_r = 1 - gpio_get_level(IR_TOP_RIGHT_GPIO);

        if (active && setting_at_angle_ctr == 0 && completed_setting_at_angle)
        {
            completed_setting_at_angle = 0;
            setting_at_angle_ctr       = 1;
            reset_scan_params();
            target_angle_to_obstacle   = 70;
            scan_range_one_way         = 90;
            ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Completed setting at angle (0)");
        }

        if (active && setting_at_angle_ctr == 1 && completed_setting_at_angle)
        {
            // completed_setting_at_angle = 0;
            setting_at_angle_ctr = 2;
            ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Completed setting at angle (1)");

        }

        if (active && !completed_setting_at_angle)
        {
            static int8_t facing_movement_dir;
            facing_movement_dir = set_at_angle_to_obstacle(target_angle_to_obstacle,
                                                           &completed_setting_at_angle);

            // Movement needed if movement dir is not zero
            if (facing_movement_dir != 0)
            {
                SET_BIT(mc->cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
                if (facing_movement_dir < 0)
                {
                    send_mot_spd(mc_q_h, mc, -1.0, 1.0, pdMS_TO_TICKS(0));
                }
                else if (facing_movement_dir > 0)
                {
                    send_mot_spd(mc_q_h, mc, 1.0, -1.0, pdMS_TO_TICKS(0));
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                moved_once = 1;
                send_mot_spd(mc_q_h, mc, 0.0, 0.0, pdMS_TO_TICKS(0));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(700));
    }
}