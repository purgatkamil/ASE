#include "scan_move.h"

#define MIN_DISTANCE_TOLERANCE_PERCENT 0.1f

TaskHandle_t scan_move_task_h;
SemaphoreHandle_t manouver_done_sem_h;

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
            ESP_LOGW(SCAN_MOVE_LOG_TAG,
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

    ESP_LOGI(SCAN_MOVE_LOG_TAG,
             "Closest angle (deg): %d, min_distance: %f",
             closest_angle,
             min_distance);

    sonar_set_servo(closest_angle);
    return closest_angle;
}

static void set_at_angle_to_obstacle(int8_t angle, bool *completed)
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
        ESP_LOGI(SCAN_MOVE_LOG_TAG, "Can continue mission!");
        *completed = true;
        return;
    }
    if (moved_once && align_count >= 3)
    {
        ESP_LOGI(SCAN_MOVE_LOG_TAG, "Found optimal position, carry on!");
        *completed = true;
        turn       = 0;
        return;
    }
    if (turn < (angle + 2))
    {
        mc_set_duty(1.0, -1.0);
        moved_once = 1;
    }
    else if (turn > -(angle + 2))
    {
        mc_set_duty(-1.0, 1.0);
        moved_once = 1;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    mc_set_duty(0.0, 0.0);
}

void scan_move_start_and_wait(uint32_t wait_ms)
{
    xTaskNotify(scan_move_task_h, 1, eSetValueWithOverwrite);
    xSemaphoreTake(manouver_done_sem_h, pdMS_TO_TICKS(wait_ms));
}

void scan_move_task(void *pvParameters)
{
    // scan_move_ctx_t *ctx              = (scan_move_ctx_t *)pvParameters;
    scan_move_task_h = xTaskGetCurrentTaskHandle();
    manouver_done_sem_h = xSemaphoreCreateBinary();
    // TaskHandle_t     job_done_ntask_h = ctx->scn_mv_finished_task_h;

    bool     completed_setting_at_angle = false;
    uint8_t  setting_at_angle_ctr       = 0;
    uint8_t  target_angle_to_obstacle   = 0;
    uint32_t ctrl_notif                 = 0;
    uint8_t  active                     = 0;
    for (;;)
    {
        if (xTaskNotifyWait(0, ULONG_MAX, &ctrl_notif, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            active = ctrl_notif == 1;

            if (active)
            {
                reset_scan_params();
                completed_setting_at_angle = false;
                setting_at_angle_ctr       = 0;
                target_angle_to_obstacle   = 0;
            }
        }

        if (!active)
            continue;

        if (setting_at_angle_ctr == 0 && completed_setting_at_angle)
        {
            completed_setting_at_angle = 0;
            setting_at_angle_ctr++;
            reset_scan_params();
            target_angle_to_obstacle = 70;
            scan_range_one_way       = 90;
            ESP_LOGI(SCAN_MOVE_LOG_TAG, "Completed setting at angle (0)");
        }

        if (setting_at_angle_ctr == 1 && completed_setting_at_angle)
        {
            completed_setting_at_angle = 0;
            setting_at_angle_ctr++;
            ESP_LOGI(SCAN_MOVE_LOG_TAG, "Completed setting at angle (1)");
        }

        if (setting_at_angle_ctr == 2)
        {
            // Reaching this step means scan move is completed
            completed_setting_at_angle = true;
            // xTaskNotifyGiveIndexed(job_done_ntask_h, 1);
            active = 0;
            completed_setting_at_angle = false;
            xSemaphoreGive(manouver_done_sem_h);
            continue;
        }

        if (!completed_setting_at_angle)
        {
            set_at_angle_to_obstacle(target_angle_to_obstacle,
                                     &completed_setting_at_angle);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
