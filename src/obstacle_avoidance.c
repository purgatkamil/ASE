#include "obstacle_avoidance.h"

QueueHandle_t sonar_queue_h;
TaskHandle_t sonar_task_h;


static void inline setServo(int16_t angle, TaskHandle_t sonar_task_h)
{
    xTaskNotifyIndexed(sonar_task_h, SONAR_SET_SERVO_ANGLE_NOTIF_IDX, (uint32_t)angle, eSetValueWithOverwrite);
    xTaskNotifyWaitIndexed(MAIN_SERVO_ANGLE_READY, 0x00, 0x00, NULL, portMAX_DELAY);
}

static ultrasonic_measurement_t inline getDistance(QueueHandle_t q, uint32_t wait_ms)
{
    static ultrasonic_measurement_t measurement;
    
    xQueueReceive(q, &measurement, pdMS_TO_TICKS(wait_ms));
    return measurement;
}

int8_t scan()
{
    bool reached = 0;
    uint32_t min_distance = 2000;
    int8_t closest_angle = 80;
    ultrasonic_measurement_t values;
    int8_t angle_dir = 1;
    static const int16_t scan_range_one_way = 80;
    int16_t servo_angle = -scan_range_one_way;

    while(servo_angle <= scan_range_one_way)
    {
        setServo(servo_angle, sonar_task_h);

        values = getDistance(sonar_queue_h, 300);

        vTaskDelay(20);

        if(values.distance < min_distance)
        {
            min_distance = values.distance;
            closest_angle = values.angle;
        }

        servo_angle += 10;
        
    }

    setServo(closest_angle, sonar_task_h);
    return closest_angle;
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

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    sonar_queue_h = xQueueCreate(1, sizeof(ultrasonic_measurement_t));

    sonar_task_ctx_t sonar_task_ctx = {
        .masurements_queue_h            = sonar_queue_h,
        .servo_angle_ready_notif_task_h = xTaskGetCurrentTaskHandle()};
    xTaskCreate(&ultrasonic_sensor_task, "sonar", 8192, (void *)&sonar_task_ctx, 10, &sonar_task_h);

    if (sonar_queue_h == NULL)
    {
        ESP_LOGE(OBSTACLE_AVOIDANCE_LOG_TAG, "Failed to create sonar queue!");
        abort();
    }

    float   ultrasonic_distance = 0.f;

    uint8_t  ir_f                   = 0;
    uint8_t  ir_l                   = 0;
    uint8_t  ir_r                   = 0;
    uint32_t orchestrator_notif_val = 0;
    uint8_t  active                 = 1;

        //SET_BIT(mc->cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
        //send_mot_spd(mc_q_h, mc, 0.0, 1.0, pdMS_TO_TICKS(0));  

    for (;;)
    {
        int turn = scan();
        //turn = turn - 80;
        if(turn > 5)
        {
            SET_BIT(mc->cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
            send_mot_spd(mc_q_h, mc, -1.0, 1.0, pdMS_TO_TICKS(0));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else if(turn < -5)
        {
            SET_BIT(mc->cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
            send_mot_spd(mc_q_h, mc, 1.0, -1.0, pdMS_TO_TICKS(0));
            vTaskDelay(pdMS_TO_TICKS(100)); 
        }
      send_mot_spd(mc_q_h, mc, 0.0, 0.0, pdMS_TO_TICKS(0));
      turn = 80;     
      
       /* if (xTaskNotifyWait(0x00, ULONG_MAX, &orchestrator_notif_val, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            ESP_LOGI(OBSTACLE_AVOIDANCE_LOG_TAG, "Notify received: %lu", orchestrator_notif_val);
            active = orchestrator_notif_val == AVOIDANCE_STATE_ACTIVE;
        }

        ir_l = 1 - gpio_get_level(IR_TOP_LEFT_GPIO);
        ir_f = 1 - gpio_get_level(IR_TOP_FRONT_GPIO);
        ir_r = 1 - gpio_get_level(IR_TOP_RIGHT_GPIO);

        if (active)
        {
            
            SET_BIT(mc->cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
            send_mot_spd(mc_q_h, mc, 220.0, 220.0, pdMS_TO_TICKS(0));


            // Mr Kamil please code your logic here :)
            // Move motors using command below
            // This one to enable motors -> SET_BIT(mc->cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
            // This one to stop the motors -> CLEAR_BIT(mc->cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
            // But they will take effect ONLY after calling the one below!!!
            // send_mot_spd(mc_q_h, mc, 0.0, 0.0, pdMS_TO_TICKS(0));
        }*/

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}