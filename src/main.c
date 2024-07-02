#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <driver/gpio.h>

#include "motor_control.h"
#include "ultrasonic_sensor.h"
#include "ase_typedefs.h"
#include "helpers.h"

static const char *TAG = "orchestrator";

#define IR_SENSOR_BOTTOM_LEFT_GPIO GPIO_NUM_23
#define IR_SENSOR_BOTTOM_RIGHT_GPIO GPIO_NUM_22
#define IR_SENSOR_BOTTOM_CENTER_GPIO GPIO_NUM_35

static inline void init_ir_gpio()
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << IR_SENSOR_BOTTOM_LEFT_GPIO) |
                        (1ULL << IR_SENSOR_BOTTOM_CENTER_GPIO) |
                        (1ULL << IR_SENSOR_BOTTOM_RIGHT_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 1};

    gpio_config(&io_conf);
}

// Define motor control output range
#define MOTOR_MAX 0.93
#define MOTOR_MIN -1.0
#define MOTOR_START_THRESHOLD 0.55

void line_follow(uint8_t left_sensor, uint8_t center_sensor, uint8_t right_sensor, double *left_motor, double *right_motor)
{
    *left_motor = 0.0;
    *right_motor = 0.0;

    double base_speed = 0.68; // Base speed for straight movement
    double turn_speed = 0.5;  // Additional speed for turning

    if (center_sensor && !left_sensor && !right_sensor)
    {
        // Go straight
        *left_motor = base_speed;
        *right_motor = base_speed;
    }
    else if (left_sensor && !center_sensor && !right_sensor)
    {
        // Turn left
        *left_motor = base_speed - turn_speed;
        *right_motor = base_speed + turn_speed;
    }
    else if (right_sensor && !center_sensor && !left_sensor)
    {
        // Turn right
        *left_motor = base_speed + turn_speed;
        *right_motor = base_speed - turn_speed;
    }
    else if (left_sensor && center_sensor && !right_sensor)
    {
        // Slight left adjustment
        *left_motor = base_speed - turn_speed / 2;
        *right_motor = base_speed + turn_speed / 2;
    }
    else if (right_sensor && center_sensor && !left_sensor)
    {
        // Slight right adjustment
        *left_motor = base_speed + turn_speed / 2;
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
        *left_motor = base_speed;
    }

    // Due to imperfections have to increase control signal for right motor
    // in order to allow robot going relatively straight
    // *right_motor += 0.12;
    *right_motor += 0.04;

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

static inline BaseType_t send_mot_spd(
    QueueHandle_t q,
    motors_control_msg_t *mc,
    double spdLeft,
    double spdRight)
{
    mc->speed_cmd.left = spdLeft;
    mc->speed_cmd.right = spdRight;
    return xQueueSend(q, mc, pdMS_TO_TICKS(0));
}

void app_main()
{
    QueueHandle_t sonar_queue_h = xQueueCreate(5, sizeof(ultrasonic_measurement_t));
    QueueHandle_t motors_control_queue_h = xQueueCreate(20, sizeof(motors_control_msg_t));
    sonar_motors_q_ok_or_abort(sonar_queue_h, motors_control_queue_h, TAG);
    init_ir_gpio();

    // Create motor control task
    xTaskCreate(&motor_control_task, "motor_control_task", 2048, (void *)motors_control_queue_h, 15, NULL);
    // Create ultrasonic sensor scanner task
    xTaskCreate(&ultrasonic_sensor_task, "ultrasonic_sensor_task", 4096, (void *)sonar_queue_h, 10, NULL);

    ultrasonic_measurement_t sonar_notif = {
        .angle = 0,
        .distance = 0.0f};

    motors_control_msg_t motors_control = {
        .speed_cmd = {
            .left = 0.85f,
            .right = 0.85f}};

    TickType_t xlast_wake_time = xTaskGetTickCount();
    mission_state_t mission_state = MISSION_STATE_IDLE;
    uint32_t ir_l_history = 0, ir_c_history = 0, ir_r_history = 0;
    for (;;)
    {
        if (xQueueReceive(sonar_queue_h, &sonar_notif, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            static uint8_t sonar_print_ctr = 0;
            if (sonar_print_ctr++ % 50 == 0)
            {
                ESP_LOGI(TAG, "Ultrasonic measurement at angle %d deg: %fcm",
                         sonar_notif.angle,
                         sonar_notif.distance);
                sonar_print_ctr = 0;
            }

            if (sonar_notif.distance > 10 && sonar_notif.distance < 30)
            {
                ESP_LOGI(TAG, "Enabling motors!");
                SET_BIT(motors_control.cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
                xQueueSend(motors_control_queue_h, &motors_control, pdMS_TO_TICKS(0));
                mission_state = MISSION_STATE_FOLLOW_LINE;
            }
            else if (sonar_notif.distance < 8)
            {
                mission_state = MISSION_STATE_STOP;
            }
        }

        // Reversing logic of IR sensors, as at GPIO level 0 they are active
        // thanks to subtracting one from actual measurement if they are active
        // the value that is stored is 1, otherwise 0.
        ir_l_history = ir_l_history << 1 | (1 - gpio_get_level(IR_SENSOR_BOTTOM_LEFT_GPIO));
        ir_c_history = ir_c_history << 1 | (1 - gpio_get_level(IR_SENSOR_BOTTOM_CENTER_GPIO));
        ir_r_history = ir_r_history << 1 | (1 - gpio_get_level(IR_SENSOR_BOTTOM_RIGHT_GPIO));

        // ESP_LOGI(TAG, "Count of ones: [center = %d], [left = %d]", count_n_of_ones(ir_c_history), count_n_of_ones(ir_l_history));
        // ESP_LOGI(TAG, "Count of ones: [center(27) = %d, center(5) = %d]",
        //     count_n_of_ones(ir_c_history & (~((uint32_t)0b11111))),
        //     count_n_of_ones((~ir_c_history) & 0b11111));

        // If recently center was active for some time and the most recent one are not so much
        // then we we can assume center actually went out of line
        // Forward checking max 100 ms
        if (mission_state != MISSION_STATE_TURN)
        {
            if (N_BITS_ONES_N_ZEROS(ir_c_history, 0b1111111, 5, 5))
            {
                ESP_LOGI(TAG, "Center outta the line!");
                if (N_BITS_ONES_N_ZEROS(ir_l_history, 0b1111111, 4, 2))
                {
                    ESP_LOGI(TAG, "Left outta the line ASWELL! Assuming LEFT turn is met!");
                    mission_state = MISSION_STATE_TURN;
                    ir_c_history = ir_l_history = ir_r_history = 0;
                }
                else if (N_BITS_ONES_N_ZEROS(ir_r_history, 0b1111111, 3, 2))
                {
                    ESP_LOGI(TAG, "Right outta the line ASWELL! Assuming RIGHT turn is met!");
                    // mission_state = MISSION_STATE_TURN;
                    // ir_c_history = ir_l_history = ir_r_history = 0;
                }
            }
        }

        switch (mission_state)
        {
        case MISSION_STATE_IDLE:
            // ESP_LOGI(TAG, "Idling!");
            break;

        case MISSION_STATE_FOLLOW_LINE:
            // ESP_LOGI(TAG, "MISSION_STATE_FOLLOW_LINE");
            line_follow(ir_l_history & 0x01, ir_c_history & 0x01, ir_r_history & 0x01,
                        &motors_control.speed_cmd.left, &motors_control.speed_cmd.right);
            xQueueSend(motors_control_queue_h, &motors_control, pdMS_TO_TICKS(5));
            break;

        case MISSION_STATE_TURN:
            ESP_LOGI(TAG, "MISSION_STATE_TURN");
            // Before turning, back off a little bit
            send_mot_spd(motors_control_queue_h, &motors_control, -0.8, -0.9);
            vTaskDelay(pdMS_TO_TICKS(100));
            ///////////////////////////////////////
            send_mot_spd(motors_control_queue_h, &motors_control, -1.0, 1.0);
            vTaskDelay(pdMS_TO_TICKS(470));
            send_mot_spd(motors_control_queue_h, &motors_control, 0.0, 0.0);
            vTaskDelay(pdMS_TO_TICKS(100));
            // mission_state = MISSION_STATE_IDLE;
            mission_state = MISSION_STATE_FOLLOW_LINE;
            break;

        case MISSION_STATE_STOP:
            ESP_LOGI(TAG, "MISSION_STATE_STOP - Disabling motors!");
            CLEAR_BIT(motors_control.cmd_flags, MOTORS_CONTROL_FLAGS_ENABLE);
            send_mot_spd(motors_control_queue_h, &motors_control, 0.0, 0.0);
            mission_state = MISSION_STATE_IDLE;
            break;

        default:
            break;
        }

        // Frequency of control loop = 100 Hz (1/0.01s)
        vTaskDelayUntil(&xlast_wake_time, pdMS_TO_TICKS(20));
    }
}
