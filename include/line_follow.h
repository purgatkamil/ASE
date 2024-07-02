#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include "freertos/FreeRTOS.h"

#include <driver/gpio.h>
#include <esp_log.h>

#include "helpers.h"
#include "ase_typedefs.h"

#define IR_SENSOR_BOTTOM_LEFT_GPIO GPIO_NUM_23
#define IR_SENSOR_BOTTOM_RIGHT_GPIO GPIO_NUM_22
#define IR_SENSOR_BOTTOM_CENTER_GPIO GPIO_NUM_35

// Define motor control output range
#define MOTOR_MAX 0.8
#define MOTOR_MIN -1.0
#define MOTOR_START_THRESHOLD 0.45

// Dead time in which robot can not take another 90 deg turn.
#define TURNING_DEAD_TIME_MS 5000

typedef enum
{
    LINE_FOLLOWER_DIR_STRAIGHT = 0,
    LINE_FOLLOWER_DIR_LEFT,
    LINE_FOLLOWER_DIR_RIGHT
} line_follower_dir_t;

typedef struct
{
    QueueHandle_t mot_cmd_q_handle;
    motors_control_msg_t* mot_ctrl_msg;
    TaskHandle_t main_task_h;
} line_follower_task_context_t;

void line_follower_task(void* pvParameters);

#endif