#ifndef OBSTACLE_AVOIDANCE
#define OBSTACLE_AVOIDANCE

#include "freertos/FreeRTOS.h"

#include "ultrasonic_sensor.h"
#include "helpers.h"

#include "ase_config.h"

typedef struct
{
    QueueHandle_t         mot_cmd_q_handle;
    motors_control_msg_t *mot_ctrl_msg;
    TaskHandle_t          main_task_h;
} obstacle_avoidance_ctx_t;

void obstacle_avoidance_task(void *pvParameters);

#endif