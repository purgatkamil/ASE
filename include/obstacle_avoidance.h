#ifndef OBSTACLE_AVOIDANCE
#define OBSTACLE_AVOIDANCE

#include "freertos/FreeRTOS.h"

#include "ultrasonic_sensor.h"
#include "helpers.h"
#include "motor_control.h"

#include "ase_config.h"

typedef struct
{
    TaskHandle_t          main_task_h;
} obstacle_avoidance_ctx_t;

void obstacle_avoidance_task(void *pvParameters);

#endif