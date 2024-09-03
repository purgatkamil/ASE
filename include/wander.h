#ifndef WANDER_H
#define WANDER_H

#include "freertos/FreeRTOS.h"

#include "helpers.h"
#include "motor_control.h"
#include "scan_move.h"
#include "ultrasonic_sensor.h"

#include "ase_config.h"

typedef struct
{
    TaskHandle_t main_task_h;
} wander_ctx_t;

void wander_task(void *pvParameters);

#endif