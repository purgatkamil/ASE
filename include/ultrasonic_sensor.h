#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <driver/ledc.h>
#include <driver/gpio.h>
#include <driver/mcpwm_cap.h>
#include <esp_log.h>
#include <esp_private/esp_clk.h>

#include "ase_typedefs.h"
#include "ase_config.h"

typedef struct
{
    int16_t angle;
    float distance;
} ultrasonic_measurement_t;

void ultrasonic_sensor_task(void* pvParameters);

#endif