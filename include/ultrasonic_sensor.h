#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/mcpwm_cap.h>
#include <esp_log.h>
#include <esp_private/esp_clk.h>

#include "ase_config.h"
#include "ase_typedefs.h"
#include "helpers.h"
#include "task_notif_indexes.h"

typedef struct
{

} sonar_task_ctx_t;

typedef struct
{
    int16_t angle;
    float   distance;
} ultrasonic_measurement_t;

void sonar_set_servo(int16_t angle);

ultrasonic_measurement_t sonar_get_measurement(uint32_t wait_ms, bool *success);

void ultrasonic_sensor_task(void *pvParameters);

#endif