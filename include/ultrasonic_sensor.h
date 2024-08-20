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
#include "task_notif_indexes.h"
#include "helpers.h"

typedef struct
{
    TaskHandle_t  servo_angle_ready_notif_task_h;
    QueueHandle_t masurements_queue_h;
} sonar_task_ctx_t;

typedef struct
{
    int16_t angle;
    float   distance;
} ultrasonic_measurement_t;

void ultrasonic_sensor_task(void *pvParameters);

#endif