#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <freertos/FreeRTOS.h>

#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>
#include <esp_log.h>

#include <math.h>

#include "ase_typedefs.h"

#define MOTOR_LEFT_EN_GPIO GPIO_NUM_14
#define MOTOR_LEFT_IN1_GPIO GPIO_NUM_27
#define MOTOR_LEFT_IN2_GPIO GPIO_NUM_26

#define MOTOR_RIGHT_EN_GPIO GPIO_NUM_12
#define MOTOR_RIGHT_IN1_GPIO GPIO_NUM_32
#define MOTOR_RIGHT_IN2_GPIO GPIO_NUM_33

#define MCPWM_RESOLUTION_HZ 10000000 // 10 MHz -> 100ns time resolution
#define MCPWM_PERIOD_TICKS 10000     // 10 000 ticks * time resolution 100ns -> 1 kHz

void motor_control_task(void *pvParameters);

#endif