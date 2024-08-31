#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <freertos/FreeRTOS.h>

#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>
#include <esp_log.h>

#include <math.h>

#include "ase_config.h"
#include "ase_typedefs.h"
#include "helpers.h"

void mc_enable_pwm(bool enable);

// Left and right duty are in range [-1; 1]
// The sign determines direction:
//  positive -> forward
//  negative -> backward
void mc_set_duty(double left, double right);

void mc_motor_control_task(void *pvParameters);

#endif