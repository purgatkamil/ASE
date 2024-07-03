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

void motor_control_task(void *pvParameters);

#endif