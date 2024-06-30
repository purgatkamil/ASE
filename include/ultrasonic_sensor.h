#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/ledc.h>
#include <driver/gpio.h>
#include <driver/mcpwm_cap.h>
#include <esp_log.h>
#include <esp_private/esp_clk.h>

#define ULTRASONIC_TRIG_GPIO GPIO_NUM_2
#define ULTRASONIC_ECHO_GPIO GPIO_NUM_17

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_20_BIT
#define LEDC_DUTY (5*105)     // Set duty to 10 us (0.0001 % * 2^20 ~ 105 at 10 Hz)
#define LEDC_FREQUENCY (5*10)   // Frequency in Hertz

void ultrasonic_sensor_task();

#endif