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

#define ULTRASONIC_TRIG_GPIO GPIO_NUM_2
#define ULTRASONIC_ECHO_GPIO GPIO_NUM_17
#define SERVO_PWM_GPIO GPIO_NUM_25

#define SONAR_LEDC_TIMER LEDC_TIMER_0
#define SONAR_LEDC_MODE LEDC_LOW_SPEED_MODE
#define SONAR_LEDC_CHANNEL LEDC_CHANNEL_0
#define SONAR_LEDC_DUTY_RES LEDC_TIMER_20_BIT
#define SONAR_LEDC_DUTY (5 * 105)     // Set duty to 10 us (0.0001 % * 2^20 ~ 105 at 10 Hz)
#define SONAR_LEDC_FREQUENCY (5 * 10) // Frequency in Hertz

#define SERVO_LEDC_TIMER LEDC_TIMER_1
#define SERVO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL LEDC_CHANNEL_1
#define SERVO_LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define SERVO_LEDC_FREQUENCY 100 // Frequency in Hertz

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE -180         // Minimum angle
#define SERVO_MAX_DEGREE 180          // Maximum angle

// Time that is waited before taking measurement after commanding 
// new servo position (to make sure servo has enough time to move) 
#define DELAY_AFTER_SERVO_MOVEMENT_MS 50

// Maximum distance that is propagated further onto the system
// bigger values are still measured, but they are not passed
// to queue that is then processed in main task.
// This approach should minimise amount of glitched readings.
#define SONAR_MAX_DISTANCE_CM 150

// If not commented servo will be scanning, otherwise fixed to 0 degrees.
// #define ENABLE_SERVO_MOVEMENT

typedef struct
{
    int16_t angle;
    float distance;
} ultrasonic_measurement_t;

void ultrasonic_sensor_task(void* pvParameters);

#endif