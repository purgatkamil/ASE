#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <freertos/FreeRTOS.h>

#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>
#include <esp_log.h>

#define MOTOR_LEFT_EN_GPIO GPIO_NUM_12
#define MOTOR_LEFT_IN1_GPIO GPIO_NUM_33
#define MOTOR_LEFT_IN2_GPIO GPIO_NUM_32

#define MOTOR_RIGHT_EN_GPIO GPIO_NUM_14
#define MOTOR_RIGHT_IN1_GPIO GPIO_NUM_27
#define MOTOR_RIGHT_IN2_GPIO GPIO_NUM_26

#define MCPWM_RESOLUTION_HZ 10000000 // 10 MHz -> 100ns time resolution
#define MCPWM_PERIOD_TICKS 10000 // 10 000 ticks * time resolution 100ns -> 1 kHz

/*
* Commenting out function declarations as they are not required to be called by the user
* Only motor control task function should be visible, as thats the point where interfacing occours.
*/

// void enable_start_mcpwm_tim(mcpwm_timer_handle_t tim_h);

// void disable_mcpwm_tim(mcpwm_timer_handle_t tim_h);

// void create_tim_oper(mcpwm_timer_handle_t *tim_h, mcpwm_oper_handle_t *oper);

// void conf_mcpwm_gen_cmp(mcpwm_oper_handle_t oper, mcpwm_cmpr_handle_t *cmp_h, int gpio_num);

void motor_control_task();

#endif