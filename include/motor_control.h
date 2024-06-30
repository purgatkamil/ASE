#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <freertos/FreeRTOS.h>

#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>
#include <esp_log.h>

#define GPIO_MOTOR_LEFT GPIO_NUM_12
#define GPIO_MOTOR_RIGHT GPIO_NUM_14

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