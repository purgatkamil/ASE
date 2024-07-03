#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include "freertos/FreeRTOS.h"

#include <driver/gpio.h>
#include <esp_log.h>

#include "helpers.h"
#include "ase_typedefs.h"
#include "ase_config.h"

typedef enum
{
    LINE_FOLLOWER_DIR_STRAIGHT = 0,
    LINE_FOLLOWER_DIR_LEFT,
    LINE_FOLLOWER_DIR_RIGHT
} line_follower_dir_t;

typedef struct
{
    QueueHandle_t mot_cmd_q_handle;
    motors_control_msg_t* mot_ctrl_msg;
    TaskHandle_t main_task_h;
} line_follower_task_context_t;

void line_follower_task(void* pvParameters);

#endif