#ifndef STATE_TRANSITIONS_H
#define STATE_TRANSITIONS_H

#include "freertos/FreeRTOS.h"

#include <esp_log.h>

#include "ase_config.h"
#include "ase_typedefs.h"
#include "helpers.h"

// void on_mission_state_enter_idle(void *param);
// void on_mission_state_quit_idle(void *param);

// void on_mission_state_enter_follow_line(void *param);
// void on_mission_state_quit_follow_line(void *param);

// void on_mission_state_enter_avoidance(void *param);
// void on_mission_state_quit_avoidance(void *param);

// void on_mission_state_enter_stop(void *param);
// void on_mission_state_quit_stop(void *param);

// void switch_mission_state(mission_state_t *current_state, mission_state_t newstate, void *param);

// typedef struct
// {
//     motors_control_msg_t *mc;
//     QueueHandle_t         mc_q_h;
// } state_trans_stop_ctx_t;

// typedef struct
// {
//     TaskHandle_t lf_task_h;
//     TickType_t ticks_when_quitted;
// } state_trans_lf_ctx_t;

// typedef struct
// {
//     TaskHandle_t avoidance_task_h;
// } state_trans_avoidance_ctx_t;

#endif