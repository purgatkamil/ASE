#include "state_transitions.h"

// void switch_mission_state(mission_state_t *current_state, mission_state_t newstate, void *param)
// {
//     if (*current_state == newstate) return;

//     switch (*current_state)
//     {
//     case MISSION_STATE_IDLE:
//         ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Quitting mission mode - IDLE");
//         on_mission_state_quit_idle(param);
//         break;

//     case MISSION_STATE_FOLLOW_LINE:
//         ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Quitting mission mode - LF");
//         on_mission_state_quit_follow_line(param);
//         break;

//     case MISSION_STATE_AVOID_OBSTACLE:
//         ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Quitting mission mode - AVOIDANCE");
//         on_mission_state_quit_avoidance(param);
//         break;

//     case MISSION_STATE_STOP:
//         ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Quitting mission mode - STOP");
//         on_mission_state_quit_stop(param);
//     }

//     switch (newstate)
//     {
//     case MISSION_STATE_IDLE:
//         ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Entering mission mode - IDLE");
//         on_mission_state_enter_idle(param);
//         break;

//     case MISSION_STATE_FOLLOW_LINE:
//         ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Entering mission mode - LF");
//         on_mission_state_enter_follow_line(param);
//         break;

//     case MISSION_STATE_AVOID_OBSTACLE:
//         ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Entering mission mode - AVOIDANCE");
//         on_mission_state_enter_avoidance(param);
//         break;

//     case MISSION_STATE_STOP:
//         ESP_LOGI(MAIN_MISSION_STATE_LOG_TAG, "Entering mission mode - STOP");
//         on_mission_state_enter_stop(param);
//     }

//     *current_state = newstate;
// }


// void on_mission_state_enter_idle(void *param) {}
// void on_mission_state_quit_idle(void *param) {}

// void on_mission_state_enter_follow_line(void *param)
// {
//     state_trans_lf_ctx_t *ctx = (state_trans_lf_ctx_t *)param;
//     xTaskNotify(ctx->lf_task_h, LF_STATE_ACTIVE, eSetValueWithOverwrite);
// }

// void on_mission_state_quit_follow_line(void *param)
// {
//     state_trans_lf_ctx_t *ctx = (state_trans_lf_ctx_t *)param;
//     // ctx->mynumbe = 1;
//     // ESP_LOGE(MAIN_TASK_LOG_TAG, "param: %d, task_h: %d, ticks_whq: %d", param == NULL, ctx->lf_task_h == NULL, ctx->ticks_when_quitted == NULL);

//     xTaskNotify(ctx->lf_task_h, LF_STATE_INACTIVE, eSetValueWithOverwrite);
//     (ctx->ticks_when_quitted) = 15u;
//     // ESP_LOGE(MAIN_TASK_LOG_TAG, "param: %d, task_h: %d, ticks_whq: %d", param == NULL, ctx->lf_task_h == NULL, ctx->ticks_when_quitted == NULL);
// }

// void on_mission_state_enter_avoidance(void *param)
// {
//     state_trans_avoidance_ctx_t *ctx = (state_trans_avoidance_ctx_t *)param;
//     xTaskNotify(ctx->avoidance_task_h, AVOIDANCE_STATE_ACTIVE, eSetValueWithOverwrite);
// }

// void on_mission_state_quit_avoidance(void *param)
// {
//     state_trans_avoidance_ctx_t *ctx = (state_trans_avoidance_ctx_t *)param;
//     xTaskNotify(ctx->avoidance_task_h, AVOIDANCE_STATE_INACTIVE, eSetValueWithOverwrite);
// }

// void on_mission_state_enter_stop(void *param)
// {
//     state_trans_stop_ctx_t *ctx = (state_trans_stop_ctx_t *)param;
//     // ESP_LOGE(MAIN_TASK_LOG_TAG, "param: %d, task_h: %d, ticks_whq: %d", param == NULL, ctx->mc == NULL, ctx->mc_q_h == NULL);
//     send_mot_spd(ctx->mc_q_h, ctx->mc, 0, 0, pdMS_TO_TICKS(5000));
// }

// void on_mission_state_quit_stop(void *param) {}