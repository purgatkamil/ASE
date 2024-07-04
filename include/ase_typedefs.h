#ifndef ASE_TYPEDEFS_H
#define ASE_TYPEDEFS_H

// For types availability
#include "freertos/FreeRTOS.h"

typedef enum
{
    MOTORS_CONTROL_FLAGS_ENABLE = (1 << 0),
} motors_control_mask;

typedef struct
{
    // Left and right speed are in range [-1; 1]
    double left;
    double right;
} motors_speed_t;

typedef struct
{
    motors_speed_t speed_cmd;
    uint8_t        cmd_flags;
} motors_control_msg_t;

typedef enum
{
    MISSION_STATE_IDLE = 0,
    MISSION_STATE_FOLLOW_LINE,
    MISSION_STATE_AVOID_OBSTACLE,
    MISSION_STATE_STOP
} mission_state_t;

typedef enum
{
    LF_STATE_ACTIVE = 0,
    LF_STATE_INACTIVE
} lf_state_command_t;

typedef enum
{
    AVOIDANCE_STATE_ACTIVE = 0,
    AVOIDANCE_STATE_INACTIVE
} avoidance_state_command_t;

#endif