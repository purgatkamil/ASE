#ifndef ASE_TYPEDEFS_H
#define ASE_TYPEDEFS_H

// For types availability
#include "freertos/FreeRTOS.h"

typedef enum
{
    MISSION_STATE_IDLE = 0,
    MISSION_STATE_AVOID_OBSTACLE,
    MISSION_STATE_CELEBRATE,
    MISSION_STATE_STOP
} mission_state_t;

typedef enum
{
    AVOIDANCE_STATE_ACTIVE = 0,
    AVOIDANCE_STATE_INACTIVE
} avoidance_state_command_t;

#endif