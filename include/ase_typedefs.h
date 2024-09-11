#ifndef ASE_TYPEDEFS_H
#define ASE_TYPEDEFS_H

// For types availability
#include "freertos/FreeRTOS.h"

typedef enum
{
    MISSION_STATE_UNKNOWN = 0,
    MISSION_STATE_IDLE,
    MISSION_STATE_WANDER,
    MISSION_STATE_CELEBRATE,
    MISSION_STATE_STOP
} mission_state_t;

typedef enum
{
    WANDER_STATE_ACTIVE = 0,
    WANDER_STATE_INACTIVE
} wander_state_command_t;

#endif