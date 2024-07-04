#ifndef HELPERS_H
#define HELPERS_H

#include "ase_config.h"
#include "ase_typedefs.h"
#include "freertos/FreeRTOS.h"

// Bitwise operations macros to easy
// set flags in motor control structure
#define SET_BIT(var, mask)    ((var) |= (mask))
#define CLEAR_BIT(var, mask)  ((var) &= ~(mask))
#define IS_BIT_SET(var, mask) (((var) & (mask)) != 0)

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)         \
    ((byte) & 0x80 ? '1' : '0'),     \
        ((byte) & 0x40 ? '1' : '0'), \
        ((byte) & 0x20 ? '1' : '0'), \
        ((byte) & 0x10 ? '1' : '0'), \
        ((byte) & 0x08 ? '1' : '0'), \
        ((byte) & 0x04 ? '1' : '0'), \
        ((byte) & 0x02 ? '1' : '0'), \
        ((byte) & 0x01 ? '1' : '0')

// Counting number of 1's in a 32 bit unsigned integer, credits to the following link
// https://web.archive.org/web/20151229003112/http://blogs.msdn.com/b/jeuge/archive/2005/06/08/hakmem-bit-count.aspx
static inline int count_n_of_ones(unsigned int u)
{
    unsigned int uCount;

    uCount = u - ((u >> 1) & 033333333333) - ((u >> 2) & 011111111111);
    return ((uCount + (uCount >> 3)) & 030707070707) % 63;
}

#define N_BITS_ONES_N_ZEROS(var, mask, nhigh, nlow)          \
    ((count_n_of_ones(var & (~((uint32_t)mask))) >= nhigh && \
      (count_n_of_ones((~var) & mask)) >= nlow))

#define GET_LSB(var) var & 0x01

static inline BaseType_t send_mot_spd(
    QueueHandle_t         q,
    motors_control_msg_t *mc,
    double                spdLeft,
    double                spdRight,
    TickType_t            q_wait_ticks)
{
    mc->speed_cmd.left  = spdLeft;
    mc->speed_cmd.right = spdRight;
    return xQueueSend(q, mc, q_wait_ticks);
}
#endif