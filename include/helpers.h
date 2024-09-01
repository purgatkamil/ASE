#ifndef HELPERS_H
#define HELPERS_H

#include "ase_config.h"
#include "ase_typedefs.h"
#include "freertos/FreeRTOS.h"

#include <sys/time.h>

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

int64_t get_sys_timestamp();

#endif