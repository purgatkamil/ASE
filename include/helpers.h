#ifndef HELPERS_H
#define HELPERS_H

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

static inline void sonar_motors_q_ok_or_abort(QueueHandle_t sonar_q, QueueHandle_t mot_ctr_q, const char* TAG)
{
    if (sonar_q == NULL || mot_ctr_q == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue (sonar_q_fail = %d, motors_control_q_fail = %d)",
                 sonar_q == NULL,
                 mot_ctr_q == NULL);
        abort();
    }
}

#define N_BITS_ONES_N_ZEROS(var, mask, nhigh, nlow)          \
    ((count_n_of_ones(var & (~((uint32_t)mask))) >= nhigh && \
      (count_n_of_ones((~var) & mask)) >= nlow))


#define GET_LSB(var) var & 0x01

// (count_n_of_ones(ir_c_history & (~((uint32_t)0x05))) >= 15 && (count_n_of_ones((~ir_c_history) & 0x05)) >= 4)

#endif