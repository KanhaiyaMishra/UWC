#ifndef __PRBS_H
#define __PRBS_H

#include <inttypes.h>

extern uint8_t taps[7][2];

typedef enum{
PRBS7, PRBS9, PRBS11, PRBS15, PRBS20, PRBS23, PRBS31}order_t;

void pattern_LFSR_bit(order_t prbs_order, uint32_t *bitsequence, uint32_t n_bits);
void pattern_LFSR_byte(order_t prbs_order, uint8_t *bitsequence, uint32_t n_bits);
void pattern_LFSR_reset();

#endif
