#include <stdio.h>
#include "prbs.h"

uint8_t taps[7][2] = { {6, 5}, {8, 4}, {10, 8}, {14, 13}, {19, 2}, {22, 17}, {30, 31} };
static uint32_t start = 0x02;
long long seed = 0x1;

void pattern_LFSR_bit(order_t prbs_order, uint32_t *bitsequence, uint32_t nBits)
{
    uint32_t i,newbit, a = start;
    uint32_t *addr;
    for( i = 0; i<nBits; i++) {
        newbit = (((a >> taps[prbs_order][0]) ^ (a >> taps[prbs_order][1])) & 1);
        a = ((a << 1) | newbit);
        addr = bitsequence + (i/32) ;
        *addr &= ~(1<<(i%32));
        *addr |= (a&0x1)<<(i%32);
    }
    start = a;
}

void pattern_LFSR_byte(order_t prbs_order, uint8_t *bitsequence, uint32_t nBits)
{
    uint32_t i,newbit, a = start;
    for( i = 0; i<nBits; i++) {
        newbit = (((a >> taps[prbs_order][0]) ^ (a >> taps[prbs_order][1])) & 1);
        a = ((a << 1) | newbit);
        *bitsequence = a&0x1;
        bitsequence++;
    }
    start = a;
}
