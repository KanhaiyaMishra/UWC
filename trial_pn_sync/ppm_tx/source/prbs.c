/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <stdio.h>
#include "prbs.h"

uint8_t taps[8][2] = { {3,2}, {6, 5}, {8, 4}, {10, 8}, {14, 13}, {19, 16}, {22, 17}, {30, 27} };
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
    uint8_t tap1 = taps[prbs_order][0], tap2 = taps[prbs_order][1];
    for( i = 0; i<nBits; i++) {
			newbit = (((a >> tap1) ^ (a >> tap2)) & 1);
      a = ((a << 1) | newbit);
			*bitsequence = a&0x1;
			bitsequence++;
    }
		start = a;
}

void pattern_LCR(uint32_t *bitsequence, uint32_t nBits)
{
	int i;
    for( i = 0; i<=nBits/32; i++) {
			*(bitsequence+i) = (seed*1103515245LL + 12345LL);
    }
		seed = *bitsequence;
}
