/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   prbs.h
 * Author: mishr
 *
 * Created on 13 September, 2017, 1:23 AM
 */

#ifndef PRBS_H
#define PRBS_H
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{ 
PRBS4, PRBS7, PRBS9, PRBS11, PRBS15, PRBS20, PRBS23, PRBS31}order_t;

void pattern_LFSR_bit(order_t prbs_order, uint32_t *bitsequence, uint32_t n_bits);
void pattern_LFSR_byte(order_t prbs_order, uint8_t *bitsequence, uint32_t n_bits);
void pattern_LCR(uint32_t *bitsequence, uint32_t nBits);

#ifdef __cplusplus
}
#endif

#endif /* PRBS_H */

