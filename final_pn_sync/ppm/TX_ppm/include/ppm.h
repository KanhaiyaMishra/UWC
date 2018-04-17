/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ppm.h
 * Author: mishr
 *
 * Created on 13 September, 2017, 1:25 AM
 */

#ifndef PPM_H
#define PPM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#ifndef ADC_BUFFER_SIZE
#define ADC_BUFFER_SIZE 16384
#endif
#define PPM 4
#define OSF 8
#define PPM_AMP 0.9999
// Samples per symbol = PPM*OSF
#define N_SAMP_SYM 32
#define FRM_NUM_BITS 16

#if (PPM == 4)
#define N_BITS 2
#elif (PPM == 8)
#define N_BITS 3
#elif (PPM == 16)
#define N_BITS 4
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define PN_SEQ_TYPE PRBS7
#define PN_SEQ_LEN 31

uint32_t ppm_init();
void ppm_mod(float *ppm_tx, uint8_t *bin_tx);
uint32_t ppm_demod(uint8_t *bin_rx, float *ppm_rx, uint32_t size, uint32_t *bits_received);

#ifdef __cplusplus
}
#endif

#endif /* PPM_H */

