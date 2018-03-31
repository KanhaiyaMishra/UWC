/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ook.h
 * Author: mishr
 *
 * Created on 23 October, 2017, 11:55 PM
 */

#ifndef OOK_H
#define OOK_H
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ADC_BUFFER_SIZE
#define ADC_BUFFER_SIZE 16384
#endif

#define OOK 2
#define OSF 8
#define OOK_AMP 0.9
#if (OOK==1)
#define N_SAMP_SYM OSF
#else
#define N_SAMP_SYM (2*OSF)
#endif
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#define PN_SEQ_TYPE PRBS7
#define PN_SEQ_LEN 127

uint32_t ook_init();
void ook_mod(float *ook_tx, uint8_t *bin_tx);
uint32_t ook_demod(uint8_t *bin_rx, float *ook_rx, uint32_t size, uint32_t *bits_received);

#ifdef __cplusplus
}
#endif

#endif /* OOK_H */

