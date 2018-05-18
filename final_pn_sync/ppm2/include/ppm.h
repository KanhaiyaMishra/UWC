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
#define N_FRAMES 1000
#define RX_BUFF_SIZE (64*ADC_BUFFER_SIZE)
#define NANO 1000000000LL
#define FRM_DUR 8.5
#define TRACE_PRINT TRUE
#define DC_ERROR 0.015
#define MAX_COUNT (1<<14)
#define AMP_ADJ 1
#define PPM 4
#define OSF 8
#define PPM_HIGH (0.9/AMP_ADJ)
#define PPM_LOW 0.0
#define N_SAMP_SYM (PPM*OSF)

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
#define PN_SEQ_LEN 32
#define FRM_NUM_BITS 16

void ppm_tx_init(float *tx_sig_buff);
void ppm_rx_init();
void ppm_mod(float *ppm_tx, uint8_t *bin_tx);
uint32_t ppm_demod(uint8_t *bin_rx, uint32_t demod_idx, uint32_t size, uint32_t *bits_received);

#ifdef __cplusplus
}
#endif

#endif /* PPM_H */
