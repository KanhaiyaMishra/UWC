/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "ook.h"
#include <math.h>
#include "prbs.h"
#include <stdio.h>

uint8_t pn_seq_buff[PN_SEQ_LEN];
uint32_t n_sym, corr_max;

uint32_t ook_init(){
    int i;
    pattern_LFSR_byte(PN_SEQ_TYPE, pn_seq_buff, PN_SEQ_LEN);
    n_sym = (ADC_BUFFER_SIZE/OSF - PN_SEQ_LEN - 1)/OOK;
    for(i=0; i<PN_SEQ_LEN; i++){
        corr_max += *(pn_seq_buff + i);
    }
    return n_sym;
}

void ook_mod(float *ook_tx, uint8_t *bin_tx){

	uint32_t i, j;

        for( i=0; i<PN_SEQ_LEN; i++){
            for( j=0; j<OSF; j++)
                *(ook_tx++) = OOK_AMP*(*(pn_seq_buff + i));
        }
        ook_tx += OSF;
    	for( i=0; i<n_sym; i++ )
	{
            for( j=0; j<OSF; j++)
                *(ook_tx + j) = OOK_AMP*(*bin_tx);
            bin_tx++;
            ook_tx += N_SAMP_SYM;
        }
}

uint32_t ook_demod(uint8_t *bin_rx, float *ook_rx, uint32_t size, uint32_t *bits_received){

    int i, delay = 0, pn_corr, demod_sym, delay_max = 0;
    static uint32_t sync_done=0, sym_count=0;
    float rx_th = 0.5;

        if (!sync_done){
            sym_count = 0;
            delay_max = size - (PN_SEQ_LEN + 1)*OSF;
            if (delay_max >= 0){
                while( delay <= delay_max ){
                    pn_corr = 0;
                    for(i=0; i<PN_SEQ_LEN; i++)
                        pn_corr += ( (*(pn_seq_buff+i)) > rx_th) * ( (*(ook_rx + OSF*i)) >rx_th);
                    if(pn_corr > 0.9*corr_max){
			sync_done = 1;
                        break;
		    }
                    ook_rx++;
                    delay++;
                }
                if (sync_done){
                    size = delay_max - delay;
                    ook_rx += (PN_SEQ_LEN + 1)*OSF;
                    printf("Demodulation not completed, Sync completed, delay = %d samples\n", delay);
                } else{
                    *bits_received = 0;
                    size = (PN_SEQ_LEN + 1)*OSF;
                    printf("Demodulation completed, Sync not completed, Remaining %d Samples\n", size);
                }
            } else{
                *bits_received = 0;
                printf("Demodulation completed, Sync not completed, Remaining %d Samples\n", size);
            }
        }
        if (sync_done){
            if (size + sym_count*N_SAMP_SYM <= n_sym*N_SAMP_SYM){
                demod_sym = size/N_SAMP_SYM;
            } else{
                demod_sym = n_sym - sym_count;
                sync_done = 0;
            }
            for(i=0; i<demod_sym; i++ ){
                *bin_rx = *ook_rx > rx_th;
                ook_rx += N_SAMP_SYM;
                bin_rx++;
            }
            *bits_received = demod_sym;
            sym_count += demod_sym;
            size -= demod_sym*N_SAMP_SYM;
            printf("Demodulation Completed, Symbol Count = %d, Received %d bits, Remaining %d Samples\n", sym_count, *bits_received, size);
        }
    return size;
}
