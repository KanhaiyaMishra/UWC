/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "ppm.h"
#include <math.h>
#include "prbs.h"
#include <stdio.h>
uint8_t pn_seq_buff[PN_SEQ_LEN];
uint32_t n_sym, corr_max;

uint32_t ppm_init(){
    int i;
    pattern_LFSR_byte(PN_SEQ_TYPE, pn_seq_buff, PN_SEQ_LEN);
    n_sym = (ADC_BUFFER_SIZE/OSF - PN_SEQ_LEN - 1)/PPM;
    for(i=0; i<PN_SEQ_LEN; i++)
        corr_max += *(pn_seq_buff + i);

    return n_sym*N_BITS;
}

void ppm_mod(float *ppm_tx, uint8_t *bin_tx){

	uint32_t i, j, pos;
        for( i=0; i<PN_SEQ_LEN; i++){
            for( j=0; j<OSF; j++)
                *(ppm_tx++) = PPM_AMP* (*(pn_seq_buff + i));
        }

        ppm_tx += OSF;
    	for( i=0; i<n_sym; i++ )
	{
            pos = 0;
            //extract the binary data from input buffer
            for( j=0; j<N_BITS; j++)
		pos |= *(bin_tx+j)<<j;

            for( j=0; j<OSF; j++)
                *(ppm_tx + pos*OSF + j) = PPM_AMP;
            ppm_tx += N_SAMP_SYM;
            bin_tx += N_BITS;
        }
}

uint32_t ppm_demod(uint8_t *bin_rx, float *ppm_rx, uint32_t size, uint32_t *bits_received){

    int i, j, pos, delay = 0, pn_corr=0, demod_sym=0, delay_max = 0; 
    static uint32_t sync_done=0, sym_count=0;        
    float rx_th = 0.5;
    
        if (!sync_done){
            sym_count = 0;
            delay_max = size - (PN_SEQ_LEN + 1)*OSF;
            if (delay_max >= 0){
                while( delay <= delay_max ){
                    pn_corr = 0;
                    for(i=0; i<PN_SEQ_LEN; i++){
                        pn_corr += ( (*(pn_seq_buff + i)) > rx_th) * ( (*(ppm_rx + OSF*i)) >rx_th);
                    }
                    if(pn_corr == corr_max)
                        break;
                    ppm_rx++;
                    delay++;
                }
                if (pn_corr == corr_max){
                    sync_done = 1;
                    size = delay_max - delay;
                    ppm_rx += (PN_SEQ_LEN + 1)*OSF;
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
                    for( j=0; j< PPM; j++){
                        if( *(ppm_rx + j*OSF) > rx_th )
                            pos=j;
                    }

                    //write the binary data into output buffer
                    for( j=0; j<N_BITS; j++)
                        *(bin_rx+j) = (pos>>j)&0x1;

                    ppm_rx += N_SAMP_SYM;
                    bin_rx += N_BITS;
                }
            *bits_received = N_BITS*demod_sym;
            sym_count += demod_sym;
            size -= demod_sym*N_SAMP_SYM;              
            printf("Demodulation Completed, Symbol Count = %d, Received %d bits, Remaining %d Samples\n", sym_count, *bits_received, size);
        }
    return size;
}	
