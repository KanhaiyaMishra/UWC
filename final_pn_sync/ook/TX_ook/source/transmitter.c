#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include "redpitaya/rp.h"
#include "ook.h"
#include "prbs.h"

#define N_FRAMES 550
#define MAX_COUNT (1<<14)

uint8_t pn_seq_buff[PN_SEQ_LEN];
uint32_t n_sym, corr_max;

uint64_t GetTimeStamp(){
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

uint32_t ook_init(){
    int i;
    prbs_gen_byte(PN_SEQ_TYPE, pn_seq_buff, PN_SEQ_LEN);
    n_sym = (ADC_BUFFER_SIZE/OSF - PN_SEQ_LEN - 1)/OOK - 16;
    for(i=0; i<PN_SEQ_LEN; i++){
        corr_max += *(pn_seq_buff + i);
    }
    return n_sym;
}

void ook_mod(float *ook_tx, uint8_t *bin_tx){

	uint32_t i, j;
    static uint32_t frm_num = 0;
    for( i=0; i<PN_SEQ_LEN; i++){
        for( j=0; j<OSF; j++)
            *(ook_tx++) = OOK_AMP*(*(pn_seq_buff + i));
    }
    ook_tx += OSF;
    for( i=0; i<16; i++){
        for( j=0; j<OSF; j++)
            *(ook_tx++) = OOK_AMP*((frm_num>>i)&1);
    #if (OOK==2)
        ook_tx += OSF;
    #endif
    }
  	for( i=0; i<n_sym; i++ )
    {
        for( j=0; j<OSF; j++)
            *(ook_tx++) = OOK_AMP*(*bin_tx);
        bin_tx++;
    #if (OOK==2)
        ook_tx += OSF;
    #endif
    }
    frm_num++;
}
/*
void ook_mod(int32_t *ook_tx, uint32_t frm_num, uint8_t *bin_tx){

	uint32_t i, j;
    int32_t dac_count = ( (int32_t)(OOK_AMP*MAX_COUNT/2 + 0.5*(2*(OOK_AMP>0)-1)) & (MAX_COUNT-1));

    for( i=0; i<PN_SEQ_LEN; i++){
        for( j=0; j<OSF; j++)
           *(ook_tx++) = dac_count*(*(pn_seq_buff + i));
    }
    ook_tx += OSF;
 	for( i=0; i<n_sym; i++ )
	{
        for( j=0; j<OSF; j++)
            *(ook_tx + j) = dac_count*(*bin_tx);
        bin_tx++;
        ook_tx += N_SAMP_SYM;
    }
}
*/
int main(int argc, char **argv){

	if(rp_Init() != RP_OK)
		printf("Initialization Failed");

	uint64_t start = 0, end=0;
	float freq = 125e6/(16384*64);
	uint32_t period = round(1e6/freq), frm_num, i, pos;
    float *tx_sig_ptr = (float *)malloc(ADC_BUFFER_SIZE * sizeof(float));
	uint32_t n_bits_total = ook_init();
    uint8_t *tx_bin_ptr = (uint8_t *)malloc(n_bits_total* sizeof(uint8_t));
    static volatile int32_t* dac_add = NULL;

	fprintf(stdout,"TX: Entered\n");
    dac_add = (volatile int32_t*)rp_GenGetAdd(RP_CH_2);
    memset(tx_sig_ptr, 0, ADC_BUFFER_SIZE*(sizeof(float)));
    rp_GenWaveform(RP_CH_2, RP_WAVEFORM_ARBITRARY);
	rp_GenAmp(RP_CH_2, OOK_AMP);
    rp_GenFreq(RP_CH_2, freq);
    rp_GenMode(RP_CH_2, RP_GEN_MODE_BURST);
    rp_GenOffset(RP_CH_2, 0.0);
    rp_GenBurstCount(RP_CH_2, 1);
    rp_GenBurstRepetitions(RP_CH_2, 1);
    rp_GenBurstPeriod(RP_CH_2, period);
//		rp_GenArbWaveform(RP_CH_2, tx_sig_ptr, ADC_BUFFER_SIZE);

    start = GetTimeStamp();
	for(frm_num=0; frm_num<N_FRAMES; frm_num++){
        rp_GenGetReadPointer(&pos, RP_CH_2);
		prbs_gen_byte(PRBS11, tx_bin_ptr, n_bits_total);
		ook_mod(tx_sig_ptr, tx_bin_ptr);
        for(i=0; i<ADC_BUFFER_SIZE;){
            rp_GenGetReadPointer(&pos, RP_CH_2);
            pos = ((pos==0)?(ADC_BUFFER_SIZE):pos);
//            fprintf(stdout,"current read pointer = %d\n",pos);
            for(;i<pos;i++)
                dac_add[i] = ( (int32_t)(tx_sig_ptr[i]*MAX_COUNT/2 + 0.5*(2*(tx_sig_ptr[i]>0)-1)) & (MAX_COUNT-1));
        }
		rp_GenOutEnable(RP_CH_2);
		printf("TX: Transmitting Frame Num = %d\n",frm_num);
	}
	usleep(period);
   	end = GetTimeStamp()-start;
	fprintf(stdout,"TX: Transmitted %d Frames in %lf ms\n", N_FRAMES, (double)end/1000);
	rp_GenOutDisable(RP_CH_2);

//  Releasing resources
    free(tx_sig_ptr);
	free(tx_bin_ptr);
	rp_Release();
	fprintf(stdout,"TX: Transmission Completed, Exiting TX.\n");
    return 0;
}

