#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include "redpitaya/rp.h"
#include "ppm.h"
#include "prbs.h"
#define N_FRAMES 1100
#define MAX_COUNT (1<<14)

uint8_t pn_seq_buff[PN_SEQ_LEN];
uint32_t n_sym, corr_max=0;

uint64_t GetTimeStamp(){
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

uint32_t ppm_init(){
    int i;
    prbs_gen_byte(PN_SEQ_TYPE, pn_seq_buff, PN_SEQ_LEN);
    n_sym = (ADC_BUFFER_SIZE/OSF - PN_SEQ_LEN - 1)/PPM;
    for(i=0; i<PN_SEQ_LEN; i++)
        corr_max += *(pn_seq_buff + i);
    fprintf(stdout,"Symbols Per Frame = %d, Max Correlation = %d", n_sym, corr_max);
    return n_sym*N_BITS;
}

void ppm_mod(float *ppm_tx, uint8_t *bin_tx){

	uint32_t i=0, j, pos;
    static uint16_t frm_num = 0;

    // insert the PN sequence for synchronization
    for( i=0; i<PN_SEQ_LEN; i++){
        for( j=0; j<OSF; j++)
            *(ppm_tx++) = PPM_AMP*(*(pn_seq_buff + i));
    }
    // leave one symbol between sync sequence and data
    ppm_tx += OSF;

/*    // insert the frame number information
    for( i=0; i<(16/N_BITS); i++){
        pos = 0;
        for( j=0; j<N_BITS; j++)
	    	pos |= ((frm_num>>(i*N_BITS+j))&1)<<j;

        for( j=0; j<OSF; j++)
            *(ppm_tx + pos*OSF + j) = PPM_AMP;

        ppm_tx += N_SAMP_SYM;
    }
*/
    // write the ppm data into the buffer
  	for( i=0; i<n_sym; i++ )
	{
        pos = 0;
        for( j=0; j<N_BITS; j++)
    	    pos |= *(bin_tx+j)<<j;

        for( j=0; j<OSF; j++)
            *(ppm_tx + pos*OSF + j) = PPM_AMP;

        ppm_tx += N_SAMP_SYM;
        bin_tx += N_BITS;
    }
    frm_num++;
}

int main(int argc, char **argv){

    // Initialize the board
	if(rp_Init() != RP_OK)
		fprintf(stderr,"TX: Initialization Failed\n");

    // Calculate frame freq, Sample rate = 125/64 Msps, One Frame = 16384 Samples
	float freq = 125e6/(16384*64);
    // Calculate frame duration in micro seconds
	uint32_t period = round(1e6/freq);
    // Temporary variables
    uint32_t frm_num, i, pos;
	uint64_t start = 0, end=0;
    // Initialize the modulator
	uint32_t n_bits_total = ppm_init();
    // Allocate memory for TX signal and TX binary data
    float *tx_sig_ptr = (float *)malloc(ADC_BUFFER_SIZE*sizeof(float));
    uint8_t *tx_bin_ptr = (uint8_t *)malloc(n_bits_total*sizeof(uint8_t));

    // Retreive the DAC buffer address
    static volatile int32_t* dac_add;
    dac_add = (volatile int32_t*)rp_GenGetAdd(RP_CH_2);

	fprintf(stdout,"TX: Entered, DAC Address=%p\n", dac_add);

    // set DAC output waveform parameters
    rp_GenWaveform(RP_CH_2, RP_WAVEFORM_ARBITRARY);
    rp_GenOffset(RP_CH_2, 0.0);
	rp_GenAmp(RP_CH_2, PPM_AMP);
    rp_GenFreq(RP_CH_2, freq);
    rp_GenMode(RP_CH_2, RP_GEN_MODE_BURST);
    rp_GenBurstCount(RP_CH_2, 1);
    rp_GenBurstRepetitions(RP_CH_2, 1);
    rp_GenBurstPeriod(RP_CH_2, period);

//	rp_GenArbWaveform(RP_CH_2, tx_sig_ptr, ADC_BUFFER_SIZE);

    // Get cpu clock
    start = GetTimeStamp();
    // start continuous transmission with single burst waveforms
	for(frm_num=0; frm_num<N_FRAMES; frm_num++){
        // read the buffer pointer once for updating from zero
        rp_GenGetReadPointer(&pos, RP_CH_2);
        // Initialize the signal buffer
        memset(tx_sig_ptr, 0, ADC_BUFFER_SIZE*(sizeof(float)));
        // insert frame number information
        for(i=0; i<FRM_NUM_BITS; i++)
            tx_bin_ptr[i] = ((frm_num>>i)&1);
        // Generate the prbs data
		prbs_gen_byte(PRBS11, tx_bin_ptr+FRM_NUM_BITS, n_bits_total-FRM_NUM_BITS);
        // PPM modulation using prbs data
		ppm_mod(tx_sig_ptr, tx_bin_ptr);
        // Write the signal into the DAC buffer following the read pointer
        for(i=0; i<ADC_BUFFER_SIZE;){
            rp_GenGetReadPointer(&pos, RP_CH_2);
            pos = ((pos==0)?(ADC_BUFFER_SIZE):pos);
            for(;i<pos;i++)
                dac_add[i] = ( (int32_t)(tx_sig_ptr[i]*MAX_COUNT/2 + 0.5*(2*(tx_sig_ptr[i]>0)-1)) & (MAX_COUNT-1));
        }
        // Transmit the current frame and publish info
        rp_GenOutEnable(RP_CH_2);
        fprintf(stdout,"TX: Transmitting Frame Num = %d\n",frm_num);
    }
    // wait till last frame is transmitted
    usleep(period);
    // get the total transmission time and publish info
    end = GetTimeStamp()-start;
    fprintf(stdout,"TX: Transmitted %d Frames in %lf ms\n", N_FRAMES, (double)end/1000);

    // Disable the DAC output
	rp_GenOutDisable(RP_CH_2);

    // Release Resources
    free(tx_sig_ptr);
	free(tx_bin_ptr);
	rp_Release();

	fprintf(stdout,"TX: Transmission complete, Exiting.\n");
    return 0;
}
