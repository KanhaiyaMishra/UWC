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

// buffer to hold synchornizations sequence
uint8_t pn_seq_buff[PN_SEQ_LEN];
// symbols per frame
uint32_t n_sym = (ADC_BUFFER_SIZE/OSF - PN_SEQ_LEN)/PPM, sync_len  = PN_SEQ_LEN*OSF;
// timediff in miliseconds
static double timediff_ms(struct timespec *begin, struct timespec *end){
    return (double)( (end->tv_sec - begin->tv_sec)*NANO + (end->tv_nsec - begin->tv_nsec) )/1000000;
}

// initialize PPM sync sequence
void ppm_tx_init(float *ppm_tx){
    int i,j;
    pattern_LFSR_byte(PN_SEQ_TYPE, pn_seq_buff, PN_SEQ_LEN);
    // insert the PN sequence (NRZ_OOK modulation)
    for( i=0; i<PN_SEQ_LEN; i++){
        for( j=0; j<OSF; j++){
            if( pn_seq_buff[i] == 1 )
                *(ppm_tx++) = PPM_HIGH;
            else
                *(ppm_tx++) = PPM_LOW;
        }
    }
}

void ppm_mod(float *ppm_tx, uint8_t *bin_tx){

	uint32_t i=0, j=0, k=0, pos=0;

    // write the ppm data into the buffer
  	for( i=0; i<n_sym; i++ )
	{
        pos = 0;
        for( j=0; j<N_BITS; j++)
    	    pos |= *(bin_tx++)<<j;

        for( k=0; k<PPM; k++){
            if(k!=pos){
                for( j=0; j<OSF; j++)
                    *(ppm_tx++) = PPM_LOW;
            } else{
                for( j=0; j<OSF; j++)
                    *(ppm_tx++) = PPM_HIGH;
            }
        }
    }
}

int main(int argc, char **argv){

    // Initialize the board
	if(rp_Init() != RP_OK)
		fprintf(stderr,"TX: Initialization Failed\n");

    // Calculate frame freq, Sample rate = 125/64 Msps, One Frame = 16384 Samples
	float freq = 125e6/(16384*64);
    // bits per frame
	uint32_t n_bits_total = n_sym*N_BITS;
    // Allocate memory for TX signal and TX binary data
    float *tx_sig_buff = (float *)malloc(ADC_BUFFER_SIZE*sizeof(float));
    uint8_t *tx_bin_buff = (uint8_t *)malloc(n_bits_total*sizeof(uint8_t));

    struct timespec begin, end;
    // Retreive the DAC buffer address
    static volatile int32_t* dac_add;
    dac_add = rp_GenGetAdd(DAC_CHANNEL);

	fprintf(stdout,"TX: Entered, DAC Address=%p\n", dac_add);

    // set DAC output waveform parameters
    rp_GenWaveform(DAC_CHANNEL, RP_WAVEFORM_ARBITRARY);
    rp_GenOffset(DAC_CHANNEL, 0);
	rp_GenAmp(DAC_CHANNEL, 1.0);
    rp_GenFreq(DAC_CHANNEL, freq);
    rp_GenMode(DAC_CHANNEL, RP_GEN_MODE_CONTINUOUS);

    // insert frame number information
    tx_bin_buff[0] = 1;
    //write the sync_sequence into the signal buffer
    ppm_tx_init(tx_sig_buff);
    // Generate the prbs data
    pattern_LFSR_byte(PRBS11, tx_bin_buff+FRM_NUM_BITS, n_bits_total-FRM_NUM_BITS);
    // PPM modulation using prbs data
    ppm_mod(tx_sig_buff+sync_len, tx_bin_buff);
    for(int i=0;i<ADC_BUFFER_SIZE;i++)
        dac_add[i] =  ((int)(tx_sig_buff[i]*MAX_COUNT/2.0f + 0.5) & (MAX_COUNT-1));
    // Get start time
    clock_gettime(CLOCK_MONOTONIC, &begin);
    // Transmit the current frame and publish info
    rp_GenOutEnable(DAC_CHANNEL);
    // start continuous transmission with single burst waveforms
    while( timediff_ms(&begin, &end) <= (N_FRAMES+1)*FRM_DUR)
        clock_gettime(CLOCK_MONOTONIC, &end);
    fprintf(stdout,"TX: Transmitted %d Frames in %lf sec\n", N_FRAMES, timediff_ms(&begin, &end)/1000);

    // Disable the DAC output
	rp_GenOutDisable(DAC_CHANNEL);

    // Release Resources
    free(tx_sig_buff);
	free(tx_bin_buff);
	rp_Release();

	fprintf(stdout,"TX: Transmission complete, Exiting.\n");
    return 0;
}
