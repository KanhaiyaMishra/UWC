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

#define NANO 1000000000LL
// # frames to transmitted
#define N_FRAMES 1020
// max value to be written to DAC
#define MAX_COUNT (1<<14)
// buffer to hold synchornizations sequence
uint8_t pn_seq_buff[PN_SEQ_LEN];
// symbols per frame
uint32_t n_sym = (ADC_BUFFER_SIZE/OSF - PN_SEQ_LEN)/PPM, sync_len  = PN_SEQ_LEN*OSF;
// timediff in miliseconds
static double timediff_ms(struct timespec *begin, struct timespec *end){
    return (double)( (end->tv_sec - begin->tv_sec)*NANO + (end->tv_nsec - begin->tv_nsec) )/1000000;
}

// initialize PPM sync sequence
void ppm_init(float *ppm_tx){
    int i,j;
    prbs_gen_byte(PN_SEQ_TYPE, pn_seq_buff, PN_SEQ_LEN);
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
    for( i=0; i<n_sym; i++ ){
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
    // Calculate frame duration in micro seconds
    uint32_t period = round(1e6/freq);
    // Temporary variables
    uint32_t frm_num, i, pos;
    // Initialize the modulator
    uint32_t n_bits_total = n_sym*N_BITS;
    // Allocate memory for TX signal and TX binary data
    float *tx_sig_buff = (float *)malloc(ADC_BUFFER_SIZE*sizeof(float));
    uint8_t *tx_bin_buff = (uint8_t *)malloc(n_bits_total*sizeof(uint8_t));

    struct timespec begin, end;
    // Retreive the DAC buffer address
    static volatile int32_t* dac_add;
    dac_add = rp_GenGetAdd(RP_CH_2);

    fprintf(stdout,"TX: Entered, DAC Address=%p\n", dac_add);

    //write the sync_sequence into the signal buffer
    ppm_init(tx_sig_buff);

    // set DAC output waveform parameters
    rp_GenWaveform(RP_CH_2, RP_WAVEFORM_ARBITRARY);
    rp_GenOffset(RP_CH_2, 0);
    rp_GenAmp(RP_CH_2, 1.0);
    rp_GenFreq(RP_CH_2, freq);
    rp_GenMode(RP_CH_2, RP_GEN_MODE_BURST);
    rp_GenBurstCount(RP_CH_2, 1);
    rp_GenBurstRepetitions(RP_CH_2, 1);
    rp_GenBurstPeriod(RP_CH_2, period);

    // Get start time
    clock_gettime(CLOCK_MONOTONIC, &begin);
    // start continuous transmission with single burst waveforms
	for(frm_num=1; frm_num<=N_FRAMES; frm_num++){
        // read the buffer pointer once for updating from zero
        rp_GenGetReadPointer(&pos, RP_CH_2);
        // insert frame number information
        for(i=0; i<FRM_NUM_BITS; i++)
            tx_bin_buff[i] = ((frm_num>>i)&1);
        // Generate the prbs data
        prbs_gen_byte(PRBS11, tx_bin_buff+FRM_NUM_BITS, n_bits_total-FRM_NUM_BITS);
        // PPM modulation using prbs data
        ppm_mod(tx_sig_buff+sync_len, tx_bin_buff);
        // Write the signal into the DAC buffer following the read pointer
        for(i=0; i<ADC_BUFFER_SIZE;){
            rp_GenGetReadPointer(&pos, RP_CH_2);
            pos = ((pos==0)?(ADC_BUFFER_SIZE):pos);
            for(;i<pos;i++)
                dac_add[i] =  ((int)(tx_sig_buff[i]*MAX_COUNT/2.0f + 0.5) & 0x3FFF);
        }
        // Transmit the current frame and publish info
        rp_GenOutEnable(RP_CH_2);
        fprintf(stdout,"TX: Transmitting Frame Num = %d\n",frm_num);
    }
    // wait till last frame is transmitted
    usleep(period);
    // get the total transmission time and publish info
    clock_gettime(CLOCK_MONOTONIC, &end);
    fprintf(stdout,"TX: Transmitted %d Frames in %lf sec\n", N_FRAMES, timediff_ms(&begin, &end)/1000);

    // Disable the DAC output
    rp_GenOutDisable(RP_CH_2);

    // Release Resources
    free(tx_sig_buff);
    free(tx_bin_buff);
    rp_Release();

    fprintf(stdout,"TX: Transmission complete, Exiting.\n");
    return 0;
}
