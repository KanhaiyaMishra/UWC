#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include "redpitaya/rp.h"
#include "ook.h"
#include "prbs.h"

int main(int argc, char **argv){

	clock_t start = 0, end=0;

	if(rp_Init() != RP_OK)
		printf("Initialization Failed");

	float freq = 125e6/(16384*64);
	uint32_t period = round(1e6/freq), i;
    float *tx_sig_ptr = (float *)malloc(ADC_BUFFER_SIZE * sizeof(float));
	uint32_t n_bits_total = ook_init();
    uint8_t *tx_bin_ptr = (uint8_t *)malloc(n_bits_total* sizeof(uint8_t));

	fprintf(stdout,"TX: Entered\n");
    memset(tx_sig_ptr, 0, ADC_BUFFER_SIZE*sizeof(float));
    rp_GenWaveform(RP_CH_1, RP_WAVEFORM_ARBITRARY);
	rp_GenAmp(RP_CH_1, OOK_AMP);
    rp_GenFreq(RP_CH_1, freq);
    rp_GenMode(RP_CH_1, RP_GEN_MODE_BURST);
    rp_GenBurstCount(RP_CH_1, 1);
    rp_GenBurstRepetitions(RP_CH_1, 1);
    rp_GenBurstPeriod(RP_CH_1, period);

	for(i=0; i<1000; i++){
		start = clock();
		pattern_LFSR_byte(PRBS11, tx_bin_ptr, n_bits_total);
		ook_mod(tx_sig_ptr, tx_bin_ptr);
		rp_GenArbWaveform(RP_CH_1, tx_sig_ptr, ADC_BUFFER_SIZE);
		end = clock()-start;
        usleep(period - end);
		printf("time taken = %lf\n",(double)end/CLOCKS_PER_SEC);
		rp_GenOutEnable(RP_CH_1);
//		usleep(130000);
	}

	rp_GenOutDisable(RP_CH_1);

        /* Releasing resources */
        free(tx_sig_ptr);
	free(tx_bin_ptr);
	rp_Release();
	fprintf(stdout,"TX: Transmission Completed \n");
        return 0;;
}

