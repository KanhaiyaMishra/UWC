#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include "redpitaya/rp.h"
#include "ppm.h"
#include "prbs.h"

int main(int argc, char **argv){

	clock_t start = 0, end=0;

	if(rp_Init() != RP_OK)
		printf("Initialization Failed");

	float freq = 125e6/(16384*64);
	uint32_t period = round(1e6/freq);
        float *tx_sig_ptr = (float *)malloc(ADC_BUFFER_SIZE * sizeof(float));
	uint32_t n_bits_total = ppm_init();
        uint8_t *tx_bin_ptr = (uint8_t *)malloc(n_bits_total* sizeof(uint8_t));

	fprintf(stdout,"TX: Entered\n");

    rp_GenWaveform(RP_CH_2, RP_WAVEFORM_ARBITRARY);
	rp_GenAmp(RP_CH_2, PPM_AMP);
        rp_GenFreq(RP_CH_2, freq);
        rp_GenMode(RP_CH_2, RP_GEN_MODE_BURST);
        rp_GenBurstCount(RP_CH_2, 1);
        rp_GenBurstRepetitions(RP_CH_2, 1);
        rp_GenBurstPeriod(RP_CH_2, period);

	for(int i=0; i<100; i++){
		start = clock();
        memset(tx_sig_ptr, 0, ADC_BUFFER_SIZE*sizeof(float));
		pattern_LFSR_byte(PRBS11, tx_bin_ptr, n_bits_total);
		ppm_mod(tx_sig_ptr, tx_bin_ptr);
		rp_GenArbWaveform(RP_CH_2, tx_sig_ptr, ADC_BUFFER_SIZE);
		printf("time taken = %lf\n",(double)end/CLOCKS_PER_SEC);
		end = clock()-start;
		rp_GenOutEnable(RP_CH_2);
		usleep(period-end);
	}

	rp_GenOutDisable(RP_CH_2);

        /* Releasing resources */
        free(tx_sig_ptr);
	free(tx_bin_ptr);
	rp_Release();

	fprintf(stdout,"TX: Transmission Completed \n");
        return 0;;
}

