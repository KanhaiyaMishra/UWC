#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include "redpitaya/rp.h"

int main(int argc, char **argv){

	clock_t start = 0, end=0;
	uint32_t delay=0, period = 10000;

	if(rp_Init() != RP_OK)
		printf("Initialization Failed\n");

        float *buff = (float *)malloc(ADC_BUFFER_SIZE * sizeof(float));
	fprintf(stdout,"TX: Entered\n");

        for(int i = 0; i < ADC_BUFFER_SIZE; i++)
o
			buff[i] = (0.8)*sin((2 * M_PI) / (ADC_BUFFER_SIZE) * i);

	fprintf(stdout,"TX: Waveform prepared \n");
        rp_GenWaveform(RP_CH_2, RP_WAVEFORM_ARBITRARY);
        rp_GenFreq(RP_CH_2, 1e6/period);
        rp_GenMode(RP_CH_2, RP_GEN_MODE_BURST);
        rp_GenBurstCount(RP_CH_2, 1);
        rp_GenBurstRepetitions(RP_CH_2, 1);
        rp_GenBurstPeriod(RP_CH_2, period);

	for(int i=0; i<1000; i++){
		rp_GenArbWaveform(RP_CH_2, buff, ADC_BUFFER_SIZE/2);
		printf("time taken = %lf, delay = %d\n",(double)end/CLOCKS_PER_SEC, delay);
		end = clock()-start;
		delay = period - 1e6*end/CLOCKS_PER_SEC - 50;
		usleep(delay);
		rp_GenOutEnable(RP_CH_2);
		start = clock();
	}

	rp_GenOutDisable(RP_CH_2);
	fprintf(stdout,"TX: output enabled \n");

    /* Releasing resources */
    free(buff);
	rp_Release();
        return 0;;
}

