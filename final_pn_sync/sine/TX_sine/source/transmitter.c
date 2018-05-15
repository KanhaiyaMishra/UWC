#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include "redpitaya/rp.h"
#define N_FRAMES 5000
#define MAX_COUNT (1<<14)

uint64_t GetTimeStamp(){
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

int main(int argc, char **argv){

	uint64_t start = 0, end=0;
    float freq = 125e6/(64*ADC_BUFFER_SIZE);
    uint32_t frm_num, i, pos, period = round(1e6/freq);
	static volatile int32_t *dac_add = NULL;
    float *buff = (float *)malloc(ADC_BUFFER_SIZE * sizeof(float));
    float *buff_ptr = buff, amp;

    if(rp_Init() != RP_OK)
        printf("Initialization Failed\n");

    dac_add = (volatile int32_t*)rp_GenGetAdd(RP_CH_2);
    fprintf(stdout,"TX: Entered\n");
    rp_GenWaveform(RP_CH_2, RP_WAVEFORM_ARBITRARY);
    rp_GenFreq(RP_CH_2, freq);
    rp_GenMode(RP_CH_2, RP_GEN_MODE_BURST);
    rp_GenBurstCount(RP_CH_2, 1);
    rp_GenBurstRepetitions(RP_CH_2, 1);
    rp_GenBurstPeriod(RP_CH_2, period);

    for(i = 0; i < ADC_BUFFER_SIZE; i++){
        buff[i] = sin((2 * M_PI) / (ADC_BUFFER_SIZE) * i);
    }

    start = GetTimeStamp();
	for(frm_num=0; frm_num<N_FRAMES; frm_num++){
        amp = ((float)(frm_num%20)+5)/25;
        rp_GenGetReadPointer(&pos, RP_CH_2);
        for(i=0; i<ADC_BUFFER_SIZE;){
            rp_GenGetReadPointer(&pos, RP_CH_2);
            pos = ((pos==0)?(ADC_BUFFER_SIZE):pos);
            for(;i<pos;i++)
                dac_add[i] = ( (int32_t)(amp*buff_ptr[i]*MAX_COUNT/2 + 0.5*(2*(buff_ptr[i]>0)-1)) & (MAX_COUNT-1));
        }
        rp_GenOutEnable(RP_CH_2);
        printf("TX: Transmitting Frame Num = %d, %f\n",frm_num, amp);
    }
    usleep(period);
    end = GetTimeStamp()-start;
    fprintf(stdout,"TX: Transmitted %d Frames in %lf ms\n", N_FRAMES, (double)end/1000);

    rp_GenOutDisable(RP_CH_2);
    /* Releasing resources */
    free(buff);
    rp_Release();
    fprintf(stdout,"TX: Trasmission Completed, Exiting.\n");
    return 0;;
}

