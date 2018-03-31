/* Red Pitaya C API example Acquiring a signal from a buffer  
 * This application acquires a signal on a specific channel */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include "redpitaya/rp.h"
#include "time.h"

int main(int argc, char** argv){

	if(rp_Init()!=RP_OK)
		fprintf(stderr,"Initialization failed");

	uint32_t size, curr_pos, prev_pos=0, count=0;
	float *rx_sig_buff = (float *)malloc(ADC_BUFFER_SIZE*sizeof(float));
	float *buff_ptr = rx_sig_buff;
    float *tx_buff = (float *)malloc(ADC_BUFFER_SIZE*sizeof(float));

    for (int i=0; i<ADC_BUFFER_SIZE; i++)
        tx_buff[i] = 0.9;

    rp_GenWaveform(RP_CH_2, RP_WAVEFORM_ARBITRARY);
    rp_GenAmp(RP_CH_2, 1.0);
    rp_GenFreq(RP_CH_2, 10000);
    rp_GenArbWaveform(RP_CH_2, tx_buff, ADC_BUFFER_SIZE);

	sleep(3);
    rp_AcqReset();
    rp_AcqSetDecimation(RP_DEC_64);
	rp_AcqSetArmKeep(true);
    rp_AcqSetTriggerDelay(0);
    rp_AcqSetTriggerSrc(RP_TRIG_SRC_NOW);
    rp_AcqStart();

	while( count<10000 ){
		rp_AcqGetWritePointer(&curr_pos);
		size = (curr_pos - prev_pos) % ADC_BUFFER_SIZE;
		fprintf(stdout,"RX: read out samples = %d, current pos = %d, prev_pos = %d \n", size, curr_pos, prev_pos);
        for(int i=0; i<size; i+=3){
            if( buff_ptr[i] > 0.63 )
                rp_GenOutEnable(RP_CH_2);
            else
                rp_GenOutDisable(RP_CH_2);
        }
		prev_pos = curr_pos;
        count++;
	}

    rp_AcqStop();
	fprintf(stdout,"RX: Acquisition Comeplete\n");
	free(rx_sig_buff);
	rp_Release();
        return 0;
}
