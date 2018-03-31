/* Red Pitaya C API example Acquiring a signal from a buffer  
 * This application acquires a signal on a specific channel */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include "redpitaya/rp.h"

int main(int argc, char** argv){

	if(rp_Init()!=RP_OK)
		fprintf(stderr,"Initialization failed");

	bool buff_filled=0;
	uint32_t i, size, curr_pos, prev_pos=0, n_buff =100;
	float *rx_sig_buff = (float *)malloc(n_buff*ADC_BUFFER_SIZE*sizeof(float));
	float *buff_end = rx_sig_buff + n_buff*ADC_BUFFER_SIZE-1;
	float *buff_start = rx_sig_buff;
	float *buff_ptr = buff_start;

	FILE *fp;
	fprintf(stdout, "RX: Entered\n");
	fp = fopen("./data.txt","w+");

	sleep(3);
    rp_AcqReset();
    rp_AcqSetDecimation(RP_DEC_64);
	rp_AcqSetArmKeep(true);
        rp_AcqSetTriggerDelay(0);
        rp_AcqSetTriggerSrc(RP_TRIG_SRC_NOW);
        rp_AcqStart();

	while( !buff_filled ){
		rp_AcqGetWritePointer(&curr_pos);
		size = (curr_pos - prev_pos) % ADC_BUFFER_SIZE;
		if ( buff_ptr + size > buff_end){
			size = buff_end - buff_ptr + 1;
			buff_filled = 1;
		}
	        rp_AcqGetDataV(RP_CH_1, prev_pos, &size, buff_ptr);
		fprintf(stdout,"RX: read out samples = %d, current pos = %d, prev_pos = %d \n", size, curr_pos, prev_pos);
		prev_pos = curr_pos;
		buff_ptr = (buff_ptr + size);
	}

        for(i = 0; i <n_buff*ADC_BUFFER_SIZE; i++){
                fprintf(fp," %f \n", *(buff_start+i));
        }

        rp_AcqStop();
	fprintf(stdout,"RX: Acquisition Comeplete\n");
	free(rx_sig_buff);
	rp_Release();
        return 0;
}
