/* Red Pitaya C API example Acquiring a signal from a buffer  
 * This application acquires a signal on a specific channel */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include "redpitaya/rp.h"
#include "ppm.h"

int main(int argc, char** argv){

	if(rp_Init()!=RP_OK)
		fprintf(stderr,"Initialization failed");

	bool buff_filled=0;
	uint32_t i, size, curr_pos, prev_pos=0, n_buff =5;
	uint32_t n_bits_total = ppm_init(), bits_recvd = 0, rem_size =0;
	uint8_t *rx_bin_start = (uint8_t *)malloc(n_buff*n_bits_total*sizeof(uint8_t));
	uint8_t *rx_bin_ptr = rx_bin_start;
	float *rx_sig_start = (float *)malloc(n_buff*ADC_BUFFER_SIZE*sizeof(float));
	float *rx_sig_end = rx_sig_start + n_buff*ADC_BUFFER_SIZE;
	float *rx_sig_ptr = rx_sig_start;

	FILE *fp;
	fprintf(stdout, "RX: Entered\n");
	fp = fopen("./data.txt","w+");
	FILE *fp1;
	fp1 = fopen("./bin.txt","w+");

	sleep(3);
        rp_AcqReset();
        rp_AcqSetDecimation(RP_DEC_1024);
	rp_AcqSetArmKeep(true);
        rp_AcqSetTriggerDelay(0);
        rp_AcqSetTriggerSrc(RP_TRIG_SRC_NOW);
        rp_AcqStart();

	while( !buff_filled ){
		rp_AcqGetWritePointer(&curr_pos);
		size = (curr_pos - prev_pos) % ADC_BUFFER_SIZE;
		if ( rx_sig_ptr + size > rx_sig_end){
			size = rx_sig_end - rx_sig_ptr + 1;
			buff_filled = 1;
		}
	        rp_AcqGetDataV(RP_CH_1, prev_pos, &size, rx_sig_ptr);
		fprintf(stdout,"RX: read out samples = %d, current pos = %d, prev_pos = %d, bits received = %d \n", size, curr_pos, prev_pos, bits_recvd);
		prev_pos = curr_pos;
		rx_sig_ptr = (rx_sig_ptr + size);
		rem_size = ppm_demod(rx_bin_ptr, rx_sig_ptr-rem_size-size, size+rem_size, &bits_recvd);
		rx_bin_ptr += bits_recvd;
	}

        for(i = 0; i <n_buff*ADC_BUFFER_SIZE; i++){
                fprintf(fp," %f \n", *(rx_sig_start+i));
        }
        for(i = 0; i <n_buff*n_bits_total; i++){
                fprintf(fp1," %d \n", *(rx_bin_start+i));
        }

        rp_AcqStop();
	free(rx_sig_start);
	free(rx_bin_start);
	rp_Release();
	fprintf(stdout,"RX: Acquisition Comeplete\n");
        return 0;
}
