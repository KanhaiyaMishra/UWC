/* Red Pitaya C API example Acquiring a signal from a buffer  
 * This application acquires a signal on a specific channel */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include "redpitaya/rp.h"
#include "time.h"
#define ADC_MASK 0x3FFF
#define MAX_COUNT 0x1FFF

int main(int argc, char** argv){

	if(rp_Init()!=RP_OK)
		fprintf(stderr,"Initialization failed");

	int32_t buff_filled=0;
	uint32_t i, size, curr_pos, prev_pos=0, n_buff =1000;
	float *rx_sig_buff = (float *)malloc(n_buff*ADC_BUFFER_SIZE*sizeof(float));
//    uint32_t *uint_buff = (uint32_t *)malloc(n_buff*ADC_BUFFER_SIZE*sizeof(int32_t));
//    uint32_t *uint_buff_ptr = uint_buff, temp;
    float *buff_end = rx_sig_buff + n_buff*ADC_BUFFER_SIZE-1;
	float *buff_start = rx_sig_buff;
	float *buff_ptr = buff_start;
    static volatile uint32_t* adc_add = NULL;

	FILE *fp;
	fprintf(stdout, "RX: Entered\n");
	fp = fopen("./data.txt","w+");
    adc_add = (volatile uint32_t*)rp_AcqGetAdd(RP_CH_1);
    sleep(1);
    rp_AcqReset();
    rp_AcqSetDecimation(RP_DEC_64);
	rp_AcqSetArmKeep(true);
    rp_AcqSetTriggerDelay(0);
    rp_AcqSetTriggerSrc(RP_TRIG_SRC_NOW);
    rp_AcqStart();
    usleep(500);

	while( !buff_filled ){
		rp_AcqGetWritePointer(&curr_pos);
		size = (curr_pos - prev_pos) % ADC_BUFFER_SIZE;
		if ( buff_ptr + size > buff_end){
			size = buff_end - buff_ptr + 1;
			buff_filled = 1;
		}
        rp_AcqGetDataV(RP_CH_1, prev_pos, &size, buff_ptr);

/*        for(i = 0; i<size; i++){
            temp = adc_add[(i+prev_pos)%ADC_BUFFER_SIZE] & 0x3FFF;
            uint_buff_ptr[i] = temp;
            if(temp&(1<<13))
                buff_ptr[i] = -1.f*((temp^ADC_MASK)+1)/((double)MAX_COUNT);
            else
                buff_ptr[i] = temp/((double)MAX_COUNT);
        }
        uint_buff_ptr = (uint_buff_ptr + size);
*/
		prev_pos = curr_pos;
		buff_ptr = (buff_ptr + size);
	}

    for(i = 0; i <n_buff*ADC_BUFFER_SIZE; i++){
       fprintf(fp," %f\n", *(buff_start+i));
    }

    rp_AcqStop();
	fprintf(stdout,"RX: Acquisition Comeplete, ADC Address = %p\n", adc_add);
	free(rx_sig_buff);
	rp_Release();
    return 0;
}
