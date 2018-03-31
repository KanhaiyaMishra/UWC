#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include "redpitaya/rp.h"
#include "ook.h"
#include <math.h>
#include "prbs.h"
#include <time.h>

#define N_FRAMES 500

uint8_t pn_seq_buff[PN_SEQ_LEN];
uint32_t n_sym, corr_max;
uint32_t indices[64];

uint32_t ook_init(){
    int i;
    uint32_t *temp = indices;
    prbs_gen_byte(PN_SEQ_TYPE, pn_seq_buff, PN_SEQ_LEN);
    n_sym = (ADC_BUFFER_SIZE/OSF - PN_SEQ_LEN - 1)/OOK;
    for(i=0; i<PN_SEQ_LEN; i++){
        if( *(pn_seq_buff+i) ){
            corr_max += *(pn_seq_buff + i);
            *temp = i;
            temp++;
        }
    }
    printf("RX: Max Correlation = %d\n",corr_max);
    return n_sym;
}

uint32_t ook_demod(uint8_t *bin_rx, float *ook_rx, uint32_t size, uint32_t *bits_received){

    int i, delay = 0, pn_corr, demod_sym, delay_max = 0;
    static uint32_t sync_done=0, sym_count=0;
    float rx_th = 0.175;
        if (!sync_done){
            sym_count = 0;
            delay_max = size - (PN_SEQ_LEN + 1)*OSF;
            if (delay_max >= 0){
                while( delay <= delay_max ){
                    pn_corr = 0;
                    for(i=0; i<corr_max; i++){
                        if( *(ook_rx + OSF*indices[i]) > rx_th )
                            pn_corr++;
		            }
		            if(pn_corr == 12){
			            sync_done = 1;
                        break;
		            }
                    ook_rx++;
                    delay++;
                }
                if (sync_done){
                    size = delay_max - delay;
                    ook_rx += (PN_SEQ_LEN + 1)*OSF;
                    fprintf(stdout,"RX: Demodulation not completed, Sync done, Delay = %d\n", delay);
                } else{
                    *bits_received = 0;
                    size = (PN_SEQ_LEN + 1)*OSF;
                    fprintf(stdout,"RX: Demodulation completed, Sync not done, Remaining Samples = %d\n", size);
                }
            } else{
                *bits_received = 0;
                fprintf(stdout,"RX: Demodulation completed, Sync not done, Remaining Samples = %d\n", size);
            }
        }

        if (sync_done){
            if (size + sym_count*N_SAMP_SYM <= n_sym*N_SAMP_SYM){
                demod_sym = size/N_SAMP_SYM;
            } else{
                demod_sym = n_sym - sym_count;
                sync_done = 0;
            }
            for(i=0; i<demod_sym; i++ ){
                *bin_rx = ( (*(ook_rx+2) + *(ook_rx+3) + *(ook_rx+4) + *(ook_rx+5))/4 > rx_th );
                ook_rx += N_SAMP_SYM;
                bin_rx++;
            }
            *bits_received = demod_sym;
            sym_count += demod_sym;
            size -= demod_sym*N_SAMP_SYM;
            fprintf(stdout,"RX: Demodulation Completed, Symbol Count = %d, Received %d bits, Remaining %d Samples\n", sym_count, *bits_received, size);
/*            if(size > N_SAMP_SYM){
                uint32_t new_bits=0;
                size = ook_demod(bin_rx, ook_rx, size, &new_bits);
                *bits_received += new_bits;
            }*/
        }
    return size;
}

int main(int argc, char** argv){

	if(rp_Init()!=RP_OK)
		fprintf(stderr,"RX: Initialization failed");

	bool buff_filled=0;
	uint32_t i, size, curr_pos, prev_pos=0;
	uint32_t n_bits_total = ook_init(), bits_recvd = 0, rem_size =0;
	uint8_t *rx_bin_start = (uint8_t *)malloc(N_FRAMES*n_bits_total*sizeof(uint8_t));
	uint8_t *rx_bin_ptr = rx_bin_start;
	float *rx_sig_start = (float *)malloc(N_FRAMES*ADC_BUFFER_SIZE*sizeof(float));
	float *rx_sig_end = rx_sig_start + N_FRAMES*ADC_BUFFER_SIZE;
	float *rx_sig_ptr = rx_sig_start;

/*    clock_t start=0, end1=0, end2 = 0;;
	FILE *fp;
	fprintf(stdout, "RX: Entered\n");
	fp = fopen("./data.txt","w+");
*/
	FILE *fp1;
	fp1 = fopen("./bin.txt","w+");

	usleep(500000);
    rp_AcqReset();
    rp_AcqSetDecimation(RP_DEC_64);
	rp_AcqSetArmKeep(true);
    rp_AcqSetTriggerDelay(0);
    rp_AcqSetTriggerSrc(RP_TRIG_SRC_NOW);
    rp_AcqStart();
	usleep(10000);

//    start = clock();
	while( !buff_filled ){
		rp_AcqGetWritePointer(&curr_pos);
		size = (curr_pos - prev_pos) % ADC_BUFFER_SIZE;
		if ( rx_sig_ptr + size > rx_sig_end){
			size = rx_sig_end - rx_sig_ptr + 1;
			buff_filled = 1;
            fprintf(stdout,"RX: Receive Buffer Filled\n");
		}
        rp_AcqGetDataV(RP_CH_2, prev_pos, &size, rx_sig_ptr);
    	fprintf(stdout,"RX: read out samples = %d, curr_pos = %d, prev_pos = %d, bits received = %d \n", size, curr_pos, prev_pos, bits_recvd);
    	prev_pos = curr_pos;
    	rx_sig_ptr = (rx_sig_ptr + size);
//        end1 = clock() - start;
    	rem_size = ook_demod(rx_bin_ptr, rx_sig_ptr-rem_size-size, size+rem_size, &bits_recvd);
//        end2 = clock() - end1 - start;
    	rx_bin_ptr += bits_recvd;
//        fprintf(stdout,"RX: Read time = %lf, Process time = %lf\n", (double)end1/CLOCKS_PER_SEC, (double)end2/CLOCKS_PER_SEC);
    }

/*    for(i = 0; i <N_FRAMES*ADC_BUFFER_SIZE; i++){
            fprintf(fp," %f \n", *(rx_sig_start+i));
    }
    fclose(fp);
*/
    for(i = 0; i <N_FRAMES*n_bits_total; i++){
        fprintf(fp1," %d \n", *(rx_bin_start+i));
    }
    fclose(fp1);

    // releasing resources
    rp_AcqStop();
	free(rx_sig_start);
	free(rx_bin_start);
	rp_Release();

	fprintf(stdout,"RX: Acquisition Comeplete, Exiting RX.\n");
    return 0;
}
