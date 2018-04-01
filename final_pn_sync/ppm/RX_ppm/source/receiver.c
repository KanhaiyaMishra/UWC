#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include "redpitaya/rp.h"
#include "ppm.h"
#include <math.h>
#include "prbs.h"
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#define AVG_DELAY 10
#define N_FRAMES 1000

// known sync sequence
uint8_t pn_seq_buff[PN_SEQ_LEN];
// symbols per frame and max correlation of sync sequence
uint32_t n_sym, corr_max;
// indices holding ones in the pn sequence
uint32_t indices[64];

uint64_t GetTimeStamp(){
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

// intilialize the modulator
uint32_t ppm_init(){
    int i;
    uint32_t *temp = indices;
    // generate pn-sequence and save it in a variable
    prbs_gen_byte(PN_SEQ_TYPE, pn_seq_buff, PN_SEQ_LEN);
    // calculate number of data symbols in the frame
    n_sym = (ADC_BUFFER_SIZE/OSF - PN_SEQ_LEN - 1)/PPM;
    // find indices holding 1's in on sequence
    for(i=0; i<PN_SEQ_LEN; i++){
        if( *(pn_seq_buff+i) ){
            corr_max += *(pn_seq_buff + i);
            *temp = i;
            temp++;
        }
    }
    // return number of bits per frame
    return n_sym*N_BITS;
}

// ppm demodulation function
uint32_t ppm_demod(uint8_t *bin_rx, float *ppm_rx, uint32_t samp_recvd, uint32_t *bits_received){

    int i, j, pos, delay = 0, pn_corr=0, demod_sym=0, delay_max = 0;
    static uint32_t sync_done=0, sym_count=0;
    //threshold setting
    float rx_th = 0.45, max, temp;

    // synchronize with data to detect frame starting
    if (!sync_done){
        sym_count = 0;
        // if input data samp_recvd is more than pn-seq samp_recvd start correlating with the known sequence
        delay_max = samp_recvd - (PN_SEQ_LEN + 1)*OSF;
        if (delay_max >= 0){
            while( delay <= delay_max ){
                pn_corr = 0;
                for(i=0; i<corr_max; i++){
                    if(*(ppm_rx + OSF*indices[i]) > rx_th )
                        pn_corr++;
	            }
                // set sync_done flag if correlation reaches maximum
		        if(pn_corr >= 0.85*corr_max){
			        sync_done = 1;
                    break;
		        }
                ppm_rx++;
                delay++;
            }
            // if sync done, advance the pointer to the start of data symbols
            if (sync_done){
                samp_recvd = delay_max - delay;
                ppm_rx += (PN_SEQ_LEN + 1)*OSF;
                fprintf(stdout,"RX: Demodulation not completed, Sync completed, delay = %d samples\n", delay);
            } else{
            // if sync is not done, exit the demod function and return the num of remaining
            // samples (not checked for synchronization) to be carried forward
                *bits_received = 0;
                samp_recvd = (PN_SEQ_LEN + 1)*OSF;
                fprintf(stdout,"RX: Demodulation completed, Sync not completed, Remaining %d Samples\n", samp_recvd);
            }
        } else{
            // if input samples are less than length of pn-seq, exit the function without
            // any processing, return the input samp_recvd as remaining samp_recvd to be carried forward
            *bits_received = 0;
            fprintf(stdout,"RX: Demodulation completed, Sync not completed, Remaining %d Samples\n", samp_recvd);
        }
    }
    // demodulate the data symbols
    if (sync_done){
        // check whether frame end is reached or not, and calculate the number of symbols
        // to be demodulated, remaining samples (samp_recvd%N_SAMP_SYM) will be carried forward
        if (samp_recvd + sym_count*N_SAMP_SYM <= n_sym*N_SAMP_SYM){
            demod_sym = samp_recvd/N_SAMP_SYM;
        } else{
            demod_sym = n_sym - sym_count;
            sync_done = 0;
        }
        // demodulate ppm data symbols
        for(i=0; i<demod_sym; i++ ){
            // reset the pulse position to be zero
		    pos = 0;
            // get the value at zeroth pulse position
		    max = *(ppm_rx)+*(ppm_rx+1)+*(ppm_rx+2)+*(ppm_rx+3)+*(ppm_rx+4)+*(ppm_rx+5);
            // detect the maximum by comparing the pulse positions
            for( j=1; j< PPM; j++){
		        temp = *(ppm_rx+j*OSF)+*(ppm_rx+j*OSF+1)+*(ppm_rx+j*OSF+2)+*(ppm_rx+j*OSF+3)+*(ppm_rx+j*OSF+4)+*(ppm_rx+j*OSF+5);
                if( temp > max ){
                    pos = j;
                    max = temp;
			    }
            }
            // convert the position to binary value and write into the binary buffer
            for( j=0; j<N_BITS; j++)
                *(bin_rx+j) = ((pos>>j)&0x1);
            // advance the receive signal and the recevie binary buffer
            ppm_rx += N_SAMP_SYM;
            bin_rx += N_BITS;
        }
        // get the recevied bits, aggregate sym count and rem samples to be carried forward
        *bits_received = N_BITS*demod_sym;
        sym_count += demod_sym;
        samp_recvd -= demod_sym*N_SAMP_SYM;
        fprintf(stdout,"RX: Demodulation Completed, Symbol Count = %d, Received %d bits, Remaining %d Samples\n", sym_count, *bits_received, samp_recvd);
/*        if(samp_recvd > N_SAMP_SYM){
            uint32_t new_bits=0;
            samp_recvd = ppm_demod(bin_rx, ppm_rx, samp_recvd, &new_bits);
            *bits_received += new_bits;
        }*/
    }
    return samp_recvd;
}


int main(int argc, char** argv){

	if(rp_Init()!=RP_OK)
		fprintf(stderr,"RX: Initialization failed");

    // received samples, remaining samples, received bits, current and previous ADC ptr pos
	uint32_t i, samp_recvd = 0, samp_remng =0, bits_recvd = 0, curr_pos, prev_pos=0, recvd_frms = 0;
    // Number of bits to be received per frame
	uint32_t n_bits_total = ppm_init();
    // receive binary buffer (one extra buffer to take care of spillage while checking end of buffer)
	uint8_t *rx_bin_start = (uint8_t *)malloc((N_FRAMES+1)*n_bits_total*sizeof(uint8_t));
    // end of last binary buffer (last frame)
	uint8_t *rx_bin_end = rx_bin_start + N_FRAMES*n_bits_total;
    // current location of pointer in binary buffer
	uint8_t *rx_bin_ptr = rx_bin_start;
    // receive signal buffer (one extra buffer for the case when current read samples spill out
    // of the first buffer. In next read cycle, pointer is reset to start of the first buffer)
	float *rx_sig_start = (float *)malloc((N_FRAMES+1)*(ADC_BUFFER_SIZE+AVG_DELAY)*sizeof(float));
    // end of first receive buffer
	float *rx_sig_end = rx_sig_start + N_FRAMES*(ADC_BUFFER_SIZE+AVG_DELAY);
    // current location of pointer in signal buffer
	float *rx_sig_ptr = rx_sig_start;

    // timing variables
    uint64_t start=0, end=0;
    clock_t start1=0, end1=0, end2 = 0;;
	fprintf(stdout, "RX: Entered\n");

/*	FILE *fp;
	fp = fopen("./data.txt","w+");*/
	FILE *fp1;
	fp1 = fopen("./bin.txt","w+");

    // wait till transmission is started
	usleep(113500);
    // reset the ADC
    rp_AcqReset();
    // set the ADC sample rate (125e6/decimation)
    rp_AcqSetDecimation(RP_DEC_64);
    // enable continuous acquisition
	rp_AcqSetArmKeep(true);
    // set the trigger delay
    rp_AcqSetTriggerDelay(0);
    // set trigger source (instantaneous triggering)
    rp_AcqSetTriggerSrc(RP_TRIG_SRC_NOW);
    // start the acquisition
    rp_AcqStart();
    // small wait till ADC has acquired some data
    usleep(10000);

    start = GetTimeStamp();
    // continue recieving untill receive signal buffer gets filled
	while( TRUE ){
        // get the cpu clock at the start
        start1 = clock();

        // get the current ADC write pointer
		rp_AcqGetWritePointer(&curr_pos);

        // calculate the #samples of the data to be acquired
		samp_recvd = (curr_pos - prev_pos) % ADC_BUFFER_SIZE;

        // acquire the data into rx signal buffer from hardware ADC buffer
	    rp_AcqGetDataV(RP_CH_2, prev_pos, &samp_recvd, rx_sig_ptr);

		fprintf(stdout,"RX: Read out samples = %d, Current pos = %d, Prev_pos = %d\n", samp_recvd, curr_pos, prev_pos);
        // calculate the acquisition time
        end1 = clock() - start1;

        // demodulate the receive signal and save the remaining unprocessed samples
		samp_remng = ppm_demod(rx_bin_ptr, rx_sig_ptr-samp_remng, samp_recvd+samp_remng, &bits_recvd);
        // update the ADC pointer position
		prev_pos = curr_pos;
        // advance the signal buffer pointer
        rx_sig_ptr += samp_recvd;
        // advance the rx binary buffer
		rx_bin_ptr += bits_recvd;

        // check for signal buffer overflow
        if (rx_sig_ptr > rx_sig_end)
            break;

        // check for binary buffer oveflow
		if ( rx_bin_ptr > rx_bin_end)
			break;

        // calculate the data processing time
        end2 = clock() - end1 - start1;
        fprintf(stdout,"RX: Read time = %lf, Process time = %lf\n", (double)end1/CLOCKS_PER_SEC, (double)end2/CLOCKS_PER_SEC);
	}

/*  // save raw received data
    for(i = 0; i <n_buff*ADC_BUFFER_SIZE; i++){
        fprintf(fp," %f \n", *(rx_sig_start+i));
    }
    fclose(fp);
*/
    // save demodulated data
    end = GetTimeStamp();
    recvd_frms = ( (rx_bin_ptr - rx_bin_start)/sizeof(uint8_t) )/n_bits_total;
    fprintf(stdout,"RX: Received %d Data Frames in %fms\n", recvd_frms, (double)(end-start)/1000);

    for(i = 0; i <recvd_frms*n_bits_total; i++){
        fprintf(fp1," %d \n", *(rx_bin_start+i));
    }
    fclose(fp1);


    // Stop acquisition and release resources
    rp_AcqStop();
	free(rx_sig_start);
	free(rx_bin_start);
	rp_Release();
	fprintf(stdout,"RX: Acquisition Comeplete, Exiting.\n");
    return 0;
}
