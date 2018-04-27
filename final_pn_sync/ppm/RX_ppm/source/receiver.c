#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <time.h>
#include "redpitaya/rp.h"
#include "ppm.h"
#include "prbs.h"

#define DC_ERROR 0.015
// #frames to be received
#define N_FRAMES 1000
// Frame duration (actual duration = 8.389us)
#define FRM_DUR 9
// recv signal buffer size (must be in powers of 2 (because of unsigned diff of indices later used in the program) )
#define RX_BUFF_SIZE (4*ADC_BUFFER_SIZE)
// constant for converting second to nano second
#define NANO 1000000000LL

// File pointer to log traces
FILE *trace_fp = NULL;
// Number of bits to be received per frame
uint32_t n_sym = (ADC_BUFFER_SIZE/OSF-PN_SEQ_LEN)/PPM;
// rx signal buffer to store data from ADC hardware buffer
float rx_sig_buff[RX_BUFF_SIZE] = {0.0};
// known sync sequence
uint8_t pn_seq_buff[PN_SEQ_LEN];
// symbols per frame and max correlation of sync sequence
uint32_t corr_max, indices[64];
// timediff in miliseconds
static double timediff_ms(struct timespec *begin, struct timespec *end){
    return (double)( (end->tv_sec - begin->tv_sec)*NANO + (end->tv_nsec - begin->tv_nsec) )/1000000;
}

// intilialize the demodulator
void ppm_init(){
    int i;
    uint32_t *temp = indices;
    // generate pn-sequence and save it in a variable
    pattern_LFSR_byte(PN_SEQ_TYPE, pn_seq_buff, PN_SEQ_LEN);
    // find indices holding 1's in sync sequence
    for(i=0; i<PN_SEQ_LEN; i++){
        if( *(pn_seq_buff+i) ){
            corr_max += *(pn_seq_buff + i);
            *temp = i;
            temp++;
        }
    }
}

// ppm demodulation function
uint32_t ppm_demod(uint8_t *bin_rx, uint32_t demod_idx, uint32_t samp_remng, uint32_t *bits_received){

    float rx_th = (PPM_HIGH + PPM_LOW)/2;
    int i, j, pos, pn_corr=0, demod_sym=0;
    static uint32_t sync_done=0, sym_count=0, frm_count=0;
    //threshold setting
    float max, adc_counts;

    // synchronize with data to detect frame starting
    if (!sync_done){
        sym_count = 0;
        // if input data samp_recvd is more than pn-seq samp_recvd start correlating with the known sequence
        while( samp_remng >= PN_SEQ_LEN*OSF ){
            pn_corr = 0;
            for(i=0; i<corr_max; i++){
                if(rx_sig_buff[(demod_idx + OSF*indices[i])%RX_BUFF_SIZE] > rx_th )
                    pn_corr++;
            }
            // set sync_done flag if correlation reaches maximum
            if(pn_corr == corr_max){
                frm_count++;
			    sync_done = 1;
                demod_idx = (demod_idx+PN_SEQ_LEN*OSF)%RX_BUFF_SIZE;
                samp_remng -= (PN_SEQ_LEN*OSF);
                fprintf(stdout,"RX: Receiving Frame number = %d \n", frm_count);
                break;
		    }
            demod_idx++;
            samp_remng--;
        }
    }
    // demodulate the data symbols
    if (sync_done){
        // check whether frame end is reached or not, and calculate the number of symbols
        // to be demodulated, remaining samples (samp_recvd%N_SAMP_SYM) will be carried forward
        if (samp_remng + sym_count*N_SAMP_SYM <= n_sym*N_SAMP_SYM){
            demod_sym = samp_remng/N_SAMP_SYM;
        } else{
            demod_sym = n_sym - sym_count;
            sync_done = 0;
        }
        // demodulate ppm data symbols
        for(i=0; i<demod_sym; i++ ){
            // reset the pulse position to be zero
		    pos = 0;
            max = rx_sig_buff[demod_idx];
            demod_idx = (demod_idx+OSF)%RX_BUFF_SIZE;
            // detect the maximum by comparing the pulse positions
            for( j=1; j< PPM; j++){
                adc_counts = rx_sig_buff[demod_idx];
                if( adc_counts > max ){
                    pos = j;
                    max = adc_counts;
			    }
                demod_idx = (demod_idx+OSF)%RX_BUFF_SIZE;
            }
            // convert the position to binary value and write into the binary buffer
            for( j=0; j<N_BITS; j++)
                *(bin_rx++) = ((pos>>j)&0x1);
        }
        // get the recevied bits, aggregate sym count and rem samples to be carried forward
        sym_count += demod_sym;
        samp_remng -= demod_sym*N_SAMP_SYM;

        // recursive call at the end of each frame when there are enough samples remaining for synchronization and/or demodulation
        if( sym_count==n_sym && samp_remng>=(OSF*PN_SEQ_LEN) ){
            uint32_t new_bits = 0;
            samp_remng = ppm_demod(bin_rx, demod_idx, samp_remng, &new_bits);
            demod_sym += (new_bits/N_BITS);
        }
    }
    *bits_received = N_BITS*demod_sym;
    return samp_remng;
}


int main(int argc, char** argv){

	if(rp_Init()!=RP_OK)
		fprintf(stderr,"RX: Initialization failed");

    uint32_t bits_per_frame = N_BITS*n_sym, data_bits = bits_per_frame - FRM_NUM_BITS;
    // received samples, remaining samples, received bits, current and previous ADC ptr pos
	uint32_t samp_recvd = 0, samp_remng =0, bits_recvd = 0, curr_pos=0, prev_pos=0, recvd_frms=1;
    // receive binary buffer (one extra buffer to take care of spillage while checking end of buffer)
    uint8_t *rx_bin_buff = (uint8_t *)malloc((N_FRAMES+1)*N_BITS*n_sym*sizeof(uint8_t));
    // end of last binary buffer (last frame)
	uint8_t *rx_bin_end = rx_bin_buff + N_FRAMES*bits_per_frame;
    // current location of pointer in binary buffer
	uint8_t *rx_bin_ptr = rx_bin_buff;
    // location receive pointer, demod pointer in rx signal buffer
    uint32_t demod_idx = 0, recv_idx = 0;
    // get the DAC hardware address
    static volatile uint32_t *adc_add;
    adc_add = (volatile uint32_t*)rp_AcqGetAdd(RP_CH_2);
    int32_t i, j, adc_counts=0;

    // initialize sync_sequence to get correlation value
    ppm_init();
    // timing variables
    struct timespec begin, end;

    FILE *bin_fp, *ber_fp;
    time_t now = time(NULL);
    char log_dir[255], ber_file[255], bin_file[255];

    strftime(log_dir, 255,"../log/PPM_%Y_%m_%d_%H_%M_%S",gmtime(&now));
    mkdir(log_dir, 0777);
    strftime(bin_file, 255,"../log/PPM_%Y_%m_%d_%H_%M_%S/bin.txt",gmtime(&now));
    strftime(ber_file, 255,"../log/PPM_%Y_%m_%d_%H_%M_%S/ber.txt",gmtime(&now));
    bin_fp = fopen(bin_file,"w+");
    ber_fp = fopen(ber_file,"w+");

	fprintf(stdout, "RX: Entered, max correlation = %d, add = %p\n", corr_max, adc_add);
    // wait till transmission is started
	usleep(100000);
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

    clock_gettime(CLOCK_MONOTONIC, &begin);
    // continue recieving untill receive signal buffer gets filled
	while( TRUE ){

        // get the current ADC write pointer
		rp_AcqGetWritePointer(&curr_pos);
        // calculate the samp_recvd of the data to be acquired
		samp_recvd = (curr_pos - prev_pos) % ADC_BUFFER_SIZE;
        // acquire the data into rx signal buffer from hardware ADC buffer
//	    rp_AcqGetDataV(RP_CH_2, prev_pos, &samp_recvd, rx_sig_buff+recv_idx);
        for (i =0; i<samp_recvd; i++){
            adc_counts = ( adc_add[(prev_pos+i)%ADC_BUFFER_SIZE] & 0x3FFF );
            adc_counts = ( (adc_counts < (1<<13)) ? adc_counts : (adc_counts - (1<<14)) );
            rx_sig_buff[(recv_idx+i)%RX_BUFF_SIZE] = ((double)adc_counts)/(1<<13) + DC_ERROR;
        }

        #if TRACE_PRINT
		fprintf(trace_fp,"RX: Recv Index = %d, Current pos = %d, Prev_pos = %d\n", recv_idx, curr_pos, prev_pos);
        // calculate the acquisition time
        clock_gettime(CLOCK_MONOTONIC, &t2);
        #endif

        // demodulate the receive signal and save the remaining unprocessed samples
		samp_remng = ppm_demod(rx_bin_ptr, demod_idx, samp_recvd+samp_remng, &bits_recvd);
        // update the ADC pointer position
		prev_pos = curr_pos;
        // advance the signal buffer pointer
        recv_idx = (recv_idx+samp_recvd)%RX_BUFF_SIZE;
        // advance the demod pointer in the buffer
        demod_idx = (recv_idx - samp_remng)%RX_BUFF_SIZE;
        // advance the rx binary buffer
		rx_bin_ptr += bits_recvd;

        // get the current clock time to determine whether or not to stop
        clock_gettime(CLOCK_MONOTONIC, &end);
        // check if all frames are received or Time is over (avoid infinite loop, in case all frames were not received)
        if(rx_bin_ptr > rx_bin_end || timediff_ms(&begin, &end)> (N_FRAMES*FRM_DUR) ){
            // Calculate the number of frames received
            recvd_frms = ( (rx_bin_ptr - rx_bin_buff)/sizeof(uint8_t) )/bits_per_frame;
            if (rx_bin_ptr > rx_bin_end)
                fprintf(stdout,"RX: All frames received, Received %d frames in %lfsec\n", recvd_frms, timediff_ms(&begin, &end)/1000);
            else
                fprintf(stdout,"RX: Time limit reached, Received %d frames in %lfsec\n", recvd_frms, timediff_ms(&begin, &end)/1000);
            break;
        }
	}

    // BER Evaluation Varibales: Error Count per frame, Last frame recevied, RX and TX Frame numbers, Valid, invalid and Missed Frame numbers
    uint32_t error_count[recvd_frms], last_rx_frm = 0, rx_frm_num=0, tx_frm_num = 1, missd_frms=0, valid_frms=0, invalid_frms=0, frm_diff=0;
    // buffer to hold tx binary data, pointer to tx binary buffer
    uint8_t *tx_bin_buff = (uint8_t *)malloc(data_bits*sizeof(uint8_t)), *tx_bin_ptr;
    float ber = 0.0;

    memset(error_count, 0, recvd_frms*sizeof(uint32_t));
    // generate first transmit frame
    pattern_LFSR_byte(PRBS11, tx_bin_buff, data_bits);
    // iterate over all received frames
    for(i=0; i<recvd_frms; i++){
        // get base pointers to the binary buffers for current frame
        tx_bin_ptr = tx_bin_buff;
        rx_bin_ptr = rx_bin_buff+i*bits_per_frame;
        // get the current received frame number
        rx_frm_num = 0;
        for(j=0; j<FRM_NUM_BITS; j++)
            rx_frm_num |= (*(rx_bin_ptr++)<<j);

        // calculate diff between current and last received frames
        frm_diff = rx_frm_num - last_rx_frm;

        // evaluate ber only if frm_diff is less than 5
        if ( frm_diff >= 1 && frm_diff <= 5 ){
            while(rx_frm_num >= tx_frm_num) {
                if(rx_frm_num == tx_frm_num){
                    for(j=0; j<(data_bits); j++)
                        error_count[i] += (*(tx_bin_ptr++) != *(rx_bin_ptr++));
                }
                pattern_LFSR_byte(PRBS11, tx_bin_buff, data_bits);
                tx_frm_num++;
            }
            valid_frms +=1;
            missd_frms += (frm_diff-1);
            ber += (double)error_count[i];
            if (error_count[i])
                fprintf(ber_fp,"RX: Received Frame number %d with %d bit errors\n", rx_frm_num, error_count[i]);
        // if frm_diff not within range, call it invalid frame
        } else {
            invalid_frms++;
            fprintf(ber_fp,"RX: Received invalid Frame %d, Expected Frame %d. Ignoring for BER Calculation\n", rx_frm_num, tx_frm_num);
        }
        last_rx_frm = rx_frm_num;
    }
    // calculate average BER
    ber = ber/(valid_frms*data_bits);
    fprintf(stdout,"\nRX: Calculating BER per received frame basis\n");
    fprintf(stdout,"RX: Received total %d valid frames with BER1 = %f\n", valid_frms, ber);
    fprintf(stdout,"RX: Missed %d frames and received %d invalid frames\n", missd_frms, invalid_frms);

    fprintf(stdout,"\nRX: Calculating BER assuming no frames are missed\n");
    pattern_LFSR_reset();
    ppm_init();
    memset(error_count, 0, recvd_frms*sizeof(uint32_t));
    ber = 0;
    // find a valid frm_num (1-10)
    tx_frm_num=1;
    rx_bin_ptr = rx_bin_buff;
    tx_bin_ptr = tx_bin_buff;
    fprintf(stdout,"RX: Searching First valid received frame\n");
    while(TRUE){
        rx_frm_num = 0;
        for(j=0; j<FRM_NUM_BITS; j++)
            rx_frm_num |= (*(rx_bin_ptr++)<<j);
        if(rx_frm_num<10){
            fprintf(stdout,"RX: First valid received frame is %d\n", rx_frm_num);
            break;
        } else if(rx_bin_ptr-rx_bin_buff > 10*bits_per_frame){
            rx_bin_ptr = rx_bin_buff+FRM_NUM_BITS;
            fprintf(stdout,"RX:Could not find any valid frame, setting first valid frame to be 1\n");
            rx_frm_num = 1;
            break;
        }
        rx_bin_ptr += bits_per_frame;
    }
    while(tx_frm_num!=rx_frm_num){
        pattern_LFSR_byte(PRBS11, tx_bin_buff, data_bits);
        tx_frm_num++;
    }

    for (i=0; i<=(recvd_frms-rx_frm_num); i++){
        pattern_LFSR_byte(PRBS11, tx_bin_buff, data_bits);
        for(j=0; j<(data_bits); j++)
            error_count[i] += (*(tx_bin_ptr++) != *(rx_bin_ptr++));
        rx_bin_ptr+= FRM_NUM_BITS;
        tx_bin_ptr = tx_bin_buff;
        ber += (double)error_count[i];
    }
    ber = ber/(valid_frms*data_bits);
    fprintf(stdout,"RX: Received total %d valid frames with BER2 = %f\n", i, ber);


    // save demodulated data if ber is non-zero
    if(ber>0.0){
        for(i = 0; i <recvd_frms*bits_per_frame; i++){
            fprintf(bin_fp," %d \n", rx_bin_buff[i]);
        }
    }

    // close the output files
    fclose(ber_fp);
    fclose(bin_fp);

    // Stop acquisition and release resources
    free(rx_bin_buff);
    rp_AcqStop();
    rp_Release();
    fprintf(stdout,"RX: Acquisition Comeplete, Exiting.\n");
    return 0;
}


