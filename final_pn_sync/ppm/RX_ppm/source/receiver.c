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

#define N_FRAMES 1000
#define MARGIN 10

FILE *trace_fp = NULL;
// Number of bits to be received per frame
uint32_t n_sym = (ADC_BUFFER_SIZE/OSF-PN_SEQ_LEN-1)/PPM;
// rx signal buffer to store data from ADC hardware buffer
float rx_sig_buff[(N_FRAMES+MARGIN)*ADC_BUFFER_SIZE] = {0.0};
// known sync sequence
uint8_t pn_seq_buff[PN_SEQ_LEN];
// symbols per frame and max correlation of sync sequence
uint32_t corr_max;
// indices holding ones in the pn sequence
uint32_t indices[64];

uint64_t GetTimeStamp(){
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

// intilialize the modulator
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

    int i, j, pos, pn_corr=0, demod_sym=0;
    static uint32_t sync_done=0, sym_count=0, frm_count=0;
    //threshold setting
    float rx_th = PPM_AMP/2, max, temp;

    // synchronize with data to detect frame starting
    if (!sync_done){
        sym_count = 0;
        // if input data samp_recvd is more than pn-seq samp_recvd start correlating with the known sequence
        while( samp_remng >= (PN_SEQ_LEN+1)*OSF ){
            pn_corr = 0;
            for(i=0; i<corr_max; i++){
                if(rx_sig_buff[demod_idx + OSF*indices[i]] > rx_th )
                    pn_corr++;
            }
            // set sync_done flag if correlation reaches maximum
            if(pn_corr >= 0.85*corr_max){
                frm_count++;
			    sync_done = 1;
                demod_idx += (PN_SEQ_LEN + 1)*OSF;
                samp_remng -= (PN_SEQ_LEN + 1)*OSF;
                fprintf(stdout,"RX: Receiving Frame number = %d \n", frm_count);
                fprintf(trace_fp,"RX: Demodulation not completed, Sync completed, sync_idx = %d\n", demod_idx);
                break;
		    }
            demod_idx++;
            samp_remng--;
        }
        if (!sync_done){
            // if sync is not done, exit the demod function and return the num of un-processed samples
            fprintf(trace_fp,"RX: Demodulation completed, Sync not completed, Remaining %d Samples\n", samp_remng);
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
            max = rx_sig_buff[demod_idx+OSF/2];
            demod_idx += OSF;
            // detect the maximum by comparing the pulse positions
            for( j=1; j< PPM; j++){
                temp = rx_sig_buff[demod_idx+OSF/2];
                if( temp > max ){
                    pos = j;
                    max = temp;
			    }
                demod_idx += OSF;
            }
            // convert the position to binary value and write into the binary buffer
            for( j=0; j<N_BITS; j++)
                *(bin_rx++) = ((pos>>j)&0x1);
        }
        // get the recevied bits, aggregate sym count and rem samples to be carried forward
        sym_count += demod_sym;
        samp_remng -= demod_sym*N_SAMP_SYM;
        fprintf(trace_fp,"RX: Demodulation Completed, Symbol Count = %d, Received %d bits, Remaining %d Samples\n", sym_count, *bits_received, samp_remng);
    }
    *bits_received = N_BITS*demod_sym;
    return samp_remng;
}


int main(int argc, char** argv){

	if(rp_Init()!=RP_OK)
		fprintf(stderr,"RX: Initialization failed");

    uint32_t bits_per_frame = N_BITS*n_sym, data_bits = bits_per_frame - FRM_NUM_BITS;
    // received samples, remaining samples, received bits, current and previous ADC ptr pos
	uint32_t samp_recvd = 0, samp_remng =0, bits_recvd = 0, curr_pos=0, prev_pos=0;
    // receive binary buffer (one extra buffer to take care of spillage while checking end of buffer)
    uint8_t *rx_bin_buff = (uint8_t *)malloc((N_FRAMES+1)*N_BITS*n_sym*sizeof(uint8_t));
    // end of last binary buffer (last frame)
	uint8_t *rx_bin_end = rx_bin_buff + N_FRAMES*bits_per_frame;
    // current location of pointer in binary buffer
	uint8_t *rx_bin_ptr = rx_bin_buff;
    // location receive pointer, demod pointer in rx signal buffer
    uint32_t demod_idx = 0, recv_idx = 0, end_idx = (MARGIN+N_FRAMES-1)*ADC_BUFFER_SIZE;
    // get the DAC hardware address
    static volatile int32_t* adc_add = NULL;
    adc_add = (volatile int32_t*)rp_AcqGetAdd(RP_CH_2);
    int32_t i, j, temp;

    ppm_init();
    // timing variables
    uint64_t start=0, end1=0, end2 = 0;;

    FILE *bin_fp, *sig_fp;
    time_t now = time(NULL);
    char log_dir[255], trace_file[255], bin_file[255], sig_file[255];

    strftime(log_dir, 255,"../log/PPM_%Y_%m_%d_%H_%M_%S",gmtime(&now));
    mkdir(log_dir, 0777);
    strftime(trace_file, 255,"../log/PPM_%Y_%m_%d_%H_%M_%S/trace.txt",gmtime(&now));
    strftime(bin_file, 255,"../log/PPM_%Y_%m_%d_%H_%M_%S/bin.txt",gmtime(&now));
    strftime(sig_file, 255,"../log/PPM_%Y_%m_%d_%H_%M_%S/sig.txt",gmtime(&now));
    trace_fp = fopen(trace_file,"w+");
    bin_fp = fopen(bin_file,"w+");
    sig_fp = fopen(sig_file,"w+");

	fprintf(stdout, "RX: Entered, max correlation = %d, add = %p\n", corr_max, trace_fp);
    // wait till transmission is started
	usleep(105000);
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

    // continue recieving untill receive signal buffer gets filled
	while( TRUE ){
        // get the cpu clock at the start
        start = GetTimeStamp();
        // get the current ADC write pointer
		rp_AcqGetWritePointer(&curr_pos);
        // calculate the samp_recvd of the data to be acquired
		samp_recvd = (curr_pos - prev_pos) % ADC_BUFFER_SIZE;
        // acquire the data into rx signal buffer from hardware ADC buffer
//	    rp_AcqGetDataV(RP_CH_2, prev_pos, &samp_recvd, rx_sig_buff+recv_idx);
        for (i =0; i<samp_recvd; i++){
            temp = (double) (adc_add[(prev_pos+i)%ADC_BUFFER_SIZE]);
            rx_sig_buff[recv_idx+i] = (temp>8191)?(temp-(1<<14))/((double)(1<<13)):temp/((double)(1<<13));
        }
		fprintf(trace_fp,"RX: Read out samples = %d, Current pos = %d, Prev_pos = %d\n", samp_recvd, curr_pos, prev_pos);
        // calculate the acquisition time
        end1 = GetTimeStamp() - start;

        // demodulate the receive signal and save the remaining unprocessed samples
		samp_remng = ppm_demod(rx_bin_ptr, demod_idx, samp_recvd+samp_remng, &bits_recvd);
        // update the ADC pointer position
		prev_pos = curr_pos;
        // advance the signal buffer pointer
        recv_idx += samp_recvd;
        // advance the demod pointer in the buffer
        demod_idx = recv_idx - samp_remng;
        // advance the rx binary buffer
		rx_bin_ptr += bits_recvd;

        if ( recv_idx > end_idx || rx_bin_ptr > rx_bin_end)
			break;

        // calculate the data processing time
        end2 = GetTimeStamp() - end1 - start;
        fprintf(trace_fp,"RX: Read time = %lf, Process time = %lf\n", (double)end1/1000, (double)end2/1000);
	}

    // Calculate the number of frames received
    uint32_t recvd_frms = ( (rx_bin_ptr - rx_bin_buff)/sizeof(uint8_t) )/bits_per_frame;
    fprintf(stdout,"RX: Receiver signal/binary buffer filled, num of received frames = %d\n", recvd_frms);

    uint32_t error_count[recvd_frms], rx_frm_num, tx_frm_num = 1, missd_frms=0, valid_frms=0, invalid_frms=0, frm_diff=0;
    uint8_t *tx_bin_buff = (uint8_t *)malloc(data_bits*sizeof(uint8_t));
    uint8_t *tx_bin_ptr;
    float ber = 0.0;

    pattern_LFSR_byte(PRBS11, tx_bin_buff, data_bits);
    for(i=0; i<recvd_frms; i++){
        tx_bin_ptr = tx_bin_buff;
        rx_bin_ptr = rx_bin_buff+i*bits_per_frame;
        rx_frm_num = 0;
        for(j=0; j<FRM_NUM_BITS; j++)
            rx_frm_num |= (*(rx_bin_ptr++)<<j);

        frm_diff = rx_frm_num-tx_frm_num;

        if ( frm_diff <= 5 && frm_diff >= 0 ){
            while(rx_frm_num >= tx_frm_num) {
                error_count[i] = 0;
                for(j=0; j<(data_bits); j++)
                    error_count[i] += (*(tx_bin_ptr++) != *(rx_bin_ptr++));
                pattern_LFSR_byte(PRBS11, tx_bin_buff, data_bits);
                tx_frm_num++;
            }
            valid_frms +=1;
            missd_frms +=frm_diff;
            ber += (double)error_count[i];
            fprintf(stdout,"RX: Received Frame number %d with %d bit errors\n", rx_frm_num, error_count[i]);
        } else {
            invalid_frms++;
            fprintf(stdout,"RX: Received invalid Frame number %d, ignoring for BER Calculation\n", rx_frm_num);
        }
    }
    ber = ber/(valid_frms*data_bits);
    fprintf(stdout,"RX: Received total %d valid frames with BER = %f\n", valid_frms, ber);
    fprintf(stdout,"RX: Missed %d frames and received %d invalid frames\n", missd_frms, invalid_frms);

    // save demodulated data
    for(i = 0; i <recvd_frms*bits_per_frame; i++){
        fprintf(bin_fp," %d \n", rx_bin_buff[i]);
    }
    if(missd_frms>0){
        // save demodulated data
        for(i = 0; i <recv_idx; i++){
            fprintf(sig_fp," %f \n", rx_sig_buff[i]);
        }
    }

    fclose(sig_fp);
    fclose(bin_fp);
    fclose(trace_fp);

    // Stop acquisition and release resources
    rp_AcqStop();
	rp_Release();
	fprintf(stdout,"RX: Acquisition Comeplete, Exiting.\n");
    return 0;
}


