/* UWC PROJECT: OFDM Demodulation Function specifically written for Redpitaya Hardware (ADC/DAC)
   AUTHOR: Kanhaiya Mishra, M Tech Student IIT Madras
   Date: April, 7 2018 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include "prbs.h"
#include "ofdm.h"
#include "deque.h"
#include "kiss_fft.h"
#include "redpitaya/rp.h"
#include <sys/stat.h>

// amplitude adjustment done at the transmitter side
#define AMP_ADJ 20
// DC error of ADC
#define DC_ERROR 0.0140
// number of frames to be received
#define N_FRAMES 1000

uint64_t GetTimeStamp(){
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

// log file pointer (updated in main)
FILE *trace_fp = NULL;
static complex_t sync_qam_tx[N_FFT] = {{0.0}};
static complex_t fft_in_buff[N_FFT] = {{0.0}};
static complex_t fft_out_buff[N_FFT] = {{0.0}};
static real_t rx_sig_buff[(N_FRAMES+100)*ADC_BUFFER_SIZE] = {0.0};
static uint8_t rx_bin_buff[(N_FRAMES+1)*N_SYM*N_QAM*N_BITS] = {0.0};
static uint8_t gray_map[16] __attribute__((aligned(1))) = { 0, 1, 3, 2, 6, 7, 5, 4, 12, 13, 15, 14, 10, 11,  9,  8 };

// Inline function to calculate complex magnitude
static inline real_t comp_mag_sqr( complex_t num ){
    return (num.r*num.r + num.i*num.i);
}

// Inline fucntion for complex division
static inline complex_t comp_div( complex_t num, complex_t den){
    complex_t div;
    div.r = (num.r*den.r + num.i*den.i)/comp_mag_sqr(den);
    div.i = (num.i*den.r - num.r*den.i)/comp_mag_sqr(den);
    return div;
}

// Generate the pilot symbols
void generate_ofdm_sync(){

    // sync symbol uses half subcarriers as compared to data symbol
    uint8_t *bin_data = malloc(N_QAM*N_BITS/2*sizeof(uint8_t));
    // generate binary data for sync symbols
    pattern_LFSR_byte(PRBS7, bin_data, N_DSC/2);

    // qam modulate the binary data
    complex_t temp, *head_ptr, *tail_ptr;
    // get the header and tail pointers of the output buffer
    // head = +1 subcarrier, tail = +N_FFT(or -1) subcarrier
    head_ptr = sync_qam_tx + 1;
    tail_ptr = sync_qam_tx + N_FFT - 1;

    // temp variable for real and imag components of the QAM symbol
    uint32_t j, i, qam_r, qam_i;
    // max integer value of the real/imag component of the QAM symbol
    real_t qam_limit = pow(2, N_BITS/2) - 1;
    // Nomalization constant for getting average unit power per ofdm symbol
    real_t norm_const = 1/sqrt( 20*2*(M_QAM - 1)/3*N_DSC/2 );

    for(i=0; i<N_QAM; i++)
    {
        // reset real and imag parts
        qam_r = 0;
        qam_i = 0;

        // separate the real and imag bits and convert to decimal for gray coding
        for (j=1; j<=N_BITS/2; j++){
            qam_r |= ( (*(bin_data + N_BITS/2 - j) ) << (j-1) );
            qam_i |= ( (*(bin_data + N_BITS - j) ) << (j-1) );
        }
        // gray coded QAM modulation and normalization
        temp.r =  (2*gray_map[qam_r] -qam_limit)*norm_const;
        temp.i = -(2*gray_map[qam_i] -qam_limit)*norm_const;

        // advance the bin pointer to next symbol
        bin_data += N_BITS;

        // sync symbols only used odd data subcarriers (+1, +3,... | -1,-3,....)
        // QAM symbol at -1 subarrier will be complex conjugate of +1 subcarrier
        (head_ptr)->r = temp.r;
        (head_ptr++)->i = temp.i;
        (head_ptr)->r = 0.0;
        (head_ptr++)->i = 0.0;
        (tail_ptr)->r = temp.r;
        (tail_ptr--)->i = -temp.i;
        (tail_ptr)->r = 0.0;
        (tail_ptr--)->i = -0.0;
        i++;
    }

}

// QAM Demodulation function: input qam_data, output binary data
void qam_demod(uint8_t *bin_data, complex_t *qam_data){

    complex_t temp;
    uint32_t i, j, qam_sym_r, qam_sym_i;
    real_t qam_limit = pow(2, N_BITS/2) - 1;
    real_t norm_const = sqrt( 20*(2*(M_QAM - 1)/3)*N_DSC)/N_FFT;

    for(i=0; i<N_QAM; i++)
    {
        //extract the qam data from input buffer
        temp.r = qam_data->r*norm_const;
        temp.i = -qam_data->i*norm_const;

        // Hard Limit Data Within QAM Symbol Boundaries
        if (temp.r > qam_limit)
            temp.r = qam_limit;
        else if (temp.r < -qam_limit)
            temp.r = -qam_limit;
        if (temp.i > qam_limit)
            temp.i = qam_limit;
        else if (temp.i < -qam_limit)
            temp.i = -qam_limit;

        // Demodulate based on minimum distance from constellation points
        qam_sym_r = gray_map[ (uint8_t)(round((temp.r+qam_limit)/2)) ];
        qam_sym_i = gray_map[ (uint8_t)(round((temp.i+qam_limit)/2)) ];

        //write the binary data into output buffer
        for (j=1; j<=N_BITS/2; j++){
            (*(bin_data + N_BITS/2 - j)) = (qam_sym_r>>(j-1))&0x1;
            (*(bin_data + N_BITS - j)) = (qam_sym_i>>(j-1))&0x1;
        }
        bin_data += N_BITS;
        qam_data++;
    }
}

// OFDM signal demodulation function receives random number of samples, first detects frame boundary
// then corrects the sync error and demodulates the signal per symbol basis depending on the #samples
uint32_t ofdm_demod(uint8_t *bin_rx, uint32_t demod_idx, uint32_t samp_remng, uint32_t *bits_recvd){

    // local pointers to the global FFT input and output buffers
    complex_t *fft_out = fft_out_buff, *fft_in = fft_in_buff;
    // local buffers for storing channel coefficients in time and frequency domain
    complex_t ht[N_FFT] = {{0.0}}, hf[N_FFT] = {{0.0}};
    // FFT/IFFT configuraiton data structure required for FFT/IFFT operation
    kiss_fft_cfg fft_cfg = kiss_fft_alloc(N_FFT, FALSE, NULL, NULL);
    kiss_fft_cfg ifft_cfg = kiss_fft_alloc(N_FFT, TRUE, NULL, NULL);
    // constants: Max sync error = pre introduced synchronization error, n_cp_rem = part of cp that is removed
    const int32_t max_sync_error = floor(N_CP_SYNC/2), n_cp_rem = (N_CP_SYNC - max_sync_error);
    // correlation length, window minimum length, auto correlation threshold to diff pilot and noise
    const int32_t corr_len = OSF*N_FFT/2/PRE_DSF, win_len = POST_DSF*N_CP_SYNC, auto_corr_th = 0.5*(N_FFT*POST_DSF/AMP_ADJ/2);
    // corr_count, sym_count, sync_correction flag, sync complettion flag, sync index with respect to base address of the signal buffer
    static int32_t corr_count = -1, sym_count = 0, sync_corrected  = 0, sync_done= 0, sync_idx=0, frm_count=0;
    // maximum of window minimum of correlation factor, auto correlation, cross correlation, cross_correlation, auto and cross correlation at sync point
    static real_t max_of_min = 0.0, auto_corr=0.0, cros_corr=0.0, corr_fact = 0.0;
    // base index of receive data, mid index (base + corr_len), end index (base + 2*corr_len), default sync error (depends on the range of coefficients over which maxima is found)
    uint32_t idx1=0, idx2=0, idx3=0, demod_sym=0, sync_err = N_FFT/4;
    // double ended queue to implement continuous window minimum using double sorted set
    // Pair Struct holds: Coor_fact and corresponding index,
    // Dequeue holds: base add of a static 'pair' array, front and rear index of within in the queue
    static pair queue[POST_DSF*N_CP_SYNC+2];
    dequeue window;

    idx1 = demod_idx;
    idx2 = (demod_idx+N_FFT*OSF/2);
    idx3 = (demod_idx+N_FFT*OSF);

    // log the receive index and number of samples received
    fprintf(trace_fp,"RX: Sync/Demod Started, Demod Index = %d, Bin Index = %d, Received Samples = %d\n", demod_idx, (bin_rx-rx_bin_buff), samp_remng);

    // check whether sync is completed or not
    if(!sync_done){
        // chech whether sync is started or not
        if(corr_count<0){
            // if sync is just started, initialize all static variables
            sym_count = 0;
            cros_corr = 0;
            auto_corr = 0;
            memset(queue, 0, (win_len+2)*(sizeof(float)+sizeof(int)));
            initialize(&window, queue, win_len+2);
            // make sure received samples are more than correlation window
            if(samp_remng >= OSF*N_FFT){
                // compute cross and auto correlation at first receive index
                for (int i=0; i<corr_len; i++){
                    auto_corr += rx_sig_buff[(idx2+i*PRE_DSF) ]*rx_sig_buff[(idx2+i*PRE_DSF) ];
                    cros_corr += rx_sig_buff[(idx1+i*PRE_DSF) ]*rx_sig_buff[(idx2+i*PRE_DSF) ];
                }
                // compute the corellation factor (absolute ration of cross and auto correlation)
                corr_fact = fabs(cros_corr/auto_corr);
                corr_count++;
            } else
                fprintf(trace_fp,"RX: Sync not completed samples not enough, demod_idx = %d, remaining samples = %d\n", demod_idx, samp_remng);
        }
        if(corr_count>=0){
            // compute correlation for forward indices untill remaining samples are less than required by correlation
            for( ;samp_remng > OSF*N_FFT; samp_remng-=PRE_DSF) {
                // auto correlation threshold (diff noise from pilot), corr fact threshold (reduces uncessary computation when pilot is yet present)
                // start computing window minimum once thresholds are crossed
                if( auto_corr > auto_corr_th){
                    // remove all elements from rear having higher corr factor than current corr_fact (they will never be the window minimum in future)
                    while( !empty(&window) && window.base_ptr[window.rear].value >= corr_fact)
                        dequeueR(&window);
                    // add the current element (corr_fact and position) from rear
                    enqueueR( &window, (pair){.value = corr_fact, .position = idx1} );
                    // remove all the elements from front having indices before window length from current index
                    while( (window.base_ptr[window.front].position + (win_len*PRE_DSF)) < idx1 )
                        dequeueF(&window);
//                    fprintf(stdout,"front location = %d, rear_location = %d\n", window.front, window.rear);
                    // check whether current corr_fact is the maximum window minimum and save its index
                    if( window.base_ptr[window.front].value > max_of_min ){
                        max_of_min = window.base_ptr[window.front].value;
                        // for current index, algorithm calculates window minimum for the index one window behind
                        sync_idx = idx1 - (win_len*PRE_DSF);
                    }else if (window.base_ptr[window.front].value < 0.6*max_of_min){
                        frm_count++;
                        // consider that maximum if the current value is less 85% of the maxima and exit sync loop
                        sync_done = 1;
                        // compute remaining samples from the sync index
                        samp_remng += (idx1 - sync_idx);
                        fprintf(stdout,"RX: Receiving Frame number = %d \n", frm_count);
                        fprintf(trace_fp,"RX: Sync completed, corr_count = %d, remaining samples = %d, SYNC_IDX[%d] = %d\n", corr_count, samp_remng, frm_count, sync_idx);
                        break;
                    }
                }
                // compute correlation values for next index
                auto_corr = auto_corr + rx_sig_buff[idx3]*rx_sig_buff[idx3] - rx_sig_buff[idx2]*rx_sig_buff[idx2];
                cros_corr = cros_corr + rx_sig_buff[idx2]*rx_sig_buff[idx3] - rx_sig_buff[idx1]*rx_sig_buff[idx2];
                corr_fact = fabs(cros_corr/auto_corr);
                // increase the correlation count and advance the index (partial downsampling)
                corr_count++;
                idx1 = (idx1 + PRE_DSF);
                idx2 = (idx2 + PRE_DSF);
                idx3 = (idx3 + PRE_DSF);
            }
            if(!sync_done){
                fprintf(trace_fp,"RX: Sync not completed, corr_count = %d, remng samples = %d, auto_corr=%f, cros_Corr = %f\n", corr_count, samp_remng, auto_corr, cros_corr);
            }
        }
    }
    if(sync_done){
        // if sync is completed do the err correction
        if (!sync_corrected){
            if ( samp_remng >= (N_FFT+n_cp_rem)*OSF ){
                // downsample and collect samples for pilot estimation
                for( int i=0; i< N_FFT; i++)
                    fft_in[i].r = rx_sig_buff[sync_idx + (n_cp_rem + i)*OSF];

                // Take fft to get pilot symbols in frequency domain
                kiss_fft( fft_cfg, (const complex_t *)fft_in, fft_out);

                // calculate channel coefficients by dividing transmit pilots (only at odd subcarriers)
                for (int i = 1; i<N_QAM; i+=2){
                    hf[i] = comp_div( fft_out[i], sync_qam_tx[i]);
                    hf[N_FFT-i] = comp_div( fft_out[N_FFT-i], sync_qam_tx[N_FFT-i]);
                }
                // Take ifft of coeficients to get time domain channel response
               	kiss_fft( ifft_cfg, (const complex_t *)hf, ht);

                // Start from mid point in the channel response since there will be two maxima, due to anti-symmetricity in the pilot
                // Only need to scan for MAX in a window of N_FFT/2 in the middle, this will give max correction of of +/-(N_FFT/4) samples
                complex_t *ch_ptr = ht+N_FFT/4+max_sync_error;

                // Set dominant tap location in the channel to 0
                float max_coef = comp_mag_sqr(ch_ptr[0]);

                // Find the dominant tap searching across the window of N_FFT/2
                for( int i=1; i<N_FFT/2; i++) {
                    if( max_coef < comp_mag_sqr(ch_ptr[i]) ){
                        max_coef = comp_mag_sqr(ch_ptr[i]);
                        sync_err = N_FFT/4-i;
                    }
                }

                // confirm if the tap is actually dominant (for cases when sync_idx happens to be on the boundary of two samples, peak could be incorrect,
                // Either side peak of the maxima might share almost equal power), correct sync will be to move slightly(2 samples) towards that side peak
                float l_side_pk = comp_mag_sqr(ch_ptr[N_FFT/4-sync_err-1]);
                float r_side_pk = comp_mag_sqr(ch_ptr[N_FFT/4-sync_err+1]);
                if(l_side_pk > 0.5*max_coef){
                    sync_idx -= 2;
                    samp_remng += 2;
                } else if(r_side_pk > 0.5*max_coef){
                    sync_idx += 2;
                    samp_remng -= 2;
                }
                // compute the demodulation index and remaining samples after sync correction
                demod_idx = sync_idx + SYNC_SYM_LEN - sync_err*OSF;
                samp_remng = samp_remng - SYNC_SYM_LEN + sync_err*OSF;
                sync_corrected = 1;
                fprintf(trace_fp,"RX: Sync correction done by %d samples, Corrected Sync Index = %d\n", sync_err*OSF, demod_idx);

            } else
                fprintf(trace_fp,"RX: Demoduation not completed, sync correction not completed\n");
        }
        // start demdoulation once sync correction is done
        if (sync_corrected){
            // check whether remaining samples more than size of all remaining OFDM symbols in the frame
            if ( samp_remng >= (N_SYM-sym_count)*DATA_SYM_LEN ){
                // if yes, demodulate till end of the frame
                demod_sym = N_SYM - sym_count;
                // reset syncronizaiton parameters for sync of next frame
                corr_count = -1;
                sync_done = 0;
                sync_corrected = 0;
                max_of_min = 0;
            } else{
                // if not demodulate all symbols contained within the received samples
                demod_sym = samp_remng/DATA_SYM_LEN;
            }

            // remove the cp for first symbol to be demdulated
            demod_idx = (demod_idx + N_CP_DATA*OSF);

	        for(int j=0; j<demod_sym; j++){
                // downsample the received data
		    for( int i=0; i< N_FFT; i++){
	    	    #ifdef FLIP
                        fft_in[i].r = rx_sig_buff[ (demod_idx + i*OSF) ] - rx_sig_buff[ (demod_idx + DATA_SYM_LEN + i*OSF) ];
	    	    #elif defined(DCO)
                        fft_in[i].r = rx_sig_buff[ (demod_idx + i*OSF) ];
		    #endif
		    }

                // Compute FFT and demdoulate QAM data into binary buffer
                kiss_fft( fft_cfg, (const complex_t *)fft_in, fft_out);
                qam_demod( bin_rx, (fft_out+1));

                // get rx buffer pointer without cp for next symbol
                demod_idx += DATA_SYM_LEN;
                // get bin buffer pointer for next symbol
                bin_rx += N_QAM*N_BITS;
                // FLIP decodes two ofdm symbols together, so advance pointer once more
                #if defined(FLIP)
                    demod_idx += DATA_SYM_LEN;
                    j++;
                #endif
	        }
            // increase the current symbol count for the frame by # demod symbols
            sym_count += demod_sym;
            // decrease the remaining samples by samples contained by demodulated symbols
            samp_remng -= demod_sym*DATA_SYM_LEN;
            // shift the demodulation index back the base from CP
            demod_idx = (demod_idx - N_CP_DATA*OSF);
            // log the demodulaion information
            fprintf(trace_fp,"RX: Demoduation completed, sym_count = %d, demod_idx = %d, remaining samples = %d, received bits = %d\n", sym_count, demod_idx, samp_remng, demod_sym*N_QAM*N_BITS);
            kiss_fft_free(fft_cfg);
            kiss_fft_free(ifft_cfg);
        }
    }
    // return the number of bits received and remaining samples
    *bits_recvd = (demod_sym*N_QAM*N_BITS);
    return samp_remng;
}


int main(int argc, char** argv){

    if(rp_Init()!=RP_OK)
        fprintf(stderr,"RX: Initialization failed");

    // received samples, remaining samples, received bits, current and previous ADC ptr pos
    uint32_t samp_recvd = 0, samp_remng =0, bits_recvd = 0, curr_pos, prev_pos=0;
    uint32_t bits_per_frame = N_SYM*N_QAM*N_BITS, data_bits = bits_per_frame-FRM_NUM_BITS;
    // end of last binary buffer (last frame)
    uint8_t *rx_bin_end = rx_bin_buff + N_FRAMES*N_SYM*N_QAM*N_BITS;
    // current location of pointer in binary buffer
    uint8_t *rx_bin_ptr = rx_bin_buff;
    // receive signal buffer (one extra buffer for the case when current read samples spill out
    // of the first buffer. In next read cycle, pointer is reset to start of the first buffer)
    uint32_t demod_idx = 0, recv_idx = 0, end_idx = (9+N_FRAMES)*ADC_BUFFER_SIZE;
    // timing variables
    uint64_t start=0, end1=0, end2 = 0;
    // get the DAC hardware address
    static volatile int32_t* adc_add = NULL;
    adc_add = (volatile int32_t*)rp_AcqGetAdd(RP_CH_2);
    FILE *bin_fp, *sig_fp;
    time_t now = time(NULL);
    char log_dir[255], trace_file[255], bin_file[255], sig_file[255];

    strftime(log_dir, 255,"../log/%Y_%m_%d_%H_%M_%S",gmtime(&now));
    mkdir(log_dir, 0777);
    strftime(trace_file, 255,"../log/%Y_%m_%d_%H_%M_%S/trace.txt",gmtime(&now));
    strftime(bin_file, 255,"../log/%Y_%m_%d_%H_%M_%S/bin.txt",gmtime(&now));
    strftime(sig_file, 255,"../log/%Y_%m_%d_%H_%M_%S/sig.txt",gmtime(&now));
    trace_fp = fopen(trace_file,"w+");
    bin_fp = fopen(bin_file,"w+");
    sig_fp = fopen(sig_file,"w+");

    fprintf(trace_fp, "RX: Entered, Address = %p\n", adc_add);
    // generate the pilots for synchronization error correction exactly same as in Transmit Program

    generate_ofdm_sync();
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

        // acquire the data into rx signal buffer from ADC buffer either using library function or using hardware address(much faster)
//	    rp_AcqGetDataV(RP_CH_2, prev_pos, &samp_recvd, (rx_sig_buff+recv_idx) );
        int32_t temp;
        for (int i =0; i<samp_recvd; i++){
            temp = (double) (adc_add[(prev_pos+i)%ADC_BUFFER_SIZE]);
            rx_sig_buff[recv_idx+i] = (temp>8191)?(temp-(1<<14))/((double)(1<<13)):temp/((double)(1<<13)) + DC_ERROR;
        }

        // log the acquisition details
        fprintf(trace_fp,"RX: Read out samples = %d, Current pos = %d, Prev_pos = %d Receive Index = %d \n", samp_recvd, curr_pos, prev_pos, recv_idx);
        // calculate the acquisition time
        end1 = GetTimeStamp() - start;
        // demodulate the receive signal and save the remaining unprocessed samples
        samp_remng = ofdm_demod(rx_bin_ptr, demod_idx, samp_recvd + samp_remng, &bits_recvd);
        // update the ADC pointer position
        prev_pos = curr_pos;
        // advance the signal buffer pointer
        recv_idx += samp_recvd;
        // advance the demod pointer in the buffer
        demod_idx = recv_idx - samp_remng;
        // advance the rx binary buffer
        rx_bin_ptr += bits_recvd;
        // break if sig buffer end is reached or all frames are received
        if ( recv_idx > end_idx || rx_bin_ptr > rx_bin_end)
   	        break;

        // calculate the data processing time
        end2 = GetTimeStamp() - end1 - start;
        fprintf(trace_fp,"RX: Read time = %lfms, Process time = %lfms\n", (double)end1/1000, (double)end2/1000);
    }

    // Calculate the number of frames received
    uint32_t recvd_frms = ( (rx_bin_ptr - rx_bin_buff)/sizeof(uint8_t) )/bits_per_frame;
    fprintf(stdout,"RX: Receiver signal/binary buffer filled, num of received frames = %d\n", recvd_frms);

    uint32_t error_count[recvd_frms], rx_frm_num, tx_frm_num = 1, i ,j, missd_frms=0, valid_frms=0, invalid_frms=0, frm_diff=0;
    uint8_t *tx_bin_buff = (uint8_t *)malloc(data_bits*sizeof(uint8_t));
    uint8_t *tx_bin_ptr;
    float ber = 0.0;

    pattern_LFSR_byte(PRBS7, tx_bin_buff, data_bits);
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
                pattern_LFSR_byte(PRBS7, tx_bin_buff, data_bits);
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

