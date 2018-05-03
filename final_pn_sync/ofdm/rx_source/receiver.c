/* UWC PROJECT: OFDM Demodulation Function written for Redpitaya Hardware (ADC/DAC)
   AUTHOR: Kanhaiya Mishra, M Tech Final Year Student IIT Madras
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

// time difference in miliseconds
static double timediff_ms(struct timespec *begin, struct timespec *end){
    return (double)( (end->tv_sec - begin->tv_sec)*NANO + (end->tv_nsec - begin->tv_nsec) )/1000000;
}

FILE *trace_fp = NULL;
#if DEBUG_INFO
static float est_attn[N_FRAMES+1] = {0.0};
static uint32_t sync_indices[N_FRAMES+1] = {0};
static uint32_t sync_corr[N_FRAMES+1] = {0};
static double sync_fact[3][N_FRAMES+1] = {{0}};
static double dom_tap[3][N_FRAMES+1] = {{0}};
static complex_t qam_raw_buff[(N_FRAMES+1)*N_QAM*N_SYM] = {{0.0}};
static complex_t qam_demod_buff[(N_FRAMES+1)*N_QAM*N_SYM] = {{0.0}};
#endif

// sync symbol buffer
float ch_attn = RX_PWR_ADJ;
static complex_t sync_qam_tx[N_FFT] = {{0.0}};
// FFT Input / Output Buffers (TBD: Check for performance enhancement with global static allocation vs local dynamic allocation)
static complex_t fft_in_buff[N_FFT] = {{0.0}};
static complex_t fft_out_buff[N_FFT] = {{0.0}};
// RX Signal Buffer will be accessed by main as well as demod function
static real_t rx_sig_buff[RX_BUFF_SIZE] = {0.0};

// Gray Coding
static uint8_t gray_map[16] __attribute__((aligned(1))) = { 0, 1, 3, 2, 6, 7, 5, 4, 12, 13, 15, 14, 10, 11,  9,  8 };

// Inline function to calculate complex magnitude
static inline real_t comp_mag_sqr( complex_t num ){
    return (num.r*num.r + num.i*num.i);
}

// Inline fucntion for complex division
static inline complex_t comp_div( complex_t num, complex_t den){
    float a = comp_mag_sqr(den);
    complex_t div = {0.0};
    if (a!=0){
        div.r = (num.r*den.r + num.i*den.i)/a;
        div.i = (num.i*den.r - num.r*den.i)/a;
        return div;
    } else{
        return div;
    }
}

// Generate the pilot symbols
void generate_ofdm_sync(){

    // sync symbol uses half subcarriers as compared to data symbol
    uint8_t sync_bin[N_QAM*N_BITS/2] = {0};
    uint8_t *bin_data = sync_bin;
    // generate binary data for sync symbols
    pattern_LFSR_byte(PRBS7, bin_data, N_QAM*N_BITS/2);

    // qam modulate the binary data
    complex_t temp, *head_ptr, *tail_ptr;
    // get the header and tail pointers of the output buffer
    // head = +1 subcarrier, tail = +N_FFT(or -1) subcarrier
    head_ptr = (sync_qam_tx + 1);
    tail_ptr = (sync_qam_tx + N_FFT - 1);

    // temp variable for real and imag components of the QAM symbol
    uint32_t j, i, qam_r, qam_i;
    // max integer value of the real/imag component of the QAM symbol
    real_t qam_limit = pow(2, N_BITS/2) - 1;
    // Nomalization constant for getting average unit power per ofdm symbol
    real_t norm_const = 1/sqrt(2*(M_QAM - 1)/3);

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
    real_t norm_const = sqrt(ch_attn*(2*(M_QAM - 1)/3)*N_DSC)/N_FFT;

    #if DEBUG_INFO
    static complex_t *qam_raw_ptr=qam_raw_buff;
    static complex_t *qam_demod_ptr=qam_demod_buff;
    #endif

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

        #if DEBUG_INFO
        qam_raw_ptr->r = temp.r;
        qam_raw_ptr->i = temp.i;
        qam_demod_ptr->r = qam_sym_r;
        qam_demod_ptr->i = qam_sym_i;
        qam_raw_ptr++;
        qam_demod_ptr++;
        #endif
    }
}

// OFDM signal demodulation function receives random number of samples, first detects frame boundary
// then corrects the sync error and demodulates the signal per symbol basis depending on the #samples
uint32_t ofdm_demod(uint8_t *bin_rx, uint32_t demod_idx, int32_t samp_remng, uint32_t *bits_recvd){

    // local pointers to the global FFT input and output buffers
    complex_t *fft_out = fft_out_buff, *fft_in = fft_in_buff;
    // FFT/IFFT configuraiton data structure required for FFT/IFFT operation
    kiss_fft_cfg fft_cfg = kiss_fft_alloc(N_FFT, FALSE, NULL, NULL);
    // correlation length, window minimum length, auto correlation threshold to diff pilot and noise
    const int32_t corr_len = OSF*N_FFT/2/PRE_DSF, win_len = POST_DSF*N_CP_SYNC;
    const float  auto_corr_th = THRESHOLD*(N_FFT*POST_DSF/2);
    // corr_count, sym_count, sync_correction flag, sync complettion flag, sync index with respect to base address of the signal buffer
    static int32_t corr_count = -1, sym_count = 0, sync_corrected  = 0, sync_done= 0, sync_idx=0, frm_count=0;
    // maximum of window minimum of correlation factor, auto correlation, cross correlation, cross_correlation, auto and cross correlation at sync point
    static real_t max_of_min = 0.0, auto_corr=0.0, cros_corr=0.0, corr_fact = 0.0;
    // base index of receive data, mid index (base + corr_len), end index (base + 2*corr_len), default sync error (depends on the range of coefficients over which maxima is found)
    uint32_t idx1=0, idx2=0, idx3=0, demod_sym=0;
    // double ended queue to implement continuous window minimum using double sorted set
    // Pair Struct holds: Corr_fact and corresponding Sample index,
    static pair queue[POST_DSF*N_CP_SYNC+2];
    // Dequeue holds: base add of a static 'pair' array, front and rear index of within in the queue
    dequeue window;

    uint32_t min_samp = OSF*N_FFT;
    idx1 = demod_idx;
    idx2 = (demod_idx+N_FFT*OSF/2)%RX_BUFF_SIZE;
    idx3 = (demod_idx+N_FFT*OSF)%RX_BUFF_SIZE;
    #ifdef FLIP_OFDM
    uint32_t idx4=0, idx5=0, idx6=0;
    idx4 = (idx1+SYNC_SYM_LEN/2)%RX_BUFF_SIZE;
    idx5 = (idx2+SYNC_SYM_LEN/2)%RX_BUFF_SIZE;
    idx6 = (idx3+SYNC_SYM_LEN/2)%RX_BUFF_SIZE;
    min_samp += SYNC_SYM_LEN/2;
    #endif

    #if TRACE_PRINT
    // log the receive index and number of samples received
    fprintf(trace_fp,"RX: Sync/Demod Started, Demod Index = %d, Received Samples = %d\n", demod_idx, samp_remng);
    #endif

    // check whether sync is completed or not
    if(!sync_done){
        // chech whether sync is started or not
        if(corr_count<0){
            // if sync is just started, initialize all static variables
            sym_count = 0;
            cros_corr = 0;
            auto_corr = 0;
            memset(queue, 0, (win_len+2)*(sizeof(float)+sizeof(int)));
            initialize(&window, queue, (win_len+2) );
            // make sure received samples are more than correlation window
            if(samp_remng >= min_samp){
                // compute cross and auto correlation at first receive index
                for (int i=0; i<corr_len; i++){
                    #ifdef DCO_OFDM
                    auto_corr += rx_sig_buff[(idx2+i*PRE_DSF)%RX_BUFF_SIZE]*rx_sig_buff[(idx2+i*PRE_DSF)%RX_BUFF_SIZE];
                    cros_corr += rx_sig_buff[(idx1+i*PRE_DSF)%RX_BUFF_SIZE]*rx_sig_buff[(idx2+i*PRE_DSF)%RX_BUFF_SIZE];
                    #elif defined(FLIP_OFDM)
                    auto_corr += ( (rx_sig_buff[(idx2+i*PRE_DSF)%RX_BUFF_SIZE] - rx_sig_buff[(idx5+i*PRE_DSF)%RX_BUFF_SIZE])
                                 * (rx_sig_buff[(idx2+i*PRE_DSF)%RX_BUFF_SIZE] - rx_sig_buff[(idx5+i*PRE_DSF)%RX_BUFF_SIZE]));
                    cros_corr += ( (rx_sig_buff[(idx1+i*PRE_DSF)%RX_BUFF_SIZE] - rx_sig_buff[(idx4+i*PRE_DSF)%RX_BUFF_SIZE])
                                 * (rx_sig_buff[(idx2+i*PRE_DSF)%RX_BUFF_SIZE] - rx_sig_buff[(idx5+i*PRE_DSF)%RX_BUFF_SIZE]));
                    #endif
                }
                // compute the corellation factor (absolute ration of cross and auto correlation)
                corr_fact = fabs(cros_corr/auto_corr);
                corr_count++;
            }
        }
        if(corr_count>=0){
            // compute correlation for forward indices untill remaining samples are less than required by correlation
            for( ;samp_remng > min_samp; samp_remng-=PRE_DSF) {
                // auto correlation threshold (diff noise from pilot), corr fact threshold (reduces uncessary computation when pilot is yet present)
                // start computing window minimum once thresholds are crossed
                if( auto_corr > auto_corr_th && corr_fact>0.3 ){
                    // remove all elements from rear having higher corr factor than current corr_fact (they will never be the window minimum in future)
                    while( !empty(&window) && window.base_ptr[window.rear].value >= corr_fact)
                        dequeueR(&window);
                    // add the current element (corr_fact and position) from rear
                    enqueueR( &window, (pair){.value = corr_fact, .position = idx1} );
                    // remove all the elements from front having indices before window length from current index
                    while( (idx1 - window.base_ptr[window.front].position)%RX_BUFF_SIZE > (win_len*PRE_DSF) )
                        dequeueF(&window);
                    // check whether current corr_fact is the maximum window minimum and save its index
                    if( window.base_ptr[window.front].value > max_of_min && corr_count > win_len){
                        max_of_min = window.base_ptr[window.front].value;
                        // for current index, algorithm calculates window minimum for the index one window behind
                        sync_idx = (idx1 - (win_len*PRE_DSF))%RX_BUFF_SIZE;
                        #if DEBUG_INFO
                        // save sync parameters for debugging
                        sync_indices[frm_count] = sync_idx;
                        sync_fact[0][frm_count] = max_of_min;
                        sync_fact[1][frm_count] = auto_corr;
                        sync_fact[2][frm_count] = cros_corr;
                        #endif
                    }else if (window.base_ptr[window.front].value < 0.6*max_of_min){
                        frm_count++;
                        // consider that maximum if the current value is less 85% of the maxima and exit sync loop
                        sync_done = 1;
                        // compute remaining samples from the sync index
                        samp_remng += ((idx1 - sync_idx)%RX_BUFF_SIZE);
                        fprintf(stdout,"RX: Receiving Frame number = %d \n", frm_count);
                        break;
                    }
                }
                // compute correlation values for next index
                #ifdef DCO_OFDM
                auto_corr = auto_corr + rx_sig_buff[idx3]*rx_sig_buff[idx3] - rx_sig_buff[idx2]*rx_sig_buff[idx2];
                cros_corr = cros_corr + rx_sig_buff[idx2]*rx_sig_buff[idx3] - rx_sig_buff[idx1]*rx_sig_buff[idx2];
                #elif defined(FLIP_OFDM)
                auto_corr = auto_corr + (rx_sig_buff[idx3]-rx_sig_buff[idx6])*(rx_sig_buff[idx3]-rx_sig_buff[idx6])
                                      - (rx_sig_buff[idx2]-rx_sig_buff[idx5])*(rx_sig_buff[idx2]-rx_sig_buff[idx5]);
                cros_corr = cros_corr + (rx_sig_buff[idx2]-rx_sig_buff[idx5])*(rx_sig_buff[idx3]-rx_sig_buff[idx6])
                                      - (rx_sig_buff[idx1]-rx_sig_buff[idx4])*(rx_sig_buff[idx2]-rx_sig_buff[idx5]);
                #endif
                corr_fact = fabs(cros_corr/auto_corr);
                // increase the correlation count and advance the index (partial downsampling)
                corr_count++;
                idx1 = (idx1 + PRE_DSF)%RX_BUFF_SIZE;
                idx2 = (idx2 + PRE_DSF)%RX_BUFF_SIZE;
                idx3 = (idx3 + PRE_DSF)%RX_BUFF_SIZE;
                #ifdef FLIP_OFDM
                idx4 = (idx1+SYNC_SYM_LEN/2)%RX_BUFF_SIZE;
                idx5 = (idx2+SYNC_SYM_LEN/2)%RX_BUFF_SIZE;
                idx6 = (idx3+SYNC_SYM_LEN/2)%RX_BUFF_SIZE;
                #endif
            }
        }
    }
    if(sync_done){
        // if sync is completed do the err correction
        if (!sync_corrected){
           if ( samp_remng >= SYNC_SYM_LEN ){
                // constants: Max sync error = pre introduced synchronization error, n_cp_rem = part of cp that is removed
                const int32_t max_sync_error = floor(N_CP_SYNC/2), n_cp_rem = (N_CP_SYNC - max_sync_error);
                 // local buffers for storing time and frequency domain channel coefficients
                complex_t ht[N_FFT] = {{0.0}}, hf[N_FFT] = {{0.0}};
                // FFT/IFFT configuraiton data structure required for FFT/IFFT operation
                kiss_fft_cfg ifft_cfg = kiss_fft_alloc(N_FFT, TRUE, NULL, NULL);

                int sync_err = N_FFT/4, error=0;

                for( int i=0; i< N_FFT; i++)
                    fft_in[i].r = rx_sig_buff[(sync_idx + (n_cp_rem + i)*OSF)%RX_BUFF_SIZE];
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
                complex_t *ch_ptr = (ht+N_FFT/4+max_sync_error);
                // Set dominant tap location in the channel to 0
                float max_coef = comp_mag_sqr(ch_ptr[0]);
                // Find the dominant tap searching across pilot locations
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
                if(l_side_pk > 0.1*max_coef){
                    error = (sync_err*OSF+2);
                } else if(r_side_pk > 0.1*max_coef){
                    error = (sync_err*OSF-2);
                } else
                    error = sync_err*OSF;

                #ifdef DCO_OFDM
                ch_attn = (error%OSF==0)?(N_FFT*N_FFT*N_QAM/max_coef):ch_attn;
                #elif defined(FLIP_OFDM)
                ch_attn = (error%OSF==0)?(N_FFT*N_FFT*N_QAM/max_coef/4):ch_attn;
                #endif

                #if DEBUG_INFO
                sync_corr[frm_count-1] = error;
                dom_tap[0][frm_count-1] = l_side_pk;
                dom_tap[1][frm_count-1] = max_coef;
                dom_tap[2][frm_count-1] = r_side_pk;
                est_attn[frm_count-1] = ch_attn;
                #endif
                kiss_fft_free(ifft_cfg);
                // compute the demodulation index and remaining samples after sync correction
                demod_idx = (sync_idx + SYNC_SYM_LEN - error)%RX_BUFF_SIZE;
                samp_remng = samp_remng - SYNC_SYM_LEN + error;
                sync_corrected = 1;
            }
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
                #if TRACE_PRINT
                fprintf(trace_fp,"RX: Demodulation Completed for Frame Num %d, Symbol Count = %d\n", frm_count, sym_count+demod_sym);
                #endif
            } else{
                // if not demodulate all symbols contained within the received samples
                demod_sym = samp_remng/DATA_SYM_LEN;
            }

            // remove the cp for first symbol to be demdulated
            demod_idx = (demod_idx + N_CP_DATA*OSF)%RX_BUFF_SIZE;

	        for(int j=0; j<demod_sym; j++){
                // downsample the received data
                for( int i=0; i< N_FFT; i++){
	    	    #ifdef FLIP_OFDM
                        fft_in[i].r = rx_sig_buff[(demod_idx + i*OSF)%RX_BUFF_SIZE] - rx_sig_buff[(demod_idx + DATA_SYM_LEN/2 + i*OSF)%RX_BUFF_SIZE];
	    	    #elif defined(DCO_OFDM)
                        fft_in[i].r = rx_sig_buff[(demod_idx + i*OSF)%RX_BUFF_SIZE];
                #endif
                }

                // Compute FFT and demdoulate QAM data into binary buffer
                kiss_fft( fft_cfg, (const complex_t *)fft_in, fft_out);
                qam_demod( bin_rx, (fft_out+1));

                // get rx buffer pointer without cp for next symbol
                demod_idx = (demod_idx + DATA_SYM_LEN)%RX_BUFF_SIZE;
                // get bin buffer pointer for next symbol
                bin_rx += N_QAM*N_BITS;
	        }

            // increase the current symbol count for the frame by #demod symbols
            sym_count += demod_sym;
            // decrease the remaining samples by samples contained by demodulated symbols
            samp_remng -= demod_sym*DATA_SYM_LEN;
            // shift the demodulation index back the base from CP
            demod_idx = (demod_idx - N_CP_DATA*OSF)%RX_BUFF_SIZE;
            // free fft configuration variables
            kiss_fft_free(fft_cfg);
            // recursive call at the end of each frame when there are enough samples remaining for synchronization and/or demodulation
            if( sym_count==N_SYM && samp_remng >= min_samp){
                uint32_t new_bits = 0;
                samp_remng = ofdm_demod(bin_rx, demod_idx, samp_remng, &new_bits);
                demod_sym += (new_bits/(N_QAM*N_BITS));
            }
        }
    }

    // return the number of bits received and remaining samples
    *bits_recvd = (demod_sym*N_QAM*N_BITS);
    return samp_remng;
}


int main(int argc, char** argv){

    if(rp_Init()!=RP_OK)
        fprintf(stderr,"RX: Initialization failed");

    // received samples, remaining samples, received bits, current and previous ADC write pointer positions
    uint32_t samp_recvd = 0, bits_recvd = 0, curr_pos, prev_pos=0, recvd_frms = 1;
    // no of bits per frame and number of data bits
    uint32_t bits_per_frame = N_SYM*N_QAM*N_BITS, data_bits = bits_per_frame-FRM_NUM_BITS;
    // buffer for storing demodulated binary data (1 extra buffer to avoid segmentation fault in case of overfill)
    uint8_t *rx_bin_buff = (uint8_t *)malloc((N_FRAMES+1)*bits_per_frame*sizeof(uint8_t));
    // end of last binary buffer (last frame)
    uint8_t *rx_bin_end = rx_bin_buff + N_FRAMES*N_SYM*N_QAM*N_BITS;
    // current location of pointer in binary buffer
    uint8_t *rx_bin_ptr = rx_bin_buff;
    // demod and recv pointers within the signal buffer
    uint32_t demod_idx = 0, recv_idx = 0;
    int32_t adc_counts =0, samp_remng = 0;
    // timing variables
    struct timespec begin, end;
    // get the DAC hardware address
    static volatile uint32_t* adc_add = NULL;
    adc_add = (volatile uint32_t*)rp_AcqGetAdd(RP_CH_2);

    // File pointers: Binary Data File, BER Result File
    time_t now = time(NULL);
    FILE *bin_fp, *ber_fp;
    char log_dir[255], ber_file[255], bin_file[255];

    strftime(log_dir, 255,"./log/OFDM_%Y_%m_%d_%H_%M_%S",gmtime(&now));
    mkdir(log_dir, 0777);
    strftime(bin_file, 255,"./log/OFDM_%Y_%m_%d_%H_%M_%S/bin.txt",gmtime(&now));
    strftime(ber_file, 255,"./log/OFDM_%Y_%m_%d_%H_%M_%S/ber.txt",gmtime(&now));
    bin_fp = fopen(bin_file,"w+");
    ber_fp = fopen(ber_file,"w+");

    #if DEBUG_INFO
    FILE *sig_fp, *qam_fp, *sync_fp;
    char sig_file[255], qam_file[255], sync_file[255];
    strftime(qam_file, 255,"./log/OFDM_%Y_%m_%d_%H_%M_%S/qam.txt",gmtime(&now));
    strftime(sync_file, 255,"./log/OFDM_%Y_%m_%d_%H_%M_%S/sync.txt",gmtime(&now));
    strftime(sig_file, 255,"./log/OFDM_%Y_%m_%d_%H_%M_%S/sig.txt",gmtime(&now));
    qam_fp = fopen(qam_file,"w+");
    sync_fp = fopen(sync_file,"w+");
    sig_fp = fopen(sig_file,"w+");
    #endif

    #if TRACE_PRINT
    // timing variables for checking process and receiving time (only needed when DEBUG_INFOging)
    struct timespec t1, t2, t3;
    // log file pointer
    char trace_file[255];
    strftime(trace_file, 255,"./log/OFDM_%Y_%m_%d_%H_%M_%S/trace.txt",gmtime(&now));
    trace_fp = fopen(trace_file,"w+");
    #endif

    // check if valid RX address is acquired
    fprintf(stdout, "RX: Entered, Address = %p, adc_counts = %d\n", adc_add, adc_counts);
    // generate the pilots for synchronization error correction exactly same as in Transmit Program
    generate_ofdm_sync();
    usleep(RX_DELAY);
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
    usleep(1000);

    clock_gettime(CLOCK_MONOTONIC, &begin);
    // continue recieving untill receive signal buffer gets filled
    while( TRUE ){

        #if TRACE_PRINT
        // get the cpu clock at the start
        clock_gettime(CLOCK_MONOTONIC, &t1);
        #endif

        // get the current ADC write pointer
        rp_AcqGetWritePointer(&curr_pos);
        // calculate the samp_recvd of the data to be acquired
        samp_recvd = (curr_pos - prev_pos) % ADC_BUFFER_SIZE;

        // acquire the data into rx signal buffer from ADC buffer either using library functon or using hardware address(much faster)
//	    rp_AcqGetDataV_WA(RP_CH_2, prev_pos, &samp_recvd, rx_sig_buff, recv_idx, RX_BUFF_SIZE );
        for (int i =0; i<samp_recvd; i++){
            adc_counts = ( adc_add[(prev_pos+i)%ADC_BUFFER_SIZE] & 0x3FFF );
            adc_counts = ( (adc_counts < (MAX_COUNT/2)) ? adc_counts : (adc_counts - MAX_COUNT) );
            rx_sig_buff[(recv_idx+i)%RX_BUFF_SIZE] = ((double)adc_counts)/(MAX_COUNT/2) + DC_ERROR;
        }

        #if TRACE_PRINT
        // log the acquisition details
        fprintf(trace_fp,"RX: Recv Index = %d, Current pos = %d, Prev_pos = %d Receive Index = %d \n", recv_idx, curr_pos, prev_pos, recv_idx);
        // calculate the acquisition time
        clock_gettime(CLOCK_MONOTONIC, &t2);
        #endif

        // demodulate the receive signal and save the remaining unprocessed samples
        samp_remng = ofdm_demod(rx_bin_ptr, demod_idx, samp_recvd + samp_remng, &bits_recvd);

        // update the ADC pointer position
        prev_pos = curr_pos;
        // advance the signal buffer pointer
        recv_idx = (recv_idx + samp_recvd)%RX_BUFF_SIZE;
        // advance the demod pointer in the buffer
        demod_idx = (recv_idx - samp_remng)%RX_BUFF_SIZE;
        // advance the rx binary buffer
        rx_bin_ptr += bits_recvd;
        // break if sig buffer end is reached or all frames are received

        // get the current clock time to determine whether or not to stop
        clock_gettime(CLOCK_MONOTONIC, &end);
        // check if all frames are received or Time is over (avoid infinite loop, in case all frames were not received)
        if(rx_bin_ptr > rx_bin_end || timediff_ms(&begin, &end)> RUN_TIME ){
            // Calculate the number of frames received
            recvd_frms = ( (rx_bin_ptr - rx_bin_buff)/sizeof(uint8_t) )/bits_per_frame;
            if (rx_bin_ptr > rx_bin_end)
                fprintf(stdout,"RX: All frames received, Received %d frames in %lfsec\n", recvd_frms, timediff_ms(&begin, &end)/1000);
            else
                fprintf(stdout,"RX: Time limit reached, Received %d frames in %lfsec\n", recvd_frms, timediff_ms(&begin, &end)/1000);
            break;
        }

        #if TRACE_PRINT
        // calculate the data processing time
        clock_gettime(CLOCK_MONOTONIC, &t3);
        fprintf(trace_fp,"RX: Read time = %lfms, Process time = %lfms, Received Bits = %d, Remaining Samples = %d\n", timediff_ms(&t1, &t2), timediff_ms(&t2, &t3), bits_recvd, samp_remng);
        #endif
    }

    if(recvd_frms>0){
    // BER Evaluation Variables: RX and TX Frame numbers, loop counters, missed, valid, invalid, zero_ber frame counts, frm diff between last and latest RX frame
    uint32_t error_count[recvd_frms], rx_frm_num, tx_frm_num = 1, i ,j, missd_frms=0, valid_frms=0, invalid_frms=0, frm_diff=0;
    uint8_t *tx_bin_buff = (uint8_t *)malloc(data_bits*sizeof(uint8_t));
    uint8_t *tx_bin_ptr;
    float ber = 0.0;

    rx_bin_ptr = rx_bin_buff;
    memset(error_count, 0, recvd_frms*sizeof(uint32_t));
    pattern_LFSR_byte(PRBS7, tx_bin_buff, data_bits);
    for(i=0; i<recvd_frms; i++){
        tx_bin_ptr = tx_bin_buff;
        rx_bin_ptr = rx_bin_buff + i*bits_per_frame;
        rx_frm_num = 0;
        for(j=0; j<FRM_NUM_BITS; j++)
            rx_frm_num |= (*(rx_bin_ptr++)<<j);

        frm_diff = rx_frm_num - tx_frm_num;

        if ( frm_diff <= 10 && frm_diff >= 0 ){

            while(rx_frm_num >= tx_frm_num) {
                if(rx_frm_num==tx_frm_num){
                    for(j=0; j<(data_bits); j++)
                        error_count[i] += (*(tx_bin_ptr++) != *(rx_bin_ptr++));
                }
                pattern_LFSR_byte(PRBS7, tx_bin_buff, data_bits);
                tx_frm_num++;
            }

            valid_frms +=1;
            missd_frms += frm_diff;
            ber += (double)error_count[i];
            if(error_count[i])
                fprintf(ber_fp,"RX: Received Frame number %d with %d bit errors\n", rx_frm_num, error_count[i]);
        } else {
            pattern_LFSR_byte(PRBS7, tx_bin_buff, data_bits);
            tx_frm_num++;
            missd_frms +=1;
            invalid_frms++;
            fprintf(ber_fp,"RX: Received invalid Frame %d, Expected Frame = %d. Ignoring for BER Calculation\n", rx_frm_num, tx_frm_num);
        }
    }
    ber = ber/(valid_frms*data_bits);
    fprintf(stdout,"\nRX: Calculating BER per received frame basis\n");
    fprintf(stdout,"RX: Received total %d valid frames with BER1 = %f\n", valid_frms, ber);
    fprintf(stdout,"RX: Missed %d frames and received %d invalid frames\n", missd_frms, invalid_frms);

    fprintf(stdout,"\nRX: Calculating BER assuming no frames are missed\n");
    pattern_LFSR_reset();
    generate_ofdm_sync();
    memset(error_count, 0, recvd_frms*sizeof(uint32_t));
    ber = 0;
    // find a valid frm_num (1-10)
    tx_frm_num=1;
    rx_bin_ptr = rx_bin_buff;
    tx_bin_ptr = tx_bin_buff;
    pattern_LFSR_byte(PRBS7, tx_bin_buff, data_bits);
    fprintf(stdout,"RX: Searching First valid received frame\n");
    while(TRUE){
        rx_frm_num = 0;
        for(j=0; j<FRM_NUM_BITS; j++)
            rx_frm_num |= (*(rx_bin_ptr++)<<j);
        if(rx_frm_num<100){
            fprintf(stdout,"RX: First valid received frame is %d\n", rx_frm_num);
            break;
        } else if(rx_bin_ptr-rx_bin_buff > 100*bits_per_frame){
            rx_bin_ptr = rx_bin_buff+FRM_NUM_BITS;
            fprintf(stdout,"RX: Could not find any valid frame, setting first valid frame to be 1\n");
            rx_frm_num = 1;
            break;
        }
        rx_bin_ptr += data_bits;
    }
    if(rx_frm_num>=1){
    while(tx_frm_num!=rx_frm_num){
        pattern_LFSR_byte(PRBS7, tx_bin_buff, data_bits);
        tx_frm_num++;
    }
    for (i=0; i<=(recvd_frms-rx_frm_num); i++){
        for(j=0; j<(data_bits); j++)
            error_count[i] += (*(tx_bin_ptr++) != *(rx_bin_ptr++));
        rx_bin_ptr+= FRM_NUM_BITS;
        tx_bin_ptr = tx_bin_buff;
        ber += (double)error_count[i];
        pattern_LFSR_byte(PRBS7, tx_bin_buff, data_bits);
    }
    ber = ber/(i*data_bits);
    fprintf(stdout,"RX: Received total %d valid frames with BER2 = %f\n", i, ber);
    }
    #if DEBUG_INFO
//    if(ber>0.0 || missd_frms>0){
    // Save post-processed and online received data
    if(ber > 0.0){
        // save demodulated data
        for(i = 0; i <recvd_frms*bits_per_frame; i++){
            fprintf(bin_fp," %d \n", rx_bin_buff[i]);
        }
    }
    for(i = 0; i <recvd_frms; i++){
        fprintf(sync_fp," %d, %d, \t%lf, \t%lf, \t%lf, \t%lf", sync_indices[i]+616, sync_corr[i], dom_tap[0][i], dom_tap[1][i], dom_tap[2][i], est_attn[i]);
        fprintf(sync_fp," \t%lf, \t%lf, \t%lf\n", sync_fact[0][i], sync_fact[1][i], sync_fact[2][i]);
    }
    for(i = 0; i <recvd_frms*N_SYM*N_QAM; i++){
        fprintf(qam_fp," %f + %fj \t", qam_raw_buff[i].r, qam_raw_buff[i].i);
        fprintf(qam_fp," %f + %fj \n", qam_demod_buff[i].r, qam_demod_buff[i].i);
    }
    for(i = 0; i <recv_idx; i++){
        fprintf(sig_fp," %f \n", rx_sig_buff[i]);
    }
//    }
    #endif
    }

    fclose(ber_fp);
    fclose(bin_fp);

    #if DEBUG_INFO
    // close files
    fclose(qam_fp);
    fclose(sync_fp);
    fclose(sig_fp);
    #endif

    #if TRACE_PRINT
    fclose(trace_fp);
    #endif

    // Stop acquisition and release resources
    free(rx_bin_buff);
    rp_AcqStop();
    rp_Release();
    fprintf(stdout,"RX: Acquisition Comeplete, Exiting.\n");
    return 0;
}

