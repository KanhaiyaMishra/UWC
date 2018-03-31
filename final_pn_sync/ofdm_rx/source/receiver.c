#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include "prbs.h"
#include "ofdm.h"
#include "deque.h"
#include "kiss_fft.h"
#include "redpitaya/rp.h"

#define N_FRAMES 1

static complex_t sync_qam_tx[N_FFT] = {{0.0}};
static complex_t fft_in_buff[N_FFT] = {{0.0}};
static complex_t fft_out_buff[N_FFT] = {{0.0}};
static uint8_t gray_map[16] __attribute__((aligned(1))) = { 0, 1, 3, 2, 6, 7, 5, 4, 12, 13, 15, 14, 10, 11,  9,  8 };

static inline real_t comp_mag_sqr( complex_t num ){
    return (num.r*num.r + num.i*num.i);
}

static inline complex_t comp_div( complex_t num, complex_t den){
    complex_t div;
    div.r = (num.r*den.r + num.i*den.i)/comp_mag_sqr(den);
    div.i = (num.i*den.r - num.r*den.i)/comp_mag_sqr(den);
    return div;
}

static real_t dot_product(real_t* arr1, real_t* arr2, uint32_t size, uint32_t offset){
    real_t result=0.0;
    for(int i=0; i<size; i++)
        result += arr1[i*offset]*arr2[i*offset];
    return result;
}

void qam_mod ( complex_t *qam_data, uint8_t *bin_data, uint8_t is_sync_sym)
{
    complex_t temp, *head_ptr, *tail_ptr;
    // temp variable for real and imag components of the QAM symbol
    uint32_t j, i, qam_r, qam_i;
    // max integer value of the real/imag component of the QAM symbol
    real_t qam_limit = pow(2, N_BITS/2) - 1;
    // Nomalization constant for getting average unit power per ofdm symbol
    real_t norm_const = 1/sqrt( 16*2*(M_QAM - 1)/3*N_DSC/(1+is_sync_sym) );

    // get the header and tail pointers of the output buffer
    head_ptr = qam_data + 1;
    tail_ptr = qam_data + N_FFT - 1;

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

        temp.r =  (2*gray_map[qam_r] -qam_limit)*norm_const;
        temp.i = -(2*gray_map[qam_i] -qam_limit)*norm_const;

        // advance the bin pointer to next symbol
        bin_data += N_BITS;

        // sync symbols only used odd data subcarriers
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

void generate_ofdm_sync(){

    // sync sequence binary data holder
    uint8_t *sync_bin = malloc(N_DSC*sizeof(uint8_t)/2);
    // generate binary data for sync sequence
    pattern_LFSR_byte(PRBS7, sync_bin, N_DSC/2);
    // qam modulate the binary data: [N_FFT] = [DC QAM_DATA (N_GAURD/2-1) conj(flip(QAM_DATA))]
    qam_mod( sync_qam_tx, sync_bin, TRUE);
}

void qam_demod(uint8_t *bin_data, complex_t *qam_data){

	complex_t temp;
	uint32_t i, j, qam_sym_r, qam_sym_i;
	real_t qam_limit = pow(2, N_BITS/2) - 1;
	real_t norm_const = sqrt( 16*(2*(M_QAM - 1)/3)*N_DSC)/N_FFT;

	for(i=0; i<N_QAM; i++)
	{
		//extract the qam data from input buffer
	  	temp.r = qam_data->r*norm_const;
	  	temp.i = -qam_data->i*norm_const;

		// Hard Limit Data Within QAM Symbol Boundaries
		if (temp.r > qam_limit)
			temp.r	= qam_limit;
		else if (temp.r < -qam_limit)
			temp.r	= -qam_limit;
		if (temp.i > qam_limit)
			temp.i	= qam_limit;
		else if (temp.i < -qam_limit)
			temp.i	= -qam_limit;

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

uint32_t ofdm_demod(uint8_t *bin_rx, real_t *sig_rx, uint32_t samp_remng, uint32_t *bits_recvd){

	complex_t *fft_out = fft_out_buff, *fft_in = fft_in_buff;
    complex_t ht[N_FFT] = {{0.0}}, hf[N_FFT] = {{0.0}};
	kiss_fft_cfg fft_cfg = kiss_fft_alloc(N_FFT, FALSE, NULL, NULL);
	kiss_fft_cfg ifft_cfg = kiss_fft_alloc(N_FFT, TRUE, NULL, NULL);
    const int32_t max_sync_error = floor(N_CP_SYNC/2), n_cp_rem = (N_CP_SYNC - max_sync_error);
    const int32_t corr_len = OSF*N_FFT/2/PRE_DSF, win_len = POST_DSF*N_CP_SYNC;
    static int32_t corr_count = -1, sym_count = 0, sync_corrected  = 0, sync_done= 0;
    static real_t auto_corr=0.0, cros_corr=0.0, corr_fact = 0.0, corr_th = 0.40, max_of_min = 0.0;
    static dequeue window;
    uint32_t idx=0, sync_point, demod_sym, sync_correction = N_FFT/4;
    real_t *sig_ptr1, *sig_ptr2, *sig_ptr3;
    complex_t *ch_ptr;
    sig_ptr1 = sig_rx;
    sig_ptr2 = sig_rx+OSF*N_FFT/2;
    sig_ptr3 = sig_rx+OSF*N_FFT;

    if(!sync_done){
        if(corr_count<0){
            fprintf(stdout," Sync Started, Rx Address = %p, Remaininig Samples = %d\n",sig_rx, samp_remng);
            sym_count = 0;
            initialize(&window);
            if(samp_remng >= OSF*N_FFT){
                auto_corr = dot_product( sig_ptr2, sig_ptr2, corr_len, PRE_DSF);
                cros_corr = dot_product( sig_ptr1, sig_ptr2, corr_len, PRE_DSF);
                corr_fact = fabs(cros_corr/auto_corr);
                corr_count++;
            } else
                fprintf(stdout,"Sync not completed samples not enough, remaining samples = %d\n", samp_remng);
        }
        if(corr_count>=0){
            for( ;samp_remng >= PRE_DSF; samp_remng-=PRE_DSF) {

                while( !empty(&window) && window.data[window.rear].value >= corr_fact)
                    dequeueR(&window);
                enqueueR( &window, (pair){.value = corr_fact, .position = corr_count} );
                while(window.data[window.front].position < (corr_count-win_len) )
                    dequeueF(&window);

                if( window.data[window.front].value > max_of_min  && corr_count >= win_len ){
                    max_of_min = window.data[window.front].value;
                    sync_point = corr_count - win_len;
//                    fprintf(stdout,"max of window minimum = %f, location = %d\n", max_of_min, sync_point);
                } else if ( window.data[window.front].value < 0.5*max_of_min && max_of_min > corr_th){
                    sync_done = 1;
                    break;
                }
                auto_corr = auto_corr + sig_ptr3[idx]*sig_ptr3[idx] - sig_ptr2[idx]*sig_ptr2[idx];
                cros_corr = cros_corr + sig_ptr2[idx]*sig_ptr3[idx] - sig_ptr1[idx]*sig_ptr2[idx];
                corr_fact = fabs(cros_corr/auto_corr);
                corr_count++;
                idx = corr_count*PRE_DSF;
            }
            if(sync_done){
                samp_remng = samp_remng + corr_count*PRE_DSF - sync_point*PRE_DSF;
                sig_rx += sync_point*PRE_DSF;
                fprintf(stdout,"Sync completed, sync_point = %d, max of min = %f, corr_count = %d, remaining samples = %d\n", sync_point, max_of_min, corr_count, samp_remng);
            } else
                fprintf(stdout,"Sync not completed, remaining samples = %d\n", samp_remng);
        }
    }
    if(sync_done){
        if (!sync_corrected){
            if ( samp_remng >= (N_FFT+n_cp_rem)*OSF ){
                // correct for sync error
                sig_rx += n_cp_rem*OSF;
    		    for( int i=0; i< N_FFT; i++){
	    	    #ifdef FLIP
		    	    fft_in[i].r = *(sig_rx + i*OSF + OSF/2);
    	        #elif defined(DCO)
		    	    fft_in[i].r = *(sig_rx + i*OSF + OSF/2);
		        #endif
		        }

    		    kiss_fft( fft_cfg, (const complex_t *)fft_in, fft_out);

               // calculate channel coefficients (only at odd subcarriers)
                for (int i = 1; i<N_QAM; i+=2){
                    hf[i] = comp_div( fft_out[i], sync_qam_tx[i]);
                    hf[N_FFT-i] = comp_div( fft_out[N_FFT-i], sync_qam_tx[N_FFT-i]);
                }

    		    kiss_fft( ifft_cfg, (const complex_t *)hf, ht);

                ch_ptr = ht+N_FFT/4+max_sync_error;
                float max_coef = comp_mag_sqr(ch_ptr[0]);
                for( int i=1; i<N_FFT/2; i++) {
                    if( max_coef < comp_mag_sqr(ch_ptr[i]) ){
                        max_coef = comp_mag_sqr(ch_ptr[i]);
                        sync_correction = N_FFT/4-i;
                    }
                }
                sig_rx = sig_rx + (N_FFT + max_sync_error - sync_correction)*OSF;
                samp_remng -= SYNC_SYM_LEN - sync_correction*OSF;
                sync_corrected = 1;
                fprintf(stdout,"Sync correction done by %d samples, max_coef = %f, rx address = %p\n", sync_correction*OSF, max_coef, sig_rx);
            } else
                fprintf(stdout,"Demoduation not completed, sync symbol not received\n");
        }
        if (sync_corrected){
            if ( samp_remng >= (N_SYM-sym_count)*DATA_SYM_LEN ){
                demod_sym = N_SYM - sym_count;
                corr_count = -1;
                sync_done = 0;
                sync_corrected = 0;
            } else
                demod_sym = samp_remng/DATA_SYM_LEN;

            sig_rx += N_CP_DATA*OSF;

            // downsample the received data
	        for(int j=0; j<demod_sym; j++){
		        for( int i=0; i< N_FFT; i++){
	    	    #ifdef FLIP
		    	    fft_in[i].r = *(sig_rx + i*OSF + OSF/2) - *(sig_rx + DATA_SYM_LEN + i*OSF + OSF/2);
	    	    #elif defined(DCO)
		    	    fft_in[i].r = *(sig_rx + i*OSF + OSF/2);
		        #endif
		        }

        		kiss_fft( fft_cfg, (const complex_t *)fft_in, fft_out);
	        	qam_demod( bin_rx, (fft_out+1));

    	    	// get rx buffer pointer without cp for next symbol
	        	sig_rx += DATA_SYM_LEN;
		        // get bin buffer pointer for next symbol
		        bin_rx += N_QAM*N_BITS;

    	    	// FLIP decodes two ofdm symbols together, so advance pointer once more
        	    #if defined(FLIP)
		        ofdm_rx += DATA_SYM_LEN;
	    	    j++;
	            #endif
	        }
            sym_count += demod_sym;
            samp_remng = samp_remng%DATA_SYM_LEN;
            *bits_recvd = demod_sym*N_QAM*N_BITS;
            fprintf(stdout,"Demoduation completed, sym_count = %d, remaining samples = %d, received bits = %d\n", sym_count, samp_remng, *bits_recvd);
	        kiss_fft_free(fft_cfg);
	        kiss_fft_free(ifft_cfg);
        }
    }
    return samp_remng;
}


int main(int argc, char** argv){

	if(rp_Init()!=RP_OK)
		fprintf(stderr,"RX: Initialization failed");

    // received samples, remaining samples, received bits, current and previous ADC ptr pos
	uint32_t samp_recvd = 0, samp_remng =0, bits_recvd = 0, curr_pos, prev_pos=0;
    // receive binary buffer (one extra buffer to take care of spillage while checking end of buffer)
	uint8_t *rx_bin_start = (uint8_t *)malloc((N_FRAMES+1)*N_SYM*N_QAM*N_BITS*sizeof(uint8_t));
    // end of last binary buffer (last frame)
	uint8_t *rx_bin_end = rx_bin_start + N_FRAMES*N_SYM*N_QAM*N_BITS;
    // current location of pointer in binary buffer
	uint8_t *rx_bin_ptr = rx_bin_start;
    // receive signal buffer (one extra buffer for the case when current read samples spill out
    // of the first buffer. In next read cycle, pointer is reset to start of the first buffer)
	float *rx_sig_start = (float *)malloc(12*ADC_BUFFER_SIZE*sizeof(float));
    // end of first receive buffer
	float *rx_sig_end = rx_sig_start + 12*ADC_BUFFER_SIZE;
    // current location of pointer in signal buffer
	float *rx_sig_ptr = rx_sig_start;

    // timing variables
    clock_t start=0, end1=0, end2 = 0;
	fprintf(stdout, "RX: Entered\n");
    // generate the pilots for synchronization error correction
    // pilots are generated exactly same as in Transmit Program
    generate_ofdm_sync();

	FILE *fp1, *fp2;
	fp1 = fopen("./bin.txt","w+");
	fp2 = fopen("./sig.txt","w+");
/*	fp2 = fopen("./data.txt","r");
    for (int i=0; i<ADC_BUFFER_SIZE+10; i++)
        fscanf(fp2,"%f", rx_sig_start+i);
*/

    // wait till transmission is started
	usleep(500000);
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
        start = clock();
        // get the current ADC write pointer
		rp_AcqGetWritePointer(&curr_pos);
        // calculate the samp_recvd of the data to be acquired
		samp_recvd = (curr_pos - prev_pos) % ADC_BUFFER_SIZE;
        // acquire the data into rx signal buffer from hardware ADC buffer
	    rp_AcqGetDataV(RP_CH_1, prev_pos, &samp_recvd, rx_sig_ptr);
		fprintf(stdout,"RX: Read out samples = %d, Current pos = %d, Prev_pos = %d, bits received = %d \n", samp_recvd, curr_pos, prev_pos, bits_recvd);
        // calculate the acquisition time
        end1 = clock() - start;
        // demodulate the receive signal and save the remaining unprocessed samples
		samp_remng = ofdm_demod(rx_bin_ptr, rx_sig_ptr-samp_remng, samp_recvd+samp_remng, &bits_recvd);
        // update the ADC pointer position
		prev_pos = curr_pos;
        // advance the signal buffer pointer
        rx_sig_ptr += samp_recvd;
        // advance the rx binary buffer
		rx_bin_ptr += bits_recvd;

        if (rx_sig_ptr >= rx_sig_end){
            for(int i=0; i<samp_remng; i++)
                rx_sig_start[i] = *(rx_sig_ptr-samp_remng+i);
            rx_sig_ptr = rx_sig_start+samp_remng;
            fprintf(stdout,"crossed boundary\n");
        }
        // check if bin buffer end is reached
		if ( rx_bin_ptr >= rx_bin_end)
			break;

        // calculate the data processing time
        end2 = clock() - end1 - start;
        fprintf(stdout,"RX: Read time = %lf, Process time = %lf\n", (double)end1/CLOCKS_PER_SEC, (double)end2/CLOCKS_PER_SEC);
	}


    // save demodulated data
    for(int i = 0; i <N_FRAMES*N_SYM*N_DSC; i++){
        fprintf(fp1," %d \n", *(rx_bin_start+i));
    }
    for(int i = 0; i < (rx_sig_ptr-rx_sig_start)/sizeof(float); i++){
        fprintf(fp2," %f \n", *(rx_sig_start+i));
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

/*	FILE *fp2;
	fp2 = fopen("./data.txt","r");
	FILE *fp2;
	fp1 = fopen("./bin.txt","w+");


    start = clock();
    samp_remng = ofdm_demod(rx_bin_start, rx_sig_start, samp_recvd, &bits_recvd);

    end1 = clock() - start;
    fprintf(stdout,"RX: Read time = %lf, Process time = %lf\n", (double)end1/CLOCKS_PER_SEC, (double)end2/CLOCKS_PER_SEC);

    fprintf(stdout,"samp remaining = %d\n", samp_remng);
    for (int i=0; i<N_SYM*N_QAM*N_BITS; i++)
        fprintf(fp1,"%d\n", rx_bin_start[i]);

    fclose(fp2);
    fclose(fp1);
*/
