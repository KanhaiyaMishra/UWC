#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include "redpitaya/rp.h"
#include "prbs.h"
#include "ofdm.h"
#include "kiss_fft.h"

#define N_FRAMES 100
#define MAX_COUNT (1<<14)

static real_t sync_sym[SYNC_SYM_LEN] = {0.0};
static complex_t ifft_in_buff[N_FFT] = {{0.0}};
static complex_t ifft_out_buff[N_FFT] = {{0.0}};
static uint8_t gray_map[16] __attribute__((aligned(1))) = { 0, 1, 3, 2, 6, 7, 5, 4, 12, 13, 15, 14, 10, 11,  9,  8 };

uint64_t GetTimeStamp(){
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

void qam_mod(complex_t *qam_data, uint8_t *bin_data, uint8_t is_sync_sym){

    // temp head and tail pointers for the output buffer
	complex_t temp, *head_ptr, *tail_ptr;
    // temp variable for real and imag components of the QAM symbol
	uint32_t j, i, qam_r, qam_i;
    // max integer value of the real/imag component of the QAM symbol
	real_t qam_limit = pow(2, N_BITS/2) - 1;
    // Nomalization constant for getting average unit power per ofdm symbol
    // Avg Energy = 2*(M_QAM - 1)/3), #QAM per OFDM Sym = N_DSC (N_DSC/2 for sync)
    // Extra factor of 4 to get Vpp less than 2 (DAC Amplitude Limitation)
	real_t norm_const = 1/sqrt( 16*2*(M_QAM - 1)/3*N_DSC/(1+is_sync_sym) );

	// get the header and tail pointers of the output buffer
    // OFDM Sym = [ DC QAM[N_DSC] Zeros[N_FFT-N_DSC-1] conj(flip(QAM[N_DSC]))]
	head_ptr = qam_data + 1;
	tail_ptr = qam_data + N_FFT - 1;

	for(i=0; i<N_QAM; i++)
	{
		// reset real and imag parts
		qam_r = 0;
		qam_i = 0;

		// separate the real and imag bits and convert to decimal for gray coding
        // e.g. 16QAM Binary - 1011, Real (first half) = 2(10), Imag (second half) = 3(11)
        // Gray Mapping: Real = (2->3), Imag = (3->2), Signal Conversion = (2*3-3) - (2*2-3)j = 3-1j
		for (j=1; j<=N_BITS/2; j++){
			qam_r |= ( (*(bin_data + N_BITS/2 - j) ) << (j-1) );
			qam_i |= ( (*(bin_data + N_BITS - j) ) << (j-1) );
		}

		temp.r =  (2*gray_map[qam_r] -qam_limit)*norm_const;
		temp.i = -(2*gray_map[qam_i] -qam_limit)*norm_const;

        // advance the bin pointer to next symbol
		bin_data += N_BITS;

		// write qam data with hermitian symmetry: head_ptr++ = a+bj, tail_ptr-- = a-bj
        if(is_sync_sym){
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
        else {
        // data symbols use all data subcarriers (except DC)
    		(head_ptr)->r = temp.r;
    		(head_ptr++)->i = temp.i;
    		(tail_ptr)->r = temp.r;
    		(tail_ptr--)->i = -temp.i;
        }
	}
}

void ofdm_mod(real_t *ofdm_tx_cp, uint8_t *bin_tx) {

    // get the first ofdm symbol pointer without cp
    real_t *ofdm_tx = ofdm_tx_cp + N_CP_DATA*OSF + SYNC_SYM_LEN;
    // pointers for ifft input and output buffers
    complex_t *ifft_out = ifft_out_buff, *ifft_in = ifft_in_buff;
    // fft state config parameter allocation
    kiss_fft_cfg ifft_cfg = kiss_fft_alloc( N_FFT, TRUE, NULL, NULL );

    // insert sync symbol
    for(int j=0; j<SYNC_SYM_LEN; j++)
        *(ofdm_tx_cp++) = sync_sym[j];

    // insert data symbols
    for(int j=0; j<N_SYM; j++){
        // Hermitian symmetric qam data preparation
        qam_mod( ifft_in, bin_tx, FALSE);

        // ifft of hermitian symmetric data
        kiss_fft( ifft_cfg, (const complex_t *)ifft_in, ifft_out);

        // save ifft data into the output buffer
        for( int i=0; i<N_FFT; i++){
            // oversampling the output
            for( int k=0; k<OSF; k++){
            #ifdef DCO_OFDM
					*ofdm_tx++ = ifft_out->r;

            #elif defined(FLIP_OFDM)
                // flip ofdm +ve and -ve symbol separation
                if (ifft_out->r < 0.0)
					*(ofdm_tx + DATA_SYM_LEN) = -(ifft_out->r);
				else
					*ofdm_tx++ = ifft_out->r;
            #endif
			}
            // advance the buffer pointer
    		ifft_out++;
		}

		// retreive cp location
		ofdm_tx -= (N_CP_DATA*OSF);

		// add cp to both +ve and -ve ofdm symbols
		for(int i=0; i<(N_CP_DATA*OSF); i++){
        #ifdef FLIP_OFDM
			*(ofdm_tx_cp + DATA_SYM_LEN) = *(ofdm_tx + DATA_SYM_LEN);
			*ofdm_tx_cp++ = *ofdm_tx++;
        #elif defined(DCO_OFDM)
			*ofdm_tx_cp++ = *ofdm_tx++;
        #endif
	    }

        #ifdef DCO_OFDM
		// advance the sig buffer pointer
		ofdm_tx_cp = ofdm_tx;
		// get sig buffer pointer with cp
		ofdm_tx = ofdm_tx_cp + N_CP_DATA*OSF;

        #elif defined(FLIP_OFDM)
		// advance the sig buffer pointer
		ofdm_tx_cp = ofdm_tx + DATA_SYM_LEN;
		// get sig buffer pointer with cp
		ofdm_tx = ofdm_tx_cp + N_CP*OSF + DATA_SYM_LEN;
        #endif

		// get bin data pointer for next symbol
		bin_tx += (N_QAM*N_BITS);
		// Reinitialize IFFT output buffer
		ifft_out = ifft_out_buff;
	}
	kiss_fft_free(ifft_cfg);
}

void generate_ofdm_sync(){

    int i, k;
    // sync sequence binary data holder
    uint8_t *sync_bin = malloc(N_DSC*sizeof(uint8_t)/2);
    // CP length and symbol length
    complex_t *ifft_out = ifft_out_buff, *ifft_in = ifft_in_buff;
    // pointer to the ofdm symbol buffer with and without CP
    real_t *sync_sym_ptr = sync_sym + (N_CP_SYNC*OSF), *sync_sym_cp_ptr = sync_sym;
    // configuration variable for kiss IFFT library
    kiss_fft_cfg ifft_cfg = kiss_fft_alloc( N_FFT, TRUE, NULL, NULL );
    // generate binary data for sync sequence
    pattern_LFSR_byte(PRBS7, sync_bin, N_DSC/2);

    // qam modulate the binary data: [N_FFT] = [DC QAM_DATA (N_GAURD/2-1) conj(flip(QAM_DATA))]
    qam_mod( ifft_in, sync_bin, TRUE);

    // Take IFFT to get the time domain ofdm symbol
    kiss_fft( ifft_cfg, (const complex_t *)ifft_in, ifft_out);

    // Oversample and save the symbol for later use (Note: IFFT output only has real components)
    for( i=0; i<N_FFT; i++){
        for( k=0; k<OSF; k++){
        // separate positive and negative for FLIP OFDM
        #ifdef FLIP_OFDM
            if (ifft_out->r > 0.0)
                *sync_sym_ptr = ifft_out->r;
            else
                *(sync_sym_ptr + sym_len) = -(ifft_out->r);
        #else
        // write the data as it is for DCO OFDM
            *sync_sym_ptr = ifft_out->r;
        #endif
            sync_sym_ptr++;
        }
        ifft_out++;
    }

    // add the CP to the sync symbol
    sync_sym_ptr-= (N_CP_SYNC*OSF);
    for( int i=0; i<(N_CP_SYNC*OSF); i++){
    #ifdef FLIP_OFDM
        *(sync_sym_cp_ptr + SYNC_SYM_LEN) = *(sync_sym_ptr + SYNC_SYM_LEN);
        *(sync_sym_cp_ptr++) = *(sync_sym_ptr++);
    #else
        *(sync_sym_cp_ptr++) = *(sync_sym_ptr++);
    #endif
    }
    kiss_fft_free(ifft_cfg);
    // return the number of bits per frame
}

int main(int argc, char **argv){

	if(rp_Init() != RP_OK)
		printf("Initialization Failed");

	uint64_t start = 0, end=0;
	real_t freq = 125e6/(16384*64);
	uint32_t period = round(1e6/freq), frm_num, i, pos;
    real_t *tx_sig_ptr = (real_t *)malloc(2*ADC_BUFFER_SIZE*sizeof(real_t));
    uint8_t *tx_bin_ptr = (uint8_t *)malloc(N_SYM*N_QAM*N_BITS*sizeof(uint8_t));
    static volatile int32_t* dac_add;

    // get the DAC hardware address
    dac_add = (volatile int32_t*)rp_GenGetAdd(RP_CH_2);
	fprintf(stdout,"TX: Entered, total bits =%d, DAC Address = %p\n", N_BITS*N_QAM*N_SYM, dac_add);
    // reset the tx signal buffer
    memset(tx_sig_ptr, 0, ADC_BUFFER_SIZE*(sizeof(real_t)));
	// DAC Output Settings
    rp_GenWaveform(RP_CH_2, RP_WAVEFORM_ARBITRARY);
    // set the maximum amplitude of the signal
    rp_GenAmp(RP_CH_2, 1);
	// 1.9Msps frequency (16384 samples) per period
    rp_GenFreq(RP_CH_2, freq);
	// Continuous waveform burst mode
    rp_GenMode(RP_CH_2, RP_GEN_MODE_BURST);
	// Single waveform per burst
    rp_GenBurstCount(RP_CH_2, 1);
	// No Repetition of Burst
    rp_GenBurstRepetitions(RP_CH_2, 1);
	// Waveform time period in micro seconds (16384/1.9Msps)
    rp_GenBurstPeriod(RP_CH_2, period);
    // initialize ofdm sync sequence
    generate_ofdm_sync();

    start = GetTimeStamp();
	for(frm_num=1; frm_num<=N_FRAMES; frm_num++){
        rp_GenGetReadPointer(&pos, RP_CH_2);

        for(i=0; i<FRM_NUM_BITS; i++)
            tx_bin_ptr[i] = ((frm_num>>i)&1);

		pattern_LFSR_byte(PRBS7, tx_bin_ptr+FRM_NUM_BITS, N_SYM*N_QAM*N_BITS-FRM_NUM_BITS);

		ofdm_mod(tx_sig_ptr, tx_bin_ptr);

        for(i=0; i<ADC_BUFFER_SIZE;){
            rp_GenGetReadPointer(&pos, RP_CH_2);
            pos = ((pos==0)?(ADC_BUFFER_SIZE):pos);
            for(;i<pos;i++)
                dac_add[i] = ((int32_t)(tx_sig_ptr[i]*MAX_COUNT/2 + 0.5*(2*(tx_sig_ptr[i]>0)-1)) & (MAX_COUNT-1));
        }
		rp_GenOutEnable(RP_CH_2);
		printf("TX: Transmitting Frame Num = %d\n",frm_num);
	}
	usleep(period);
   	end = GetTimeStamp()-start;
	fprintf(stdout,"TX: Transmitted %d Frames in %lf ms\n", N_FRAMES, (double)end/1000);
	rp_GenOutDisable(RP_CH_2);

    FILE *fp;
    fp = fopen("./data.txt","w+");
    for(i=0; i<ADC_BUFFER_SIZE; i++)
        fprintf(fp,"%f\n",tx_sig_ptr[i]);


//  Releasing resources
    free(tx_sig_ptr);
	free(tx_bin_ptr);
	rp_Release();
	fprintf(stdout,"TX: Transmission Completed, Exiting TX.\n");
    return 0;
}
/*
        // Calculate symbol power to verify
        Ps = 0.0;
        dot_product( (real_t*)ifft_out, (real_t*)ifft_out, 2*N_FFT, &Ps);
        max = maximum( (real_t*)ifft_out, 2*N_FFT);
        min = minimum( (real_t*)ifft_out, 2*N_FFT);
        fprintf(stdout,"Sym Power = %f, Max, Min Amp = %f, %f\n",Ps, max, min);
*/
/*
static inline complex_t conjugate(complex_t num){
	num.i = -num.i;
	return num;
}

static real_t dot_product(real_t* arr1, real_t* arr2, uint32_t size){
    real_t temp=0.0;
    for(int i=0; i<size; i++)
		temp += (*arr1++)*(*arr2++);
    return temp;
}

static real_t maximum(real_t* arr, uint32_t size){
	real_t max=0.0;
	for(int i=0; i<size; i++){
		max = max*( max > *arr) + (*arr)*( *arr > max);
		arr++;
	}
	return max;
}

static real_t minimum(real_t* arr, uint32_t size){
	real_t min=0.0;
	for(int i=0; i<size; i++){
		min = min*( min < *arr) + (*arr)*( *arr < min);
		arr++;
	}
	return min;
}
*/
