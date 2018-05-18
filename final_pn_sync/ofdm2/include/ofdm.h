#ifndef __OFDM_H
#define __OFDM_H

#include <inttypes.h>
#include "kiss_fft.h"
#define TRUE 1
#define FALSE 0
#define N_FRAMES 1000
#define DEBUG_FRAMES 50

typedef kiss_fft_cpx complex_t;
typedef kiss_fft_scalar real_t;

// Transmit Power adjustment to keep signal within +/-1V range (Without Adjustment OFDM output will vary +/-4V)
// Amplitude after adjustment = Actual Max/sqrt(PWR_ADJ)
#define PWR_ADJ 120.0f
// QAM16 Pilot Power = 160/20/(Channel Attenuation), QAM4 Pilot Power = 128/20/(Channel Attenuation)
// Noise Power = (Channel Noise Level)^2*128. For Back to back channel noise level = (10^-5)
// Based on expected attenuation from channel threshold will be set, Threshold must be less than Pilot Power
// Auto Correlation Threshold to distinguish between pilot and noise. Actual Threshold Level = 128/THRESHOLD.
#define THRESHOLD 0.00001f
#define OSF 8               // TX over sampling factor
#define PRE_DSF 2           // RX downsampling before synchronization
#define POST_DSF 4          // RX downsampling after synchronization
#define M_QAM 16            // QAM order
#define N_BITS 4            // bits per qam symbol
#define N_FFT 64            // Total Subcarriers (FFT Size)
#define N_DSC 52            // Data Subcarriers
#define N_QAM 26            // qam syms per ofdm symbol
#define N_CP_DATA 9         // CP length for symbols with data
#define FRM_NUM_BITS 16     // bits for frame no info (start of each frame)

///! choose whether DCO or FLIP OFDM
#define DCO_OFDM            // OFDM Type: DCO/FLIP

#if (M_QAM==4)
#define SIGDATA "./sigqam4.txt"
#elif (M_QAM==16)
#define SIGDATA "./sigqam16.txt"
#endif

#ifdef DCO_OFDM
#define N_CP_SYNC 13        // CP length for symbol with sync info
#define N_SYM 27            // OFDM Symbols Carrying Data
#define SYNC_SYM_LEN 616    // OFDM Sync Symbol: (64+13)*8 samples
#define DATA_SYM_LEN 584    // OFDM Data Symbol: (64+9)*8 samples
#else
#define N_CP_SYNC 11         // CP length for symbol with sync info
#define N_SYM 13             // OFDM Symbols Carrying Data
#define SYNC_SYM_LEN 1200    // OFDM Sync Symbol: (64+11)*8*2 samples
#define DATA_SYM_LEN 1168    // OFDM Data Symbol: (64+9)*8*2 samples
#endif

#define DAC_CHANNEL RP_CH_1
#define ADC_CHANNEL RP_CH_2
#define FRM_DUR 8.5
#define RX_DELAY 100000
#define RUN_TIME ((N_FRAMES*FRM_DUR+40))
#define DC_ERROR 0.0147
#define NANO 1000000000LL
#define MAX_COUNT (1<<14)
#define DEBUG_INFO (DEBUG_FRAMES>N_FRAMES)
#define TRACE_PRINT FALSE
#define RX_BUFF_SIZE (64*ADC_BUFFER_SIZE)
#define FRM_SAMP (ADC_BUFFER_SIZE/OSF)

/*! Each OFDM Symbol contains one OFDM Symbol carrying Synchronization Information
DCO OFDM - length of symbol is (N_FFT+N_CP)
FFT=64: Total Frame = 16384 [73*27*8 (data) + 77*1*8 (sync as well as pilot)]
FFT=128: Total Frame = 16384 [146*13*8 (data) + 150*1*8 (sync as well as pilot)]
FLIP OFDM - length of symbol is 2*(N_FFT+N_CP)
FFT=64: Total Frame = 16384 [73*2*13*8 (data) + 77*2*1*8 (sync as well as pilot)]
FFT=128: Total Frame = 16384 [146*2*6*8 (data) + 150*2*1*8 (sync as well as pilot)]
*/
void qam_mod(complex_t *qam_tx, uint8_t *bin_tx, uint8_t sym_type);
void qam_demod(uint8_t *bin_rx, complex_t *qam_rx);
void ofdm_mod(real_t *ofdm_tx, uint8_t *bin_tx);
uint32_t ofdm_demod(uint8_t *bin_rx, uint32_t demod_idx, int32_t samp_remng, uint32_t *bits_recvd);

#endif
