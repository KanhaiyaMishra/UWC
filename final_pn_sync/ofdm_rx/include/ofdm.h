#ifndef __OFDM_H
#define __OFDM_H

#include <inttypes.h>
#include "kiss_fft.h"
#define TRUE 1
#define FALSE 0

typedef kiss_fft_cpx complex_t;
typedef kiss_fft_scalar real_t;

typedef struct{
    real_t auto_corr;
    real_t cross_corr;
    real_t corr_factor;
}sync_params_t;

#define OSF 8               // TX over sampling factor
#define PRE_DSF 4           // RX downsampling before synchronization
#define POST_DSF 2          // RX downsampling after synchronization
#define DCO                 // OFDM Type: DCO/FLIP
#define M_QAM  4            // QAM order
#define N_BITS 2            // bits per qam symbol
#define N_FFT 64            // Total Subcarriers (FFT Size)
#define N_DSC 52            // Data Subcarriers
#define N_QAM 26            // qam syms per ofdm symbol
#define N_SYM 27            // OFDM Symbols Carrying Data
#define N_CP_DATA 9         // CP length for symbols with data
#define N_CP_SYNC 13        // CP length for symbol with sync info
#define SYNC_SYM_LEN 616    // OFDM Sync Symbol: (64+13)*8 samples
#define DATA_SYM_LEN 584    // OFDM Data Symbol: (64+9)*8 samples
#define FRM_NUM_BITS 16     // bits for frame no info (start of each frame)

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
uint32_t ofdm_demod(uint8_t *bin_rx, real_t *ofdm_rx, uint32_t samp_remng, uint32_t *bits_recvd);

#endif
