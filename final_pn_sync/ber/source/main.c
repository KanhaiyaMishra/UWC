#include <stdio.h>
#include <inttypes.h>
#include "prbs.h"
#include <math.h>
#include <stdlib.h>
#define NBITSFRM 992
#define NFRAMES 1100
#define SYNCLEN 31
int start_frame = 47;

int main(int argc, char** argv)
{
    uint8_t* bin_tx = (uint8_t*)malloc(NFRAMES*NBITSFRM*sizeof(uint8_t));
    uint8_t* sync_seq = (uint8_t*)malloc(SYNCLEN*sizeof(uint8_t));
    prbs_gen_byte(PRBS7, sync_seq, SYNCLEN);
    prbs_gen_byte(PRBS11, bin_tx, NFRAMES*NBITSFRM);
    int err_count[1000] = {0}, bit, last_frm, first_frm, frm_num[1000] = {0};
    float ber = 0.0;
    fprintf(stdout, "entered main\n");
    FILE *fp = NULL;
    fp = fopen("./bin.txt","r");
    for(int j=0; j<NFRAMES; j++)
    {
        for(int i=0; i<1008; i++)
        {
            if(fp!=NULL)
            {
                fscanf(fp,"%d", &bit);
                if(i<15) {
                    frm_num[j] = frm_num[j] + bit*pow(2,i);
                } else
                    err_count[j] = err_count[j] + (bin_tx[frm_num[j]*992+i] != bit);
            } else {
                first_frm = frm_num[0];
                last_frm = frm_num[j-1];
                j=NFRAMES;
                break;
            }
        }
        ber = ber + err_count[j];
    }
    ber = ber/(last_frm-first_frm+1);
    fprintf(stdout, "last frame = %d, first frame = %d, BER = %f\n", last_frm, first_frm, ber);
    return 0;
}

