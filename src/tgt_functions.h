#ifndef _TGT_FUNCTIONS
#define _TGT_FUNCTIONS

//Function prototypes
void tgt_init_calmode(int calmode, calmode_t *tgt);
int  tgt_calibrate(int calmode, double *zernikes, uint32_t *step, int procid, int reset);


#endif
