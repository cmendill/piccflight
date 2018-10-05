#ifndef _ALP_FUNCTIONS
#define _ALP_FUNCTIONS

//Function prototypes
void alp_init_calmode(int calmode, calmode_t *alp);
int  alp_zern2alp(double *zernikes,double *actuators);
void alp_get_command(sm_t *sm_p, alp_t *cmd);
int  alp_send_command(sm_t *sm_p, alp_t *cmd, int proc_id, int n_dither);
int  alp_revert_flat(sm_t *sm_p, int proc_id);
int  alp_save_flat(sm_t *sm_p);
int  alp_load_flat(sm_t *sm_p,int proc_id);
int  alp_set_random(sm_t *sm_p,int proc_id);
int  alp_calibrate(int calmode, alp_t *alp, uint32_t *step, int procid, int reset);


//Calibration Modes
enum alpcalmodes {ALP_CALMODE_NONE,
		  ALP_CALMODE_ZERO,
		  ALP_CALMODE_FLAT,
		  ALP_CALMODE_POKE,
		  ALP_CALMODE_ZPOKE,
		  ALP_CALMODE_FLIGHT,
		  ALP_CALMODE_RAMP,
		  ALP_CALMODE_ZRAMP,
		  ALP_NCALMODES};

#endif
