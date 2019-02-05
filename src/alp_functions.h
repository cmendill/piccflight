#ifndef _ALP_FUNCTIONS
#define _ALP_FUNCTIONS

//Function prototypes
void alp_init_calmode(int calmode, calmode_t *alp);
int  alp_zern2alp(double *zernikes,double *actuators,int reset);
void alp_get_command(sm_t *sm_p, alp_t *cmd);
int  alp_send_command(sm_t *sm_p, alp_t *cmd, int proc_id, int n_dither);
int  alp_revert_flat(sm_t *sm_p, int proc_id);
int  alp_zero_flat(sm_t *sm_p, int proc_id);
int  alp_save_flat(sm_t *sm_p);
int  alp_load_flat(sm_t *sm_p,int proc_id);
int  alp_set_bias(sm_t *sm_p, double bias, int proc_id);
int  alp_set_random(sm_t *sm_p,int proc_id);
int  alp_set_zrandom(sm_t *sm_p,int proc_id);
int  alp_calibrate(int calmode, alp_t *alp, uint32_t *step, int procid, int reset);

#endif
