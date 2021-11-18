#ifndef _BMC_FUNCTIONS
#define _BMC_FUNCTIONS

//Function prototypes
void bmc_function_reset(sm_t *sm_p);
void bmc_init_calmode(int calmode, calmode_t *bmc);
void bmc_limit_command(bmc_t *cmd);
void bmc_rotate_command(bmc_t *cmd, int reset);
int bmc_get_command(sm_t *sm_p, bmc_t *cmd);
int bmc_get_flat(sm_t *sm_p, bmc_t *cmd,int iflat);
int bmc_send_command(sm_t *sm_p, bmc_t *cmd, int proc_id, int set_flat);
int bmc_set_bias(sm_t *sm_p, float bias, int proc_id);
int bmc_set_random(sm_t *sm_p, int proc_id);
int bmc_zero_flat(sm_t *sm_p, int proc_id);
int bmc_revert_flat(sm_t *sm_p, int proc_id);
int bmc_save_flat(sm_t *sm_p);
int bmc_load_flat(sm_t *sm_p,int proc_id);
int bmc_set_flat(sm_t *sm_p,int proc_id, int iflat);
void bmc_init_calibration(sm_t *sm_p);
void bmc_add_length(float *input, float *output, double *dl, int reset);
void bmc_add_probe(float *input, float *output, int ihowfs, int reset);
void bmc_add_test(float *input, float *output, int itest);
void bmc_add_sine(float *input, float *output,int isine);
int bmc_calibrate(sm_t *sm_p, int calmode, bmc_t *bmc, uint32_t *step, int advance, int delta_only, int procid, int reset);
#endif
