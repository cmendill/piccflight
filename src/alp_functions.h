#ifndef _ALP_FUNCTIONS
#define _ALP_FUNCTIONS

//Function prototypes
void alp_init_calmode(int calmode, calmode_t *alp);
int alp_zern2alp(double *zernikes,double *actuators);
int alp_calibrate(int calmode, alp_t *alp, int reset);
void alp_check(alp_t *alp);
int alp_open(char *name);
int alp_write(int devId, alp_t* alp);
int alp_close(int devId);
int alp_zero(int devId);


//Calibration Modes
enum alpcalmodes {ALP_CALMODE_NONE,
		  ALP_CALMODE_POKE,
		  ALP_CALMODE_ZPOKE,
		  ALP_CALMODE_FLIGHT,
		  ALP_NCALMODES};

#endif
