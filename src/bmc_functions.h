#ifndef _BMC_FUNCTIONS
#define _BMC_FUNCTIONS

//Function prototypes
void bmc_init_calmode(int calmode, calmode_t *bmc);

//Calibration Modes
enum bmccalmodes {BMC_CALMODE_NONE,
		  BMC_NCALMODES};

#endif
