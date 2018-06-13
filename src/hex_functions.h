#ifndef _HEX_FUNCTIONS
#define _HEX_FUNCTIONS

//Function prototypes
void hex_init_calmode(int calmode, calmode_t *hex);
int hex_connect(void);
void hex_disconnect(int id);
int hex_hex2scope(double *position, double *result);
int scope2hex(double *position, double *result);
int hex_move(int id, double *pos);
int hex_reference(int id,int force);
int hex_getpos(int id, double *pos);
int hex_setpivot(int id, double *pivot);
int hex_zern2hex(double *zernikes, double *axes);
int hex_calibrate(int calmode, hex_t *hex,uint64 *step, int reset, uint64 counter);



//Calibration Modes
enum hexcalmodes {HEX_CALMODE_NONE,
		  HEX_CALMODE_POKE,
		  HEX_CALMODE_SPIRAL,
		  HEX_NCALMODES};


#endif
