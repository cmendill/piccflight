#ifndef _HEX_FUNCTIONS
#define _HEX_FUNCTIONS

//Function prototypes
void hex_init_calmode(int calmode, calmode_t *hex);
int  hex_connect(void);
void hex_disconnect(int id);
void hex_get_error(int id);
int  hex_init(int *hexfd);
int  hex_get_command(sm_t *sm_p, hex_t *cmd);
int  hex_send_command(sm_t *sm_p, hex_t *cmd, int proc_id);
int  hex_hex2scope(double *position, double *result);
int  hex_scope2hex(double *position, double *result);
int  hex_move(int id, double *pos);
int  hex_reference(int id, int force);
int  hex_getpos(int id, double *pos);
int  hex_printpos(int id);
int  hex_setpivot(int id, double *pivot);
int  hex_zern2hex(double *zernikes, double *axes);
void hex_zern2hex_alt(double *zernikes, double *axes);
int  hex_calibrate(int calmode, hex_t *hex, uint32_t *step, int procid, int reset);

#endif
