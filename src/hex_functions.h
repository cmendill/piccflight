void hex_init(hex_t *hex);
int hex_calibrate(int calmode, hex_t *hex, int reset);
int hex_connect(void);
int hex_move(int id, double *pos);
int hex_reference(int id, int force);
int hex_getpos(int id, double *pos);
int hex_setpivot(int id, double *pivot);
