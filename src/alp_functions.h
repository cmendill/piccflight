#include "acedev5.h"

void alp_init(alp_t *iwc);
int alp_calibrate(int calmode, alp_t *alp, int reset);
void alp_check(alp_t *alp);
const int* alp_open(int*);
int alp_closeDev(const int*);
int alp_write(const int*, alp_t*);
int alp_zero(const int*);
