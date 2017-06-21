void iwc_init(iwc_t *iwc);
int iwc_calibrate(int calmode, iwc_t *iwc, int reset);
void iwc_check(iwc_t *iwc);
int xin_open(void);
int xin_writeUsb(signed short hDevice, unsigned char* data, unsigned long length);
int xin_readUsb(signed short hDevice, unsigned char* data, unsigned long length);
int xin_closeDev(signed short hDevice);
int xin_zero(signed short hDevice);
int xin_write(signed short hDevice, iwc_t *iwc, dm_t *dm, pez_t *pez);
