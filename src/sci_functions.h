void sci_function_reset(sm_t *sm_p);
uint64_t sci_xy2index_full(int x,int y,int lrx, int lry);
uint64_t sci_xy2index_roi(int x,int y,int lrx, int lry);
void sci_setorigin(sm_t *sm_p,uint16_t *img_buffer);
void sci_findorigin(sm_t *sm_p,uint16_t *img_buffer);
void sci_loadorigin(sm_t *sm_p);
void sci_saveorigin(sm_t *sm_p);
void sci_revertorigin(sm_t *sm_p);
double sci_get_temp(flidev_t dev);
double sci_get_tec_power(flidev_t dev);
int sci_expose(sm_t *sm_p, flidev_t dev, uint16 *img_buffer);
void sci_howfs_construct_field(sm_t *sm_p,sci_howfs_t *frames,scievent_t *scievent,sci_field_t *field,int reset);
void sci_howfs_efc(sm_t *sm_p,sci_field_t *field, double *delta_length,int reset);
void sci_process_image(uint16 *img_buffer, float img_exptime, sm_t *sm_p);



