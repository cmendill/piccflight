uint64_t sci_xy2index(int x,int y);
void sci_setorigin(scievent_t *sci,uint16_t *img_buffer);
void sci_findorigin(scievent_t *sci,uint16_t *img_buffer);
void sci_loadorigin(scievent_t *sci);
void sci_saveorigin(scievent_t *sci);
void sci_revertorigin(scievent_t *sci);
double sci_get_temp(flidev_t dev);
int sci_expose(sm_t *sm_p, flidev_t dev, uint16 *img_buffer);
void sci_howfs_construct_field(sci_howfs_t *frames,sci_field_t *field,int reset);
void sci_howfs_efc(sci_field_t *field, double *delta_length,int reset);
void sci_process_image(uint16 *img_buffer, sm_t *sm_p);



