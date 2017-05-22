#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>


/* piccflight headers */
#include "../common/controller.h"
#include "../common/common_functions.h"
#include "../phx/include/phx_api.h"
#include "../phx/config.h"
#include "lyt_proc.h"

/* lyt_process_image */
void lyt_process_image(stImageBuff *buffer,sm_t *sm_p){
  static lytfull_t lytfull;
  static lytevent_t lytevent;
  
  //Buffer Layout
  //buffer->pvAddress;
  //buffer->pvContext;

  //Process image

  //Calculate update

  //Apply update

  //Write event

  //Fill out full image header
  lytfull.frame_number = 0;
  lytfull.exptime = 0;
  lytfull.ontime = 0;
  lytfull.temp = 0;
  lytfull.timestamp = 0;
  lytfull.imxsize = 0;
  lytfull.imysize = 0;
  lytfull.state = 0;
  lytfull.mode = 0;

  //Copy full image
  memcpy(buffer->pvAddress,lytfull.image.data[0],sizeof(lytfull.image.data));
  
  //Write full image
  write_to_buffer(sm_p,(void *)&lytfull,LYTFULL);
}
