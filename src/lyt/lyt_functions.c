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

#define LYT_FULL_IMAGE_TIME 100000000 //nsec

/* lyt_process_image */
void lyt_process_image(stImageBuff *buffer,sm_t *sm_p, uint32 frame_number){
  static lytfull_t lytfull;
  static lytevent_t lytevent;
  static struct timespec start,now,delta;

  //Get time immidiately
  clock_gettime(CLOCK_REALTIME,&now);
  if(frame_number == 0)
    memcpy(&start,&now,sizeof(struct timespec));

  //Fill out event header
  lytevent.frame_number = frame_number;
  lytevent.exptime = 0;
  lytevent.ontime = 0;
  lytevent.temp = 0;
  lytevent.imxsize = LYTXS;
  lytevent.imysize = LYTYS;
  lytevent.state = 0;
  lytevent.mode = 0;
  memcpy(&lytevent.time,&now,sizeof(struct timespec));
  
  //Process image
  
  //Calculate update

  //Apply update

  //Write event

  //Full image code
  if(timespec_subtract(&delta,&now,&start))
    printf("LYT: lyt_process_image --> timespec_subtract error!\n");
  if(delta.tv_nsec > LYT_FULL_IMAGE_TIME){
    //Fill out full image header
    lytfull.frame_number = frame_number;
    lytfull.exptime = 0;
    lytfull.ontime = 0;
    lytfull.temp = 0;
    lytfull.imxsize = LYTXS;
    lytfull.imysize = LYTYS;
    lytfull.state = 0;
    lytfull.mode = 0;
    memcpy(&lytfull.time,&now,sizeof(struct timespec));
    
    //Copy full image
    memcpy(buffer->pvAddress,lytfull.image.data[0],sizeof(lytfull.image.data));
  
    //Write full image
    write_to_buffer(sm_p,(void *)&lytfull,LYTFULL);

    //Reset time
    memcpy(&start,&now,sizeof(struct timespec));
  }
}
