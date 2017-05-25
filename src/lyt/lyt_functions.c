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

#define LYT_FULL_IMAGE_TIME 0.5 //seconds

/* lyt_process_image */
void lyt_process_image(stImageBuff *buffer,sm_t *sm_p, uint32 frame_number){
  static lytfull_t lytfull;
  static lytevent_t lytevent;
  static struct timespec start,now,delta;
  double dt;
  int32 i,j;
  uint16 fakepx=0;
  
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
  ts2double(&delta,&dt);
  if(dt > LYT_FULL_IMAGE_TIME){
    //Fill out full image header
    lytfull.packet_type  = LYTFULL;
    lytfull.frame_number = frame_number;
    lytfull.exptime = 0;
    lytfull.ontime = 0;
    lytfull.temp = 0;
    lytfull.imxsize = LYTXS;
    lytfull.imysize = LYTYS;
    lytfull.mode = 0;
    lytfull.time_sec = now.tv_sec;
    lytfull.time_nsec = now.tv_nsec;

    //Copy full image
    memcpy(&(lytfull.image.data[0][0]),buffer->pvAddress,sizeof(lytfull.image.data));

    //Fake data
    if(sm_p->lyt_fake_mode == 1){
      for(i=0;i<LYTXS;i++)
	for(j=0;j<LYTYS;j++)
	  lytfull.image.data[i][j]=fakepx++;
    }
    if(sm_p->lyt_fake_mode == 2){
      for(i=0;i<LYTXS;i++)
	for(j=0;j<LYTYS;j++)
	  lytfull.image.data[i][j]=2*fakepx++;
    }
    if(sm_p->lyt_fake_mode == 3){
      for(i=0;i<LYTXS;i++)
	for(j=0;j<LYTYS;j++)
	  lytfull.image.data[i][j]=3*fakepx++;
    }
    
    //Write full image
    write_to_buffer(sm_p,(void *)&lytfull,LYTFULL);

    //Reset time
    memcpy(&start,&now,sizeof(struct timespec));
  }
}
