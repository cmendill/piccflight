#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <phx_api.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "numeric.h"
#include "phx_config.h"
#include "fakemodes.h"

/**************************************************************/
/* LYT_PROCESS_IMAGE                                          */
/*  - Main image processing function for LYT                  */
/**************************************************************/
void lyt_process_image(stImageBuff *buffer,sm_t *sm_p, uint32 frame_number){
  static lytfull_t lytfull;
  static lytevent_t lytevent;
  lytfull_t *lytfull_p;
  lytevent_t *lytevent_p;
  static struct timespec first,start,end,delta,last;
  static int init=0;
  double dt;
  int32 i,j;
  uint16 fakepx=0;
  int state;
  
  //Get time immidiately
  clock_gettime(CLOCK_REALTIME,&start);

  //Get state
  state = sm_p->state;
  
  //Check reset
  if(sm_p->lyt_reset){
    init=0;
    sm_p->lyt_reset=0;
  }
  
  //Initialize
  if(!init){
    //Zero out events & commands
    memset(&lytfull,0,sizeof(lytfull_t));
    memset(&lytevent,0,sizeof(lytevent_t));
    //Set init flag
    init=1;
    //Debugging
    if(LYT_DEBUG) printf("LYT: Initialized\n");
  }
  
  //Measure exposure time
  if(timespec_subtract(&delta,&start,&last))
    printf("LYT: lyt_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);
  
  //Fill out event header
  lytevent.hed.packet_type  = LYTEVENT;
  lytevent.hed.frame_number = frame_number;
  lytevent.hed.exptime      = 0;
  lytevent.hed.ontime       = dt;
  lytevent.hed.temp         = 0;
  lytevent.hed.state        = state;
  lytevent.hed.imxsize      = LYTXS;
  lytevent.hed.imysize      = LYTYS;
  lytevent.hed.mode         = 0;
  lytevent.hed.start_sec    = start.tv_sec;
  lytevent.hed.start_nsec   = start.tv_nsec;

  
  //Open circular buffer
  lytevent_p=(lytevent_t *)open_buffer(sm_p,LYTEVENT);

  //Copy data
  memcpy(lytevent_p,&lytevent,sizeof(lytevent_t));;

  //Get final timestamp
  clock_gettime(CLOCK_REALTIME,&end);
  lytevent_p->hed.end_sec  = end.tv_sec;
  lytevent_p->hed.end_nsec = end.tv_nsec;

  //Close buffer
  close_buffer(sm_p,LYTEVENT);

  //Save time
  memcpy(&last,&start,sizeof(struct timespec));


  /*******************  Full Image Code  *****************/
  if(timespec_subtract(&delta,&start,&first))
    printf("LYT: lyt_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);
  if(dt > LYT_FULL_IMAGE_TIME){
    //Copy packet header
    memcpy(&lytfull.hed,&lytevent.hed,sizeof(pkthed_t));
    lytfull.hed.packet_type = LYTFULL;

    //Fake data
    if(sm_p->lyt_fakemode > FAKEMODE_NONE){
      if(sm_p->lyt_fakemode == FAKEMODE_GEN_IMAGE_CAMERA_SYNC)
	for(i=0;i<LYTXS;i++)
	  for(j=0;j<LYTYS;j++)
	    lytfull.image.data[i][j]=fakepx++;
    }
    else{
      //Copy full image
      memcpy(&(lytfull.image.data[0][0]),buffer->pvAddress,sizeof(lytfull.image.data));
    }

    //Open circular buffer
    lytfull_p=(lytfull_t *)open_buffer(sm_p,LYTFULL);

    //Copy data
    memcpy(lytfull_p,&lytfull,sizeof(lytfull_t));;

    //Get final timestamp
    clock_gettime(CLOCK_REALTIME,&end);
    lytfull_p->hed.end_sec  = end.tv_sec;
    lytfull_p->hed.end_nsec = end.tv_nsec;

    //Close buffer
    close_buffer(sm_p,LYTFULL);

    //Reset time
    memcpy(&first,&start,sizeof(struct timespec));
  }
}
