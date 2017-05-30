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


#define SHK_FULL_IMAGE_TIME 0.5 //seconds

/* shk_process_image */
void shk_process_image(stImageBuff *buffer,sm_t *sm_p, uint32 frame_number){
  static shkfull_t shkfull;
  static shkevent_t shkevent;
  static struct timespec start,now,delta;
  double dt;
  int32 i,j;
  uint16 fakepx=0;
  
  //Get time immidiately
  clock_gettime(CLOCK_REALTIME,&now);
  if(frame_number == 0)
    memcpy(&start,&now,sizeof(struct timespec));

  //Fill out event header
  shkevent.frame_number = frame_number;
  shkevent.exptime = 0;
  shkevent.ontime = 0;
  shkevent.temp = 0;
  shkevent.imxsize = SHKXS;
  shkevent.imysize = SHKYS;
  shkevent.state = 0;
  shkevent.mode = 0;
  memcpy(&shkevent.time,&now,sizeof(struct timespec));
  
  //Process image
  
  //Calculate update

  //Apply update

  //Write event

  //Full image code
  if(timespec_subtract(&delta,&now,&start))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);
  if(dt > SHK_FULL_IMAGE_TIME){
    //Fill out full image header
    shkfull.packet_type  = SHKFULL;
    shkfull.frame_number = frame_number;
    shkfull.exptime = 0;
    shkfull.ontime = 0;
    shkfull.temp = 0;
    shkfull.imxsize = SHKXS;
    shkfull.imysize = SHKYS;
    shkfull.mode = 0;
    shkfull.time_sec = now.tv_sec;
    shkfull.time_nsec = now.tv_nsec;

    //Copy full image
    memcpy(&(shkfull.image.data[0][0]),buffer->pvAddress,sizeof(shkfull.image.data));

    //Fake data
    if(sm_p->shk_fake_mode == 1){
      for(i=0;i<SHKXS;i++)
	for(j=0;j<SHKYS;j++)
	  shkfull.image.data[i][j]=fakepx++;
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	shkfull.zernikes[i]=i;
      
    }
    if(sm_p->shk_fake_mode == 2){
      for(i=0;i<SHKXS;i++)
	for(j=0;j<SHKYS;j++)
	  shkfull.image.data[i][j]=2*fakepx++;
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	shkfull.zernikes[i]=i;
    }
    if(sm_p->shk_fake_mode == 3){
      for(i=0;i<SHKXS;i++)
	for(j=0;j<SHKYS;j++)
	  shkfull.image.data[i][j]=3*fakepx++;
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	shkfull.zernikes[i]=i;
    }
    
    //Write full image
    write_to_buffer(sm_p,(void *)&shkfull,SHKFULL);

    //Reset time
    memcpy(&start,&now,sizeof(struct timespec));
  }
}
