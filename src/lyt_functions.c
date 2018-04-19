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

#define LYT_FULL_IMAGE_TIME 0.5 //seconds

/* lyt_process_image */
void lyt_process_image(stImageBuff *buffer,sm_t *sm_p, uint32 frame_number){
  static lytfull_t lytfull;
  static lytevent_t lytevent;
  static struct timespec first,start,delta;
  double dt;
  int32 i,j,k;
  uint16 fakepx=0;
  double lyt_zernike_matrix_inv[LOWFS_N_ZERNIKE*LYTXS*LYTYS];
  double lyt_image_arr[LYTXS*LYTYS];
  const int matrix_m=LYTXS*LYTYS;
  const int matrix_n=LOWFS_N_ZERNIKE;
  
  //Get time immidiately
  clock_gettime(CLOCK_REALTIME,&start);
  if(frame_number == 0)
    memcpy(&first,&start,sizeof(struct timespec));

  //Fill out event header
  lytevent.frame_number = frame_number;
  lytevent.exptime = 0;
  lytevent.ontime = 0;
  lytevent.temp = 0;
  lytevent.imxsize = LYTXS;
  lytevent.imysize = LYTYS;
  lytevent.mode = 0;
  lytevent.start_sec = start.tv_sec;
  lytevent.start_nsec = start.tv_nsec;

  //Copy image into event
  memcpy(&(lytevent.image.data[0][0]),buffer->pvAddress,sizeof(lytevent.image.data));
  
  //Transform image into array
  k=0;
  for(i=0;i<LYTXS;i++)
    for(j=0;j<LYTYS;j++)
      lyt_image_arr[k++]=(double)lytevent.image.data[i][j];
  
  //Matrix multiply
  //num_dgemv(lyt_zernike_matrix_inv,lyt_image_arr,lytevent.zernikes,matrix_m,matrix_n);
  
  //Apply update

  //Write event
  write_to_buffer(sm_p,(void *)&lytevent,LYTEVENT);

  //Full image code
  if(timespec_subtract(&delta,&start,&first))
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
    lytfull.start_sec = start.tv_sec;
    lytfull.start_nsec = start.tv_nsec;

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
    memcpy(&first,&start,sizeof(struct timespec));
  }
}
