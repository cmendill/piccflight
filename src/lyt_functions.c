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
#include "alp_functions.h"

/**************************************************************/
/* LYT_ALP_ZERNPID                                            */
/*  - Run PID controller on measured Zernikes for ALP         */
/**************************************************************/
void lyt_alp_zernpid(lytevent_t *lytevent, double *zernike_delta, int reset){
  static int init = 0;
  static double zint[LOWFS_N_ZERNIKE] = {0};
  double error;
  int i;
  #define LYT_ALP_ZERN_INT_MAX  0.1
  #define LYT_ALP_ZERN_INT_MIN -0.1

  //Initialize
  if(!init || reset){
    memset(zint,0,sizeof(zint));
    init=1;
    if(reset) return;
  }

  //Run PID
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    //Calculate error
    error = lytevent->zernike_measured[i] - lytevent->zernike_target[i];
    //Calculate integral
    zint[i] += error;

    if(zint[i] > LYT_ALP_ZERN_INT_MAX) zint[i]=LYT_ALP_ZERN_INT_MAX;
    if(zint[i] < LYT_ALP_ZERN_INT_MIN) zint[i]=LYT_ALP_ZERN_INT_MIN;

    //Calculate command
    zernike_delta[i] = lytevent->gain_alp_zern[i][0] * error + lytevent->gain_alp_zern[i][1] * zint[i];
  }
}

/**************************************************************/
/* LYT_ZERNIKE_FIT                                            */
/*  - Fit Zernikes to LYT centroids                           */
/**************************************************************/
void lyt_zernike_fit(lyt_t *image, double *zernikes){
  FILE   *fd=NULL;
  char   filename[MAX_FILENAME];
  static lytevent_t lytevent;
  static int init=0;
  static double lyt2zern_matrix[LYTXS*LYTYS*LOWFS_N_ZERNIKE]={0}; //zernike fitting matrix (max size)
  static double lyt_delta[LYTXS*LYTYS]={0};     //measured image - reference (max size)
  static double lyt_refimg[LYTXS][LYTYS]={{0}}; //reference image
  static uint16 lyt_pxmask[LYTXS][LYTYS]={{0}}; //pixel selection mask
  static int    lyt_npix=0;
  uint64 fsize,rsize;
  double total;
  int    i,j,count;

  /* Initialize Fitting Matrix */
  if(!init){

    /****** READ REFERENCE IMAGE FILE ******/
    //--setup filename
    sprintf(filename,LYT_REFIMG_FILE);
    //--open file
    if((fd = fopen(filename,"r")) == NULL){
      perror("fopen");
      printf("lyt_refimg file\n");
      goto end_of_init;
    }
    //--check file size
    fseek(fd, 0L, SEEK_END);
    fsize = ftell(fd);
    rewind(fd);
    rsize = sizeof(lyt_refimg);
    if(fsize != rsize){
      printf("LYT: incorrect lyt_refimg file size %lu != %lu\n",fsize,rsize);
      fclose(fd);
      goto end_of_init;
    }
    //--read file
    if(fread(lyt_refimg,rsize,1,fd) != 1){
      perror("fread");
      printf("lyt_refimg file\n");
      fclose(fd);
      goto end_of_init;
    }
    //--close file
    fclose(fd);
    printf("LYT: Read: %s\n",filename);

    /****** READ PIXEL MASK FILE ******/
    //--setup filename
    sprintf(filename,LYT_PXMASK_FILE);
    //--open file
    if((fd = fopen(filename,"r")) == NULL){
      perror("fopen");
      printf("lyt_pxmask file\n");
      goto end_of_init;
    }
    //--check file size
    fseek(fd, 0L, SEEK_END);
    fsize = ftell(fd);
    rewind(fd);
    rsize = sizeof(lyt_pxmask);
    if(fsize != rsize){
      printf("LYT: incorrect lyt_pxmask file size %lu != %lu\n",fsize,rsize);
      fclose(fd);
      goto end_of_init;
    }
    //--read file
    if(fread(lyt_pxmask,rsize,1,fd) != 1){
      perror("fread");
      printf("lyt_pxmask file\n");
      fclose(fd);
      goto end_of_init;
    }
    //--close file
    fclose(fd);
    printf("LYT: Read: %s\n",filename);

    /****** COUNT NUMBER OF CONTROLED PIXELS ******/
    lyt_npix=0;
    for(i=0;i<LYTXS;i++)
      for(j=0;j<LYTYS;j++)
	if(lyt_pxmask[i][j])
	  lyt_npix++;


    /****** READ ZERNIKE MATRIX FILE ******/
    //--setup filename
    sprintf(filename,LYTPIX2ALPZER_FILE);
    //--open file
    if((fd = fopen(filename,"r")) == NULL){
      perror("fopen");
      printf("lyt2zern file\n");
      goto end_of_init;
    }
    //--calculate
    //--check file size
    fseek(fd, 0L, SEEK_END);
    fsize = ftell(fd);
    rewind(fd);
    rsize = lyt_npix*LOWFS_N_ZERNIKE*sizeof(double);
    if(fsize != rsize){
      printf("LYT: incorrect lyt2zern matrix file size %lu != %lu\n",fsize,rsize);
      fclose(fd);
      goto end_of_init;
    }
    //--read matrix
    if(fread(lyt2zern_matrix,rsize,1,fd) != 1){
      perror("fread");
      printf("lyt2zern file\n");
      fclose(fd);
      goto end_of_init;
    }
    //--close file
    fclose(fd);
    printf("LYT: Read: %s\n",filename);


  end_of_init:
    //--set init flag
    init=1;
  }


  //Normalize image pixels & subtract reference image
  total=0;
  count=0;
  for(i=0;i<LYTXS;i++)
    for(j=0;j<LYTYS;j++)
      if(lyt_pxmask[i][j])
	total += (double)image->data[i][j];
  for(i=0;i<LYTXS;i++)
    for(j=0;j<LYTYS;j++)
      if(lyt_pxmask[i][j])
	lyt_delta[count++]  = ((double)image->data[i][j])/total - lyt_refimg[i][j];

  //Do matrix multiply
  num_dgemv(lyt2zern_matrix, lyt_delta, zernikes, LOWFS_N_ZERNIKE, lyt_npix);
}


/**************************************************************/
/* LYT_PROCESS_IMAGE                                          */
/*  - Main image processing function for LYT                  */
/**************************************************************/
void lyt_process_image(stImageBuff *buffer,sm_t *sm_p, uint32 frame_number){
  static lytfull_t lytfull;
  static lytevent_t lytevent;
  lytfull_t *lytfull_p;
  lytevent_t *lytevent_p;
  static calmode_t alpcalmodes[ALP_NCALMODES];
  static struct timespec start,end,delta,last,full_last;
  static int init=0;
  double dt;
  int i,j;
  uint16_t fakepx=0;
  int state;
  alp_t alp,alp_try,alp_delta;
  int zernike_control=0;
  uint32_t n_dither=1;

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
    //Reset calibration routines
    alp_calibrate(0,NULL,NULL,LYTID,FUNCTION_RESET);
    //Reset PID controllers
    lyt_alp_zernpid(NULL,NULL,FUNCTION_RESET);
    for(i=0;i<ALP_NCALMODES;i++){
      alp_init_calmode(i,&alpcalmodes[i]);
    }
    //Reset last times
    memcpy(&full_last,&start,sizeof(struct timespec));
    memcpy(&last,&start,sizeof(struct timespec));
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

  //Save modes and gains
  lytevent.alp_calmode      = sm_p->alp_calmode;
  memcpy(&lytevent.gain_alp_zern[0][0],(double *)&sm_p->lyt_gain_alp_zern[0][0],sizeof(lytevent.gain_alp_zern));
  
  //Copy image out of buffer
  memcpy(&(lytevent.image.data[0][0]),buffer->pvAddress,sizeof(lytevent.image.data));

  //Fit Zernikes
  if(sm_p->state_array[state].lyt.fit_zernikes)
    lyt_zernike_fit(&lytevent.image,lytevent.zernike_measured);

  /*************************************************************/
  /*******************  ALPAO DM Control Code  *****************/
  /*************************************************************/

  //Get last ALP command
  alp_get_command(sm_p,&alp);
  memcpy(&alp_try,&alp,sizeof(alp_t));

  //Check if we will send a command
  if((sm_p->state_array[state].alp_commander == LYTID) && sm_p->alp_ready){

    //Check if ALP is controlling any Zernikes
    zernike_control = 0;
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      if(sm_p->state_array[state].lyt.zernike_control[i] == ACTUATOR_ALP)
	zernike_control = 1;

    //Run Zernike control
    if(zernike_control){
      // - run Zernike PID
      lyt_alp_zernpid(&lytevent, alp_delta.zernike_cmd, FUNCTION_NO_RESET);

      // - zero out uncontrolled Zernikes
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	if(sm_p->state_array[state].lyt.zernike_control[i] != ACTUATOR_ALP)
	  alp_delta.zernike_cmd[i] = 0;

      // - convert zernike deltas to actuator deltas
      alp_zern2alp(alp_delta.zernike_cmd,alp_delta.act_cmd);

      // - add Zernike PID output deltas to ALP command
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	if(sm_p->state_array[state].lyt.zernike_control[i] == ACTUATOR_ALP)
	  alp_try.zernike_cmd[i] += alp_delta.zernike_cmd[i];

      // - add actuator deltas to ALP command
      for(i=0;i<ALP_NACT;i++)
	alp_try.act_cmd[i] += alp_delta.act_cmd[i];
    }

    //Calibrate ALP
    if(sm_p->alp_calmode != ALP_CALMODE_NONE)
      sm_p->alp_calmode = alp_calibrate(lytevent.alp_calmode,&alp_try,&lytevent.alp_calstep,LYTID,FUNCTION_NO_RESET);

    //Send command to ALP
    if(alp_send_command(sm_p,&alp_try,LYTID,n_dither)){

      // - copy command to current position
      memcpy(&alp,&alp_try,sizeof(alp_t));
    }
  }

  //Copy ALP command to lytevent
  memcpy(&lytevent.alp,&alp,sizeof(alp_t));

  //Open LYTEVENT circular buffer
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

  //Save end timestamps for full image code
  lytevent.hed.end_sec  = end.tv_sec;
  lytevent.hed.end_nsec = end.tv_nsec;


  /*************************************************************/
  /**********************  Full Image Code  ********************/
  /*************************************************************/
  if(timespec_subtract(&delta,&start,&full_last))
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

    //Copy event
    memcpy(&lytfull.lytevent,&lytevent,sizeof(lytevent));

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
    memcpy(&full_last,&start,sizeof(struct timespec));
  }
}
