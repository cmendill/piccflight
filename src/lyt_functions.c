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
  #define LYT_ALP_ZERN_INT_MAX  0.0
  #define LYT_ALP_ZERN_INT_MIN -0.0

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
    zernike_delta[i] = lytevent->kP_alp_zern * error + lytevent->kI_alp_zern * zint[i];
  }
}

/**************************************************************/
/* LYT_ZERNIKE_FIT                                            */
/*  - Fit Zernikes to LYT centroids                           */
/**************************************************************/
void lyt_zernike_fit(lyt_t *image, double *zernikes){
  FILE *matrix=NULL;
  FILE *flatfile=NULL;
  static FILE *calfile=NULL;
  char matrix_file[MAX_FILENAME];
  static lytevent_t lytevent;
  static int init=0;
  static double lyt2zern_matrix[(LYTXS*LYTYS)*LOWFS_N_ZERNIKE]={0};
  static double lyt_image_flat[LYTXS*LYTYS];
  static double image_flat_total=0;
  double lyt_image_delta[LYTXS*LYTYS];
  double image_meas_total=0;

  int i,j,n, ret;
  uint64 fsize,rsize;

  /* Initialize Fitting Matrix */
  if(!init){

  /* Open matrix file */
  //--setup filename
  sprintf(matrix_file,LYT2ZERNIKE_FILE);
  //--open file
  if((matrix = fopen(matrix_file,"r")) == NULL){
    perror("fopen");
    printf("lyt2zern file\n");
    goto end_of_init;
  }
  //--check file size
  fseek(matrix, 0L, SEEK_END);
  fsize = ftell(matrix);
  rewind(matrix);
  rsize = LYTXS*LYTYS*LOWFS_N_ZERNIKE*sizeof(double);
  if(fsize != rsize){
    printf("SHK: incorrect lyt2zern matrix file size %lu != %lu\n",fsize,rsize);
    fclose(matrix);
    goto end_of_init;
  }
  //--read matrix
  if(fread(lyt2zern_matrix,LYTXS*LYTYS*LOWFS_N_ZERNIKE*sizeof(double),1,matrix) != 1){
    perror("fread");
    printf("lyt2zern file\n");
    fclose(matrix);
    goto end_of_init;
  }
  //--close file
  fclose(matrix);
  printf("LYT: Read: %s\n",matrix_file);

///////////////////////////////////////////////////////////////////////
  // read in lyt alpao calibration images
  //--open file
  if((calfile = fopen("output/calibration/lyt_alp_zpoke_caldata.dat", "r")) == NULL){
    perror("lyt_zernike_matrix: fopen()\n");
  }
  if(fread(&lytevent, sizeof(lytevent), 1, calfile) != 1){
    perror("fread");
    printf("lyot calibration file\n");
    fclose(calfile);
    // goto end_of_init;
  }

  for(n=0;n<ALP_NCALIM;n++){
    ret=fread(&lytevent, sizeof(lytevent), 1, calfile);
    for(i=0;i<LYTYS;i++){
      for(j=0;j<LYTXS;j++){
        lyt_image_flat[i*LYTYS+j] += (double)lytevent.image.data[i][j]/ALP_NCALIM;
        image_flat_total += (double)lytevent.image.data[i][j]/ALP_NCALIM;
      }
    }
  }
  image_flat_total /= 1000;
  fclose(calfile);

/////////////////////////////////////////////////////////////////
  // read in lyt ideal flat image
  //--open file
  // if((flatfile = fopen("config/lyt_flat_image.dat", "r")) == NULL){
  //   perror("lyt_flat_image: fopen()\n");
  // }
  //
  // //--check file size
  // fseek(flatfile, 0L, SEEK_END);
  // fsize = ftell(flatfile);
  // rewind(flatfile);
  // rsize = LYTXS*LYTYS*sizeof(double);
  // if(fsize != rsize){
  //   printf("SHK: incorrect lyot flat image file size %lu != %lu\n",fsize,rsize);
  //   fclose(flatfile);
  //   goto end_of_init;
  // }
  //
  // //--read image file
  // if(fread(lyt_image_flat,LYTXS*LYTYS*sizeof(double),1,flatfile) != 1){
  //   perror("fread");
  //   printf("lyot flat image file\n");
  //   fclose(flatfile);
  //   goto end_of_init;
  // }
  // printf("LYT: Loaded lyot flat image\n");
  //   for(i=0;i<LYTYS;i++){
  //     for(j=0;j<LYTXS;j++){
  //       image_flat_total += lyt_image_flat[i*LYTYS+j];
  //     }
  //   }
  // image_flat_total /= 1000;
/////////////////////////////////////////////////////////////
  end_of_init:
    //--set init flag
    init=1;
  }

  for(i=0;i<LYTYS;i++){
    for(j=0;j<LYTXS;j++){
      image_meas_total += (double)image->data[i][j];
    }
  }
  image_meas_total /= 1000;

  for(i=0;i<LYTYS;i++){
    for(j=0;j<LYTXS;j++){
      lyt_image_delta[i*LYTYS + j] = ((double)image->data[i][j]/image_meas_total) - (lyt_image_flat[i*LYTYS + j]/image_flat_total);
    }
  }
  //Do matrix multiply
  num_dgemv(lyt2zern_matrix, lyt_image_delta, zernikes, LOWFS_N_ZERNIKE, LYTXS*LYTYS);
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
    alp_calibrate(0,NULL,NULL,FUNCTION_RESET);
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
  lytevent.kP_alp_zern      = sm_p->lyt_kP_alp_zern;
  lytevent.kI_alp_zern      = sm_p->lyt_kI_alp_zern;
  lytevent.kD_alp_zern      = sm_p->lyt_kD_alp_zern;

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
      sm_p->alp_calmode = alp_calibrate(lytevent.alp_calmode,&alp_try,&lytevent.alp_calstep,FUNCTION_NO_RESET);

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
