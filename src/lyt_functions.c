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
#include "hex_functions.h"
#include "bmc_functions.h"
#include "tgt_functions.h"

/***************************************************************/
/* LYT_SETORIGIN                                               */
/*  - Set band origins from current image                      */
/***************************************************************/
void lyt_setorigin(lytevent_t *lyt,uint16_t *img_buffer){
  int i,j,k;
  int imax=0,jmax=0;
  uint16_t pmax=0,p=0;
  
  /* Loop through band images, find max pixel */
  for(k=0;k<LYT_NBANDS;k++){
    pmax=0;
    for(i=0;i<LYTXS;i++){
      for(j=0;j<LYTYS;j++){
	p = img_buffer[lyt_xy2index(lyt->xorigin[k]-(LYTXS/2)+i,lyt->yorigin[k]-(LYTYS/2)+j)];
	if(p > pmax){
	  pmax = p;
	  imax = i;
	  jmax = j;
	}
      }
    }
    //Set new origin
    lyt->xorigin[k] += imax - (LYTXS/2);
    lyt->yorigin[k] += jmax - (LYTYS/2);
  }  
}


/**************************************************************/
/* LYT_LOADORIGIN                                             */
/*  - Load band origins from file                             */
/**************************************************************/
void lyt_loadorigin(lytevent_t *lyt){
  FILE *fd=NULL;
  char filename[MAX_FILENAME];
  uint64 fsize,rsize;
  uint32 xorigin[LYT_NBANDS] = LYT_XORIGIN;
  uint32 yorigin[LYT_NBANDS] = LYT_YORIGIN;
  
  /* Initialize with default origins */
  memcpy(lyt->xorigin,xorigin,sizeof(xorigin));
  memcpy(lyt->yorigin,yorigin,sizeof(yorigin));
  
  /* Open origin file */
  //--setup filename
  sprintf(filename,LYT_ORIGIN_FILE);
  //--open file
  if((fd = fopen(filename,"r")) == NULL){
    perror("LYT: loadorigin fopen");
    return;
  }
  //--check file size
  fseek(fd, 0L, SEEK_END);
  fsize = ftell(fd);
  rewind(fd);
  rsize = sizeof(xorigin) + sizeof(yorigin);
  if(fsize != rsize){
    printf("LYT: incorrect LYT_ORIGIN_FILE size %lu != %lu\n",fsize,rsize);
    fclose(fd);
    return;
  }
  
  //Read file
  if(fread(xorigin,sizeof(xorigin),1,fd) != 1){
    perror("LYT: loadorigin fread");
    fclose(fd);
    return;
  }
  if(fread(yorigin,sizeof(yorigin),1,fd) != 1){
    perror("LYT: loadorigin fread");
    fclose(fd);
    return;
  }
  
  //Close file
  fclose(fd);
  printf("LYT: Read: %s\n",filename);

  //Copy origins
  memcpy(lyt->xorigin,xorigin,sizeof(xorigin));
  memcpy(lyt->yorigin,yorigin,sizeof(yorigin));
}

/***************************************************************/
/* LYT_SAVEORIGIN                                              */
/*  - Saves cell origins to file                               */
/***************************************************************/
void lyt_saveorigin(lytevent_t *lyt){
  struct stat st = {0};
  FILE *fd=NULL;
  static char outfile[MAX_FILENAME];
  char temp[MAX_FILENAME];
  char path[MAX_FILENAME];
  int i;

  /* Open output file */
  //--setup filename
  sprintf(outfile,"%s",LYT_ORIGIN_FILE);
  //--create output folder if it does not exist
  strcpy(temp,outfile);
  strcpy(path,dirname(temp));
  if (stat(path, &st) == -1){
    printf("LYT: creating folder %s\n",path);
    recursive_mkdir(path, 0777);
  }
  //--open file
  if((fd = fopen(outfile, "w")) == NULL){
    perror("LYT: saveorigin fopen()\n");
    return;
  }
  
  //Save origins
  if(fwrite(lyt->xorigin,sizeof(lyt->xorigin),1,fd) != 1){
    printf("LYT: saveorigin fwrite error!\n");
    fclose(fd);
    return;
  }
  if(fwrite(lyt->yorigin,sizeof(lyt->yorigin),1,fd) != 1){
    printf("LYT: saveorigin fwrite error!\n");
    fclose(fd);
    return;
  }
  printf("LYT: Wrote: %s\n",outfile);

  //Close file
  fclose(fd);
  return;
}

/**************************************************************/
/* LYT_REVERTORIGIN                                           */
/*  - Set band origins to default                             */
/**************************************************************/
void lyt_revertorigin(lytevent_t *lyt){
  const uint32 xorigin[LYT_NBANDS] = LYT_XORIGIN;
  const uint32 yorigin[LYT_NBANDS] = LYT_YORIGIN;
  
  /* Copy default origins */
  memcpy(lyt->xorigin,xorigin,sizeof(xorigin));
  memcpy(lyt->yorigin,yorigin,sizeof(yorigin));
    
}

/**************************************************************/
/* LYT_ALP_ZERNPID                                            */
/*  - Run PID controller on measured Zernikes for ALP         */
/**************************************************************/
void lyt_alp_zernpid(lytevent_t *lytevent, double *zernike_delta, int *zernike_switch, int reset){
  static int init = 0;
  static double zint[LOWFS_N_ZERNIKE] = {0};
  double error;
  int i;

  //Initialize
  if(!init || reset){
    memset(zint,0,sizeof(zint));
    init=1;
    if(reset) return;
  }

  //Run PID
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    if(zernike_switch[i]){
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
}

/**************************************************************/
/* LYT_COPY_LYTPIX2ALPZER_REFIMG                              */
/*  - Copy reference image into image pointer                 */
/**************************************************************/
void lyt_copy_lytpix2alpzer_refimg(lyt_t *image, int reset){
  static double lyt_refimg[LYTXS][LYTYS]={{0}}; //reference image
  static int init=0;
  int i,j;
  uint64 fsize,rsize;
  FILE   *fd=NULL;
  char   filename[MAX_FILENAME];

  /* Initialize */
  if(!init || reset){
    /****** READ REFERENCE IMAGE FILE ******/
    //--setup filename
    sprintf(filename,LYTPIX2ALPZER_REFIMG_FILE);
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

  end_of_init:
    //--set init flag
    init=1;
    //--return if reset
    if(reset) return;
  }

  /* Copy image */
  for(i=0;i<LYTXS;i++)
    for(j=0;j<LYTYS;j++)
      image->data[i][j] = lyt_refimg[i][j]*40000;
  
}


/**************************************************************/
/* LYT_ZERNIKE_FIT                                            */
/*  - Fit Zernikes to LYT pixels                              */
/**************************************************************/
void lyt_zernike_fit(lyt_t *image, double *zernikes,int reset){
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
  if(!init || reset){

    /****** READ REFERENCE IMAGE FILE ******/
    //--setup filename
    sprintf(filename,LYTPIX2ALPZER_REFIMG_FILE);
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
    sprintf(filename,LYTPIX2ALPZER_PXMASK_FILE);
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
    //--return if reset
    if(reset) return;
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
  static lytevent_t lytevent;
  static lytpkt_t lytpkt;
  lytevent_t *lytevent_p;
  lytpkt_t *lytpkt_p;
  static calmode_t alpcalmodes[ALP_NCALMODES];
  static calmode_t hexcalmodes[HEX_NCALMODES];
  static calmode_t bmccalmodes[BMC_NCALMODES];
  static calmode_t tgtcalmodes[TGT_NCALMODES];
  static struct timespec start,end,delta,last;
  static int init=0;
  double dt;
  int i,j;
  uint16_t fakepx=0;
  int state;
  alp_t alp,alp_try,alp_delta;
  int zernike_control=0;
  int zernike_switch[LOWFS_N_ZERNIKE] = {0};
  uint32_t n_dither=1;
  int sample;
  uint16_t lytread[LYTREADXS][LYTREADYS];
  
  //Get time immidiately
  clock_gettime(CLOCK_REALTIME,&start);

  //Get state
  state = sm_p->state;

  //Get sample
  sample = frame_number % LYT_NSAMPLES;
  
  //Check reset
  if(sm_p->w[LYTID].reset){
    init=0;
    sm_p->w[LYTID].reset=0;
  }

  //Initialize
  if(!init){
    //Zero out events & commands
    memset(&lytevent,0,sizeof(lytevent_t));
    memset(&lytpkt,0,sizeof(lytpkt_t));
    //Reset zern2alp mapping
    alp_zern2alp(NULL,NULL,FUNCTION_RESET);
    //Reset calibration routines
    alp_calibrate(0,NULL,NULL,LYTID,FUNCTION_RESET);
    tgt_calibrate(0,NULL,NULL,LYTID,FUNCTION_RESET);
    //Reset PID controllers
    lyt_alp_zernpid(NULL,NULL,NULL,FUNCTION_RESET);
    //Reset zernike fitter
    lyt_zernike_fit(NULL,NULL,FUNCTION_RESET);
    //Reset reference image copying
    lyt_copy_lytpix2alpzer_refimg(NULL,FUNCTION_RESET);
    //Init ALP calmodes
    for(i=0;i<ALP_NCALMODES;i++)
      alp_init_calmode(i,&alpcalmodes[i]);
    //Init HEX calmodes
    for(i=0;i<HEX_NCALMODES;i++)
      hex_init_calmode(i,&hexcalmodes[i]);
    //Init BMC calmodes
    for(i=0;i<BMC_NCALMODES;i++)
      bmc_init_calmode(i,&bmccalmodes[i]);
    //Init TGT calmodes
    for(i=0;i<TGT_NCALMODES;i++)
      tgt_init_calmode(i,&tgtcalmodes[i]);
    //Reset last times
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

  //Save time
  memcpy(&last,&start,sizeof(struct timespec));


  //Fill out event header
  lytevent.hed.version      = PICC_PKT_VERSION;
  lytevent.hed.type         = BUFFER_LYTEVENT;
  lytevent.hed.frame_number = frame_number;
  lytevent.hed.exptime      = sm_p->lyt_exptime;
  lytevent.hed.ontime       = dt;
  lytevent.hed.state        = state;
  lytevent.hed.start_sec    = start.tv_sec;
  lytevent.hed.start_nsec   = start.tv_nsec;
  lytevent.hed.hex_calmode  = sm_p->hex_calmode;
  lytevent.hed.alp_calmode  = sm_p->alp_calmode;
  lytevent.hed.bmc_calmode  = sm_p->bmc_calmode;
  lytevent.hed.tgt_calmode  = sm_p->tgt_calmode;
 
  //Save gains
  for(i=0;i<LOWFS_N_ZERNIKE;i++)
    for(j=0;j<LOWFS_N_PID;j++)
      lytevent.gain_alp_zern[i][j] = sm_p->lyt_gain_alp_zern[i][j] * sm_p->zernike_control[i];
  
  //Save zernike targets
  memcpy(lytevent.zernike_target,(double *)sm_p->lyt_zernike_target,sizeof(lytevent.zernike_target));

  //Run target calibration
  if((sm_p->state_array[state].alp_commander == LYTID))
    if(lytevent.hed.tgt_calmode != TGT_CALMODE_NONE)
      sm_p->tgt_calmode = tgt_calibrate(lytevent.hed.tgt_calmode,lytevent.zernike_target,&lytevent.hed.tgt_calstep,LYTID,FUNCTION_NO_RESET);

  //Fake data
  if(sm_p->w[LYTID].fakemode != FAKEMODE_NONE){
    if(sm_p->w[LYTID].fakemode == FAKEMODE_TEST_PATTERN)
      for(i=0;i<LYTXS;i++)
	for(j=0;j<LYTYS;j++)
	  lytevent.image.data[i][j]=fakepx++;
    if(sm_p->w[LYTID].fakemode == FAKEMODE_LYTPIX2ALPZER_REFIMG)
      lyt_copy_lytpix2alpzer_refimg(&lytevent.image, FUNCTION_NO_RESET);
  }
  else{
    //Copy image data
    memcpy(&(lytread[0][0]),buffer->pvAddress,sizeof(lytread));
    //Cut out ROI -- transpose offsets
    for(i=0;i<LYTXS;i++)
      for(j=0;j<LYTYS;j++)
	lytevent.image.data[i][j]=lytread[i+sm_p->lyt_yorigin][j+sm_p->lyt_xorigin];
   
  }
  
  //Command: lyt_setorigin 
  if(sm_p->lyt_setorigin){
    lyt_setorigin(&lytevent);
    sm_p->lyt_setorigin=0;
  }
  //Command: lyt_saveorigin 
  if(sm_p->lyt_saveorigin){
    lyt_saveorigin(&lytevent);
    sm_p->lyt_saveorigin=0;
  }
  //Command: lyt_loadorigin 
  if(sm_p->lyt_loadorigin){
    lyt_loadorigin(&lytevent);
    sm_p->lyt_loadorigin=0;
  }
  //Command: lyt_revertorigin 
  if(sm_p->lyt_revertorigin){
    lyt_revertorigin(&lytevent);
    sm_p->lyt_revertorigin=0;
  }

  //Fit Zernikes
  if(sm_p->state_array[state].lyt.fit_zernikes)
    lyt_zernike_fit(&lytevent.image,lytevent.zernike_measured, FUNCTION_NO_RESET);


  /*************************************************************/
  /*******************  ALPAO DM Control Code  *****************/
  /*************************************************************/

  //Get last ALP command
  alp_get_command(sm_p,&alp);
  memcpy(&alp_try,&alp,sizeof(alp_t));

  //Check if we will send a command
  if((sm_p->state_array[state].alp_commander == LYTID) && sm_p->alp_ready){

    //Check if ALP is controlling any Zernikes
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      if(sm_p->zernike_control[i] && (sm_p->state_array[state].lyt.zernike_control[i] == ACTUATOR_ALP)){
	zernike_switch[i] = 1;
	zernike_control = 1;
      }
    }

    //Run Zernike control
    if(zernike_control){
      // - run Zernike PID
      lyt_alp_zernpid(&lytevent, alp_delta.zcmd, zernike_switch, FUNCTION_NO_RESET);

      // - zero out uncontrolled Zernikes
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	if(!zernike_switch[i])
	  alp_delta.zcmd[i] = 0;
      
      // - convert zernike deltas to actuator deltas
      alp_zern2alp(alp_delta.zcmd,alp_delta.acmd,FUNCTION_NO_RESET);
      
      // - add Zernike PID output deltas to ALP command
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	if(zernike_switch[i])
	  alp_try.zcmd[i] += alp_delta.zcmd[i];

      // - add actuator deltas to ALP command
      for(i=0;i<ALP_NACT;i++)
	alp_try.acmd[i] += alp_delta.acmd[i];
    }
    //Calibrate ALP
    if(lytevent.hed.alp_calmode != ALP_CALMODE_NONE)
      sm_p->alp_calmode = alp_calibrate(lytevent.hed.alp_calmode,&alp_try,&lytevent.hed.alp_calstep,LYTID,FUNCTION_NO_RESET);
    
    //Send command to ALP
    if(alp_send_command(sm_p,&alp_try,LYTID,n_dither)){
      
      // - copy command to current position
      memcpy(&alp,&alp_try,sizeof(alp_t));
    }
  }

  //Copy ALP command to lytevent
  memcpy(&lytevent.alp,&alp,sizeof(alp_t));
  
  //Write LYTEVENT circular buffer
  if(sm_p->write_circbuf[BUFFER_LYTEVENT]){
    //Open LYTEVENT circular buffer
    lytevent_p=(lytevent_t *)open_buffer(sm_p,BUFFER_LYTEVENT);
    
    //Copy data
    memcpy(lytevent_p,&lytevent,sizeof(lytevent_t));;
    
    //Get final timestamp
    clock_gettime(CLOCK_REALTIME,&end);
    lytevent_p->hed.end_sec  = end.tv_sec;
    lytevent_p->hed.end_nsec = end.tv_nsec;
    
    //Close buffer
    close_buffer(sm_p,BUFFER_LYTEVENT);
  }
  
  /*************************************************************/
  /**********************  LYT Packet Code  ********************/
  /*************************************************************/
  if(sm_p->write_circbuf[BUFFER_LYTPKT]){
    //Samples, collected each time through
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      lytpkt.zernike_measured[i][sample] = lytevent.zernike_measured[i];
      lytpkt.alp_zcmd[i][sample]         = lytevent.alp.zcmd[i];
    }
  
    //Last sample, fill out rest of packet and write to circular buffer
    if(sample == LYT_NSAMPLES-1){
      //Header
      memcpy(&lytpkt.hed,&lytevent.hed,sizeof(pkthed_t));
      //Image
      memcpy(&lytpkt.image,&lytevent.image,sizeof(lyt_t));
      //Zernike gains and targets
      for(i=0;i<LOWFS_N_ZERNIKE;i++){
	lytpkt.zernike_target[i]         = lytevent.zernike_target[i]; 
	for(j=0;j<LOWFS_N_PID;j++){
	  lytpkt.gain_alp_zern[i][j]     = lytevent.gain_alp_zern[i][j];
	}
      }

      //Open LYTPKT circular buffer
      lytpkt_p=(lytpkt_t *)open_buffer(sm_p,BUFFER_LYTPKT);

      //Copy data
      memcpy(lytpkt_p,&lytpkt,sizeof(lytpkt_t));;
    
      //Get final timestamp
      clock_gettime(CLOCK_REALTIME,&end);
      lytpkt_p->hed.end_sec  = end.tv_sec;
      lytpkt_p->hed.end_nsec = end.tv_nsec;
    
      //Close buffer
      close_buffer(sm_p,BUFFER_LYTPKT);
    }
  }
}
