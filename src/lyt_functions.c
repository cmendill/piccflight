#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <libgen.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/io.h>
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

/**************************************************************/
/* LYT_INITREF                                                */
/*  - Initialize reference image structure                    */
/**************************************************************/
void lyt_initref(lytref_t *lytref){
  FILE   *fd=NULL;
  char   filename[MAX_FILENAME];
  uint64 fsize,rsize;
  
  /****** READ DEFAULT REFERENCE IMAGE FILE ******/
  //--setup filename
  sprintf(filename,LYTPIX2ALPZER_REFIMG_FILE);
  //--open file
  if((fd = fopen(filename,"r")) == NULL){
    perror("LYT: lyt_refimg fopen");
  }else{
    //--check file size
    fseek(fd, 0L, SEEK_END);
    fsize = ftell(fd);
    rewind(fd);
    rsize = sizeof(lytref->refdef);
    if(fsize != rsize){
      printf("LYT: incorrect lyt_refimg file size %lu != %lu\n",fsize,rsize);
      fclose(fd);
    }else{
      //--read file
      if(fread(&lytref->refdef[0][0],rsize,1,fd) != 1){
	perror("LYT: lyt_refimg fread");
	fclose(fd);
      }else{
	//--close file
	fclose(fd);
	printf("LYT: Read: %s\n",filename);
	//--copy to current reference image
	memcpy(&lytref->refimg[0][0],&lytref->refdef[0][0],sizeof(lytref->refimg));
      }
    }
  }
  
  /****** READ MODEL REFERENCE IMAGE FILE ******/
  //--setup filename
  sprintf(filename,LYTPIX2ALPZER_REFMOD_FILE);
  //--open file
  if((fd = fopen(filename,"r")) == NULL){
    perror("LYT: lyt_refmod fopen");
  }else{
    //--check file size
    fseek(fd, 0L, SEEK_END);
    fsize = ftell(fd);
    rewind(fd);
    rsize = sizeof(lytref->refmod);
    if(fsize != rsize){
      printf("LYT: incorrect lyt_refmod file size %lu != %lu\n",fsize,rsize);
      fclose(fd);
    }else{
      //--read file
      if(fread(&lytref->refmod[0][0],rsize,1,fd) != 1){
	perror("LYT: lyt_refmod fread");
	fclose(fd);
      }else{
	//--close file
	fclose(fd);
	printf("LYT: Read: %s\n",filename);
      }
    }
  }
  
  /****** READ PIXEL MASK FILE ******/
  //--setup filename
  sprintf(filename,LYTPIX2ALPZER_PXMASK_FILE);
  //--open file
  if((fd = fopen(filename,"r")) == NULL){
    perror("LYT: lyt_pxmask fopen");
  }else{
    //--check file size
    fseek(fd, 0L, SEEK_END);
    fsize = ftell(fd);
    rewind(fd);
    rsize = sizeof(lytref->pxmask);
    if(fsize != rsize){
      printf("LYT: incorrect lyt_pxmask file size %lu != %lu\n",fsize,rsize);
      fclose(fd);
    }else{
      //--read file
      if(fread(&lytref->pxmask[0][0],rsize,1,fd) != 1){
	perror("LYT: lyt_pxmaxk fread");
	fclose(fd);
      }else{
	//--close file
	fclose(fd);
	printf("LYT: Read: %s\n",filename);
      }
    }
  }
}

/**************************************************************/
/* LYT_LOADREF                                                */
/*  - Load reference image from file                          */
/**************************************************************/
void lyt_loadref(lytref_t *lytref){
  FILE *fd=NULL;
  char filename[MAX_FILENAME];
  uint64 fsize,rsize;
  double refimg[LYTXS][LYTYS];
  
  /* Open refimg file */
  //--setup filename
  sprintf(filename,LYT_REFIMG_FILE);
  //--open file
  if((fd = fopen(filename,"r")) == NULL){
    perror("LYT: loadref fopen");
    return;
  }
  //--check file size
  fseek(fd, 0L, SEEK_END);
  fsize = ftell(fd);
  rewind(fd);
  rsize = sizeof(refimg);
  if(fsize != rsize){
    printf("LYT: incorrect LYT_REFIMG_FILE size %lu != %lu\n",fsize,rsize);
    fclose(fd);
    return;
  }
  
  //Read file
  if(fread(&refimg[0][0],sizeof(refimg),1,fd) != 1){
    perror("LYT: loadref fread");
    fclose(fd);
    return;
  }
  
  //Close file
  fclose(fd);
  printf("LYT: Read: %s\n",filename);

  //Copy image into reference structure
  memcpy(&lytref->refimg[0][0],&refimg[0][0],sizeof(refimg));
}

/***************************************************************/
/* LYT_SAVEREF                                                 */
/*  - Save reference image to file                             */
/***************************************************************/
void lyt_saveref(lytref_t *lytref){
  struct stat st = {0};
  FILE *fd=NULL;
  static char outfile[MAX_FILENAME];
  char temp[MAX_FILENAME];
  char path[MAX_FILENAME];
  int i;
  
  /* Open output file */
  //--setup filename
  sprintf(outfile,"%s",LYT_REFIMG_FILE);
  //--create output folder if it does not exist
  strcpy(temp,outfile);
  strcpy(path,dirname(temp));
  if (stat(path, &st) == -1){
    printf("LYT: creating folder %s\n",path);
    recursive_mkdir(path, 0777);
  }
  //--open file
  if((fd = fopen(outfile, "w")) == NULL){
    perror("LYT: saveref fopen()\n");
    return;
  }
  
  //Save refimg
  if(fwrite(&lytref->refimg[0][0],sizeof(lytref->refimg),1,fd) != 1){
    printf("LYT: saveref fwrite error!\n");
    fclose(fd);
    return;
  }
  printf("LYT: Wrote: %s\n",outfile);
  
  //Close file
  fclose(fd);
  return;
}

/**************************************************************/
/* LYT_ALP_ZERNPID                                            */
/*  - Run PID controller on measured Zernikes for ALP         */
/**************************************************************/
void lyt_alp_zernpid(lytevent_t *lytevent, double *zernike_delta, int *zernike_switch, int reset){
  static int init = 0;
  static double zint[LOWFS_N_ZERNIKE] = {0};
  double zerr;
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
      zerr = lytevent->zernike_measured[i] - lytevent->zernike_target[i];
      //Calculate integral
      zint[i] += zerr;
      
      if(zint[i] > LYT_ALP_ZERN_INT_MAX) zint[i]=LYT_ALP_ZERN_INT_MAX;
      if(zint[i] < LYT_ALP_ZERN_INT_MIN) zint[i]=LYT_ALP_ZERN_INT_MIN;
      
      //Calculate command
      zernike_delta[i] = lytevent->gain_alp_zern[i][0] * zerr + lytevent->gain_alp_zern[i][1] * zint[i];
    }
  }
}

/**************************************************************/
/* LYT_ZERNIKE_FIT                                            */
/*  - Fit Zernikes to LYT pixels                              */
/**************************************************************/
void lyt_zernike_fit(lyt_t *image, lytref_t *lytref, double *zernikes, int reset){
  FILE   *fd=NULL;
  char   filename[MAX_FILENAME];
  static lytevent_t lytevent;
  static int init=0;
  static double lyt2zern_matrix[LYTXS*LYTYS*LOWFS_N_ZERNIKE]={0}; //zernike fitting matrix (max size)
  static int lyt_npix=0;
  double lyt_delta[LYTXS*LYTYS]={0}; //measured image - reference (max size)
  uint64 fsize,rsize;
  double img_total;
  double ref_total;
  int    i,j,count;
  uint16 maxpix;
  
  /* Initialize Fitting Matrix */
  if(!init || reset){

    /****** COUNT NUMBER OF CONTROLED PIXELS ******/
    lyt_npix=0;
    for(i=0;i<LYTXS;i++)
      for(j=0;j<LYTYS;j++)
	if(lytref->pxmask[i][j])
	  lyt_npix++;


    /****** READ ZERNIKE MATRIX FILE ******/
    //--setup filename
    sprintf(filename,LYTPIX2ALPZER_FILE);
    //--open file
    if((fd = fopen(filename,"r")) == NULL){
      perror("LYT: lyt2zern fopen");
    }else{
      //--check file size
      fseek(fd, 0L, SEEK_END);
      fsize = ftell(fd);
      rewind(fd);
      rsize = lyt_npix*LOWFS_N_ZERNIKE*sizeof(double);
      if(fsize != rsize){
	printf("LYT: incorrect lyt2zern matrix file size %lu != %lu\n",fsize,rsize);
	fclose(fd);
      }else{
	//--read matrix
	if(fread(lyt2zern_matrix,rsize,1,fd) != 1){
	  perror("LYT: lyt2zern fread");
	  fclose(fd);
	}else{
	  //--close file
	  fclose(fd);
	  printf("LYT: Read: %s\n",filename);
	}
      }
    }
    
    //--set init flag
    init=1;
    //--return if reset
    if(reset) return;
  }

  //Normalize image pixels & subtract reference image
  img_total=0;
  ref_total=0;
  maxpix=0;
  count=0;
  for(i=0;i<LYTXS;i++){
    for(j=0;j<LYTYS;j++){
      if(lytref->pxmask[i][j]){
	if(image->data[i][j] > maxpix) maxpix = image->data[i][j];
	img_total += (double)image->data[i][j];
	ref_total += lytref->refimg[i][j];
      }
    }
  }

  //Fit zernikes if above pixel threshold
  if(maxpix > LYT_PIXEL_THRESH){
    
    //Fill out pixel delta array
    if(img_total > 0 && ref_total > 0){
      for(i=0;i<LYTXS;i++)
	for(j=0;j<LYTYS;j++)
	  if(lytref->pxmask[i][j])
	    lyt_delta[count++]  = ((double)image->data[i][j])/img_total - lytref->refimg[i][j]/ref_total;
    }
    
    //Do matrix multiply
    num_dgemv(lyt2zern_matrix, lyt_delta, zernikes, LOWFS_N_ZERNIKE, lyt_npix);

    //Limit zernikes
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      zernikes[i] = zernikes[i] < LYT_ZERNIKE_MIN ? LYT_ZERNIKE_MIN : zernikes[i];
      zernikes[i] = zernikes[i] > LYT_ZERNIKE_MAX ? LYT_ZERNIKE_MAX : zernikes[i];
    }

  }else{
    //Zero out zernikes
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      zernikes[i] = 0;
  }
    
}

/**************************************************************/
/* LYT_PROCESS_IMAGE                                          */
/*  - Main image processing function for LYT                  */
/**************************************************************/
int lyt_process_image(stImageBuff *buffer,sm_t *sm_p){
  static lytevent_t lytevent;
  static lytpkt_t lytpkt;
  lytevent_t *lytevent_p;
  lytpkt_t *lytpkt_p;
  static calmode_t alpcalmodes[ALP_NCALMODES];
  static calmode_t hexcalmodes[HEX_NCALMODES];
  static calmode_t bmccalmodes[BMC_NCALMODES];
  static calmode_t tgtcalmodes[TGT_NCALMODES];
  static struct timespec start,end,delta,last;
  static uint32 frame_number=0;
  static int init=0;
  static lytref_t lytref;
  double dt;
  int i,j;
  uint16_t fakepx=0;
  int state;
  alp_t alp,alp_try,alp_delta;
  int zernike_control=0;
  int zernike_switch[LOWFS_N_ZERNIKE] = {0};
  uint32_t n_dither=1;
  int sample;

  //Image magnification
  double x,y,f_x_y1,f_x_y2;
  int x1,x2,y1,y2;
  
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
    memset(&lytevent,0,sizeof(lytevent_t));
    memset(&lytpkt,0,sizeof(lytpkt_t));
    //Init frame number
    frame_number=0;
    //Reset zern2alp mapping
    alp_zern2alp(NULL,NULL,FUNCTION_RESET);
    //Reset calibration routines
    alp_calibrate(sm_p,0,NULL,NULL,NULL,LYTID,FUNCTION_RESET);
    tgt_calibrate(0,NULL,NULL,LYTID,FUNCTION_RESET);
    //Reset PID controllers
    lyt_alp_zernpid(NULL,NULL,NULL,FUNCTION_RESET);
    //Init reference image structure
    lyt_initref(&lytref);
    //Reset zernike fitter
    lyt_zernike_fit(NULL,&lytref,NULL,FUNCTION_RESET);
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

  //Get sample
  sample = frame_number % LYT_NSAMPLES;

  //Fill out event header
  lytevent.hed.version       = PICC_PKT_VERSION;
  lytevent.hed.type          = BUFFER_LYTEVENT;
  lytevent.hed.frame_number  = frame_number;
  lytevent.hed.exptime       = sm_p->lyt_exptime;
  lytevent.hed.frmtime       = sm_p->lyt_frmtime;
  lytevent.hed.ontime        = dt;
  lytevent.hed.state         = state;
  lytevent.hed.alp_commander = sm_p->state_array[state].alp_commander;
  lytevent.hed.hex_commander = sm_p->state_array[state].hex_commander;
  lytevent.hed.bmc_commander = sm_p->state_array[state].bmc_commander;
  lytevent.hed.start_sec     = start.tv_sec;
  lytevent.hed.start_nsec    = start.tv_nsec;
  lytevent.hed.hex_calmode   = sm_p->hex_calmode;
  lytevent.hed.alp_calmode   = sm_p->alp_calmode;
  lytevent.hed.bmc_calmode   = sm_p->bmc_calmode;
  lytevent.hed.tgt_calmode   = sm_p->tgt_calmode;

  //Increment frame number
  frame_number++;

  //Save temperature
  lytevent.ccd_temp          = sm_p->lyt_ccd_temp;

  //Save origin
  lytevent.xorigin           = sm_p->lyt_xorigin;
  lytevent.yorigin           = sm_p->lyt_yorigin;
  
  //Save gains
  for(i=0;i<LOWFS_N_ZERNIKE;i++)
    for(j=0;j<LOWFS_N_PID;j++)
      lytevent.gain_alp_zern[i][j] = sm_p->lyt_gain_alp_zern[i][j] * sm_p->lyt_zernike_control[i];
  
  //Save zernike targets
  memcpy(lytevent.zernike_target,(double *)sm_p->lyt_zernike_target,sizeof(lytevent.zernike_target));

  //Run target calibration
  if((sm_p->state_array[state].alp_commander == LYTID))
    if(lytevent.hed.tgt_calmode != TGT_CALMODE_NONE)
      sm_p->tgt_calmode = tgt_calibrate(lytevent.hed.tgt_calmode,lytevent.zernike_target,&lytevent.hed.tgt_calstep,LYTID,FUNCTION_NO_RESET);

  //Copy full readout image to event
  memcpy(&lytevent.readimage.data[0][0],buffer->pvAddress,sizeof(lytread_t));
  
  //Fake data
  if(sm_p->w[LYTID].fakemode != FAKEMODE_NONE){
    if(sm_p->w[LYTID].fakemode == FAKEMODE_TEST_PATTERN)
      for(i=0;i<LYTXS;i++)
	for(j=0;j<LYTYS;j++)
	  lytevent.image.data[i][j]=fakepx++;
    if(sm_p->w[LYTID].fakemode == FAKEMODE_LYT_REFIMG)
      for(i=0;i<LYTXS;i++)
	for(j=0;j<LYTYS;j++)
	  lytevent.image.data[i][j]=lytref.refimg[i][j];
  }
  else{
    if(sm_p->lyt_mag_enable){
      //Run image magnification
      for(i=0;i<LYTXS;i++){
	for(j=0;j<LYTYS;j++){
	  //Define location of interpolated pixel -- transpose origin offsets
	  x = (i - LYTXS/2)/sm_p->lyt_mag + (LYTREADXS/2) + lytevent.yorigin - (LYTREADXS-LYTXS)/2;
	  y = (j - LYTYS/2)/sm_p->lyt_mag + (LYTREADYS/2) + lytevent.xorigin - (LYTREADYS-LYTYS)/2;
	  
	  //Pick 4 pixels for interpolation
	  x1 = (int)x;
	  x2 = x1 + 1;
	  y1 = (int)y;
	  y2 = y1 + 1;

	  //Calculate interpolated value
	  if(x1 >= 0 && x1 < LYTREADXS && x2 >= 0 && x2 < LYTREADXS && y1 >= 0 && y1 < LYTREADYS && y2 >= 0 && y2 < LYTREADYS){
	    f_x_y1  = (x2 - x) * lytevent.readimage.data[x1][y1] / (x2 - x1) + (x - x1) * lytevent.readimage.data[x2][y1] / (x2 - x1);
	    f_x_y2  = (x2 - x) * lytevent.readimage.data[x1][y2] / (x2 - x1) + (x - x1) * lytevent.readimage.data[x2][y2] / (x2 - x1);
	    lytevent.image.data[i][j] = (y2 - y) * f_x_y1 / (y2 - y1) + (y - y1) * f_x_y2 / (y2-y1);
	  }else{
	    //1 of 4 pixels is out of bounds --> use closest value
	    x = x < 0 ? 0 : x;
	    y = y < 0 ? 0 : y;
	    x = x >= LYTREADXS ? LYTREADXS-1 : x;
	    y = y >= LYTREADYS ? LYTREADYS-1 : y;
	    lytevent.image.data[i][j] = lytevent.readimage.data[(int)x][(int)y];
	  }
	}
      }
    }else{
      //Cut out ROI -- transpose origin offsets
      for(i=0;i<LYTXS;i++)
	for(j=0;j<LYTYS;j++)
	  lytevent.image.data[i][j]=lytevent.readimage.data[i+lytevent.yorigin][j+lytevent.xorigin];
    }
  }
  
  //Command: lyt_setref 
  if(sm_p->lyt_setref){
    //Copy current image to reference image
    for(i=0;i<LYTXS;i++)
      for(j=0;j<LYTYS;j++)
	lytref.refimg[i][j] = lytevent.image.data[i][j];
    sm_p->lyt_setref=0;
  }
  //Command: lyt_defref 
  if(sm_p->lyt_defref){
    //Switch to default reference image
    memcpy(&lytref.refimg[0][0],&lytref.refdef[0][0],sizeof(lytref.refimg));
    sm_p->lyt_defref=0;
  }
  //Command: lyt_modref 
  if(sm_p->lyt_modref){
    //Switch to model reference image
    memcpy(&lytref.refimg[0][0],&lytref.refmod[0][0],sizeof(lytref.refimg));
    sm_p->lyt_modref=0;
  }
  //Command: lyt_saveref 
  if(sm_p->lyt_saveref){
    //Save current reference image to disk
    lyt_saveref(&lytref);
    sm_p->lyt_saveref=0;
  }
  //Command: lyt_loadref 
  if(sm_p->lyt_loadref){
    //Load saved reference image from disk
    lyt_loadref(&lytref);
    sm_p->lyt_loadref=0;
  }

  //Fit Zernikes
  if(sm_p->state_array[state].lyt.fit_zernikes)
    lyt_zernike_fit(&lytevent.image,&lytref,lytevent.zernike_measured, FUNCTION_NO_RESET);


  /*************************************************************/
  /*******************  ALPAO DM Control Code  *****************/
  /*************************************************************/

  //Check if we will send a command
  if((sm_p->state_array[state].alp_commander == LYTID) && sm_p->alp_ready){

    //Get last ALP command
    if(alp_get_command(sm_p,&alp)){
      //Skip this image
      return 0;
    }
    memcpy(&alp_try,&alp,sizeof(alp_t));

    //Check if ALP is controlling any Zernikes
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      if(sm_p->lyt_zernike_control[i] && (sm_p->state_array[state].lyt.zernike_control[i] == ACTUATOR_ALP)){
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
      sm_p->alp_calmode = alp_calibrate(sm_p,lytevent.hed.alp_calmode,&alp_try,&lytevent.hed.alp_calstep,lytevent.zernike_calibrate,LYTID,FUNCTION_NO_RESET);
    
    //Send command to ALP
    if(alp_send_command(sm_p,&alp_try,LYTID,n_dither)){
      // - command failed
      // - do nothing for now
    }else{
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
      lytpkt.alp_zcmd[i][sample] = lytevent.alp.zcmd[i];
    }
  
    //Last sample, fill out rest of packet and write to circular buffer
    if(sample == LYT_NSAMPLES-1){
      //Header
      memcpy(&lytpkt.hed,&lytevent.hed,sizeof(pkthed_t));
      lytpkt.hed.type = BUFFER_LYTPKT;
      //Image
      memcpy(&lytpkt.image,&lytevent.image,sizeof(lyt_t));
      //Zernike gains and targets
      for(i=0;i<LOWFS_N_ZERNIKE;i++){
	lytpkt.zernike_target[i]         = lytevent.zernike_target[i]; 
	for(j=0;j<LOWFS_N_PID;j++){
	  lytpkt.gain_alp_zern[i][j]     = lytevent.gain_alp_zern[i][j];
	}
      }
      
      //Actuator commands
      for(i=0;i<ALP_NACT;i++){
	lytpkt.alp_acmd[i] = lytevent.alp.acmd[i];
      }
      
      //CCD Temp
      lytpkt.ccd_temp = lytevent.ccd_temp;

      //Origins
      lytpkt.xorigin  = lytevent.xorigin;
      lytpkt.yorigin  = lytevent.yorigin;
            
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

  return 0;
}
