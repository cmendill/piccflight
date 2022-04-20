#define _XOPEN_SOURCE 500
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <libfli.h>
#include <libgen.h>
#include <sys/stat.h>
#include <gsl_multimin.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "sci_functions.h"

/* Process File Descriptor */
int sci_shmfd;

/* FLI File Descriptor */
flidev_t dev;

/**************************************************************/
/* SCICTRLC                                                   */
/*  - Main process interrupt routine                          */
/**************************************************************/
void scictrlC(int sig){
  uint32 err = 0;

  /* Cancel Exposure */
  if((err = FLICancelExposure(dev))){
    fprintf(stderr, "SCI: Error FLICancelExposure: %s\n", strerror((int)-err));
  }else{
    if(SCI_DEBUG) printf("SCI: Exposure stopped\n");
  }
  /* Disable TEC */
  if((err = FLISetTemperature(dev, SCI_TEC_SETPOINT_MAX))){
    fprintf(stderr, "SCI: Error FLISetTemperature: %s\n", strerror((int)-err));
  }else{
    if(SCI_DEBUG) printf("SCI: FLI TEC Disabled\n");
  }
  /* Close FLI camera */
  if((err = FLIClose(dev))){
    fprintf(stderr, "SCI: Error FLIClose: %s\n", strerror((int)-err));
  }else{
    if(SCI_DEBUG) printf("SCI: FLI closed\n");
  }
  
  /* Close shared memory */
  close(sci_shmfd);
  
  if(MSG_CTRLC) printf("SCI: exiting\n");

  /* Exit */
  exit(sig);
}

/**************************************************************/
/* SCI_PHASE_MERIT                                            */
/*  - Phase flattening merit function                         */
/**************************************************************/
double sci_phase_zernike_merit(const gsl_vector *v, void *params){
  int i,j,rc;
  double merit,meritA,meritB,meritC;
  double meritAA=0,meritAB=0,meritAC=0;
  double meritBA=0,meritBB=0,meritBC=0;
  double meritCA=0,meritCB=0,meritCC=0;
  double maxval=0;
  sci_t  rawA,rawB,rawC;
  double imageA[SCIXS][SCIYS];
  double imageB[SCIXS][SCIYS];
  double imageC[SCIXS][SCIYS];
  double zernike_target[LOWFS_N_ZERNIKE]={0};
  float exptime;
  uint32_t err = 0;
  static double targetA[SCIXS][SCIYS];
  static double targetB[SCIXS][SCIYS];
  static double targetC[SCIXS][SCIYS];
  static double weightA[SCIXS][SCIYS];
  static double weightB[SCIXS][SCIYS];
  static double weightC[SCIXS][SCIYS];
  static int init=0;
  struct gsl_param{
    sm_t *sm_p;
    uint16_t *img_buffer;
  } *gsl_param;
  
  //Read in 3 target & weight images
  if(!init){
    if(read_file(SCI_PHASE_TARGET_A_FILE,&targetA[0][0],sizeof(targetA)))
      printf("SCI: Read phase target A image failed\n");
    if(read_file(SCI_PHASE_TARGET_B_FILE,&targetB[0][0],sizeof(targetB)))
      printf("SCI: Read phase target B image failed\n");
    if(read_file(SCI_PHASE_TARGET_C_FILE,&targetC[0][0],sizeof(targetC)))
      printf("SCI: Read phase target C image failed\n");
    if(read_file(SCI_PHASE_WEIGHT_A_FILE,&weightA[0][0],sizeof(weightA)))
      printf("SCI: Read phase weight A image failed\n");
    if(read_file(SCI_PHASE_WEIGHT_B_FILE,&weightB[0][0],sizeof(weightB)))
      printf("SCI: Read phase weight B image failed\n");
    if(read_file(SCI_PHASE_WEIGHT_C_FILE,&weightC[0][0],sizeof(weightC)))
      printf("SCI: Read phase weight C image failed\n");
    printf("SCI: sci_phase_zernike_merit initialized\n");
    init=1;
  }

  //Get parameters
  gsl_param = params;
  sm_t *sm_p = gsl_param->sm_p;
  uint16_t *img_buffer = gsl_param->img_buffer;
  
  //Get zernike target command
  for(i=0;i<sm_p->sci_phase_n_zernike;i++)
    zernike_target[i] = gsl_vector_get(v,i);

  //Generate 3 images 0,1,2 | B,C,A | +0.2,-0.2,0
  for(i=0;i<3;i++){
    memcpy((double *)sm_p->shk_zernike_target,zernike_target,sizeof(zernike_target));
    if(i == 0) sm_p->shk_zernike_target[2] += 0.2;
    if(i == 1) sm_p->shk_zernike_target[2] -= 0.2;
    sm_p->sci_iphase = i;
    printf("SCI: iphase = %d\n",sm_p->sci_iphase);
    
    /* Allow SHK to settle */
    sleep(1);

    /* Set exposure time */
    if(i==0) exptime = sm_p->sci_exptime*10;
    if(i==1) exptime = sm_p->sci_exptime*10;
    if(i==2) exptime = sm_p->sci_exptime;
    if((err = FLISetExposureTime(dev, lround(exptime * 1000)))){
      fprintf(stderr, "SCI: Error FLISetExposureTime: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI exposure time set\n");
    }
   
    /* Run Exposure */
    if((rc=sci_expose(sm_p,dev,img_buffer))){
      if(rc==SCI_EXP_RETURN_FAIL)
	printf("SCI: Exposure failed\n");
      if(rc==SCI_EXP_RETURN_KILL)
	scictrlC(0);
      if(rc==SCI_EXP_RETURN_ABORT)
	printf("SCI: Exposure aborted\n");
    }
    else{
      /* Process Image */
      sci_process_image(img_buffer,exptime,sm_p);
    }
  }
  
  //Read in measured images
  if(read_file(SCI_PHASE_IMAGE_A_FILE,&rawA,sizeof(sci_t)))
    printf("SCI: Read phase frames failed\n");
  if(read_file(SCI_PHASE_IMAGE_B_FILE,&rawB,sizeof(sci_t)))
    printf("SCI: Read phase frames failed\n");
  if(read_file(SCI_PHASE_IMAGE_C_FILE,&rawC,sizeof(sci_t)))
    printf("SCI: Read phase frames failed\n");

  //Copy measured images -- divide by exposure time if different
  for(i=0;i<SCIXS;i++){
    for(j=0;j<SCIYS;j++){
      imageA[i][j]=(double)rawA.data[i][j];
      imageB[i][j]=(double)rawB.data[i][j];
      imageC[i][j]=(double)rawC.data[i][j];
      if(imageA[i][j] > maxval) maxval = imageA[i][j];
    }
  }
  
  //Scale measured images
  for(i=0;i<SCIXS;i++){
    for(j=0;j<SCIYS;j++){
      imageA[i][j]/=maxval;
      imageB[i][j]/=maxval;
      imageC[i][j]/=maxval;
    }
  }
  
  //Calculate individual merit functions
  for(i=0;i<SCIXS;i++){
    for(j=0;j<SCIYS;j++){
      meritAA += weightA[i][j]*imageA[i][j]*targetA[i][j];
      meritAB += weightA[i][j]*imageA[i][j]*imageA[i][j];
      meritAC += weightA[i][j]*targetA[i][j]*targetA[i][j];

      meritBA += weightB[i][j]*imageB[i][j]*targetB[i][j];
      meritBB += weightB[i][j]*imageB[i][j]*imageB[i][j];
      meritBC += weightB[i][j]*targetB[i][j]*targetB[i][j];
      
      meritCA += weightC[i][j]*imageC[i][j]*targetC[i][j];
      meritCB += weightC[i][j]*imageC[i][j]*imageC[i][j];
      meritCC += weightC[i][j]*targetC[i][j]*targetC[i][j];
    }
  }
  meritA = meritAA*meritAA / (meritAB * meritAC);
  meritB = meritBA*meritBA / (meritBB * meritBC);
  meritC = meritCA*meritCA / (meritCB * meritCC);

  //Combine merit functions
  merit = 1.0 - (1.0/(double)3)*(meritA + meritB + meritC);
  //printf("SCI: Phase merit = %f\n",merit);
  
  //Return single value of merit function
  return merit;
}

/**************************************************************/
/* SCI_PROC                                                   */
/*  - Main SCI camera process                                 */
/**************************************************************/
void sci_proc(void){
  char file[MAX_FILENAME], name[MAX_FILENAME];
  uint32_t err = 0;
  long domain = (FLIDOMAIN_USB | FLIDEVICE_CAMERA);
  uint16_t *img_buffer;
  int camera_running=0;
  float exptime;
  flimode_t mode_index;
  char mode_string[128];
  size_t mode_size=128;
  int i;
  int rc;
  int phase_n_zernike;
  
  //GSL Minimizer Setup
  int min_status;
  double min_size;
  const gsl_multimin_fminimizer_type *T;
  gsl_multimin_fminimizer *s;
  gsl_vector *ss,*x;
  gsl_multimin_function my_func;
  int gsl_iter=0;
  struct gsl_param{
    sm_t *sm_p;
    uint16_t *img_buffer;
  } gsl_param;
  
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&sci_shmfd)) == NULL){
    printf("openshm fail: sci_proc\n");
    scictrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, scictrlC);	/* usually ^C */


  /**************************************************************/
  /*                      FLI Camera Setup                      */
  /**************************************************************/
  
  /* Malloc image buffer */
  img_buffer = (uint16_t *)malloc(SCI_ROI_XSIZE*SCI_ROI_YSIZE*sizeof(uint16_t));
  
  /* Create Device List */
  if((err = FLICreateList(domain))){
    fprintf(stderr, "SCI: Error FLICreateList: %s\n", strerror((int)-err));
    scictrlC(0);
  }else{
    if(SCI_DEBUG) printf("SCI: FLI list created\n");
    
    
    /* Create Device List */
    if((err = FLIListFirst(&domain, file, MAX_FILENAME, name, MAX_FILENAME))){
      fprintf(stderr, "SCI: Error FLIListFirst: %s\n", strerror((int)-err));
      scictrlC(0);
    }else{
      if(SCI_DEBUG) printf("SCI: FLI first device found\n");
      FLIDeleteList();
    }
  }
  
  /* Open Device */
  if((err = FLIOpen(&dev, file, domain))){
    fprintf(stderr, "SCI: Error FLIOpen: %s\n", strerror((int)-err));
    scictrlC(0);
  }

  
  /* ----------------------- Enter Main Loop ----------------------- */
  while(1){
    
    /* Check in with the watchdog */
    checkin(sm_p,SCIID);

    /**************************************************************/
    /*                         FLI Config                         */
    /**************************************************************/
    
    /* Cancel Exposure, put camera in known state */
    if((err = FLICancelExposure(dev))){
      fprintf(stderr, "SCI: Error FLICancelExposure: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: Exposure stopped\n");
    }
    camera_running = 0;
      
    /* Set temperature */
    if(sm_p->sci_tec_enable){
      if((err = FLISetTemperature(dev, sm_p->sci_tec_setpoint))){
	fprintf(stderr, "SCI: Error FLISetTemperature: %s\n", strerror((int)-err));
      }else{
	if(SCI_DEBUG) printf("SCI: FLI temperature set to %d\n",sm_p->sci_tec_setpoint);
      }
    }else{
      /* Disable TEC */
      if((err = FLISetTemperature(dev, SCI_TEC_SETPOINT_MAX))){
	fprintf(stderr, "SCI: Error FLISetTemperature: %s\n", strerror((int)-err));
      }else{
	if(SCI_DEBUG) printf("SCI: FLI TEC Disabled\n");
      }
    }
    
    /* Set exposure time */
    exptime = sm_p->sci_exptime;
    if((err = FLISetExposureTime(dev, lround(exptime * 1000)))){
      fprintf(stderr, "SCI: Error FLISetExposureTime: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI exposure time set\n");
    }

    /* Set frame type */
    if((err = FLISetFrameType(dev, FLI_FRAME_TYPE_NORMAL))){
      fprintf(stderr, "SCI: Error FLISetFrameType: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI frame type set\n");
    }

    /* Set ROI */
    if((err = FLISetImageArea(dev, SCI_UL_X, SCI_UL_Y, SCI_LR_X, SCI_LR_Y))){
      fprintf(stderr, "SCI: Error FLISetImageArea: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI image area set\n");
    }
      
    /* Set horizontal binning */
    if((err = FLISetHBin(dev, SCI_HBIN))){
      fprintf(stderr, "SCI: Error FLISetHBin: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI HBin set\n");
    }

    /* Set vertical binning */
    if((err = FLISetVBin(dev, SCI_VBIN))){
      fprintf(stderr, "SCI: Error FLISetVBin: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI VBin set\n");
    }

    /* Set number of flushes */
    if((err = FLISetNFlushes(dev, SCI_NFLUSHES))){
      fprintf(stderr, "SCI: Error FLISetNFlushes: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI NFlushes set\n");
    }

    /* Set camera mode */
    mode_index = SCI_MODE_10MHZ;
    if((err = FLISetCameraMode(dev, mode_index))){
      fprintf(stderr, "SCI: Error FLI SetCameraMode: %s\n", strerror((int)-err));
    }
     
    /* Get camera mode */
    if(SCI_DEBUG){
      if((err = FLIGetCameraMode(dev, &mode_index))){
	fprintf(stderr, "SCI: Error FLIGetCameraMode: %s\n", strerror((int)-err));
      }else{
	FLIGetCameraModeString(dev,mode_index,mode_string,mode_size);
	printf("SCI: Mode %d = %s\n",(int)mode_index,mode_string);
      
	for(mode_index=0;mode_index<10;mode_index++){
	  if(FLIGetCameraModeString(dev,mode_index,mode_string,mode_size))
	    break;
	  else printf("SCI: Supported mode %d = %s\n",(int)mode_index,mode_string);
	}
      }
    }

    /* ----------------------- Enter Exposure Loop ----------------------- */
    while(1){
      /* Check if we've been asked to exit */
      if(sm_p->w[SCIID].die)
	scictrlC(0);

      /* Check if we've been asked to reset the exposure */
      if(sm_p->sci_reset_camera){
	printf("SCI: Resetting exposure\n");
	sm_p->sci_reset_camera = 0;
	break;
      }
      
      /* Check if camera should start */
      if(!camera_running){
	camera_running = 1;
	printf("SCI: Camera started\n");
      }
      
      /* Get CCD Temperature */
      sm_p->sci_ccd_temp = sci_get_temp(dev);
      
      /* Get TEC Power */
      sm_p->sci_tec_power = sci_get_tec_power(dev);
      
      /* Perform Phase Flattening */
      if(sm_p->sci_phase_run){
	//Initialize GSL Minimizer
	if(gsl_iter == 0){
	  printf("SCI: Starting GSL initialization...\n");
	  //Setup user function
	  phase_n_zernike = sm_p->sci_phase_n_zernike;
	  my_func.n = phase_n_zernike;
	  my_func.f = sci_phase_zernike_merit;
	  gsl_param.sm_p = sm_p;
	  gsl_param.img_buffer = img_buffer;
	  my_func.params = &gsl_param;
  
	  //Define algorithm
	  T = gsl_multimin_fminimizer_nmsimplex2;
    
	  //Init algorithm
	  s = gsl_multimin_fminimizer_alloc(T,phase_n_zernike);
    
	  //Define starting point (initial guess)
	  x = gsl_vector_alloc(phase_n_zernike);
	  for(i=0;i<phase_n_zernike;i++)
	    gsl_vector_set(x,i,sm_p->shk_zernike_target[i]);
    
	  //Set inital step sizes to 5nm
	  ss = gsl_vector_alloc(phase_n_zernike);
	  gsl_vector_set_all(ss, 0.005);
  
	  //Set runtime parameters for the minimizer
	  gsl_multimin_fminimizer_set(s, &my_func, x, ss);
	  printf("SCI: GSL initialization done\n");
	}
	
	//Increment counter
	gsl_iter++;

	//Iterate minimizer
	min_status = gsl_multimin_fminimizer_iterate(s);
	if(min_status)
	  printf("SCI: gsl_multimin_fminimizer_iterate error\n");
	
	//Get size
	min_size = gsl_multimin_fminimizer_size(s);
	min_status = gsl_multimin_test_size(min_size, 0.01);
	
	//Print status
	printf("SCI: Phase iter=%d f=%10.5f size=%.3f\n", gsl_iter, s->fval, min_size);

 	//Check for convergence
	if(min_status == GSL_SUCCESS)
	  printf("SCI: Minimum found after %d iterations\n",gsl_iter);
      }
      else{
	/* Reset GSL */
	gsl_iter=0;
	
	/* Run Normal Exposure */
	if((rc=sci_expose(sm_p,dev,img_buffer))){
	  if(rc==SCI_EXP_RETURN_FAIL)
	    printf("SCI: Exposure failed\n");
	  if(rc==SCI_EXP_RETURN_KILL)
	    scictrlC(0);
	  if(rc==SCI_EXP_RETURN_ABORT)
	    printf("SCI: Exposure aborted\n");
	}
	else{
	  /* Process Image */
	  sci_process_image(img_buffer,exptime,sm_p);
	}
      }
    }
  }
  
  /* Exit */
  scictrlC(0);
  return;
}
