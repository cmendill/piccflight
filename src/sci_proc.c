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
#include <libtnc.h>
#include <sys/stat.h>
#include <gsl_multimin.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "sci_functions.h"
#include "alp_functions.h"
#include "fakemodes.h"

/* Process File Descriptor */
int sci_shmfd;

/* FLI File Descriptor */
flidev_t dev;

typedef struct opt_param{
  sm_t *sm_p;
  uint16_t *img_buffer;
  int nvar;
  alp_t alp;
  int phasemode;
  int calc_gradient;
  double gstep;
} opt_param_t;

//Optimizer Prototypes
#if PHASE_OPTIMIZER == OPTIMIZER_GSL
double sci_phase_merit(const gsl_vector *v, void *params);
void sci_phase_merit_gradient(const gsl_vector *v, void *params, gsl_vector *df);
#endif
#if PHASE_OPTIMIZER == OPTIMIZER_TNC
int sci_phase_merit(double v[], double *fval, double df[], void *params);
void sci_phase_merit_gradient(double *var, double merit, double df[], opt_param_t *opt_param);
#endif

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
  /* Turn off background flushing */
  if((err = FLIControlBackgroundFlush(dev, FLI_BGFLUSH_STOP))){
    fprintf(stderr, "SCI: Error FLIControlBackgroundFlush: %s\n", strerror((int)-err));
  }else{
    if(SCI_DEBUG) printf("SCI: FLI Background Flushing Stopped\n");
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
/* SCI_PHASE_MERIT_GRADIENT                                   */
/*  - Phase flattening merit function gradient                */
/**************************************************************/
#if PHASE_OPTIMIZER == OPTIMIZER_GSL
void sci_phase_merit_gradient(const gsl_vector *v, void *params, gsl_vector *df){

}
#endif

#if PHASE_OPTIMIZER == OPTIMIZER_TNC
void sci_phase_merit_gradient(double *var, double merit, double df[], opt_param_t *opt_param){
  int i;
  double dp,dm;
  opt_param_t grad_param;
  
  //Create delta array
  double *del = malloc(sizeof(double) * opt_param->nvar);

  //Copy parameters, zero out gradient switch
  memcpy(&grad_param,opt_param,sizeof(opt_param_t));
  grad_param.calc_gradient = 0;
  
  //Loop over variables
  for(i=0;i<opt_param->nvar;i++){
    //Copy variables
    memcpy(del,var,sizeof(double) * opt_param->nvar);
    
    //Run positive delta
    if(opt_param->calc_gradient >= 1){
      del[i] += opt_param->gstep;
      sci_phase_merit(del, &dp, df, (void *)&grad_param);	
    }

    //Run negative delta
    if(opt_param->calc_gradient == 2){
      del[i] -= 2*opt_param->gstep;
      sci_phase_merit(del, &dm, df, (void *)&grad_param);	
    }

    //Calculate derivative
    if(opt_param->calc_gradient == 1) df[i]=(dp - merit)/(opt_param->gstep);
    if(opt_param->calc_gradient == 2) df[i]=(dp - dm)/(2*opt_param->gstep);

    //printf("TNC: grad[%d]=%f\n",i,df[i]);
  }
  
  //Free malloc
  if(del) free(del);
  
}
#endif

/**************************************************************/
/* SCI_PHASE_MERIT                                            */
/*  - Phase flattening merit function                         */
/**************************************************************/
#if PHASE_OPTIMIZER == OPTIMIZER_GSL
double sci_phase_merit(const gsl_vector *v, void *params){
#endif

#if PHASE_OPTIMIZER == OPTIMIZER_TNC
tnc_function sci_phase_merit;
int sci_phase_merit(double v[], double *fval, double df[], void *params){
#endif
  
  int i,j,rc;
  double merit,meritA,meritB,meritC,dp,dm;
  double meritAA=0,meritAB=0,meritAC=0;
  double meritBA=0,meritBB=0,meritBC=0;
  double meritCA=0,meritCB=0,meritCC=0;
  double totval=0;
  double imageA[SCIXS][SCIYS];
  double imageB[SCIXS][SCIYS];
  double imageC[SCIXS][SCIYS];
  float exptime;
  uint32_t err = 0;
  static double targetA[SCIXS][SCIYS];
  static double targetB[SCIXS][SCIYS];
  static double targetC[SCIXS][SCIYS];
  static double weightA[SCIXS][SCIYS];
  static double weightB[SCIXS][SCIYS];
  static double weightC[SCIXS][SCIYS];
  static int init=0;
  opt_param_t *opt_param;
  double dZ[3]={0.2,-0.2,0};
  double zcmd[LOWFS_N_ZERNIKE]={0};
  double act[ALP_NACT];
  alp_t alp;
  int n_dither=1;
  double focus=0;
  
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
  opt_param = params;
  sm_t *sm_p = opt_param->sm_p;
  uint16_t *img_buffer = opt_param->img_buffer;
  int nvar = opt_param->nvar;
  int phasemode = opt_param->phasemode;

  //Check phasemode
#if PHASE_OPTIMIZER == OPTIMIZER_TNC
  if(sm_p->sci_phasemode == SCI_PHASEMODE_NONE)
    return 0;
#endif
  
  //Check in with the watchdog
  checkin(sm_p,SCIID);
  
  //Initialize variable array
  double *var = malloc(sizeof(double) * nvar);

  //Get variable values
#if PHASE_OPTIMIZER == OPTIMIZER_GSL
  for(j=0;j<nvar;j++)
    var[j] = gsl_vector_get(v,j);
#endif
#if PHASE_OPTIMIZER == OPTIMIZER_TNC
  for(j=0;j<nvar;j++)
    var[j] = v[j];
#endif

  
  //Print ZCOMMAND
  if(0){
    if(phasemode == SCI_PHASEMODE_ZCOMMAND){
      printf("SCI: ");
      for(j=0;j<nvar;j++)
	printf("%4.1f ",var[j]*1000);
      printf("\n");
    }
  }
  
  //Generate 3 images 0,1,2 | B,C,A | +0.2,-0.2,0
  for(i=0;i<3;i++){
    //Set iphase
    sm_p->sci_iphase = i;
    //printf("SCI: iphase = %d\n",sm_p->sci_iphase);
    
    //Send target command
    if(phasemode == SCI_PHASEMODE_ZTARGET){
      //Copy command into zernike target array
      for(j=0;j<nvar;j++)
	sm_p->shk_zernike_target[j] = var[j];
      //Apply defocus
      if(i==0) focus = sm_p->shk_zernike_target[2];
      sm_p->shk_zernike_target[2] = focus + dZ[i];
      //Allow SHK to Settle
      sleep(1);
    }
    
    //Send zernike command
    if(phasemode == SCI_PHASEMODE_ZCOMMAND){
      //Copy command into full zernike array
      for(j=0;j<nvar;j++)
	zcmd[j] = var[j];
      //Apply defocus
      if(i==0) focus = zcmd[2];
      zcmd[2] = focus + dZ[i];
      //Convert current zernike command to actuator deltas
      alp_zern2alp(zcmd,act,FUNCTION_NO_RESET);
      //Add to starting actuator command
      for(j=0;j<ALP_NACT;j++)
	alp.acmd[j] = opt_param->alp.acmd[j] + act[j];
      //Send command to ALP
      if(sm_p->state_array[sm_p->state].alp_commander == SCIID){
	if(alp_send_command(sm_p,&alp,SCIID,n_dither)==0){
	  //Allow ALP to Settle
	  usleep(100000);
	}
      }
    }
    
    //Send actuator command
    if(phasemode == SCI_PHASEMODE_ACOMMAND){
      //Copy command into full actuator array
      for(j=0;j<nvar;j++)
	alp.acmd[j] = var[j];
      //Apply defocus
      zcmd[2] = dZ[i];
      alp_zern2alp(zcmd,act,FUNCTION_NO_RESET);
      for(j=0;j<ALP_NACT;j++)
	alp.acmd[j] += act[j];
      //Send command to ALP
      if(sm_p->state_array[sm_p->state].alp_commander == SCIID){
	if(alp_send_command(sm_p,&alp,SCIID,n_dither)==0){
	  //Allow ALP to Settle
	  usleep(100000);
	}
      }
    }

    
    /* Set exposure time */
    exptime = sm_p->sci_exptime;
    if(sm_p->w[SCIID].fakemode == FAKEMODE_NONE)
      if(i<2) exptime = sm_p->sci_exptime*10;
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
  
  //Read in measured images -- already background subtracted and divided by exposure time
  if(read_file(SCI_PHASE_IMAGE_A_FILE,&imageA[0][0],sizeof(imageA)))
    printf("SCI: Read phase frames failed\n");
  if(read_file(SCI_PHASE_IMAGE_B_FILE,&imageB[0][0],sizeof(imageB)))
    printf("SCI: Read phase frames failed\n");
  if(read_file(SCI_PHASE_IMAGE_C_FILE,&imageC[0][0],sizeof(imageC)))
    printf("SCI: Read phase frames failed\n");

  //Normalize images
  totval=0;
  for(i=0;i<SCIXS;i++)
    for(j=0;j<SCIYS;j++)
      if(weightA[i][j]) totval += imageA[i][j];
  for(i=0;i<SCIXS;i++)
    for(j=0;j<SCIYS;j++)
      imageA[i][j] /= totval;
  totval=0;
  for(i=0;i<SCIXS;i++)
    for(j=0;j<SCIYS;j++)
      if(weightB[i][j]) totval += imageB[i][j];
  for(i=0;i<SCIXS;i++)
    for(j=0;j<SCIYS;j++)
      imageB[i][j] /= totval;
  totval=0;
  for(i=0;i<SCIXS;i++)
    for(j=0;j<SCIYS;j++)
      if(weightC[i][j]) totval += imageC[i][j];
  for(i=0;i<SCIXS;i++)
    for(j=0;j<SCIYS;j++)
      imageC[i][j] /= totval;

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
  merit = 1.0 - (1.0/3.0)*(meritA + meritB + meritC);

  
  //Calculate gradients
#if PHASE_OPTIMIZER == OPTIMIZER_TNC
  if(opt_param->calc_gradient > 0){
    printf("SCI: %f: ",merit);
    for(j=0;j<nvar;j++)
      printf("%4.1f ",var[j]*1000);
    printf("\n");
    sci_phase_merit_gradient(var, merit, df, opt_param);
 
  }
#endif

  //Free malloc
  if(var) free(var);
  
  //Return single value of merit function
#if PHASE_OPTIMIZER == OPTIMIZER_GSL
  return merit;
#endif
#if PHASE_OPTIMIZER == OPTIMIZER_TNC
  *fval = merit;
  return 0;
#endif
 
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
  int nvar=0;
  alp_t alp;
  int phasemode;
  long int ulx,uly,lrx,lry;
  
  //GSL Minimizer Setup
  int min_status;
  double min_size;
  const gsl_multimin_fminimizer_type *T;
  gsl_multimin_fminimizer *s;
  gsl_vector *ss,*x;
  gsl_multimin_function my_func;
  int opt_iter=0;
  opt_param_t opt_param;
  
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
    ulx = SCI_UL_X;
    uly = SCI_UL_Y;
    lrx = SCI_LR_X;
    lry = SCI_LR_Y;
    if((SCI_NBANDS==1) && (sm_p->sci_fastmode==1)){
      ulx = sm_p->sci_xorigin[0] - SCIXS/2;
      uly = sm_p->sci_yorigin[0] - SCIYS/2;
      lrx = ulx + SCIXS;
      lry = uly + SCIYS;
    }
    printf("SCI: Setting ROI [%ld, %ld, %ld, %ld]\n", ulx, uly, lrx, lry);
    if((err = FLISetImageArea(dev, ulx, uly, lrx, lry))){
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

    /* Turn on background flushing */
    if((err = FLIControlBackgroundFlush(dev, FLI_BGFLUSH_START))){
      fprintf(stderr, "SCI: Error FLIControlBackgroundFlush: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI Background Flushing Started\n");
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
      phasemode = sm_p->sci_phasemode;
      if(phasemode != SCI_PHASEMODE_NONE){
	//Initialize Optimizer
	if(opt_iter == 0){

	  //Get starting ALP command
	  if(sm_p->state_array[sm_p->state].alp_commander == SCIID){
	    if(alp_get_command(sm_p,&alp)){
	      printf("SCI: Failed to get ALP Command\n");
	      memset(&alp,0,sizeof(alp));
	    }
	    memcpy(&opt_param.alp,&alp,sizeof(alp));
	  }

	  //Setup optimizer parameters
	  if(phasemode == SCI_PHASEMODE_ZTARGET) nvar = sm_p->sci_phase_n_zernike;
	  if(phasemode == SCI_PHASEMODE_ZCOMMAND) nvar = sm_p->sci_phase_n_zernike;
	  if(phasemode == SCI_PHASEMODE_ACOMMAND) nvar = ALP_NACT;
	  opt_param.sm_p = sm_p;
	  opt_param.img_buffer = img_buffer;
	  opt_param.nvar = nvar;
	  opt_param.phasemode = phasemode;
	  opt_param.calc_gradient = 0;
	  
#if PHASE_OPTIMIZER == OPTIMIZER_GSL
	  printf("SCI: Starting GSL initialization...\n");
	  //Setup user function
	  my_func.n = nvar;
	  my_func.f = sci_phase_merit;
	  my_func.params = &opt_param;
	  
	  //Define algorithm
	  T = gsl_multimin_fminimizer_nmsimplex2;
    
	  //Init algorithm
	  s = gsl_multimin_fminimizer_alloc(T,nvar);
    
	  //Define starting point (initial guess)
	  x = gsl_vector_alloc(nvar);
	  
	  if(phasemode == SCI_PHASEMODE_ZTARGET)
	    for(i=0;i<nvar;i++)
	      gsl_vector_set(x,i,sm_p->shk_zernike_target[i]);

	  if(phasemode == SCI_PHASEMODE_ZCOMMAND)
	    gsl_vector_set_all(x,0);

	  if(phasemode == SCI_PHASEMODE_ACOMMAND)
	    for(i=0;i<nvar;i++)
	      gsl_vector_set(x,i,alp.acmd[i]);
	  
	  //Set inital step sizes
	  ss = gsl_vector_alloc(nvar);
	  if(phasemode == SCI_PHASEMODE_ZTARGET) gsl_vector_set_all(ss, 0.005);
	  if(phasemode == SCI_PHASEMODE_ZCOMMAND) gsl_vector_set_all(ss, 0.005);
	  if(phasemode == SCI_PHASEMODE_ACOMMAND) gsl_vector_set_all(ss, 0.01);
	  
	  //Set runtime parameters for the minimizer
	  gsl_multimin_fminimizer_set(s, &my_func, x, ss);
	  printf("SCI: GSL initialization done\n");
#endif
	  
#if PHASE_OPTIMIZER == OPTIMIZER_TNC
	  //Define TNC parameters
	  double *x;               //Starting variable values
	  double *llimit,*ulimit;  //Variable limits
	  int maxCGit = -1;        //Max number of hessian*vector evaluation per main iteration
	  int maxnfeval = 200;     //Max number of function evaluations
	  double eta = -1.0;       //Severity of the line search
	  double stepmx = 10;      //Maximum step for the line search
	  double accuracy = -1.0;  //Relative precision for finite difference calculations
	  double fmin = 0.0;       //Minimum function value estimate
	  double ftol = 0.01;      //Stop if abs(f_n-f_n-1) < ftol
	  double xtol = 0.001;     //Stop if abs(x_n-x_n-1) < xtol
	  double pgtol = 0.1;      //Stop if abs(g_n-g_n-1) < pgtol
	  double rescale = -1;     //Scaling factor (in log10) used to trigger rescaling
	  int nfeval;              //On output, the number of function evaluations
	  double fval;             //On output, the final function value
	  double *gval;            //On output, gradient values
	  int rc;                  //Return value
	  
	  //Malloc
	  x      = malloc(nvar*sizeof(double));
	  gval   = malloc(nvar*sizeof(double));
	  llimit = malloc(nvar*sizeof(double));
	  ulimit = malloc(nvar*sizeof(double));
	  
	  //Define starting point and limits
	  double limit=0;
	  for(i=0;i<nvar;i++){
	    if(phasemode == SCI_PHASEMODE_ZTARGET){
	      x[i]=sm_p->shk_zernike_target[i];
	      if(i<3) limit = 0.1; else limit = 0.05;
	      ulimit[i] = x[i] + limit;
	      llimit[i] = x[i] - limit;
	    }
	  
	    if(phasemode == SCI_PHASEMODE_ZCOMMAND){
	      x[i]=0;
	      if(i<3) limit = 0.1; else limit = 0.05;
	      ulimit[i] = x[i] + limit;
	      llimit[i] = x[i] - limit;
	    }
	      
	    if(phasemode == SCI_PHASEMODE_ACOMMAND){
	      x[i]=alp.acmd[i];
	      ulimit[i] = x[i] + 0.1;
	      llimit[i] = x[i] - 0.1;
	    }
	  }
	  
	  //Define gradient parameters
	  opt_param.gstep = 0.010;
	  opt_param.calc_gradient = 2;

	  //Run TNC Optimizer
	  rc = tnc(nvar, x, &fval, gval, sci_phase_merit, &opt_param, llimit, ulimit, NULL, NULL, TNC_MSG_ALL,
		   maxCGit, maxnfeval, eta, stepmx, accuracy, fmin, ftol, xtol, pgtol, rescale, &nfeval);

	  printf("SCI: After %d function evaluations, TNC returned:\n%s\n", nfeval,
		 tnc_rc_string[rc - TNC_MINRC]);

	  //Free malloc
	  if(x) free(x);
	  if(ulimit) free(ulimit);
	  if(llimit) free(llimit);

	  //Stop phasemode
	  sm_p->sci_phasemode = SCI_PHASEMODE_NONE;
	  
#endif
	}

#if PHASE_OPTIMIZER == OPTIMIZER_GSL
	//Iterate minimizer
	min_status = gsl_multimin_fminimizer_iterate(s);
	if(min_status)
	  printf("SCI: gsl_multimin_fminimizer_iterate error\n");
	
	//Get size
	min_size = gsl_multimin_fminimizer_size(s);
	min_status = gsl_multimin_test_size(min_size, 0.001);
	
	//Print status
	printf("SCI: %5.3f,%5.3f: ",s->fval,min_size);
	for(i=0;i<nvar;i++)
	  printf("%4.1f ",gsl_vector_get(s->x,i)*1000);
	printf("\n");
	//printf("SCI: Phase iter=%d f=%10.5f size=%.3f\n", opt_iter, s->fval, min_size);
	sm_p->sci_phasemerit=s->fval;
	
 	//Check for convergence
	if(min_status == GSL_SUCCESS){
	  printf("SCI: Minimum found after %d iterations\n",opt_iter);
	  sm_p->sci_phasemode = SCI_PHASEMODE_NONE;
	}
#endif
	//Increment counter
	opt_iter++;
      }
      else{
	/* Reset Optimizer */
	opt_iter=0;
	
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
