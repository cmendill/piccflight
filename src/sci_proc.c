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
/* SCI_PROC                                                   */
/*  - Main SCI camera process                                 */
/**************************************************************/
void sci_proc(void){
  char file[MAX_FILENAME], name[MAX_FILENAME];
  uint32_t err = 0;
  uint32_t counter = 0;
  long domain = (FLIDOMAIN_USB | FLIDEVICE_CAMERA);
  uint16_t *img_buffer;
  int camera_running=0;
  long exptime;
  flimode_t mode_index;
  char mode_string[128];
  size_t mode_size=128;
  int i;
  int rc;
    
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
    }
    
    /* Set exposure time */
    exptime = lround(sm_p->sci_exptime * 1000);
    if((err = FLISetExposureTime(dev, exptime))){
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
    mode_index = 0;
    if((err = FLISetCameraMode(dev, mode_index))){
      fprintf(stderr, "SCI: Error FLI SetCameraMode: %s\n", strerror((int)-err));
    }
     
    /* Get camera mode */
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

    /* ----------------------- Enter Exposure Loop ----------------------- */
    while(1){
      /* Check if we've been asked to exit */
      if(sm_p->w[SCIID].die)
	scictrlC(0);

      /* Check if we've been asked to reset the exposure */
      if(sm_p->sci_reset_camera){
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
      
      /* Run Camera */
      if(camera_running){
	/* Run Exposure */
	if((rc=sci_expose(sm_p,dev,img_buffer))){
	  if(rc==1)
	    printf("SCI: Exposure failed\n");
	  if(rc==2)
	    scictrlC(0);
	}
	else{
	  /*Process Image*/
	  sci_process_image(img_buffer,sm_p);
	}
      }
      
      /* Increment counter */
      counter++;
	
      /* Sleep */
      sleep(sm_p->w[SCIID].per);
    }
  }

  /* Exit */
  scictrlC(0);
  return;
}
