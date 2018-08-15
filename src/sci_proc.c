#define _XOPEN_SOURCE 500
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <libfli.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "fakemodes.h"

#define SCI_EXP_TIME 2000   //Milliseconds
#define SCI_TEMP_TEC_OFF 20 //Degrees C
#define SCI_ROI_XSIZE SCIXS
#define SCI_ROI_YSIZE SCIYS
#define SCI_HBIN 1
#define SCI_VBIN 1
#define SCI_UL_X 500
#define SCI_UL_Y 500
#define SCI_LR_X (SCI_UL_X+(SCI_ROI_XSIZE/SCI_HBIN))
#define SCI_LR_Y (SCI_UL_Y+(SCI_ROI_YSIZE/SCI_VBIN))
#define SCI_NFLUSHES 4


/* Process File Descriptor */
int sci_shmfd;

/* FLI File Descriptor */
flidev_t dev;

/* CTRL-C Function */
void scictrlC(int sig){
  uint32 err = 0;
  if (MSG_CTRLC) printf("SCI: ctrlC! exiting.\n");
  /* Cancel Exposure */
  if((err = FLICancelExposure(dev))){
    fprintf(stderr, "SCI: Error FLICancelExposure: %s\n", strerror((int)-err));
  }else{
    if(SCI_DEBUG) printf("SCI: Exposure stopped\n");
  }
  /* Close FLI camera */
  if((err = FLIClose(dev))){
    fprintf(stderr, "SCI: Error FLIClose: %s\n", strerror((int)-err));
  }else{
    if(SCI_DEBUG) printf("SCI: FLI closed\n");
  }
  /* Close shared memory */
  close(sci_shmfd);

  /* Exit */
  exit(sig);
}

/* Get CCD Temperature */
double sci_get_temp(flidev_t dev){
  double temp;
  uint32 err;
  
  /* Get temperature */
  if((err = FLIGetTemperature(dev, &temp))){
    fprintf(stderr, "SCI: Error FLIGetTemperature: %s\n", strerror((int)-err));
    return 99;
  }else{
    if(SCI_DEBUG) printf("SCI: FLI temperature: %f\n",temp);
  }
  return temp;
}

/* Exposure function */
int sci_expose(sm_t *sm_p, flidev_t dev, uint16 *img_buffer){
  int row,i;
  uint32 err;
  long int timeleft;
  int exp_timeout = (SCI_EXP_TIME/1000)+2;
  
  /* Start exposure */
  if((err = FLIExposeFrame(dev))){
    fprintf(stderr, "SCI: Error FLIExposeFrame: %s\n", strerror((int)-err));
    return 1;
  }else{
    if(SCI_DEBUG) printf("SCI: Exposure started\n");
  }

  /* Wait for exposure to finish */
  for(i=0;i<exp_timeout;i++){
    /* Check if we've been asked to exit */
    if(sm_p->w[SCIID].die)
      scictrlC(0);

    /* Check in with the watchdog */
    checkin(sm_p,SCIID);
       
    //Sleep
    sleep(1);
    
    //Get exposure status
    if((err = FLIGetExposureStatus(dev,&timeleft))){
      fprintf(stderr, "SCI: Error FLIGetExposureStatus: %s\n", strerror((int)-err));
      return 1;
    }
    if(timeleft == 0){
      if(SCI_DEBUG) printf("SCI: Exposure done after %d checks\n",i+1);
      break;
    }
  }
  if(i==exp_timeout){
    printf("SCI: Image timeout!\n");
    return 1;
  }
  
  /* Grab data one row at a time */
  for(row=0;row<SCI_ROI_YSIZE;row++)
    err = err | FLIGrabRow(dev, img_buffer+(row*SCI_ROI_XSIZE), SCI_ROI_XSIZE);

  /* Error checking */
  if(err){
    fprintf(stderr, "SCI: Error FLIGrabRow: %s\n", strerror((int)-err));
    return 1;
  }else{
    if(SCI_DEBUG) printf("SCI: FLI rows grabbed\n");
  }
  return 0;
}

/* SCI Process Image*/
void sci_process_image(sm_t *sm_p,uint16 *img_buffer,double ccdtemp){
  static scievent_t scievent;
  static scifull_t scifull;
  scifull_t* scifull_p;
  scievent_t* scievent_p;
  static struct timespec first, start, end, delta, last;
  static int init = 0;
  double dt;
  uint16 fakepx=0;
  uint32 i,j;

  /* Get time immidiately */
  clock_gettime(CLOCK_REALTIME,&start);

  /* Debugging */
  if(SCI_DEBUG) printf("SCI: Got frame\n"); 

  /* Check in with the watchdog */
  checkin(sm_p,SCIID);

  /* Check reset */
  if(sm_p->sci_reset){ //need to add this to controller.h
   init=0;
   sm_p->sci_reset=0;
  }

  /* Initialize */
  if(!init){
    memset(&scifull,0,sizeof(scifull));
    memset(&scievent,0,sizeof(scievent));
    memcpy(&first,&start,sizeof(struct timespec));
    init=1;
    if(SCI_DEBUG) printf("SCI: Initialized\n");
  }

  /* Measure exposure time */
  if(timespec_subtract(&delta,&start,&last))
    printf("SCI: call back --> timespec_subtract error!\n");
  ts2double(&delta,&dt);

  /* Fill out event header */
  scievent.hed.packet_type  = SCIEVENT;
  scievent.hed.frame_number = 0; //need to put something here
  scievent.hed.exptime      = 0;
  scievent.hed.ontime       = dt;
  scievent.hed.temp         = ccdtemp;
  scievent.hed.imxsize      = SCIXS;
  scievent.hed.imysize      = SCIYS;
  scievent.hed.mode         = 0;
  scievent.hed.start_sec    = start.tv_sec;
  scievent.hed.start_nsec   = start.tv_nsec;

  /* Open circular buffer */
  scievent_p=(scievent_t *)open_buffer(sm_p,SCIEVENT);

  /* Copy scievent */
  memcpy(scievent_p,&scievent,sizeof(scievent_t));;

  /* Get final timestamp */
  clock_gettime(CLOCK_REALTIME,&end);
  scievent_p->hed.end_sec = end.tv_sec;
  scievent_p->hed.end_nsec = end.tv_nsec;

  /* Close buffer */
  close_buffer(sm_p,SCIEVENT);

  /* Save time */
  memcpy(&last,&start,sizeof(struct timespec));


  /*******************  Full Image Code  *****************/
  if(timespec_subtract(&delta,&start,&first))
    printf("SCI: sci_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);
  if(dt > SCI_FULL_IMAGE_TIME){
    /* Debugging */
    if(SCI_DEBUG) printf("SCI: Buffer Size: %lu\n",sizeof(sci_t));  
    /* Copy packet header */
    memcpy(&scifull.hed,&scievent.hed,sizeof(pkthed_t));
    scifull.hed.packet_type = SCIFULL;

    //Fake data
    if(sm_p->sci_fakemode > FAKEMODE_NONE){
      if(sm_p->sci_fakemode == FAKEMODE_GEN_IMAGE_CAMERA_SYNC)
	for(i=0;i<SCIXS;i++)
	  for(j=0;j<SCIYS;j++)
	    scifull.image.data[i][j]=fakepx++;
    }
    else{
      /* Copy image */
      memcpy(&(scifull.image.data[0][0]),img_buffer,sizeof(sci_t));
    }
    
    /* Open circular buffer */
    scifull_p=(scifull_t *)open_buffer(sm_p,SCIFULL);

    /* Copy data */
    memcpy(scifull_p,&scifull,sizeof(scifull_t));;

    /* Get final timestamp */
    clock_gettime(CLOCK_REALTIME,&end);
    scifull_p->hed.end_sec = end.tv_sec;
    scifull_p->hed.end_nsec = end.tv_nsec;

    /* Close buffer */
    close_buffer(sm_p,SCIFULL);

    /* Reset time */
    memcpy(&first,&start,sizeof(struct timespec));
  }
}


/* Main Process */
void sci_proc(void){
  char file[MAX_FILENAME], name[MAX_FILENAME];
  uint32 err = 0;
  uint32 count = 0;
  long domain = (FLIDOMAIN_USB | FLIDEVICE_CAMERA);
  uint16 img_buffer[SCI_ROI_XSIZE * SCI_ROI_YSIZE];
  double ccdtemp;
  int camera_running=0;
  
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&sci_shmfd)) == NULL){
    printf("openshm fail: sci_proc\n");
    scictrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, scictrlC);	/* usually ^C */

  /* Create Device List */
  if((err = FLICreateList(domain))){
    fprintf(stderr, "SCI: Error FLICreateList: %s\n", strerror((int)-err));
    scictrlC(0);
  }else{
    if(SCI_DEBUG) printf("SCI: FLI list created\n");
    
    
    /* Create Device List */
    if((err = FLIListFirst(&domain, file, MAX_FILENAME, name, MAX_FILENAME))){
      fprintf(stderr, "SCI: Error FLIListFirst: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI first device found\n");
      FLIDeleteList();
    }
  }
  
  /* Open Device */
  if((err = FLIOpen(&dev, file, domain))){
    fprintf(stderr, "SCI: Error FLIOpen: %s\n", strerror((int)-err));
  }else{
    if(SCI_DEBUG) printf("SCI: FLI device opened\n");

    /* Cancel Exposure, put camera in known state */
    if((err = FLICancelExposure(dev))){
      fprintf(stderr, "SCI: Error FLICancelExposure: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: Exposure stopped\n");
    }
    camera_running = 0;
    
    /* Set temperature */
    if((err = FLISetTemperature(dev, SCI_TEMP_TEC_OFF))){
	fprintf(stderr, "SCI: Error FLISetTemperature: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI temperature set\n");
    }
    
    /* Set exposure time */
    if((err = FLISetExposureTime(dev, SCI_EXP_TIME))){
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
  }


  /* ----------------------- Enter Main Loop ----------------------- */
  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[SCIID].die)
      scictrlC(0);

    /* Get CCD Temperature */
    ccdtemp = sci_get_temp(dev);
    
    /* Check if camera should start/stop */
    if(!camera_running && sm_p->state_array[sm_p->state].sci.run_camera){
      camera_running = 1;
      printf("SCI: Camera started\n");
    }
    if(camera_running && !sm_p->state_array[sm_p->state].sci.run_camera){
      /* Cancel Exposure */
      if((err = FLICancelExposure(dev))){
	fprintf(stderr, "SCI: Error FLICancelExposure: %s\n", strerror((int)-err));
      }else{
	if(SCI_DEBUG) printf("SCI: Exposure stopped\n");
      }
      /* Disable TEC */
      if((err = FLISetTemperature(dev, SCI_TEMP_TEC_OFF))){
	fprintf(stderr, "SCI: Error FLISetTemperature: %s\n", strerror((int)-err));
      }else{
	if(SCI_DEBUG) printf("SCI: FLI TEC Disabled\n");
      }
      camera_running = 0;
      printf("SCI: Camera stopped\n");
    }
    
    /* Run Camera */
    if(camera_running){
      /* Run Exposure */
      if(sci_expose(sm_p,dev,img_buffer))
	printf("SCI: Exposure failed\n");
      else{
	/*Process Image*/
	sci_process_image(sm_p,img_buffer,ccdtemp);
      }
    }
    
    /* Check in with the watchdog */
    checkin(sm_p,SCIID);
    
    /* Sleep */
    sleep(sm_p->w[SCIID].per);
  }

  scictrlC(0);
  return;
}
