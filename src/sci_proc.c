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

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"

/* Things to put in other places once code overhaul is finished */
#include "libfli.h" //
#define FLIDEVICE_CAMERA (0x100) //from libfli.h
#define FLIDOMAIN_USB (0x02) //from libfli.h
#define MICROLINE_DOMAIN FLIDOMAIN_USB | FLIDEVICE_CAMERA
#define MAX_PATH 64
#define SCI_EXP_TIME 1000  //Milliseconds
#define SCI_TEMP 20        //Degrees C
#define SCI_FRAME_NORM FLI_FRAME_TYPE_NORMAL
#define SCI_FRAME_DARK FLI_FRAME_TYPE_DARK
#define SCI_ROI_XSIZE SCIXS
#define SCI_ROI_YSIZE SCIYS
#define SCI_UL_X 500
#define SCI_UL_Y 500
#define SCI_LR_X (SCI_UL_X+SCI_ROI_XSIZE)
#define SCI_LR_Y (SCI_UL_Y+SCI_ROI_YSIZE)
#define SCI_HBIN 1
#define SCI_VBIN 1
#define SCI_NFLUSHES 4
#define SCI_BIT_DEPTH FLI_MODE_16BIT //from libfli.h


/* Process File Descriptor */
int sci_shmfd;

/* FLI File Descriptor */
flidev_t dev;

/* CTRL-C Function */
void scictrlC(int sig){
  uint32 err = 0;
  if (MSG_CTRLC) printf("SCI: ctrlC! exiting.\n");
  /* Close FLI camera */
  if((err = FLIClose(dev))){
    fprintf(stderr, "Error FLIClose: %s\n", strerror((int)-err));
  }else{
    if(SCI_DEBUG) printf("SCI: FLI closed\n");
  }
  /* Close shared memory */
  close(sci_shmfd);

  /* Exit */
  exit(sig);
}

/* Exposure function */
int sci_expose(flidev_t dev, uint16 *img_buffer){
  int row;
  uint32 err = 0;

  /* Expose a frame */
  if((err = FLIExposeFrame(dev))){
    fprintf(stderr, "Error FLIExposeFrame: %s\n", strerror((int)-err));
    return 1;
  }else{
    if(SCI_DEBUG) printf("SCI: FLI frame exposed\n");
  }

  for(row=0;row<SCI_ROI_YSIZE;row++){
    err = err | FLIGrabRow(dev, img_buffer, SCI_ROI_XSIZE);
  }

  if(err){
    fprintf(stderr, "Error FLIGrabRow: %s\n", strerror((int)-err));
    return 1;
  }else{
    if(SCI_DEBUG) printf("SCI: FLI rows grabbed\n");
  }
  return 0;
}

/* SCI Process Image*/
void sci_process_image(sm_t *sm_p,uint16 *img_buffer){
  static scievent_t scievent;
  static scifull_t scifull;
  scifull_t* scifull_p;
  scievent_t* scievent_p;
  static struct timespec first, start, end, delta, last;
  static int init = 0;
  double dt;
  
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
  scievent.hed.temp         = 0;
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
    if(SCI_DEBUG) printf("SCI: Frame Size: --  Buffer Size: %lu\n",sizeof(sci_t));  
    /* Copy packet header */
    memcpy(&scifull.hed,&scievent.hed,sizeof(pkthed_t));
    scifull.hed.packet_type = SCIFULL;

    /* Copy image */
    //memcpy(&(scifull.image.data[0][0]),img_buffer,sizeof(sci_t));

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
  char file[MAX_PATH], name[MAX_PATH];
  double temperature_set = SCI_EXP_TIME;
  long sci_exp_time = SCI_TEMP;
  fliframe_t frame_dark = SCI_FRAME_DARK;
  fliframe_t frame_norm = SCI_FRAME_NORM;
  uint32 err = 0;
  uint32 count = 0;
  long domain = MICROLINE_DOMAIN;
  uint16 img_buffer[SCI_ROI_XSIZE * SCI_ROI_YSIZE];

  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&sci_shmfd)) == NULL){
    printf("openshm fail: sci_proc\n");
    scictrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, scictrlC);	/* usually ^C */

  /* Create Device List */
  if((err = FLICreateList(MICROLINE_DOMAIN))){
    fprintf(stderr, "Error FLICreateList: %s\n", strerror((int)-err));
    scictrlC(0);
  }else{
    if(SCI_DEBUG) printf("SCI: FLI list created\n");
    
    
    /* Create Device List */
    if((err = FLIListFirst(&domain, file, MAX_PATH, name, MAX_PATH))){
      fprintf(stderr, "Error FLIListFirst: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI first device found\n");
      FLIDeleteList();
    }
  }
  
  /* Open Device */
  if((err = FLIOpen(&dev, file, domain))){
    fprintf(stderr, "Error FLIOpen: %s\n", strerror((int)-err));
  }else{
    if(SCI_DEBUG) printf("SCI: FLI device opened\n");

    /* Open Device */
    if((err = FLISetTemperature(dev, temperature_set))){
      fprintf(stderr, "Error FLISetTemperature: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI temperature set\n");
    }

    /* Set exposure time */
    if((err = FLISetExposureTime(dev, sci_exp_time))){
      fprintf(stderr, "Error FLISetExposureTime: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI exposure time set\n");
    }

    /* Set frame type */
    if((err = FLISetFrameType(dev, frame_norm))){
      fprintf(stderr, "Error FLISetFrameType: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI frame type set\n");
    }

    /* Set ROI */
    if((err = FLISetImageArea(dev, SCI_UL_X, SCI_UL_Y, SCI_LR_X, SCI_LR_Y))){
      fprintf(stderr, "Error FLISetImageArea: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI image area set\n");
    }

    /* Set horizontal binning */
    if((err = FLISetHBin(dev, SCI_HBIN))){
      fprintf(stderr, "Error FLISetHBin: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI HBin set\n");
    }

    /* Set vertical binning */
    if((err = FLISetVBin(dev, SCI_VBIN))){
      fprintf(stderr, "Error FLISetVBin: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI VBin set\n");
    }

    /* Set number of flushes */
    if((err = FLISetNFlushes(dev, SCI_NFLUSHES))){
      fprintf(stderr, "Error FLISetNFlushes: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI NFlushes set\n");
    }

    /* Set bit depth (this causes an error for some reason) */
    // if((err = FLISetBitDepth(dev, SCI_BITDEPTH))){
    //   fprintf(stderr, "Error FLISetBitDepth: %s\n", strerror((int)-err)); //do we need this?
    // }else{
    //   if(SCI_DEBUG) printf("SCI: FLI bit depth set\n");
    // }
  }


  /* Start Loop */
  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[SCIID].die)
      scictrlC(0);
    
    /* Run Exposure */
    if(sci_expose(dev,img_buffer))
      printf("SCI: Exposure failed\n");
    else{
      /*Process Image*/
      sci_process_image(sm_p,img_buffer);
    }
    
    /* Check in with the watchdog */
    checkin(sm_p,SCIID);

    /* Sleep */
    sleep(sm_p->w[SCIID].per);
  }

  scictrlC(0);
  return;
}
