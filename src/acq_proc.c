#define _XOPEN_SOURCE 500
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <libusb.h>
#include <libuvc.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "fakemodes.h"
#include "acq_proc.h"

/* Globals */
uvc_context_t *ctx;
uvc_device_t *dev;
uvc_device_handle_t *devh;
int acq_shmfd;


/* CTRL-C Function */
void acqctrlC(int sig)
{
  if(MSG_CTRLC) printf("ACQ: ctrlC! exiting.\n");
  
  /* End the stream. Blocks until last callback is serviced */
  uvc_stop_streaming(devh);
  if(ACQ_DEBUG) printf("ACQ: Stream stopped\n");
  
  /* Release our handle on the device */
  uvc_close(devh);
  if(ACQ_DEBUG) printf("ACQ: Device closed\n");

  /* Release the device descriptor */
  uvc_unref_device(dev);
  if(ACQ_DEBUG) printf("ACQ: Device released\n");

  /* Close the UVC context */
  uvc_exit(ctx);
  if(ACQ_DEBUG) printf("ACQ: Context closed\n");

  /* Close shared memory */
  close(acq_shmfd);

  /* Exit */
  exit(sig);
}


/* Camera Callback Function */
void cb(uvc_frame_t *frame, void *ptr) {
  static acqevent_t acqevent;
  static acqfull_t acqfull;
  acqfull_t *acqfull_p;
  acqevent_t *acqevent_p;
  static struct timespec first,start,end,delta,last;
  static int init=0;
  double dt;
  uint16_t fakepx=0;
  sm_t *sm_p = (sm_t *)ptr;
  int i,j;
  
  /* Get time immidiately */
  clock_gettime(CLOCK_REALTIME,&start);
  
  /* Debugging */
  if(ACQ_DEBUG) printf("ACQ: Got frame: %d\n",frame->sequence);

  /* Check in with the watchdog */
  checkin(ptr,ACQID);
  
  /* Check reset */
  if(sm_p->w[ACQID].reset){
    init=0;
    sm_p->w[ACQID].reset=0;
  }
  
  /* Initialize */
  if(!init){
    memset(&acqfull,0,sizeof(acqfull));
    memset(&acqevent,0,sizeof(acqevent));
    memcpy(&first,&start,sizeof(struct timespec));
    init=1;
    if(ACQ_DEBUG) printf("ACQ: Initialized\n");
  }


  /* Measure exposure time */
  if(timespec_subtract(&delta,&start,&last))
    printf("ACQ: call back --> timespec_subtract error!\n");
  ts2double(&delta,&dt);

  /* Fill out event header */
  acqevent.hed.packet_type  = ACQEVENT;
  acqevent.hed.frame_number = frame->sequence;
  acqevent.hed.exptime      = 0;
  acqevent.hed.ontime       = dt;
  acqevent.hed.temp         = 0;
  acqevent.hed.imxsize      = ACQXS;
  acqevent.hed.imysize      = ACQYS;
  acqevent.hed.mode         = 0;
  acqevent.hed.start_sec    = start.tv_sec;
  acqevent.hed.start_nsec   = start.tv_nsec;

  /* Open circular buffer */
  acqevent_p=(acqevent_t *)open_buffer(sm_p,ACQEVENT);

  /* Copy acqevent */
  memcpy(acqevent_p,&acqevent,sizeof(acqevent_t));;

  /* Get final timestamp */
  clock_gettime(CLOCK_REALTIME,&end);
  acqevent_p->hed.end_sec = end.tv_sec;
  acqevent_p->hed.end_nsec = end.tv_nsec;

  /* Close buffer */
  close_buffer(sm_p,ACQEVENT);

  /* Save time */
  memcpy(&last,&start,sizeof(struct timespec));
  
  /*************************************************************/
  /**********************  Full Image Code  ********************/
  /*************************************************************/
  if(timespec_subtract(&delta,&start,&first))
    printf("ACQ: acq_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);
  if(dt > ACQ_FULL_IMAGE_TIME){
    //Debugging 
    if(ACQ_DEBUG) printf("ACQ: Full Image: %d\n",frame->sequence);
    if(ACQ_DEBUG) printf("ACQ: Frame Size: %lu  Buffer Size: %lu\n",frame->data_bytes,sizeof(acq_t));  
    
    //Copy packet header
    memcpy(&acqfull.hed,&acqevent.hed,sizeof(pkthed_t));
    acqfull.hed.packet_type = ACQFULL;
    
    //Fake data
    if(sm_p->w[ACQID].fakemode != FAKEMODE_NONE){
      if(sm_p->w[ACQID].fakemode == FAKEMODE_GEN_IMAGE_CAMERA_SYNC)
	for(i=0;i<ACQXS;i++)
	  for(j=0;j<ACQYS;j++)
	    acqfull.image.data[i][j]=fakepx++;
    }
    else{
      //Copy full image
      memcpy(&(acqfull.image.data[0][0]),frame->data,sizeof(acq_t));
    }

    //Open circular buffer
    acqfull_p=(acqfull_t *)open_buffer(sm_p,ACQFULL);
    
    //Copy data
    memcpy(acqfull_p,&acqfull,sizeof(acqfull_t));;

    //Get final timestamp
    clock_gettime(CLOCK_REALTIME,&end);
    acqfull_p->hed.end_sec = end.tv_sec;
    acqfull_p->hed.end_nsec = end.tv_nsec;

    //Close buffer
    close_buffer(sm_p,ACQFULL);

    //Reset time
    memcpy(&first,&start,sizeof(struct timespec));
  }
 }

/* Main Process */
void acq_proc(void){
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;
  int fps=5;
  int camera_running=0;
  
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&acq_shmfd)) == NULL){
    printf("openshm fail: acq_proc\n");
    acqctrlC(0);
  }
  
  /* Set soft interrupt handler */
  sigset(SIGINT, acqctrlC);	/* usually ^C */

  /* Initialize a UVC service context. Libuvc will set up its own libusb
   * context. Replace NULL with a libusb_context pointer to run libuvc
   * from an existing libusb context. */
  if((res = uvc_init(&ctx, NULL))<0){
    uvc_perror(res, "uvc_init");
    acqctrlC(0);
  }

  if(ACQ_DEBUG) printf("ACQ: UVC initialized\n");

  /* Locates the first attached UVC device, stores in dev */
  if((res = uvc_find_device(ctx, &dev, ACQ_VENDOR_ID, ACQ_PRODUCT_ID, NULL))<0){ /* filter devices: vendor_id, product_id, "serial_num" */
    uvc_perror(res, "uvc_find_device"); /* no devices found */
    acqctrlC(0);
  } 
  if(ACQ_DEBUG) printf("ACQ: Device found\n");
  
  /* Try to open the device: requires exclusive access */
  if((res = uvc_open(dev, &devh))<0){
    uvc_perror(res, "uvc_open"); /* unable to open device */
    acqctrlC(0);
  }
  if(ACQ_DEBUG) printf("ACQ: Device opened\n");

  /* Setup stream profile */
  if((res = uvc_get_stream_ctrl_format_size(devh, &ctrl, UVC_FRAME_FORMAT_GRAY16, ACQXS, ACQYS, fps))<0){
    uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
    acqctrlC(0);
  }
  if(ACQ_DEBUG) printf("ACQ: Stream profile configured\n");

  /* STOP stream, put camera in known state */
  uvc_stop_streaming(devh);
  camera_running = 0;
  if(ACQ_DEBUG) printf("ACQ: Camera stopped\n");
  

  /* ----------------------- Enter Main Loop ----------------------- */
  while(1){
    
    /* Check if camera should start/stop */
    if(!camera_running && sm_p->state_array[sm_p->state].acq.run_camera){
      /*START stream*/
      if((res = uvc_start_streaming(devh, &ctrl, cb, (void *)sm_p, 0))<0){
	uvc_perror(res, "start_streaming"); /* unable to start stream */
	acqctrlC(0);
      }
      /* Turn on auto exposure */
      uvc_set_ae_mode(devh, 1);
      camera_running = 1;
      printf("ACQ: Camera started\n");
    }
    if(camera_running && !sm_p->state_array[sm_p->state].acq.run_camera){
      /* STOP stream, put camera in known state */
      uvc_stop_streaming(devh);
      camera_running = 0;
      printf("ACQ: Camera stopped\n");
    }

    /* Check if we've been asked to exit */
    if(sm_p->w[ACQID].die)
      acqctrlC(0);
    
    /* Sleep */
    sleep(sm_p->w[ACQID].per);
  }
  
  /* Exit */
  acqctrlC(0);
  return;
}
