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
#include <libbmp.h>

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


/**************************************************************/
/* ACQCTRLC                                                   */
/*  - SIGINT function                                         */
/**************************************************************/
void acqctrlC(int sig)
{
  
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
  
  if(MSG_CTRLC) printf("ACQ: exiting\n");

  /* Exit */
  exit(sig);
}

/**************************************************************/
/* ACQ_BUILD_GIF                                              */
/*  - Function to compress an image using GIF                 */
/**************************************************************/
void acq_build_gif(unsigned char *input, unsigned char *build_output, int *build_size){
  uint64_t i;
  
  //Create bitmap structure and populate as grayscale
  Bitmap *b = bm_create(ACQXS/ACQBIN, ACQYS/ACQBIN);
  for(i=0;i<(ACQXS/ACQBIN)*(ACQYS/ACQBIN);i++){
    b->data[4*i+0]=input[i];    //Blue
    b->data[4*i+1]=input[i];    //Green
    b->data[4*i+2]=input[i];    //Red
    b->data[4*i+3]=0;           //Alpha
  }

  //Encode gif
  bm_encode_gif(b, build_output, build_size);

  //Free bitmap structure
  bm_free(b);
}

/**************************************************************/
/* ACQ_PROCESS_IMAGE                                          */
/*  - Process ACQ camera image                                */
/**************************************************************/
void acq_process_image(uvc_frame_t *frame, sm_t *sm_p) {
  static acqevent_t acqevent;
  static acqfull_t acqfull;
  acqfull_t *acqfull_p;
  acqevent_t *acqevent_p;
  static struct timespec start,end,delta,last,full_last;
  static int init=0;
  double dt;
  uint16_t fakepx=0;
  int i,j;
  uint8_t  full_image[ACQYS][ACQXS];
  uint16_t binned_image16[ACQYS/ACQBIN][ACQXS/ACQBIN]={{0}};
  uint8_t  binned_image8[ACQYS/ACQBIN][ACQXS/ACQBIN]={{0}};
  uint8_t  gif_data[(ACQXS/ACQBIN)*(ACQYS/ACQBIN)];
  int      gif_nbytes = 0;
  int      state;
  
  //Get time immidiately
  clock_gettime(CLOCK_REALTIME,&start);
  
  //Get state
  state = sm_p->state;

  //Debugging
  if(ACQ_DEBUG) printf("ACQ: Got frame: %d\n",frame->sequence);

  //Check in with the watchdog
  checkin(sm_p,ACQID);
  
  //Check reset
  if(sm_p->acq_reset){
    init=0;
    sm_p->acq_reset=0;
  }
  
  //Initialize
  if(!init){
    memset(&acqfull,0,sizeof(acqfull));
    memset(&acqevent,0,sizeof(acqevent));
    memcpy(&last,&start,sizeof(struct timespec));
    memcpy(&full_last,&start,sizeof(struct timespec));
    init=1;
    if(ACQ_DEBUG) printf("ACQ: Initialized\n");
  }

  
  //Measure exposure time
  if(timespec_subtract(&delta,&start,&last))
    printf("ACQ: call back --> timespec_subtract error!\n");
  ts2double(&delta,&dt);

  //Save time
  memcpy(&last,&start,sizeof(struct timespec));

  //Fill out event header
  acqevent.hed.version      = PICC_PKT_VERSION;
  acqevent.hed.type         = BUFFER_ACQEVENT;
  acqevent.hed.frame_number = frame->sequence;
  acqevent.hed.exptime      = sm_p->acq_exptime;
  acqevent.hed.frmtime      = sm_p->acq_frmtime;
  acqevent.hed.ontime       = dt;
  acqevent.hed.state         = state;
  acqevent.hed.alp_commander = sm_p->state_array[state].alp_commander;
  acqevent.hed.hex_commander = sm_p->state_array[state].hex_commander;
  acqevent.hed.bmc_commander = sm_p->state_array[state].bmc_commander;
  acqevent.hed.start_sec    = start.tv_sec;
  acqevent.hed.start_nsec   = start.tv_nsec;

  //Compress and send every other image
  if(acqevent.hed.frame_number % 2 == 0){
    //Copy full image
    memcpy(&full_image[0][0],frame->data,sizeof(full_image));

    //Bin image
    for(i=0;i<ACQYS;i++){
      for(j=0;j<ACQXS;j++){
	binned_image16[i/ACQBIN][j/ACQBIN] += (uint16_t)full_image[i][j];
      }
    }

    //Average bins
    for(i=0;i<ACQYS/ACQBIN;i++){
      for(j=0;j<ACQXS/ACQBIN;j++){
	binned_image8[i][j] = binned_image16[i][j] / (ACQBIN*ACQBIN);
	//Threshold
	if(binned_image8[i][j] < sm_p->acq_thresh) binned_image8[i][j] = 0;
      }
    }
  
    //Compress image
    acq_build_gif(&binned_image8[0][0], gif_data, &gif_nbytes);
  
    //Write gif data to event
    if(gif_nbytes <= ACQ_MAX_GIF_SIZE){
      memcpy(acqevent.gif,gif_data,gif_nbytes);
      acqevent.gif_nbytes = gif_nbytes;
    }
    else{
      printf("ACQ: Compressed image too large %d\n",gif_nbytes);
    }

    //Write ACQEVENT to circular buffer 
    if(sm_p->write_circbuf[BUFFER_ACQEVENT]){
      //Open circular buffer
      acqevent_p=(acqevent_t *)open_buffer(sm_p,BUFFER_ACQEVENT);

      //Copy acqevent
      memcpy(acqevent_p,&acqevent,sizeof(acqevent_t));;

      //Get final timestamp
      clock_gettime(CLOCK_REALTIME,&end);
      acqevent_p->hed.end_sec = end.tv_sec;
      acqevent_p->hed.end_nsec = end.tv_nsec;

      //Close buffer
      close_buffer(sm_p,BUFFER_ACQEVENT);
    }
  }
  
  /*************************************************************/
  /**********************  Full Image Code  ********************/
  /*************************************************************/
  if(sm_p->write_circbuf[BUFFER_ACQFULL]){
    if(timespec_subtract(&delta,&start,&full_last))
      printf("ACQ: acq_process_image --> timespec_subtract error!\n");
    ts2double(&delta,&dt);
    if(dt > ACQ_FULL_IMAGE_TIME){
      //Debugging 
      if(ACQ_DEBUG) printf("ACQ: Full Image: %d\n",frame->sequence);
      if(ACQ_DEBUG) printf("ACQ: Frame Size: %lu  Buffer Size: %lu\n",frame->data_bytes,sizeof(acq_t));  
    
      //Copy packet header
      memcpy(&acqfull.hed,&acqevent.hed,sizeof(pkthed_t));
      acqfull.hed.type = BUFFER_ACQFULL;
    
      //Fake data
      if(sm_p->w[ACQID].fakemode != FAKEMODE_NONE){
	if(sm_p->w[ACQID].fakemode == FAKEMODE_TEST_PATTERN)
	  for(i=0;i<ACQXS;i++)
	    for(j=0;j<ACQYS;j++)
	      acqfull.image.data[i][j]=fakepx++;
      }
      else{
	//Copy full image
	memcpy(&(acqfull.image.data[0][0]),frame->data,sizeof(acq_t));
      }

      //Open circular buffer
      acqfull_p=(acqfull_t *)open_buffer(sm_p,BUFFER_ACQFULL);
    
      //Copy data
      memcpy(acqfull_p,&acqfull,sizeof(acqfull_t));;

      //Get final timestamp
      clock_gettime(CLOCK_REALTIME,&end);
      acqfull_p->hed.end_sec = end.tv_sec;
      acqfull_p->hed.end_nsec = end.tv_nsec;

      //Close buffer
      close_buffer(sm_p,BUFFER_ACQFULL);

      //Reset time
      memcpy(&full_last,&start,sizeof(struct timespec));
    }
  }
}

/**************************************************************/
/* ACQ_CALLBACK                                               */ 
/*  - ACQ ISR callback function                               */
/**************************************************************/
void acq_callback(uvc_frame_t *frame, void *ptr) {
  sm_t *sm_p = (sm_t *)ptr;
  acq_process_image(frame,sm_p);
}

/**************************************************************/
/* ACQ_PROC                                                   */
/*  - Main ACQ camera process                                 */
/**************************************************************/
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


  /* ----------------------- Enter Main Loop ----------------------- */
  while(1){
    /* Check in with the watchdog */
    checkin(sm_p,ACQID);
    
    /* STOP stream, put camera in known state */
    uvc_stop_streaming(devh);
    camera_running = 0;
    if(ACQ_DEBUG) printf("ACQ: Camera stopped\n");
  

    /* Setup stream profile */
    if((res = uvc_get_stream_ctrl_format_size(devh, &ctrl, UVC_FRAME_FORMAT_GRAY8, ACQXS, ACQYS, fps))<0){
      uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
      acqctrlC(0);
    }
    if(ACQ_DEBUG) printf("ACQ: Stream profile configured\n");

    /* ----------------------- Enter Exposure Loop ----------------------- */
    while(1){
      /* Check if we've been asked to exit */
      if(sm_p->w[ACQID].die)
	acqctrlC(0);
    
      /* Check if we've been asked to reset the exposure */
      if(sm_p->acq_reset_camera){
	sm_p->acq_reset_camera = 0;
	break;
      }
      
      /* Check if camera should start */
      if(!camera_running){
	/* START stream */
	if((res = uvc_start_streaming(devh, &ctrl, acq_callback, (void *)sm_p, 0))<0){
	  uvc_perror(res, "start_streaming"); /* unable to start stream */
	  acqctrlC(0);
	}
	/* Turn on auto exposure */
	uvc_set_ae_mode(devh, 1);
	camera_running = 1;
	printf("ACQ: Camera started\n");
      }
      
      /* Sleep */
      sleep(sm_p->w[ACQID].per);
    }
  }
  
  /* Exit */
  acqctrlC(0);
  return;
}
