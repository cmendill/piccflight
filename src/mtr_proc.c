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
#include <sys/io.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"

/* Relay Bit Masks */
#define RELAY_1_ON     (1 << 0)
#define RELAY_2_ON     (1 << 1)
#define RELAY_3_ON     (1 << 2)
#define RELAY_4_ON     (1 << 3)
#define RELAY_5_ON     (1 << 4)
#define RELAY_6_ON     (1 << 5)
#define RELAY_7_ON     (1 << 6)
#define RELAY_8_ON     (1 << 7)
#define RELAY_9_ON     (1 << 8)
#define RELAY_10_ON    (1 << 9)
#define RELAY_11_ON    (1 << 10)
#define RELAY_12_ON    (1 << 11)
#define RELAY_13_ON    (1 << 12)
#define RELAY_14_ON    (1 << 13)
#define RELAY_15_ON    (1 << 14)
#define RELAY_16_ON    (1 << 15)

/* Door Status Bits */
#define DOOR_STATUS_OPEN    (1 << 0)
#define DOOR_STATUS_CLOSED  (1 << 1)
#define DOOR_STATUS_OPENING (1 << 2)
#define DOOR_STATUS_CLOSING (1 << 3)
#define DOOR_STATUS_STOPPED (1 << 4)

/* Door Parameters */
#define DOOR_TIMEOUT        {180,7,7,1}


/* Process File Descriptor */
int mtr_shmfd;

/* CTRL-C Function */
void mtrctrlC(int sig)
{
#if MSG_CTRLC
  printf("MTR: ctrlC! exiting.\n");
#endif
  close(mtr_shmfd);
  exit(sig);
}

/**************************************************************/
/* MTR_GET_STATUS                                             */
/*  - Get door status                                         */
/**************************************************************/
void mtr_get_status(mtrevent_t *mtrevent){
  char status;

  //Read status from SSR input port
  status = inb(SSR_BASE+1);
  
  //Set status words for each door
  mtrevent->door_status[0] = (DOOR_STATUS_OPEN * ((status & (1 << 0))>0)) | (DOOR_STATUS_CLOSED * ((status & (1 << 1))>0));
  mtrevent->door_status[1] = (DOOR_STATUS_OPEN * ((status & (1 << 2))>0)) | (DOOR_STATUS_CLOSED * ((status & (1 << 3))>0));
  mtrevent->door_status[2] = (DOOR_STATUS_OPEN * ((status & (1 << 4))>0)) | (DOOR_STATUS_CLOSED * ((status & (1 << 5))>0));
  mtrevent->door_status[3] = (DOOR_STATUS_OPEN * ((status & (1 << 6))>0)) | (DOOR_STATUS_CLOSED * ((status & (1 << 7))>0));
}

/* Main Process */
void mtr_proc(void){
  static uint32 count = 0;
  static mtrevent_t mtrevent;
  static struct timespec start,end;
  static int door_open_count[MTR_NDOORS] = {0};
  static int door_close_count[MTR_NDOORS] = {0};
  const  int door_timeout[MTR_NDOORS] = DOOR_TIMEOUT;
  int i;
  
  /* Setup relays */
  uint16_t door_open_relays[MTR_NDOORS] = {0};
  uint16_t door_close_relays[MTR_NDOORS] = {0};
  uint16_t door_activate_relay[MTR_NDOORS] = {0};
  door_activate_relay[0] = RELAY_1_ON;
  door_activate_relay[1] = RELAY_4_ON;
  door_activate_relay[2] = RELAY_9_ON;
  door_activate_relay[3] = RELAY_13_ON;
  door_close_relays[0]   = RELAY_2_ON | RELAY_3_ON;
  door_close_relays[1]   = RELAY_5_ON | RELAY_6_ON;
  door_close_relays[2]   = RELAY_10_ON | RELAY_11_ON;
  door_close_relays[3]   = RELAY_14_ON | RELAY_15_ON;
  door_open_relays[0]    = 0;
  door_open_relays[1]    = 0;
  door_open_relays[2]    = 0;
  door_open_relays[3]    = 0;

  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&mtr_shmfd)) == NULL){
    printf("openshm fail: mtr_proc\n");
    mtrctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, mtrctrlC);	/* usually ^C */

  /* Turn all relays off */
  outw(0x0000,REL_BASE);

  /* Start main loop */
  while(1){

    /* Get start time */
    clock_gettime(CLOCK_REALTIME,&start);

    /* Check if we've been asked to exit */
    if(sm_p->w[MTRID].die)
      mtrctrlC(0);
    
    /* Check in with the watchdog */
    checkin(sm_p,MTRID);

    /* Fill out event header */
    mtrevent.hed.version      = PICC_PKT_VERSION;
    mtrevent.hed.type         = MTREVENT;
    mtrevent.hed.frame_number = count++;
    mtrevent.hed.start_sec    = start.tv_sec;
    mtrevent.hed.start_nsec   = start.tv_nsec;

    /* Get door status */
    mtr_get_status(&mtrevent);
    
    /* Check for commands */
    for(i=0;i<MTR_NDOORS;i++){
      //OPEN DOOR
      if(sm_p->open_door[i]){
	if((mtrevent.door_status[i] & DOOR_STATUS_OPEN) || (door_open_count[i] > door_timeout[i])){
	  //Issue timeout warning
	  if(door_open_count[i] > door_timeout[i])
	    printf("MTR: Door %d OPENING TIMEOUT!\n",i+1);
	  //Set door status
	  mtrevent.door_status[i] &= ~DOOR_STATUS_OPENING;
	  //Set all relays OFF
	  outw(0x0000,REL_BASE);
	  //Clear command
	  sm_p->open_door[i] = 0;
	  //Reset counter
	  door_open_count[i] = 0;
	}
	else{
	  //Set door status
	  mtrevent.door_status[i] |= DOOR_STATUS_OPENING;
	  //Open door
	  if(door_open_count[i] == 0){
	    //Set directional relays
	    outw(door_open_relays[i],REL_BASE);
	    //Sleep
	    usleep(10000);
	    //Activate door
	    outw(door_activate_relay[i] | door_open_relays[i],REL_BASE);
	  }
	  //Increment counter
	  door_open_count[i]++;
	}
      }
      else{
	//Reset counter
	door_open_count[i] = 0;
      }
      
      //CLOSE DOOR
      if(sm_p->close_door[i]){
	if((mtrevent.door_status[i] & DOOR_STATUS_CLOSED) || (door_close_count[i] > door_timeout[i])){
	  //Issue timeout warning
	  if(door_close_count[i] > door_timeout[i])
	    printf("MTR: Door %d CLOSING TIMEOUT!\n",i+1);
	  //Set door status
	  mtrevent.door_status[i] &= ~DOOR_STATUS_CLOSING;
	  //Set all relays OFF
	  outw(0x0000,REL_BASE);
	  //Clear command
	  sm_p->close_door[i] = 0;
	  //Reset counter
	  door_close_count[i] = 0;
	}
	else{
	  //Set door status
	  mtrevent.door_status[i] |= DOOR_STATUS_CLOSING;
	  //Close door
	  if(door_close_count[i] == 0){
	    //Set directional relays
	    outw(door_close_relays[i],REL_BASE);
	    //Sleep
	    usleep(10000);
	    //Activate door
	    outw(door_activate_relay[i] | door_close_relays[i],REL_BASE);
	  }
	  //Increment counter
	  door_close_count[i]++;
	}
      }
      else{
	//Reset counter
	door_close_count[i] = 0;
      }
    
      //STOP DOOR
      if(sm_p->stop_door[i]){
	//Set all relays OFF
	outw(0x0000,REL_BASE);
	//Set door status
	mtrevent.door_status[i] |= DOOR_STATUS_STOPPED;
	mtrevent.door_status[i] &= ~DOOR_STATUS_CLOSING;
	mtrevent.door_status[i] &= ~DOOR_STATUS_OPENING;
	//Clear commands
	sm_p->stop_door[i]  = 0;
	sm_p->open_door[i]  = 0;
	sm_p->close_door[i] = 0;
	//Reset counters
	door_open_count[i]  = 0;
	door_close_count[i] = 0;
      }
    }
    
    /* Get end time */
    clock_gettime(CLOCK_REALTIME,&end);
    mtrevent.hed.end_sec    = end.tv_sec;
    mtrevent.hed.end_nsec   = end.tv_nsec;

    /* Write event to circular buffer */
    write_to_buffer(sm_p,&mtrevent,MTREVENT);

    /* Print status messages */
    for(i=0;i<MTR_NDOORS;i++){
      if(mtrevent.door_status[i] & DOOR_STATUS_OPEN)    printf("MTR: Door %d OPEN\n",i+1); 
      if(mtrevent.door_status[i] & DOOR_STATUS_OPENING) printf("MTR: Door %d OPENING %d/%d\n",i+1,door_open_count[i],door_timeout[i]); 
      if(mtrevent.door_status[i] & DOOR_STATUS_CLOSED)  printf("MTR: Door %d CLOSED\n",i+1); 
      if(mtrevent.door_status[i] & DOOR_STATUS_CLOSING) printf("MTR: Door %d CLOSING %d/%d\n",i+1,door_close_count[i],door_timeout[i]); 
      if(mtrevent.door_status[i] & DOOR_STATUS_STOPPED) printf("MTR: Door %d STOPPED\n",i+1); 
    }
        
    
    /* Print debugging info */
    if(MTR_DEBUG){
      for(i=0;i<MTR_NDOORS;i++){
	printf("MTR: *******************\n");
	printf("MTR: Door %d    OPEN: %d\n",i+1,(mtrevent.door_status[i] & DOOR_STATUS_OPEN) > 0); 
	printf("MTR: Door %d  CLOSED: %d\n",i+1,(mtrevent.door_status[i] & DOOR_STATUS_CLOSED) > 0); 
	printf("MTR: Door %d OPENING: %d\n",i+1,(mtrevent.door_status[i] & DOOR_STATUS_OPENING) > 0); 
	printf("MTR: Door %d CLOSING: %d\n",i+1,(mtrevent.door_status[i] & DOOR_STATUS_CLOSING) > 0); 
	printf("MTR: Door %d STOPPED: %d\n",i+1,(mtrevent.door_status[i] & DOOR_STATUS_STOPPED) > 0); 
      }
    }

    /* Sleep */
    sleep(1);
  }
  
  mtrctrlC(0);
  return;
}
