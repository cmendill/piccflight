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

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"

/* Relay bit masks */
#define RELAY_1_ON     1 << 0
#define RELAY_2_ON     1 << 1
#define RELAY_3_ON     1 << 2
#define RELAY_4_ON     1 << 3
#define RELAY_5_ON     1 << 4
#define RELAY_6_ON     1 << 5
#define RELAY_7_ON     1 << 6
#define RELAY_8_ON     1 << 7
#define RELAY_9_ON     1 << 8
#define RELAY_10_ON    1 << 9
#define RELAY_11_ON    1 << 10
#define RELAY_12_ON    1 << 11
#define RELAY_13_ON    1 << 12
#define RELAY_14_ON    1 << 13
#define RELAY_15_ON    1 << 14
#define RELAY_16_ON    1 << 15


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

/* Main Process */
void mtr_proc(void){
  uint32 count = 0;
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&mtr_shmfd)) == NULL){
    printf("openshm fail: mtr_proc\n");
    mtrctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, mtrctrlC);	/* usually ^C */

  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[MTRID].die)
      mtrctrlC(0);
    
    /* Check in with the watchdog */
    checkin(sm_p,MTRID);

    /* Turn all relays off */
    outs(0x0000,REL_BASE);

    /* Check for commands */
    if(sm_p->open_door_inst_fwd){
      //Set Relay 2 & 3 OFF
      outs(0x0000,REL_BASE);
      //Set Relay 1 ON
      outs(RELAY_1_ON,REL_BASE);
      //Wait for interrupt or timeout
      sleep(10);
      //Set Relay 1 OFF
      outs(0x0000,REL_BASE);
      //Clear command
      sm_p->open_door_inst_fwd = 0;
    }
      
    
    /* Sleep */
    sleep(sm_p->w[MTRID].per);
  }
  
  mtrctrlC(0);
  return;
}
