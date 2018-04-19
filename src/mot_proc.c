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

/* Process File Descriptor */
int mot_shmfd;

/* CTRL-C Function */
void motctrlC(int sig)
{
#if MSG_CTRLC
  printf("MOT: ctrlC! exiting.\n");
#endif
  close(mot_shmfd);
  exit(sig);
}

/* Main Process */
void mot_proc(void){
  uint32 count = 0;
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&mot_shmfd)) == NULL){
    printf("openshm fail: mot_proc\n");
    motctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, motctrlC);	/* usually ^C */

  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[MOTID].die)
      motctrlC(0);
    
    /* Check in with the watchdog */
    checkin(sm_p,MOTID);
    
    /* Sleep */
    sleep(sm_p->w[MOTID].per);
  }
  
  motctrlC(0);
  return;
}
