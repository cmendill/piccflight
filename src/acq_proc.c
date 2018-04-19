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
int acq_shmfd;

/* CTRL-C Function */
void acqctrlC(int sig)
{
#if MSG_CTRLC
  printf("ACQ: ctrlC! exiting.\n");
#endif
  close(acq_shmfd);
  exit(sig);
}

/* Main Process */
void acq_proc(void){
  uint32 count = 0;
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&acq_shmfd)) == NULL){
    printf("openshm fail: acq_proc\n");
    acqctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, acqctrlC);	/* usually ^C */

  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[ACQID].die)
      acqctrlC(0);
    
    /* Check in with the watchdog */
    checkin(sm_p,ACQID);
    
    /* Sleep */
    sleep(sm_p->w[ACQID].per);
  }
  
  acqctrlC(0);
  return;
}
