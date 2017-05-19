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
#include "../common/controller.h"
#include "../common/common_functions.h"

/* Process File Descriptor */
int sci_shmfd;

/* CTRL-C Function */
void scictrlC(int sig)
{
#if MSG_CTRLC
  printf("SCI: ctrlC! exiting.\n");
#endif
  close(sci_shmfd);
  exit(sig);
}

/* Main Process */
void sci_proc(void){
  uint32 count = 0;
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&sci_shmfd)) == NULL){
    printf("openshm fail: sci_proc\n");
    scictrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, scictrlC);	/* usually ^C */

  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[SCIID].die)
      scictrlC(0);
    
    /* Check in with the watchdog */
    checkin(sm_p,SCIID);
    
    /* Sleep */
    sleep(sm_p->w[SCIID].per);
  }
  
  scictrlC(0);
  return;
}
