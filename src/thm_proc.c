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
int thm_shmfd;

/* CTRL-C Function */
void thmctrlC(int sig)
{
#if MSG_CTRLC
  printf("THM: ctrlC! exiting.\n");
#endif
  close(thm_shmfd);
  exit(sig);
}

/* Main Process */
void thm_proc(void){
  uint32 count = 0;
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&thm_shmfd)) == NULL){
    printf("openshm fail: thm_proc\n");
    thmctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, thmctrlC);	/* usually ^C */

  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[THMID].die)
      thmctrlC(0);
    
    /* Check in with the watchdog */
    checkin(sm_p,THMID);
    
    /* Sleep */
    sleep(sm_p->w[THMID].per);
  }
  
  thmctrlC(0);
  return;
}
