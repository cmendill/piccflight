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
#include "../common/controller.h"
#include "../common/common_functions.h"

/* Process File Descriptor */
int hsk_shmfd;

/* CTRL-C Function */
void hskctrlC(int sig)
{
#if MSG_CTRLC
  printf("HSK: ctrlC! exiting.\n");
#endif
  close(hsk_shmfd);
  exit(sig);
}

/* Main Process */
void hsk_proc(void){
  uint32 count = 0;
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&hsk_shmfd)) == NULL){
    printf("openshm fail: hsk_proc\n");
    hskctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, hskctrlC);	/* usually ^C */

  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[HSKID].die)
      hskctrlC(0);
    
    /* Check in with the watchdog */
    checkin(sm_p,HSKID);
    
    /* Sleep */
    sleep(sm_p->w[HSKID].per);
  }
  
  hskctrlC(0);
  return;
}
