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
int tmp_shmfd;

/* CTRL-C Function */
void tmpctrlC(int sig)
{
#if MSG_CTRLC
  printf("TMP: ctrlC! exiting.\n");
#endif
  close(tmp_shmfd);
  exit(sig);
}

/* Main Process */
void tmp_proc(void){
  uint32 count = 0;
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&tmp_shmfd)) == NULL){
    printf("openshm fail: tmp_proc\n");
    tmpctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, tmpctrlC);	/* usually ^C */

  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[TMPID].die)
      tmpctrlC(0);
    
    /* Check in with the watchdog */
    checkin(sm_p,TMPID);
    
    /* Sleep */
    sleep(sm_p->w[TMPID].per);
  }
  
  tmpctrlC(0);
  return;
}
