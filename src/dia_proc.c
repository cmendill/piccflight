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
int dia_shmfd;

/* CTRL-C Function */
void diactrlC(int sig)
{
#if MSG_CTRLC
  printf("DIA: ctrlC! exiting.\n");
#endif
  close(dia_shmfd);
  exit(sig);
}

/* Main Process */
void dia_proc(void){
  uint32 count = 0;
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&dia_shmfd)) == NULL){
    printf("openshm fail: dia_proc\n");
    diactrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, diactrlC);	/* usually ^C */

  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[DIAID].die)
      diactrlC(0);
    
    /* Check in with the watchdog */
    checkin(sm_p,DIAID);
    
    /* Sleep */
    sleep(sm_p->w[DIAID].per);
  }
  
  diactrlC(0);
  return;
}
