#define _XOPEN_SOURCE 500
#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>
#include <pthread.h>

/* piccflight headers */
#include "../common/controller.h"
#include "../common/common_functions.h"

/* Globals */
volatile int clientfd=-1;
pthread_t listener_thread;
int srv_shmfd;    // shared memory file desciptor

/* Prototypes */
void *listener_loop(void *t);


void srvctrlC(int sig)
{
  close(srv_shmfd);
  close(clientfd);
  exit(sig);
}

void srv_proc(void) {
  
  int nbytes;
  shkevent_t shk;
  volatile uint32 count=0;

  /* Set soft interrupt handler */
  sigset(SIGINT, srvctrlC);	/* usually ^C */

  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&srv_shmfd)) == NULL){
    printf("openshm fail: srv_proc\n");
    srvctrlC(0);
  }

  /* Start Listener */
  pthread_create(&listener_thread,NULL,listener_loop,(void *)0);
  
  while(1){
    //every 1 sec
    if((count++ % (1000/sm_p->w[SRVID].per)) == 0){
      //check if we've been killed
      if(sm_p->w[SRVID].die)
	srvctrlC(0);
      //check in with watchdog
      checkin(sm_p,SRVID);
    }
    if(clientfd >= 0){
      while(read_from_buffer(sm_p, &shk, SHKBUF, SRVID)){
	nbytes=write_to_socket(clientfd,&shk,sizeof(shk));
	if(nbytes <=0){
	  close(clientfd);
	  clientfd=-1;
	}
      }
    }
    usleep(sm_p->w[SRVID].per*1000);
  }
  srvctrlC(0);
}
