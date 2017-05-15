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
#include "../common/controller.h"

/* Globals */
volatile int clientfd=-1;
pthread_t listener_thread;
int srv_shmfd;    // shared memory file desciptor

/* Prototypes */
void  checkin(sm_t *sm_p,int id);
sm_t *openshm(int *shmfd);
int read_from_image_buffer(sm_t *buffer, struct fullimageevent *fie, int id);
void *listener_loop(void *t);
int write_to_socket(int s,void *buf,int num);


void srvctrlC(int sig)
{
  if(CTRLC_DEBUG) printf("SRV: Got CTRL-C\n");
  close(srv_shmfd);
  close(clientfd);
  exit(sig);
}

void srv_proc(void) {
  
  int nbytes;
  struct fullimageevent fie;
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
      while(read_from_image_buffer(sm_p, &fie, SRVID)){
	nbytes=write_to_socket(clientfd,&fie,sizeof(fie));
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
