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
volatile int srv_send[NCIRCBUF]; //switches to turn sending on and off
pthread_t listener_thread;
int srv_shmfd;    // shared memory file desciptor
uint32 srv_packet_count=0;

/* Prototypes */
void *srv_listener(void *t);

void srvctrlC(int sig)
{
#if MSG_CTRLC
  printf("SRV: ctrlC! exiting.\n");
  printf("SRV: sent %lu packets.\n",srv_packet_count);
#endif
  close(srv_shmfd);
  close(clientfd);
  memset((void *)srv_send,0,sizeof srv_send);
  exit(sig);
}

void srv_proc(void) {
  int nbytes,i;
  void *buffer;
  int max_buf_size=0;
  
  /* Set soft interrupt handler */
  sigset(SIGINT, srvctrlC);	/* usually ^C */

  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&srv_shmfd)) == NULL){
    printf("openshm fail: srv_proc\n");
    srvctrlC(0);
  }

  /* Zero out srv_send */
  memset((void *)srv_send,0,sizeof srv_send);

  /* Find maximum buffer size */
  for(i=0;i<NCIRCBUF;i++){
    if(sm_p->circbuf[i].nbytes > max_buf_size)
      max_buf_size = sm_p->circbuf[i].nbytes;
    if(SRV_DEBUG)
      printf("SRV: %10s Buffer Size: %lu bytes\n",sm_p->circbuf[i].name,sm_p->circbuf[i].nbytes);
  }

  /* Allocate buffer */
  if((buffer = malloc(max_buf_size))==NULL){
    printf("SRV: Error allocating memory for buffer\n");
    perror("malloc");
    srvctrlC(0);
  }
  
  /* Start Listener */
  pthread_create(&listener_thread,NULL,srv_listener,(void *)0);
  
  while(1){
    //check in with watchdog
    checkin(sm_p,SRVID);
    
    if(clientfd >= 0){
      for(i=0;i<NCIRCBUF;i++){
	if(srv_send[i]){
	  while(read_from_buffer(sm_p, buffer, i, SRVID)){
	    //write data to socket
	    nbytes=write_to_socket(clientfd,buffer,sm_p->circbuf[i].nbytes);
	    if(nbytes <=0){
	      close(clientfd);
	      clientfd=-1;
	      memset((void *)srv_send,0,sizeof srv_send);
	    }
	    //increment packet counter
	    srv_packet_count++;
	    //check in with watchdog
	    checkin(sm_p,SRVID);
	  }
	}
      }
    }
  }
  srvctrlC(0);
}
