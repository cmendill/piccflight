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
#include "controller.h"
#include "common_functions.h"
#include "watchdog.h"

/* Globals */
volatile int clientfd=-1;
volatile int srv_send[NCIRCBUF]; //switches to turn sending on and off
pthread_t listener_thread;
int srv_shmfd;    // shared memory file desciptor
uint32 srv_packet_count=0;

/* Prototypes */
void *srv_listen(void *t);

void srvctrlC(int sig)
{
#if MSG_CTRLC
  printf("SRV: ctrlC! exiting.\n");
  printf("SRV: sent %d packets.\n",srv_packet_count);
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
  unsigned long data_ready;
  
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

  /* Set data switches to defaults */
  sm_p->write_circbuf[BUFFER_SCIEVENT] = WRITE_SCIEVENT_DEFAULT;
  sm_p->write_circbuf[BUFFER_SHKEVENT] = WRITE_SHKEVENT_DEFAULT;
  sm_p->write_circbuf[BUFFER_LYTEVENT] = WRITE_LYTEVENT_DEFAULT;
  sm_p->write_circbuf[BUFFER_ACQEVENT] = WRITE_ACQEVENT_DEFAULT;
  sm_p->write_circbuf[BUFFER_MTREVENT] = WRITE_MTREVENT_DEFAULT;
  sm_p->write_circbuf[BUFFER_THMEVENT] = WRITE_THMEVENT_DEFAULT;
  sm_p->write_circbuf[BUFFER_SCIFULL]  = WRITE_SCIFULL_DEFAULT;
  sm_p->write_circbuf[BUFFER_SHKFULL]  = WRITE_SHKFULL_DEFAULT;
  sm_p->write_circbuf[BUFFER_LYTFULL]  = WRITE_LYTFULL_DEFAULT;
  sm_p->write_circbuf[BUFFER_ACQFULL]  = WRITE_ACQFULL_DEFAULT;
  sm_p->write_circbuf[BUFFER_SHKPKT]   = WRITE_SHKPKT_DEFAULT;
  sm_p->write_circbuf[BUFFER_LYTPKT]   = WRITE_LYTPKT_DEFAULT;

  /* Find maximum buffer size */
  for(i=0;i<NCIRCBUF;i++){
    if(sm_p->circbuf[i].nbytes > max_buf_size)
      max_buf_size = sm_p->circbuf[i].nbytes;
    if(SRV_DEBUG)
      printf("SRV: %10s Buffer Size: %d bytes\n",sm_p->circbuf[i].name,sm_p->circbuf[i].nbytes);
  }

  /* Allocate buffer */
  if((buffer = malloc(max_buf_size))==NULL){
    printf("SRV: Error allocating memory for buffer\n");
    perror("malloc");
    srvctrlC(0);
  }
  
  /* Start Listener */
  pthread_create(&listener_thread,NULL,srv_listen,(void *)0);
  
  while(1){
    if(clientfd >= 0){
      //first check if there is any data
      data_ready = 0;
      for(i=0;i<NCIRCBUF;i++)
	if(srv_send[i]){
	  //enable data
	  sm_p->write_circbuf[i] = 1;
	  //check data
	  data_ready += check_buffer(sm_p,i,SRVID);
	}
      //read data if its ready
      if(data_ready){
	for(i=0;i<NCIRCBUF;i++){
	  if(srv_send[i]){
	    if(read_from_buffer(sm_p, buffer, i, SRVID)){
	      //write data to socket
	      if(SRV_DEBUG) printf("SRV: Writing Packet %d\n",i);
	      nbytes=write_to_socket(clientfd,buffer,sm_p->circbuf[i].nbytes);
	      if(nbytes <=0){
		close(clientfd);
		clientfd=-1;
		//reset srv_send
		memset((void *)srv_send,0,sizeof srv_send);
		//set data switches back to defaults
		sm_p->write_circbuf[BUFFER_SCIEVENT] = WRITE_SCIEVENT_DEFAULT;
		sm_p->write_circbuf[BUFFER_SHKEVENT] = WRITE_SHKEVENT_DEFAULT;
		sm_p->write_circbuf[BUFFER_LYTEVENT] = WRITE_LYTEVENT_DEFAULT;
		sm_p->write_circbuf[BUFFER_ACQEVENT] = WRITE_ACQEVENT_DEFAULT;
		sm_p->write_circbuf[BUFFER_MTREVENT] = WRITE_MTREVENT_DEFAULT;
		sm_p->write_circbuf[BUFFER_THMEVENT] = WRITE_THMEVENT_DEFAULT;
		sm_p->write_circbuf[BUFFER_SCIFULL]  = WRITE_SCIFULL_DEFAULT;
		sm_p->write_circbuf[BUFFER_SHKFULL]  = WRITE_SHKFULL_DEFAULT;
		sm_p->write_circbuf[BUFFER_LYTFULL]  = WRITE_LYTFULL_DEFAULT;
		sm_p->write_circbuf[BUFFER_ACQFULL]  = WRITE_ACQFULL_DEFAULT;
		sm_p->write_circbuf[BUFFER_SHKPKT]   = WRITE_SHKPKT_DEFAULT;
		sm_p->write_circbuf[BUFFER_LYTPKT]   = WRITE_LYTPKT_DEFAULT;
				
		printf("SRV: Client hung up\n");
	      }
	      //increment packet counter
	      srv_packet_count++;
	      //check in with watchdog -- make this slower
	      checkin(sm_p,SRVID);
	    }
	  }
	}
      }else{
	//sleep if there is no data
	usleep(100000);
      }
    }else{
      //sleep
      sleep(sm_p->w[SRVID].per);
      //check in with watchdog
      checkin(sm_p,SRVID);
    }
  }
  srvctrlC(0);
}
