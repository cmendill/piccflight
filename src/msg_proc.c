#define _XOPEN_SOURCE 500
#define _POSIX_C_SOURCE 200809L
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <libgen.h>
#include <sys/types.h>
#include <sys/stat.h>


/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "watchdog.h"

/* Process File Descriptor */

/* CTRL-C Function */
void msgctrlC(int sig)
{
#if MSG_CTRLC
  printf("MSG: ctrlC! exiting.\n");
#endif
  exit(sig);
}

void msg_proc(void){
  msgevent_t msgevent;
  int shmfd;
  FILE *input;
  size_t len;
  ssize_t nread;
  char *line=NULL;
  static struct timespec start;
  static uint32 count = 0;
  int state;
  
  /* Set soft interrupt handler */
  sigset(SIGINT, msgctrlC);	/* usually ^C */

  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&shmfd)) == NULL){
    perror("MSG: openshm()");
    msgctrlC(0);
  }

  /* Open input file */
  //--open file
  if((input = fopen("output/data/output.txt","r")) == NULL){
    perror("MSG: fopen()\n");
    close(shmfd);
    msgctrlC(0);
  }
  
  /* Start main loop */
  printf("MSG: Running message capture\n");
  
  while(1){
    /* Get start time */
    clock_gettime(CLOCK_REALTIME,&start);

    //Get state
    state = sm_p->state;

    //Fill out event header
    msgevent.hed.version       = PICC_PKT_VERSION;
    msgevent.hed.type          = BUFFER_MSGEVENT;
    msgevent.hed.frame_number  = count++;
    msgevent.hed.state         = state;
    msgevent.hed.alp_commander = sm_p->state_array[state].alp_commander;
    msgevent.hed.hex_commander = sm_p->state_array[state].hex_commander;
    msgevent.hed.bmc_commander = sm_p->state_array[state].bmc_commander;
    msgevent.hed.start_sec     = start.tv_sec;
    msgevent.hed.start_nsec    = start.tv_nsec;
    memset(msgevent.message,0,sizeof(msgevent.message));
    
    //Read data
    if(getline(&line,&len,input) > 0){
      //Copy message
      strncpy(msgevent.message,line,MAX_LINE);
      free(line);
      line=NULL;
      //Write event to circular buffer
      if(sm_p->circbuf[BUFFER_MSGEVENT].write) write_to_buffer(sm_p,&msgevent,BUFFER_MSGEVENT);
    }
    else{
      sleep(sm_p->w[MSGID].per);
    }
      
    //Check in with the watchdog
    checkin(sm_p,MSGID);
  }
  
    
  /* Cleanup and exit */
  close(shmfd);
  fclose(input);
  
  return;
}
