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
#include <libgen.h>
#include <sys/types.h>
#include <sys/stat.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "watchdog.h"

/* Process File Descriptor */

/* CTRL-C Function */
void getlytctrlC(int sig)
{
#if MSG_CTRLC
  printf("GETLYT: ctrlC! exiting.\n");
#endif
  exit(sig);
}

void getlyt_proc(void){
  lytevent_t lytevent[LYT_NSAMPLES];
  int shmfd;
  FILE *out=NULL;
  unsigned long int count=0,clearcount=0;
  char outfile[MAX_FILENAME];

  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&shmfd)) == NULL){
    perror("GETLYT: openshm()");
    exit(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, getlytctrlC);	/* usually ^C */

  /* Open output file */
  //--setup filename
  sprintf(outfile,"%s",(char *)sm_p->calfile);
  //--create output folder if it does not exist
  check_and_mkdir(outfile);
  //--open file
  if((out = fopen(outfile, "w")) == NULL){
    perror("GETLYT: fopen()\n");
    close(shmfd);
    exit(0);
  }

  /* Start circular buffer */
  sm_p->write_circbuf[BUFFER_LYTEVENT] = 1;
  
  /* Enter loop to read LYT events */
  while(1){
    if(clearcount < LYTEVENTSIZE){
      //Clear the circular buffer to prevent stale data
      if(read_from_buffer(sm_p, &lytevent[0], BUFFER_LYTEVENT, DIAID))
	clearcount++;
    }
    else if(read_from_buffer(sm_p, &lytevent[count % LYT_NSAMPLES], BUFFER_LYTEVENT, DIAID)){
      if(++count % LYT_NSAMPLES == 0){
	//Save lytevent
	fwrite(&lytevent,sizeof(lytevent),1,out);
	
	//Check in with the watchdog
	checkin(sm_p,DIAID);

	//Check if we've been asked to exit
	if(sm_p->w[DIAID].die) break;
      }
    }
  }
  
  /* Set circular buffer back to default */
  sm_p->write_circbuf[BUFFER_LYTEVENT] = WRITE_LYTEVENT_DEFAULT;

  /* Cleanup and exit */
  printf("GETLYT: %lu to %s\n",count,outfile);
  close(shmfd);
  fclose(out);
  
  return;
}
