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
static int getlyt_shmfd;
static FILE *out=NULL;
static int getlyt_run=1;
static unsigned long int getlyt_count=0;
static char outfile[MAX_FILENAME];

/* CTRL-C Function */
void getlytctrlC(int sig)
{
#if MSG_CTRLC
  printf("GETLYT: ctrlC! exiting.\n");
#endif
  getlyt_run = 0;
  sleep(1);
  printf("GETLYT: Wrote %lu lytevents to %s\n",getlyt_count,outfile);
  close(getlyt_shmfd);
  fclose(out);
  exit(sig);
}

void getlyt_proc(void){
  static lytevent_t lytevent;
  
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&getlyt_shmfd)) == NULL){
    printf("openshm fail: main\n");
    close(getlyt_shmfd);
    exit(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, getlytctrlC);	/* usually ^C */

  /* Open output file */
  //--setup filename
  sprintf(outfile,"%s",(char *)sm_p->calfile);
  //--open file
  out = fopen(outfile,"w");
  if(out==NULL){
    printf("open failed!\n");
    fclose(out);
    exit(0);
  }
  
  /* Enter loop to read shack-hartmann events */
  while(getlyt_run){
    if(read_from_buffer(sm_p, &lytevent, LYTEVENT, DIAID)){
      //Save lytevent
      fwrite(&lytevent,sizeof(lytevent),1,out);

      //Check in with the watchdog
      if(getlyt_count % 10 == 0) checkin(sm_p,DIAID);

      getlyt_count++;
    }
  }
  sleep(100);
  return;
}
