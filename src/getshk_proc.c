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
int getshk_shmfd;
FILE *out=NULL;
int getshk_run=1;
unsigned long int getshk_count=0;
char outfile[MAX_FILENAME];

/* CTRL-C Function */
void getshkctrlC(int sig)
{
#if MSG_CTRLC
  printf("GETSHK: ctrlC! exiting.\n");
#endif
  getshk_run = 0;
  sleep(1);
  printf("GETSHK: Wrote %lu shkevents to %s\n",getshk_count,outfile);
  close(getshk_shmfd);
  fclose(out);
  exit(sig);
}

void getshk_proc(void){
  static shkevent_t shkevent;
  
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&getshk_shmfd)) == NULL){
    printf("openshm fail: main\n");
    close(getshk_shmfd);
    exit(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, getshkctrlC);	/* usually ^C */

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
  while(getshk_run){
    if(read_from_buffer(sm_p, &shkevent, SHKEVENT, DIAID)){
      //Save shkevent
      fwrite(&shkevent,sizeof(shkevent),1,out);

      //Check in with the watchdog
      if(getshk_count % 10 == 0) checkin(sm_p,DIAID);

      getshk_count++;
    }
  }
  sleep(100);
  return;
}
