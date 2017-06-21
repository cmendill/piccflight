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
#include "../common/controller.h"
#include "../common/common_functions.h"

/* Process File Descriptor */
int getshk_shmfd;
FILE *out=NULL;
int getshk_run=1;
unsigned long int getshk_count=0;

/* CTRL-C Function */
void getshkctrlC(int sig)
{
#if MSG_CTRLC
  printf("GETSHK: ctrlC! exiting.\n");
#endif
  getshk_run = 0;
  sleep(1);
  printf("GETSHK: Collected %lu shkevents\n",getshk_count);
  close(getshk_shmfd);
  fclose(out);
  exit(sig);
}

void getshk_proc(void){
  char outfile[256];
  static shkevent_t shkevent;
  
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&getshk_shmfd)) == NULL){
    printf(WARNING);
    printf("openshm fail: main\n");
    close(getshk_shmfd);
    exit(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, getshkctrlC);	/* usually ^C */

  /* Open output file */
  //--setup filename
  sprintf(outfile,"data/getshk_output.dat");
  //--open file
  out = fopen(outfile,"w");
  if(out==NULL){
    printf("open failed!\n");
    fclose(out);
    exit(0);
  }
  else
    printf("GETSHK: opened %s\n",outfile);
  
  /* Enter loop to read shack-hartmann events */
  /* NOTE: IM NOT SURE WHY READ_BUFFER_EVENT DOES NOT WORK HERE. 
     NEED TO FIGURE THIS OUT. GIVES A SEGFAULT IF USED. 
  */
  while(getshk_run){
    if(read_from_buffer(sm_p, &shkevent, SHKEVENT, DIAID)){
      //Save shkevent
      fwrite(&shkevent,sizeof(shkevent),1,out);
      getshk_count++;
    }
  }
  sleep(100);
  return;
}
