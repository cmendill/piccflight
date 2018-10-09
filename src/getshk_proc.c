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

/* Process File Descriptor */

/* CTRL-C Function */
void getshkctrlC(int sig)
{
#if MSG_CTRLC
  printf("GETSHK: ctrlC! exiting.\n");
#endif
  exit(sig);
}

void getshk_proc(void){
  shkevent_t shkevent;
  struct stat st = {0};
  int shmfd;
  FILE *out=NULL;
  unsigned long int count=0;
  char outfile[MAX_FILENAME];
  char temp[MAX_FILENAME];
  char path[MAX_FILENAME];

  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&shmfd)) == NULL){
    perror("GETSHK: openshm()");
    exit(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, getshkctrlC);	/* usually ^C */

  /* Open output file */
  //--setup filename
  sprintf(outfile,"%s",(char *)sm_p->calfile);
  //--create output folder if it does not exist
  strcpy(temp,outfile);
  strcpy(path,dirname(temp));
  if (stat(path, &st) == -1){
    printf("GETSHK: creating folder %s\n",path);
    recursive_mkdir(path, 0777);
  }
  //--open file
  if((out = fopen(outfile, "w")) == NULL){
    perror("GETSHK: fopen()\n");
    close(shmfd);
    exit(0);
  }

  /* Enter loop to read SHK events */
  while(!sm_p->w[DIAID].die){
    if(read_from_buffer(sm_p, &shkevent, SHKEVENT, DIAID)){
      //Save shkevent
      fwrite(&shkevent,sizeof(shkevent),1,out);
      
      //Check in with the watchdog
      if(count++ % 10 == 0) checkin(sm_p,DIAID);
    }
  }

  /* Cleanup and exit */
  printf("GETSHK: Wrote %lu shkevents to %s\n",count,outfile);
  close(shmfd);
  fclose(out);
  
  return;
}
