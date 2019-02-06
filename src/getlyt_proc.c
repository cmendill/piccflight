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
  lytevent_t lytevent;
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
    perror("GETLYT: openshm()");
    exit(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, getlytctrlC);	/* usually ^C */

  /* Open output file */
  //--setup filename
  sprintf(outfile,"%s",(char *)sm_p->calfile);
  //--create output folder if it does not exist
  strcpy(temp,outfile);
  strcpy(path,dirname(temp));
  if (stat(path, &st) == -1){
    printf("GETLYT: creating folder %s\n",path);
    recursive_mkdir(path, 0777);
  }
  //--open file
  if((out = fopen(outfile, "w")) == NULL){
    perror("GETLYT: fopen()\n");
    close(shmfd);
    exit(0);
  }

  /* Start circular buffer */
  sm_p->write_circbuf[BUFFER_LYTEVENT] = 1;

  /* Enter loop to read LYT events */
  while(!sm_p->w[DIAID].die){
    if(read_from_buffer(sm_p, &lytevent, BUFFER_LYTEVENT, DIAID)){
      //Save lytevent
      fwrite(&lytevent,sizeof(lytevent),1,out);
      
      //Check in with the watchdog
      if(count++ % 10 == 0) checkin(sm_p,DIAID);
    }
  }

  /* Set circular buffer back to default */
  sm_p->write_circbuf[BUFFER_LYTEVENT] = WRITE_LYTEVENT_DEFAULT;

  /* Cleanup and exit */
  printf("GETLYT: Wrote %lu lytevents to %s\n",count,outfile);
  close(shmfd);
  fclose(out);
  
  return;
}
