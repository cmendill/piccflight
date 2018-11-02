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
void getscictrlC(int sig)
{
#if MSG_CTRLC
  printf("GETSCI: ctrlC! exiting.\n");
#endif
  exit(sig);
}

void getsci_proc(void){
  scievent_t scievent;
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
    perror("GETSCI: openshm()");
    exit(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, getscictrlC);	/* usually ^C */

  /* Open output file */
  //--setup filename
  sprintf(outfile,"%s",(char *)sm_p->calfile);
  //--create output folder if it does not exist
  strcpy(temp,outfile);
  strcpy(path,dirname(temp));
  if (stat(path, &st) == -1){
    printf("GETSCI: creating folder %s\n",path);
    recursive_mkdir(path, 0777);
  }
  //--open file
  if((out = fopen(outfile, "w")) == NULL){
    perror("GETSCI: fopen()\n");
    close(shmfd);
    exit(0);
  }

  /* Enter loop to read SCI events */
  while(!sm_p->w[DIAID].die){
    if(read_from_buffer(sm_p, &scievent, SCIEVENT, DIAID)){
      //Save scievent
      fwrite(&scievent,sizeof(scievent),1,out);
      
      //Check in with the watchdog
      if(count++ % 10 == 0) checkin(sm_p,DIAID);
    }
  }

  /* Cleanup and exit */
  printf("GETSCI: Wrote %lu scievents to %s\n",count,outfile);
  close(shmfd);
  fclose(out);
  
  return;
}
