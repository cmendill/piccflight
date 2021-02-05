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
void getscictrlC(int sig)
{
#if MSG_CTRLC
  printf("GETSCI: ctrlC! exiting.\n");
#endif
  exit(sig);
}

void getsci_proc(void){
  scievent_t scievent[SCI_NSAMPLES];
  int shmfd;
  FILE *out=NULL;
  unsigned long int count=0,clearcount=0;
  char outfile[MAX_FILENAME];

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
  check_and_mkdir(outfile);
  //--open file
  if((out = fopen(outfile, "w")) == NULL){
    perror("GETSCI: fopen()\n");
    close(shmfd);
    exit(0);
  }

  /* Start circular buffer */
  sm_p->write_circbuf[BUFFER_SCIEVENT] = 1;
  
  /* Enter loop to read SCI events */
  while(1){
    if(clearcount < SCIEVENTSIZE){
      //Clear the circular buffer to prevent stale data
      if(read_from_buffer(sm_p, &scievent[0], BUFFER_SCIEVENT, DIAID))
	clearcount++;
    }
    else if(read_from_buffer(sm_p, &scievent[count % SCI_NSAMPLES], BUFFER_SCIEVENT, DIAID)){
      if(++count % SCI_NSAMPLES == 0){
	//Save scievent
	fwrite(&scievent,sizeof(scievent),1,out);
	
	//Check in with the watchdog
	checkin(sm_p,DIAID);

	//Check if we've been asked to exit
	if(sm_p->w[DIAID].die) break;
      }
    }
  }
  
  /* Set circular buffer back to default */
  sm_p->write_circbuf[BUFFER_SCIEVENT] = WRITE_SCIEVENT_DEFAULT;

  /* Cleanup and exit */
  printf("GETSCI: %lu to %s\n",count,outfile);
  close(shmfd);
  fclose(out);
  
  return;
}
