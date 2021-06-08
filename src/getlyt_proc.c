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

/* number of samples per write */
#define NSAMPLES LYTEVENTSIZE

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
  lytevent_t lytevent[NSAMPLES];
  int shmfd;
  FILE *out=NULL;
  unsigned long int count=0,clearcount=0;
  char outfile[MAX_FILENAME];
  double dt,maxdt=0;
  struct timespec start,end,delta;
  int circbuf_save[NCIRCBUF];
  int i;
  
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
  
  /* Disable tlm_proc data saving */
  for(i=0;i<NCIRCBUF;i++){
    circbuf_save[i] = sm_p->circbuf[i].save;
    sm_p->circbuf[i].save = 0;
  }

  /* Start circular buffer */
  sm_p->write_circbuf[BUFFER_LYTEVENT] = 1;

  /* Clear the circular buffer to prevent stale data */
  while(clearcount < (2*LYTEVENTSIZE))
    if(read_from_buffer(sm_p, &lytevent[0], BUFFER_LYTEVENT, DIAID))
      clearcount++;
  
  /* Enter loop to read LYT events */
  while(1){
    if(read_from_buffer(sm_p, &lytevent[count % NSAMPLES], BUFFER_LYTEVENT, DIAID)){
      if((++count % NSAMPLES) == 0){
	//Get start time
	clock_gettime(CLOCK_REALTIME,&start);

	//Save lytevent
	fwrite(&lytevent,sizeof(lytevent),1,out);
	
	//Check in with the watchdog
	checkin(sm_p,DIAID);

	//Check if we've been asked to exit
	if(sm_p->w[DIAID].die) break;

	//Get end time
	clock_gettime(CLOCK_REALTIME,&end);
	if(timespec_subtract(&delta,&end,&start))
	  printf("GETLYT: timespec_subtract error!\n");
	ts2double(&delta,&dt);
	if(dt > maxdt) maxdt=dt;
	
      }
    }
  }
  
  /* Set circular buffer back to default */
  sm_p->write_circbuf[BUFFER_LYTEVENT] = WRITE_LYTEVENT_DEFAULT;

  /* Enable tlm_proc data saving */
  for(i=0;i<NCIRCBUF;i++){
    sm_p->circbuf[i].save = circbuf_save[i];
  }

  /* Cleanup and exit */
  printf("GETLYT: %lu to %s\n",count,outfile);
  printf("GETLYT: maxdt = %ld ms | bufdt = %ld ms\n",lround(maxdt*1000),lround(lytevent[0].hed.frmtime * LYTEVENTSIZE * 1000));
  close(shmfd);
  fclose(out);
    
  return;
}
