#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include "../src/common/controller.h"
#include "../src/common/common_functions.h"

#define DEF_NEVENTS     100     //default number of events to record
#define MAX_NEVENTS     1000000 //maximum number of events to record
#define MIN_NEVENTS     1       //minimum number of events to record

int main(int argc,char **argv){
  char outfile[256];
  FILE *out=NULL;
  unsigned long int count,i,temp,nevents;
  static shkevent_t shkevent;
  
  
  /* Get number of events from command line */
  nevents=DEF_NEVENTS;
  if(argc == 2){
    temp = atol(argv[1]);
    if(temp >= MIN_NEVENTS && temp <= MAX_NEVENTS)
      nevents=temp;
    else{
      printf("Number of events must be between %d and %d\n",MIN_NEVENTS,MAX_NEVENTS);
      exit(0);
    }
  }
  
  /* Open Shared Memory */
  sm_t *sm_p;
  int shmfd;
  if((sm_p = openshm(&shmfd)) == NULL){
    printf(WARNING);
    printf("openshm fail: main\n");
    close(shmfd);
    exit(0);
  }
  /* Open output file */
  //--setup filename
  sprintf(outfile,"getshk.txt");
  //--open file
  out = fopen(outfile,"w");
  if(out==NULL){
    printf("open failed!\n");
    fclose(out);
    exit(0);
  }
  else
    printf("getshk: opened %s\n",outfile);
  
  /* Enter loop to read shack-hartmann events */
  /* NOTE: IM NOT SURE WHY READ_BUFFER_EVENT DOES NOT WORK HERE. 
     NEED TO FIGURE THIS OUT. GIVES A SEGFAULT IF USED. 
  */
  count=0;
  while(1){
    if(check_buffer(sm_p,SHKEVENT,DIAID)){
      memcpy((void *)&shkevent,
	     (void *)&sm_p->shkevent[sm_p->circbuf[SHKEVENT].read_offsets[DIAID] % sm_p->circbuf[SHKEVENT].bufsize],sizeof(shkevent_t));
      sm_p->circbuf[SHKEVENT].read_offsets[DIAID]++;
      for(i=0;i<SHK_NCELLS;i++)
	fprintf(out,"%10d,%5d,%5d,%5d,%15.8f,%15.8f,%15.8f,%15.8f,%15.8f,%15.8f\n",
		shkevent.frame_number,shkevent.cells[i].index,shkevent.cells[i].spot_found,shkevent.cells[i].spot_captured,
		shkevent.cells[i].origin[0],shkevent.cells[i].origin[1],
		shkevent.cells[i].centroid[0],shkevent.cells[i].centroid[1],
		shkevent.cells[i].deviation[0],shkevent.cells[i].deviation[1]
		);
      if(++count == nevents)
	break;
    }
  }
  printf("getshk: recorded %lu events\n",nevents);
  //clean up
  fclose(out);
  close(shmfd);
  return 0;
}
