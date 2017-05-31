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

int main(int argc,char **argv){
  char outfile[250];
  FILE *out=NULL;
  int count,i;
  static shkevent_t shkevent;
  static shkfull_t shkfull;
  
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

  //memcpy((void *)&shkevent,(void *)&sm_p->shkevent[0],sizeof(shkevent_t));
  printf("ID %d\n",DIAID);
  printf("Read Offset %d\n",sm_p->circbuf[SHKEVENT].read_offsets[DIAID]);
  printf("Write Offset %d\n",sm_p->circbuf[SHKEVENT].write_offset);
  usleep(1000);
  
  /* Enter loop to read shack-hartmann events */
  count=0;
  while(1){
    if(read_from_buffer(sm_p,(void *)&shkevent,SHKEVENT,DIAID)){
      //for(i=0;i<SHK_NCELLS;i++)
      //fprintf(out,"%10d,%10.4f,%10.4f\n",shkevent.frame_number,shkevent.cells[i].deviation[0],shkevent.cells[i].deviation[1]);
      //if(++count == 100)
      break;
    }
  }
  //clean up
  fclose(out);
  close(shmfd);
  return 0;
}
