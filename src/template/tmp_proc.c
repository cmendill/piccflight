#include user_header.h
#include controller.h
#include common_functions.h
int tmp_shmfd;


void tmpctrlC(int sig)
{
#if MSG_CTRLC
  printf("TMP: ctrlC! exiting.\n");
#endif
  close(tmp_shmfd);
  exit(sig);
}

void tmp_proc(void){
  uint32 count = 0;
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&tmp_shmfd)) == NULL){
    printf("openshm fail: tmp_proc\n");
    tmpctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, tmpctrlC);	/* usually ^C */

  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[TMPID].die)
      tmpctrlC(0);
    
    /* Check in with the watchdog */
    checkin(sm_p,TMPID);
    
    /* Sleep */
    sleep(sm_p->w[TMPID].per);
  }
  
  tmpctrlC(0);
  return;
}
