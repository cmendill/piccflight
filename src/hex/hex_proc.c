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
#include "../common/controller.h"
#include "../common/common_functions.h"
#include "PI_GCS2_DLL.h"
#include "hex_functions.h"


/* Process File Descriptor */
int hex_shmfd;
int hexfd;

/* Hexapod Home Position */
double hex_home[HEX_NAXES] = HEX_POS_HOME;

/* CTRL-C Function */
void hexctrlC(int sig)
{
#if MSG_CTRLC
  printf("HEX: ctrlC! exiting.\n");
#endif
  hex_move(hexfd,hex_home);
  close(hex_shmfd);
  PI_CloseConnection(hexfd);
  exit(sig);
}

/* Main Process */
void hex_proc(void){
  uint32 count = 0;
  int bFlag,i;
  
 
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&hex_shmfd)) == NULL){
    printf("openshm fail: hex_proc\n");
    hexctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, hexctrlC);	/* usually ^C */

  /* Connect to Hexapod */
  if((hexfd=hex_connect()) < 0){
    printf("HEX: hex_connect error!\n");
    sleep(1);
    hexctrlC(0);
  }

  /* Reference Hexapod */
  if(hex_reference(hexfd, 1)){
    printf("HEX: hex_reference error!\n");
    sleep(1);
    hexctrlC(0);    
  }
  
  /* Wait for Referencing to Finish */
  for(i=0;i<HEX_REF_TIMEOUT;i++){
    bFlag = 0;
    if(!PI_IsControllerReady(hexfd, &bFlag)){
      printf("HEX: PI_IsControllerReady error!\n");
      sleep(1);
      hexctrlC(0);    
    }
    if(bFlag) break;
    /* Check in with the watchdog */
    checkin(sm_p,HEXID);
    sleep(1);
  }
  if(i==HEX_REF_TIMEOUT){
    printf("HEX: Referencing timeout!!\n");
    sleep(1);
    hexctrlC(0);    
  }else{
    printf("HEX: Referencing complete after %d seconds\n",i);
  }
  

  
  
  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[HEXID].die)
      hexctrlC(0);
    
    /* Check in with the watchdog */
    checkin(sm_p,HEXID);
    
    /* Move Hexapod */
    if(hex_move(hexfd,(double *)sm_p->hex)){
      printf("HEX: hex_move error!\n");
      sleep(1);
      hexctrlC(0);    
    }
    
    /* Sleep */
    sleep(sm_p->w[HEXID].per);
  }
  
  hexctrlC(0);
  return;
}
