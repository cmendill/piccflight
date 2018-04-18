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

/* CTRL-C Function */
void hexctrlC(int sig)
{
  double hex_home[HEX_NAXES] = HEX_POS_HOME;
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
  double hexpos[6]={0};
  double hexhome[6] = HEX_POS_HOME;
  double hexdef[6]  = HEX_POS_DEFAULT;
  double pivot[3]   = {HEX_PIVOT_X,HEX_PIVOT_Y,HEX_PIVOT_Z};

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
  if(hex_reference(hexfd, 0)){
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

  /* Set Pivot Point*/
  if(hex_setpivot(hexfd, pivot)){
    printf("HEX: hex_setpivot error!\n");
    sleep(1);
    hexctrlC(0);
  }

  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[HEXID].die)
      hexctrlC(0);

    /* Check in with the watchdog */
    checkin(sm_p,HEXID);

    //Go home
    if(sm_p->hex_gohome){
      memcpy((void *)sm_p->hex,(void *)hexhome,sizeof(hexhome));
      sm_p->hex_gohome=0;
    }

    //Go to default
    if(sm_p->hex_godef){
      memcpy((void *)sm_p->hex,(void *)hexdef,sizeof(hexdef));
      sm_p->hex_godef=0;
    }

    /* Move Hexapod */
    if(hex_move(hexfd,(double *)sm_p->hex)){
      printf("HEX: hex_move error!\n");
      sleep(1);
      hexctrlC(0);
    }

    /* Get Hexapod Position */
    if(sm_p->hex_getpos){
      if(hex_getpos(hexfd,hexpos)){
	printf("HEX: hex_getpos error!\n");
	sleep(1);
	hexctrlC(0);
      }
      //Print position
      printf("HEX: X = %f\n",sm_p->hex[0]);
      printf("HEX: Y = %f\n",sm_p->hex[1]);
      printf("HEX: Z = %f\n",sm_p->hex[2]);
      printf("HEX: U = %f\n",sm_p->hex[3]);
      printf("HEX: V = %f\n",sm_p->hex[4]);
      printf("HEX: W = %f\n",sm_p->hex[5]);
      sm_p->hex_getpos = 0;
    }

    /* Sleep */
    if(sm_p->hex_calmode){
      usleep(ONE_MILLION / 50);
    }else{
      sleep(sm_p->w[HEXID].per);
    }
  }

  hexctrlC(0);
  return;
}
