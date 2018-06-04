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
#include <PI_GCS2_DLL.h>


/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
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
  double scopepos[6] = {0};
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

  /************** If Hexapod is Enabled **************/
  if(HEX_ENABLE){
    
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
  }

  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[HEXID].die)
      hexctrlC(0);

    /* Check in with the watchdog */
    checkin(sm_p,HEXID);

    /************** If Hexapod is Enabled **************/
    if(HEX_ENABLE){
      /* USER COMMANDS: Command collisiion protected by state permissions */

      /* Command: Go to home position */
      if(sm_p->hex_gohome){
	memcpy((double *)sm_p->hex_command,hexhome,sizeof(hexhome));
	sm_p->hex_gohome=0;
	sm_p->hex_last_sent++;
      }

      /* Command: Go to default position */
      if(sm_p->hex_godef){
	memcpy((double *)sm_p->hex_command,hexdef,sizeof(hexdef));
	sm_p->hex_godef=0;
	sm_p->hex_last_sent++;
      }
    
      /* Command: Get Hexapod Position */
      if(sm_p->hex_getpos){
	if(hex_getpos(hexfd,hexpos)){
	  printf("HEX: hex_getpos error!\n");
	  sleep(1);
	  hexctrlC(0);
	}
	for(i=0;i<HEX_NAXES;i++){
	  scopepos[i] = 0;
	}
	hex_hex2scope(hexpos, scopepos);
	//Print position
	printf("HEX: X = %f \n",scopepos[0]);
	printf("HEX: Y = %f \n",scopepos[1]);
	printf("HEX: Z = %f \n",scopepos[2]);
	printf("HEX: U = %f \n",scopepos[3]);
	printf("HEX: V = %f \n",scopepos[4]);
	printf("HEX: W = %f \n",scopepos[5]);
	sm_p->hex_getpos = 0;
      }

      /**** Write Commands to Hexapod ****/
      /* Read Newest Command */
      if(sm_p->hex_last_sent > sm_p->hex_last_recv){ 
	
	/* Move Hexapod */
	if(hex_move(hexfd,(double *)sm_p->hex_command)){
	  printf("HEX: hex_move error!\n");
	  hexctrlC(0);
	}

	/* Acknowledge Command */
	sm_p->hex_last_recv = sm_p->hex_last_sent;
      }
    }

    /* Sleep */
    sleep(sm_p->w[HEXID].per);
  }
  
  hexctrlC(0);
  return;
}

