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


/* Return to Sender */
void return_to_sender(sm_t *sm_p, hexevent_t *hexevent_p){
  if(hexevent_p->clientid == SHKID)
    write_to_buffer(sm_p,hexevent_p,SHK_HEXRECV);
  if(hexevent_p->clientid == LYTID)
    write_to_buffer(sm_p,hexevent_p,LYT_HEXRECV);
  if(hexevent_p->clientid == ACQID)
    write_to_buffer(sm_p,hexevent_p,ACQ_HEXRECV);
  if(hexevent_p->clientid == WATID)
    write_to_buffer(sm_p,hexevent_p,WAT_HEXRECV);
}

/* Main Process */
void hex_proc(void){
  int bFlag,i,state,commander;
  double hexpos[6]={0};
  double scopepos[6] = {0};
  double pivot[3]   = {HEX_PIVOT_X,HEX_PIVOT_Y,HEX_PIVOT_Z};
  hexevent_t hexbuf,hexcmd;
  int gotcmd=0;
  int buffer[4] = {SHK_HEXSEND,LYT_HEXSEND,ACQ_HEXSEND,WAT_HEXSEND};
  int nbuffers  = 4;
  
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

    /* Get State */
    state = sm_p->state;
    
    /* Get HEX Commander */
    commander = sm_p->state_array[state].hex_commander;
    
    /************** If Hexapod is Enabled **************/
    if(HEX_ENABLE){
      /* User Command: Get Hexapod Position */
      if(sm_p->hex_getpos){
	if(hex_getpos(hexfd,hexpos)){
	  printf("HEX: hex_getpos error!\n");
	  sleep(1);
	  hexctrlC(0);
	}
	for(i=0;i<HEX_NAXES;i++)
	  scopepos[i] = 0;
	
	hex_hex2scope(hexpos, scopepos);
	//Print position
	printf("HEX: X = %f \n",scopepos[0]);
	printf("HEX: Y = %f \n",scopepos[1]);
	printf("HEX: Z = %f \n",scopepos[2]);
	printf("HEX: U = %f \n",scopepos[3]);
	printf("HEX: V = %f \n",scopepos[4]);
	printf("HEX: W = %f \n",scopepos[5]);
	printf("HEX: {%f,%f,%f,%f,%f,%f}\n",scopepos[0],scopepos[1],scopepos[2],scopepos[3],scopepos[4],scopepos[5]);
	sm_p->hex_getpos = 0;
      }

      /**** GET BUFFERED HEXAPOD COMMANDS ****/
      gotcmd=0;
      /* Loop through all command buffers */
      /*  - Goal is to return all commands to sender immediately, */
      /*    except the one we will execute.                       */
      for(i=0;i<nbuffers;i++){
	
	/* Read all commands from buffer*/
	while(read_from_buffer(sm_p,&hexbuf,buffer[i],HEXID)){ 
	  
	  /* If this is the commanding process */
	  if(commander == hexbuf.clientid){
	    /* If we already got a command, return last one to sender */
	    if(gotcmd)
	      return_to_sender(sm_p,&hexcmd);

	    /* Copy to cmd event and set command trigger */
	    memcpy(&hexcmd,&hexbuf,sizeof(hexevent_t));
	    gotcmd = 1;
	  }
	  else{
	    /* Return to sender */
	    return_to_sender(sm_p,&hexbuf);
	  }
	}
      }

      /* Execute command */
      if(gotcmd){
	
	/* Move Hexapod */
	if(hex_move(hexfd,hexcmd.hex.axis_cmd)){
	  printf("HEX: hex_move error!\n");
	  hexctrlC(0);
	}
	
	/* Write event to main recv buffer */
	write_to_buffer(sm_p,&hexcmd,HEXRECV);

	/* Return executed command to sender */
	return_to_sender(sm_p,&hexcmd);
	
	/* Debugging */
	if(HEX_DEBUG){
	  fprintf(stderr,"HEX: Accepted command %lu from: %d\n",hexcmd.command_number,hexcmd.clientid);
	  fprintf(stderr,"HEX: ZERN: %f, %f, %f, %f, %f\n",
		  hexcmd.hex.zernike_cmd[0],hexcmd.hex.zernike_cmd[1],hexcmd.hex.zernike_cmd[2],
		  hexcmd.hex.zernike_cmd[3],hexcmd.hex.zernike_cmd[4]);
	  fprintf(stderr,"HEX: AXES: %f, %f, %f, %f, %f, %f\n",
		 hexcmd.hex.axis_cmd[0],hexcmd.hex.axis_cmd[1],hexcmd.hex.axis_cmd[2],
		 hexcmd.hex.axis_cmd[3],hexcmd.hex.axis_cmd[4],hexcmd.hex.axis_cmd[5]);
	}
      }
    }
    
    /* Sleep */
    usleep(ONE_MILLION/HEX_CMD_PER_SEC);
  }
  
  hexctrlC(0);
  return;
}

