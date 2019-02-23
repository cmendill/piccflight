#define _XOPEN_SOURCE 500
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <phx_api.h>
#include <pbl_api.h>
#include <sys/io.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "phx_config.h"
#include "../drivers/phxdrv/picc_dio.h"
#include "alp_functions.h"

/* LYT board number */
#define LYT_BOARD_NUMBER PHX_BOARD_NUMBER_2

/* Process File Descriptor */
int lyt_shmfd;

/* Global Variables */
tHandle lytCamera = 0; /* Camera Handle   */

/* Prototypes */
void lyt_process_image(stImageBuff *buffer,sm_t *sm_p);
float BOBCAT_GetTemp(tHandle hCamera);

/**************************************************************/
/* LYTCTRLC                                                   */
/*  - SIGINT function                                         */
/**************************************************************/
void lytctrlC(int sig)
{
  close(lyt_shmfd);
  if(lytCamera)
    PHX_StreamRead( lytCamera, PHX_ABORT, NULL ); /* Now cease all captures */
  
  if(lytCamera) {            /* Release the Phoenix board */
    PHX_Close(&lytCamera);   /* Close the Phoenix board */
    PHX_Destroy(&lytCamera); /* Destroy the Phoenix handle */
  }

#if MSG_CTRLC
  printf("LYT: exiting\n");
#endif

  exit(sig);
}

/* Define an application specific structure to hold user information */
typedef struct {
  sm_t *sm_p; /* Shared memory pointer */
} tContext;

/**************************************************************/
/* LYT_CALLBACK                                               */
/*  - Exposure ISR callback function                          */
/**************************************************************/
static void lyt_callback( tHandle lytCamera, ui32 dwInterruptMask, void *pvParams ) {
  if ( dwInterruptMask & PHX_INTRPT_BUFFER_READY ) {
    stImageBuff stBuffer;
    tContext *aContext = (tContext *)pvParams;
    //Set DIO bit B1
    #if PICC_DIO_ENABLE
    outb(0x02,PICC_DIO_BASE+PICC_DIO_PORTB);
    #endif
    
    etStat eStat = PHX_StreamRead( lytCamera, PHX_BUFFER_GET, &stBuffer );
    if ( PHX_OK == eStat ) {
      //Process image
      lyt_process_image(&stBuffer,aContext->sm_p);
      //Unset DIO bit B1
      #if PICC_DIO_ENABLE
      outb(0x00,PICC_DIO_BASE+PICC_DIO_PORTB);
      #endif
      //Check in with watchdog
      checkin(aContext->sm_p,LYTID);
    }
    PHX_StreamRead( lytCamera, PHX_BUFFER_RELEASE, NULL );
  }
}

/**************************************************************/
/* LYT_PROC                                                   */
/*  - Main LYT camera process                                 */
/**************************************************************/
int lyt_proc(void){
  char *configFileName = LYT_CONFIG_FILE;
  etStat eStat = PHX_OK;
  etParamValue eParamValue;
  bobcatParamValue bParamValue,expmin,expmax,expcmd,frmmin,frmcmd,lnmin;
  int nLastEventCount = 0;
  tContext lytContext;
  ui64 dwParamValue;
  etParamValue roiWidth, roiHeight, bufferWidth, bufferHeight;
  int camera_running = 0;
  alp_t alp;     //dummy to init alp_calibrate
  uint32_t step; //dummy to init alp_calibrate
  
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&lyt_shmfd)) == NULL){
    printf("openshm fail: lyt_proc\n");
    lytctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, lytctrlC);	/* usually ^C */

  /* Init alp_calibrate */
  alp_calibrate(ALP_CALMODE_NONE,&alp,&step,LYTID,FUNCTION_NO_RESET);

  /* Set up context for callback */
  memset( &lytContext, 0, sizeof( tContext ) );
  lytContext.sm_p = sm_p;

  /* Mark top of the device code */
  device_top:
  while(1){
    /* Check if the camera should run in this state */
    if(sm_p->state_array[sm_p->state].lyt.run_camera){

      /* Create a Phoenix handle */
      eStat = PHX_Create( &lytCamera, PHX_ErrHandlerDefault );
      if ( PHX_OK != eStat ){
	printf("LYT: Error PHX_Create\n");
	lytctrlC(0);
      }
  
      /* Set the board number */
      eParamValue = LYT_BOARD_NUMBER;
      eStat = PHX_ParameterSet( lytCamera, PHX_BOARD_NUMBER, &eParamValue );
      if ( PHX_OK != eStat ){
	printf("LYT: Error PHX_ParameterSet --> Board Number\n");
	lytctrlC(0);
      }

      /* Open the Phoenix board */
      eStat = PHX_Open( lytCamera );
      if ( PHX_OK != eStat ){
	printf("LYT: Error PHX_Open\n");
	lytctrlC(0);
      }

      /* Run the config file */
      eStat = CONFIG_RunFile( lytCamera, configFileName );
      if ( PHX_OK != eStat ){
	printf("LYT: Error CONFIG_RunFile\n");
	lytctrlC(0);
      }

      /* Setup our own event context */
      eStat = PHX_ParameterSet( lytCamera, PHX_EVENT_CONTEXT, (void *) &lytContext );
      if ( PHX_OK != eStat ){
	printf("LYT: Error PHX_ParameterSet --> PHX_EVENT_CONTEXT\n");
	lytctrlC(0);
      }

      /* Get debugging info */
      if(LYT_DEBUG){
	eStat = PHX_ParameterGet( lytCamera, PHX_ROI_XLENGTH, &roiWidth );
	eStat = PHX_ParameterGet( lytCamera, PHX_ROI_YLENGTH, &roiHeight );
	printf("LYT: roi                     : [%d x %d]\n", roiWidth, roiHeight);
	eStat = PHX_ParameterGet( lytCamera, PHX_BUF_DST_XLENGTH, &bufferWidth );
	eStat = PHX_ParameterGet( lytCamera, PHX_BUF_DST_YLENGTH, &bufferHeight );
	printf("LYT: destination buffer size : [%d x %d]\n", bufferWidth, bufferHeight);
	eStat = BOBCAT_ParameterGet( lytCamera, BOBCAT_INFO_MIN_MAX_XLENGTHS, &bParamValue );
	printf("LYT: Camera x size (width)      : [%d to %d]\n", (bParamValue&0x0000FFFF), (bParamValue&0xFFFF0000)>>16);
	eStat = BOBCAT_ParameterGet( lytCamera, BOBCAT_INFO_MIN_MAX_YLENGTHS, &bParamValue );
	printf("LYT: Camera y size (height)     : [%d to %d]\n", (bParamValue&0x0000FFFF), (bParamValue&0xFFFF0000)>>16 );
	eStat = BOBCAT_ParameterGet( lytCamera, BOBCAT_INFO_XYLENGTHS, &bParamValue );
	printf("LYT: Camera current size        : [%d x %d]\n", (bParamValue&0x0000FFFF), (bParamValue&0xFFFF0000)>>16 );
      }
    

      /* ----------------------- Enter Main Loop ----------------------- */
      while(1){
	/* STOP Capture to put camera in known state */
	eStat = PHX_StreamRead( lytCamera, PHX_STOP, (void*)lyt_callback );
	if ( PHX_OK != eStat ){
	  printf("LYT: PHX_StreamRead --> PHX_STOP\n");
	  lytctrlC(0);
	}
	camera_running = 0;
	printf("LYT: Camera stopped\n");
    
	/* Setup exposure */
	usleep(500000);
	//Get minimum line time
	eStat = BOBCAT_ParameterGet( lytCamera, BOBCAT_INFO_MIN_LN_TIME, &lnmin );
	lnmin = ((lnmin & 0xFFFF0000) >> 16) & 0x0000FFFF;
	//Get minimum frame time and check against command
	eStat = BOBCAT_ParameterGet( lytCamera, BOBCAT_INFO_MIN_FRM_TIME, &frmmin );
	frmmin &= 0x00FFFFFF;
	printf("LYT: Min ln = %d | frm = %d\n",lnmin,frmmin);
	frmcmd = lround(sm_p->lyt_frmtime*ONE_MILLION);
	frmcmd = frmcmd < frmmin ? frmmin : frmcmd;
	//Set the frame time
	eStat = BOBCAT_ParameterSet( lytCamera, BOBCAT_FRM_TIME, &frmcmd );
	if ( PHX_OK != eStat ){
	  printf("LYT: BOBCAT_ParameterSet --> BOBCAT_FRM_TIME %d\n",frmcmd);
	  lytctrlC(0);
	}
	//Get minimum and maximum exposure time and check against command
	usleep(500000);
	eStat = BOBCAT_ParameterGet( lytCamera, BOBCAT_INFO_MIN_EXP_TIME, &expmin );
	eStat = BOBCAT_ParameterGet( lytCamera, BOBCAT_INFO_MAX_EXP_TIME, &expmax );
	expmin &= 0x00FFFFFF;
	expmax &= 0x00FFFFFF;
	expcmd = lround(sm_p->lyt_exptime*ONE_MILLION);
	expcmd = expcmd > expmax ? expmax : expcmd;
	expcmd = expcmd < expmin ? expmin : expcmd;
	eStat = BOBCAT_ParameterSet( lytCamera, BOBCAT_EXP_TIME, &expcmd );
	if ( PHX_OK != eStat ){
	  printf("LYT: BOBCAT_ParameterSet --> BOBCAT_EXP_TIME %d\n",expcmd);
	  lytctrlC(0);
	}
	//Get set exposure and frame times
	eStat = BOBCAT_ParameterGet( lytCamera, BOBCAT_INFO_EXP_TIME, &expcmd );
	eStat = BOBCAT_ParameterGet( lytCamera, BOBCAT_INFO_FRM_TIME, &frmcmd );
	expcmd &= 0x00FFFFFF;
	frmcmd &= 0x00FFFFFF;
	sm_p->lyt_exptime = (double)expcmd / ONE_MILLION;
	sm_p->lyt_frmtime = (double)frmcmd / ONE_MILLION;
	printf("LYT: Set frm = %d | exp = %d\n",frmcmd,expcmd);
    
	/* ----------------------- Enter Exposure Loop ----------------------- */
	while(1){
	  /* Check if we've been asked to exit */
	  if(sm_p->w[LYTID].die)
	    lytctrlC(0);
      
	  /* Check if we've been asked to reset the exposure */
	  if(sm_p->lyt_reset_camera){
	    sm_p->lyt_reset_camera = 0;
	    break;
	  }
      
	  /* Check if the camera has been disabled in this state */
	  if(!sm_p->state_array[sm_p->state].lyt.run_camera){
	    eStat = PHX_StreamRead( lytCamera, PHX_STOP, (void*)lyt_callback );
	    if ( PHX_OK != eStat ){
	      printf("LYT: PHX_StreamRead --> PHX_STOP\n");
	      lytctrlC(0);
	    }
	    camera_running = 0;
	    printf("LYT: Camera stopped\n");
	    /* Go back to the top */
	    goto device_top;
	  }
	
	  /* Check if camera should start */
	  if(!camera_running && sm_p->state_array[sm_p->state].lyt.run_camera){
	    eStat = PHX_StreamRead( lytCamera, PHX_START, (void*)lyt_callback );
	    if ( PHX_OK != eStat ){
	      printf("LYT: PHX_StreamRead --> PHX_START\n");
	      lytctrlC(0);
	    }
	    camera_running = 1;
	    printf("LYT: Camera started\n");
	  }
      
	  /* Get CCD temperature */
	  sm_p->lyt_ccd_temp = BOBCAT_GetTemp(lytCamera);
	
	  /* Sleep */
	  sleep(sm_p->w[LYTID].per);
	}
      }
    }else{
      /* Camera is disabled in this state */
    
      /* Check if we've been asked to exit */
      if(sm_p->w[LYTID].die)
	lytctrlC(0);
      
      /* Check in with the watchdog */
      checkin(sm_p,LYTID);
    
      /* Sleep */
      sleep(sm_p->w[LYTID].per);
    }
  }
  
  /* Exit */
  lytctrlC(0);
  return 0;
}
