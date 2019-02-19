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

/* SHK board number */
#define SHK_BOARD_NUMBER PHX_BOARD_NUMBER_1

/* Process File Descriptor */
int shk_shmfd;

/* Global Variables */
tHandle shkCamera = 0; /* Camera Handle   */
uint32 shk_frame_count=0;

/* Prototypes */
void shk_process_image(stImageBuff *buffer,sm_t *sm_p,uint32 frame_number);
float BOBCAT_GetTemp(tHandle hCamera);

/* CTRL-C Function */
void shkctrlC(int sig)
{
  close(shk_shmfd);
  if(shkCamera)
    PHX_StreamRead( shkCamera, PHX_ABORT, NULL ); /* Now cease all captures */
  
  if(shkCamera) {            /* Release the Phoenix board */
    PHX_Close(&shkCamera);   /* Close the Phoenix board */
    PHX_Destroy(&shkCamera); /* Destroy the Phoenix handle */
  }

#if MSG_CTRLC
  printf("SHK: exiting\n");
#endif

  exit(sig);
}

/* Define an application specific structure to hold user information */
typedef struct {
  sm_t *sm_p; /* Shared memory pointer */
} tContext;

/* Callback Function */
static void shk_callback( tHandle shkCamera, ui32 dwInterruptMask, void *pvParams ) {
  if ( dwInterruptMask & PHX_INTRPT_BUFFER_READY ) {
    stImageBuff stBuffer;
    tContext *aContext = (tContext *)pvParams;
    //Set DIO bit C1
    #if PICC_DIO_ENABLE
    outb(0x02,PICC_DIO_BASE+PICC_DIO_PORTC);
    #endif
    
    etStat eStat = PHX_StreamRead( shkCamera, PHX_BUFFER_GET, &stBuffer );
    if ( PHX_OK == eStat ) {
      //Process image
      shk_process_image(&stBuffer,aContext->sm_p,shk_frame_count);
      //Unset DIO bit C1
      #if PICC_DIO_ENABLE
      outb(0x00,PICC_DIO_BASE+PICC_DIO_PORTC);
      #endif
      //Check in with watchdog
      checkin(aContext->sm_p,SHKID);
      //Increment frame counter
      shk_frame_count++;
    }
    PHX_StreamRead( shkCamera, PHX_BUFFER_RELEASE, NULL );
  }
}

/* Main Process */
int shk_proc(void){
  char *configFileName = SHK_CONFIG_FILE;
  etStat eStat = PHX_OK;
  etParamValue eParamValue;
  bobcatParamValue bParamValue,expmin,expmax,expcmd,frmmin,frmcmd,lnmin;
  int nLastEventCount = 0;
  tContext shkContext;
  ui64 dwParamValue;
  etParamValue roiWidth, roiHeight, bufferWidth, bufferHeight;
  int camera_running = 0;
  alp_t alp;     //dummy to init alp_calibrate
  uint32_t step; //dummy to init alp_calibrate
  
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&shk_shmfd)) == NULL){
    printf("openshm fail: shk_proc\n");
    shkctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, shkctrlC);	/* usually ^C */

  /* Init alp_calibrate */
  alp_calibrate(ALP_CALMODE_NONE,&alp,&step,SHKID,FUNCTION_NO_RESET);

  /* Set up context for callback */
  memset( &shkContext, 0, sizeof( tContext ) );
  shkContext.sm_p = sm_p;

  /* Create a Phoenix handle */
  eStat = PHX_Create( &shkCamera, PHX_ErrHandlerDefault );
  if ( PHX_OK != eStat ){
    printf("SHK: Error PHX_Create\n");
    shkctrlC(0);
  }
  
  /* Set the board number */
  eParamValue = SHK_BOARD_NUMBER;
  eStat = PHX_ParameterSet( shkCamera, PHX_BOARD_NUMBER, &eParamValue );
  if ( PHX_OK != eStat ){
    printf("SHK: Error PHX_ParameterSet --> Board Number\n");
    shkctrlC(0);
  }

  /* Open the Phoenix board */
  eStat = PHX_Open( shkCamera );
  if ( PHX_OK != eStat ){
    printf("SHK: Error PHX_Open\n");
    shkctrlC(0);
  }

  /* Run the config file */
  eStat = CONFIG_RunFile( shkCamera, configFileName );
  if ( PHX_OK != eStat ){
    printf("SHK: Error CONFIG_RunFile\n");
    shkctrlC(0);
  }

  /* Setup our own event context */
  eStat = PHX_ParameterSet( shkCamera, PHX_EVENT_CONTEXT, (void *) &shkContext );
  if ( PHX_OK != eStat ){
    printf("SHK: Error PHX_ParameterSet --> PHX_EVENT_CONTEXT\n");
    shkctrlC(0);
  }

  /* Get debugging info */
  if(SHK_DEBUG){
    eStat = PHX_ParameterGet( shkCamera, PHX_ROI_XLENGTH, &roiWidth );
    eStat = PHX_ParameterGet( shkCamera, PHX_ROI_YLENGTH, &roiHeight );
    printf("SHK: roi                     : [%d x %d]\n", roiWidth, roiHeight);
    eStat = PHX_ParameterGet( shkCamera, PHX_BUF_DST_XLENGTH, &bufferWidth );
    eStat = PHX_ParameterGet( shkCamera, PHX_BUF_DST_YLENGTH, &bufferHeight );
    printf("SHK: destination buffer size : [%d x %d]\n", bufferWidth, bufferHeight);
    eStat = BOBCAT_ParameterGet( shkCamera, BOBCAT_INFO_MIN_MAX_XLENGTHS, &bParamValue );
    printf("SHK: Camera x size (width)      : [%d to %d]\n", (bParamValue&0x0000FFFF), (bParamValue&0xFFFF0000)>>16);
    eStat = BOBCAT_ParameterGet( shkCamera, BOBCAT_INFO_MIN_MAX_YLENGTHS, &bParamValue );
    printf("SHK: Camera y size (height)     : [%d to %d]\n", (bParamValue&0x0000FFFF), (bParamValue&0xFFFF0000)>>16 );
    eStat = BOBCAT_ParameterGet( shkCamera, BOBCAT_INFO_XYLENGTHS, &bParamValue );
    printf("SHK: Camera current size        : [%d x %d]\n", (bParamValue&0x0000FFFF), (bParamValue&0xFFFF0000)>>16 );
  }
    

  /* ----------------------- Enter Main Loop ----------------------- */
  while(1){
    
    /* STOP Capture to put camera in known state */
    eStat = PHX_StreamRead( shkCamera, PHX_STOP, (void*)shk_callback );
    if ( PHX_OK != eStat ){
      printf("SHK: PHX_StreamRead --> PHX_STOP\n");
      shkctrlC(0);
    }
    camera_running = 0;
    printf("SHK: Camera stopped\n");
    
    /* Setup exposure */
    usleep(500000);
    //Get minimum line time
    eStat = BOBCAT_ParameterGet( shkCamera, BOBCAT_INFO_MIN_LN_TIME, &lnmin );
    lnmin = ((lnmin & 0xFFFF0000) >> 16) & 0x0000FFFF;
    //Get minimum frame time and check against command
    eStat = BOBCAT_ParameterGet( shkCamera, BOBCAT_INFO_MIN_FRM_TIME, &frmmin );
    frmmin &= 0x00FFFFFF;
    printf("SHK: Min line = %d | frame = %d\n",lnmin,frmmin);
    frmcmd = lround(sm_p->shk_frmtime*ONE_MILLION);
    frmcmd = frmcmd < frmmin ? frmmin : frmcmd;
    //Set the frame time
    eStat = BOBCAT_ParameterSet( shkCamera, BOBCAT_FRM_TIME, &frmcmd );
    if ( PHX_OK != eStat ){
      printf("SHK: BOBCAT_ParameterSet --> BOBCAT_FRM_TIME %d\n",frmcmd);
      shkctrlC(0);
    }
    //Get minimum and maximum exposure time and check against command
    usleep(500000);
    eStat = BOBCAT_ParameterGet( shkCamera, BOBCAT_INFO_MIN_EXP_TIME, &expmin );
    eStat = BOBCAT_ParameterGet( shkCamera, BOBCAT_INFO_MAX_EXP_TIME, &expmax );
    expmin &= 0x00FFFFFF;
    expmax &= 0x00FFFFFF;
    expcmd = lround(sm_p->shk_exptime*ONE_MILLION);
    expcmd = expcmd > expmax ? expmax : expcmd;
    expcmd = expcmd < expmin ? expmin : expcmd;
    eStat = BOBCAT_ParameterSet( shkCamera, BOBCAT_EXP_TIME, &expcmd );
    if ( PHX_OK != eStat ){
      printf("SHK: BOBCAT_ParameterSet --> BOBCAT_EXP_TIME %d\n",expcmd);
      shkctrlC(0);
    }
    //Get set exposure and frame times
    eStat = BOBCAT_ParameterGet( shkCamera, BOBCAT_INFO_EXP_TIME, &expcmd );
    eStat = BOBCAT_ParameterGet( shkCamera, BOBCAT_INFO_FRM_TIME, &frmcmd );
    expcmd &= 0x00FFFFFF;
    frmcmd &= 0x00FFFFFF;
    sm_p->shk_exptime = (double)expcmd / ONE_MILLION;
    sm_p->shk_frmtime = (double)frmcmd / ONE_MILLION;
    printf("SHK: Set frm = %d | exp = %d\n",frmcmd,expcmd);
    
    /* ----------------------- Enter Exposure Loop ----------------------- */
    while(1){
      /* Check if we've been asked to exit */
      if(sm_p->w[SHKID].die)
	shkctrlC(0);
      
      /* Check if we've been asked to reset the exposure */
      if(sm_p->shk_reset_camera){
	sm_p->shk_reset_camera = 0;
	break;
      }
      
      /* Check if camera should start/stop */
      if(!camera_running && sm_p->state_array[sm_p->state].shk.run_camera){
	eStat = PHX_StreamRead( shkCamera, PHX_START, (void*)shk_callback );
	if ( PHX_OK != eStat ){
	  printf("SHK: PHX_StreamRead --> PHX_START\n");
	  shkctrlC(0);
	}
	camera_running = 1;
	printf("SHK: Camera started\n");
      }
      if(camera_running && !sm_p->state_array[sm_p->state].shk.run_camera){
	eStat = PHX_StreamRead( shkCamera, PHX_STOP, (void*)shk_callback );
	if ( PHX_OK != eStat ){
	  printf("SHK: PHX_StreamRead --> PHX_STOP\n");
	  shkctrlC(0);
	}
	camera_running = 0;
	printf("SHK: Camera stopped\n");
      }
    
      /* Check in with the watchdog */
      if(!camera_running)
	checkin(sm_p,SHKID);

      /* Sleep */
      sleep(sm_p->w[SHKID].per);
    }
  }

  shkctrlC(0);
  return 0;
}
