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
#include <phx_api.h>
#include <pbl_api.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "phx_config.h"

/* LYT board number */
#define LYT_BOARD_NUMBER PHX_BOARD_NUMBER_2

/* Process File Descriptor */
int lyt_shmfd;

/* Global Variables */
tHandle lytCamera = 0; /* Camera Handle   */
uint32 lyt_frame_count=0;

/* Prototypes */
void lyt_process_image(stImageBuff *buffer,sm_t *sm_p,uint32 frame_number);

/* CTRL-C Function */
void lytctrlC(int sig)
{
#if MSG_CTRLC
  printf("LYT: ctrlC! exiting.\n");
  printf("LYT: Got %d frames.\n",lyt_frame_count);
#endif
  close(lyt_shmfd);
  if ( lytCamera ) PHX_StreamRead( lytCamera, PHX_ABORT, NULL ); /* Now cease all captures */

  if ( lytCamera ) { /* Release the Phoenix board */
    PHX_Close( &lytCamera ); /* Close the Phoenix board */
    PHX_Destroy( &lytCamera ); /* Destroy the Phoenix handle */
  }


  exit(sig);
}

/* Define an application specific structure to hold user information */
typedef struct {
  sm_t *sm_p; /* Shared memory pointer */
} tContext;

/* Callback Function */
static void lyt_callback( tHandle lytCamera, ui32 dwInterruptMask, void *pvParams ) {
  if ( dwInterruptMask & PHX_INTRPT_BUFFER_READY ) {
    stImageBuff stBuffer;
    tContext *aContext = (tContext *)pvParams;

    etStat eStat = PHX_StreamRead( lytCamera, PHX_BUFFER_GET, &stBuffer );
    if ( PHX_OK == eStat ) {
      //Process image
      lyt_process_image(&stBuffer,aContext->sm_p,lyt_frame_count);
      //Check in with watchdog
      checkin(aContext->sm_p,LYTID);
      //Increment frame counter
      lyt_frame_count++;
    }
    PHX_StreamRead( lytCamera, PHX_BUFFER_RELEASE, NULL );
  }
}

/* Main Process */
int lyt_proc(void){
  char *configFileName = LYT_CONFIG_FILE;
  etStat eStat = PHX_OK;
  etParamValue eParamValue;
  bobcatParamValue bParamValue;
  int nLastEventCount = 0;
  tContext lytContext;
  ui64 dwParamValue;
  etParamValue roiWidth, roiHeight, bufferWidth, bufferHeight;
  int camera_running = 0;

  
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&lyt_shmfd)) == NULL){
    printf("openshm fail: lyt_proc\n");
    lytctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, lytctrlC);	/* usually ^C */

  /* Set up context for callback */
  memset( &lytContext, 0, sizeof( tContext ) );
  lytContext.sm_p = sm_p;

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
    
  /* STOP Capture to put camera in known state */
  eStat = PHX_StreamRead( lytCamera, PHX_STOP, (void*)lyt_callback );
  if ( PHX_OK != eStat ){
    printf("LYT: PHX_StreamRead --> PHX_STOP\n");
    lytctrlC(0);
  }
  camera_running = 0;

  /* -------------------- Enter Waiting Loop -------------------- */
  while(1){
    /* Check if camera should start/stop */
    if(!camera_running && sm_p->state_array[sm_p->state].lyt.run_camera){
      eStat = PHX_StreamRead( lytCamera, PHX_START, (void*)lyt_callback );
      if ( PHX_OK != eStat ){
	printf("LYT: PHX_StreamRead --> PHX_START\n");
	lytctrlC(0);
      }
      camera_running = 1;
    }
    if(camera_running && !sm_p->state_array[sm_p->state].lyt.run_camera){
      eStat = PHX_StreamRead( lytCamera, PHX_STOP, (void*)lyt_callback );
      if ( PHX_OK != eStat ){
	printf("LYT: PHX_StreamRead --> PHX_STOP\n");
	lytctrlC(0);
      }
      camera_running = 0;
    }
    
    /* Check if we've been asked to exit */
    if(sm_p->w[LYTID].die)
      lytctrlC(0);

    /* Check in with the watchdog */
    if(!camera_running)
      checkin(sm_p,LYTID);

    /* Sleep */
    sleep(sm_p->w[LYTID].per);
  }

  lytctrlC(0);
  return 0;
}
