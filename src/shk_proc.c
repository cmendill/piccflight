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

/* Process File Descriptor */
int shk_shmfd;

/* Global Variables */
tHandle shkCamera = 0; /* Camera Handle   */
uint32 shk_frame_count=0;

/* Prototypes */
void shk_process_image(stImageBuff *buffer,sm_t *sm_p,uint32 frame_number);

/* CTRL-C Function */
void shkctrlC(int sig)
{
#if MSG_CTRLC
  printf("SHK: ctrlC! exiting.\n");
  printf("SHK: Got %d frames.\n",shk_frame_count);
#endif
  close(shk_shmfd);
  if ( shkCamera ) PHX_StreamRead( shkCamera, PHX_ABORT, NULL ); /* Now cease all captures */

  if ( shkCamera ) { /* Release the Phoenix board */
    PHX_Close( &shkCamera ); /* Close the Phoenix board */
    PHX_Destroy( &shkCamera ); /* Destroy the Phoenix handle */
  }


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

    etStat eStat = PHX_StreamRead( shkCamera, PHX_BUFFER_GET, &stBuffer );
    if ( PHX_OK == eStat ) {
      //Process image
      shk_process_image(&stBuffer,aContext->sm_p,shk_frame_count);
      //Check in with watchdog

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
  bobcatParamValue bParamValue;
  int nLastEventCount = 0;
  tContext shkContext;
  ui64 dwParamValue;
  etParamValue roiWidth, roiHeight, bufferWidth, bufferHeight;

  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&shk_shmfd)) == NULL){
    printf("openshm fail: shk_proc\n");
    shkctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, shkctrlC);	/* usually ^C */

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
  eParamValue = PHX_BOARD_NUMBER_1;
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
  eStat = CONFIG_RunFile( shkCamera, &configFileName );
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




  /* -------------------- Now start our capture -------------------- */
  eStat = PHX_StreamRead( shkCamera, PHX_START, (void*)shk_callback );
  if ( PHX_OK != eStat ){
    printf("SHK: PHX_StreamRead --> PHX_START\n");
    shkctrlC(0);
  }


  /* -------------------- Enter Waiting Loop -------------------- */
  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[SHKID].die)
      shkctrlC(0);

    /* Check in with the watchdog */
    checkin(sm_p,SHKID);

    /* Sleep */
    sleep(sm_p->w[SHKID].per);

  }

  shkctrlC(0);
  return 0;
}
