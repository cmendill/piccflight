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

/* SHK config */
#define SHK_BOARD_NUMBER PHX_BOARD_NUMBER_1

/* Global Variables */
int shk_run=1;

/* Prototypes */
void shk_process_image(stImageBuff *buffer,sm_t *sm_p,uint32 frame_number);

/* CTRL-C Function */
void shkctrlC(int sig)
{
#if MSG_CTRLC
  printf("SHK: ctrlC! exiting.\n");
#endif
  //Trigger cleanup
  shk_run=0;
}

/* Define callback structure */
typedef struct {
  sm_t *sm_p;           
  uint64_t frame_count;
} tContext;

/* Callback Function */
static void shk_callback(tHandle shkCamera, ui32 dwInterruptMask, void *pvParams) {
  if (dwInterruptMask & PHX_INTRPT_BUFFER_READY) {
    stImageBuff stBuffer;
    tContext *aContext = (tContext *)pvParams;
    
    etStat eStat = PHX_StreamRead( shkCamera, PHX_BUFFER_GET, &stBuffer );
    if(PHX_OK == eStat) {
      //Process image
      shk_process_image(&stBuffer,aContext->sm_p,aContext->frame_count);
      //Check in with watchdog
      checkin(aContext->sm_p,SHKID);
      //Increment frame counter
      aContext->frame_count++;
    }
    PHX_StreamRead(shkCamera, PHX_BUFFER_RELEASE, NULL);
  }
}

/* Cleanup Function */
static void shk_cleanup(tHandle *camera_p,int shmfd){
  //Close shared memory
  close(shmfd);
  //Close camera
  if(*camera_p){
    PHX_StreamRead(*camera_p, PHX_ABORT, NULL );
    PHX_Close(camera_p);
    PHX_Destroy(camera_p);
  }
  //Exit
  exit(0);
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
  int camera_running = 0;
  tHandle shkCamera = 0;
  int shmfd;
  
  
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&shmfd)) == NULL){
    printf("openshm fail: shk_proc\n");
    shk_cleanup(&shkCamera,shmfd);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, shkctrlC);	/* usually ^C */

  /* Set up context for callback */
  memset(&shkContext, 0, sizeof(tContext));
  shkContext.sm_p = sm_p;

  /* Create a Phoenix handle */
  eStat = PHX_Create(&shkCamera, PHX_ErrHandlerDefault);
  if (PHX_OK != eStat){
    printf("SHK: Error PHX_Create\n");
    shk_cleanup(&shkCamera,shmfd);
  }
  
  /* Set the board number */
  eParamValue = SHK_BOARD_NUMBER;
  eStat = PHX_ParameterSet(shkCamera, PHX_BOARD_NUMBER, &eParamValue);
  if (PHX_OK != eStat){
    printf("SHK: Error PHX_ParameterSet --> Board Number\n");
    shk_cleanup(&shkCamera,shmfd);
  }

  /* Open the Phoenix board */
  eStat = PHX_Open(shkCamera);
  if (PHX_OK != eStat){
    printf("SHK: Error PHX_Open\n");
    shk_cleanup(&shkCamera,shmfd);
  }

  /* Run the config file */
  eStat = CONFIG_RunFile(shkCamera, &configFileName);
  if (PHX_OK != eStat){
    printf("SHK: Error CONFIG_RunFile\n");
    shk_cleanup(&shkCamera,shmfd);
  }

  /* Setup our own event context */
  eStat = PHX_ParameterSet(shkCamera, PHX_EVENT_CONTEXT, (void *) &shkContext);
  if (PHX_OK != eStat){
    printf("SHK: Error PHX_ParameterSet --> PHX_EVENT_CONTEXT\n");
    shk_cleanup(&shkCamera,shmfd);
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
    
  /* STOP Capture to put camera in known state */
  eStat = PHX_StreamRead( shkCamera, PHX_STOP, (void*)shk_callback );
  if ( PHX_OK != eStat ){
    printf("SHK: PHX_StreamRead --> PHX_STOP\n");
    shk_cleanup(&shkCamera,shmfd);
  }
  camera_running = 0;

  /* -------------------- Enter Waiting Loop -------------------- */
  while(shk_run){
    /* Check if camera should start/stop */
    if(!camera_running && sm_p->state_array[sm_p->state].shk.run_camera){
      eStat = PHX_StreamRead( shkCamera, PHX_START, (void*)shk_callback );
      if ( PHX_OK != eStat ){
	printf("SHK: PHX_StreamRead --> PHX_START\n");
	break;
      }
      camera_running = 1;
    }
    if(camera_running && !sm_p->state_array[sm_p->state].shk.run_camera){
      eStat = PHX_StreamRead( shkCamera, PHX_STOP, (void*)shk_callback );
      if ( PHX_OK != eStat ){
	printf("SHK: PHX_StreamRead --> PHX_STOP\n");
	break;
      }
      camera_running = 0;
    }
    
    /* Check if we've been asked to exit */
    if(sm_p->w[SHKID].die)
      break;
    
    /* Check in with the watchdog */
    if(!camera_running)
      checkin(sm_p,SHKID);

    /* Sleep */
    sleep(sm_p->w[SHKID].per);
  }
  
  //Cleanup
  shk_cleanup(&shkCamera,shmfd);

  return 0;
}
