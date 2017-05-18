#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>


/* piccflight headers */
#include "../common/controller.h"
#include "../common/common_functions.h"
#include "../phx/include/phx_api.h"
#include "../phx/include/pbl_api.h"
#include "../phx/config.h"
#include "shk_proc.h"

/* Process File Descriptor */
int shk_shmfd;


/* Global Variables */
tHandle shkCamera = 0; /* Camera Handle   */
tPHX shkBuffer1, shkBuffer2;

/* CTRL-C Function */
void shkctrlC(int sig)
{
#if MSG_CTRLC
  printf("SHK: ctrlC! exiting.\n");
#endif
  close(shk_shmfd);
  if ( shkCamera ) PHX_StreamRead( shkCamera, PHX_ABORT, NULL ); /* Now cease all captures */
  
  if (shkBuffer1) PBL_BufferDestroy( &shkBuffer1 );
  if (shkBuffer2) PBL_BufferDestroy( &shkBuffer1 );

  if ( shkCamera ) { /* Release the Phoenix board */
    PHX_Close( &shkCamera ); /* Close the Phoenix board */
    PHX_Destroy( &shkCamera ); /* Destroy the Phoenix handle */
  }
  exit(sig);
}

typedef struct { /* Define an application specific structure to hold user information */
  volatile int nCurrentEventCount; /* Event counter   */
  volatile double coefficients[15]; /* Event counter   */
} tContext;

/* Callback Function */
static void phxlyotwfs_callback( tHandle shkCamera, ui32 dwInterruptMask, void *pvParams ) {
  if ( dwInterruptMask & PHX_INTRPT_BUFFER_READY ) {
    stImageBuff stBuffer;
    etStat eStat = PHX_StreamRead( shkCamera, PHX_BUFFER_GET, &stBuffer );
    if ( PHX_OK == eStat ) {
      //Do something
    }
    PHX_StreamRead( shkCamera, PHX_BUFFER_RELEASE, NULL );
  }
}

/* Main Process */
int shk_proc(void){
  etParamValue eBoardNumber = SHK_BOARD_NUMBER;
  char *configFileName = SHK_CONFIG_FILE;
  etStat eStat = PHX_OK; /* Status variable */
  etParamValue eParamValue; /* Parameter for use with PHX_ParameterSet/Get calls */
  int nLastEventCount = 0;
  tContext aContext;
  ui32 dwAcqNumImages = 0;
  ui32 dwAcqBufferStart = 0;
  ui32 dwBufferWidth = 0;
  ui32 dwBufferHeight = 0;
  ui32 dwBufferStride = 0;
  ui32 dwLoop;

  stImageBuff pasImageBuffs[3];
  ui32 dwParamValue;

  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&shk_shmfd)) == NULL){
    printf("openshm fail: shk_proc\n");
    shkctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, shkctrlC);	/* usually ^C */

  memset( &aContext, 0, sizeof( tContext ) ); /* Initialise the user defined Event context structure */

  aContext.nCurrentEventCount = 0;

  eStat = PHX_Create( &shkCamera, PHX_ErrHandlerDefault ); /* Create a Phoenix handle */
  if ( PHX_OK != eStat ) shkctrlC(0);

  eStat = PHX_ParameterSet( shkCamera, PHX_BOARD_NUMBER, &eBoardNumber ); /* Set the board number */
  if ( PHX_OK != eStat ) shkctrlC(0);

  eStat = PHX_Open( shkCamera ); /* Open the Phoenix board using the above configuration file */
  if ( PHX_OK != eStat ) shkctrlC(0);

  eStat = CONFIG_RunFile( shkCamera, &configFileName );  /* set the settings from the compound (BOBCAT and PHX) configuration file */
  if ( eStat != PHX_OK ) shkctrlC(0);

  PBL_BufferCreate( &shkBuffer1, PBL_BUFF_SYSTEM_MEM_DIRECT, 0, shkCamera, PHX_ErrHandlerDefault );
  PBL_BufferCreate( &shkBuffer2, PBL_BUFF_SYSTEM_MEM_DIRECT, 0, shkCamera, PHX_ErrHandlerDefault );

  eStat = PHX_ParameterGet( shkCamera, PHX_BUF_DST_XLENGTH, &dwBufferWidth );
  if ( PHX_OK != eStat ) shkctrlC(0);

  eStat = PHX_ParameterGet( shkCamera, PHX_BUF_DST_YLENGTH, &dwBufferHeight );
  if ( PHX_OK != eStat ) shkctrlC(0);

  printf("PHX_BUF_DST_XLENGTH : %d\nPHX_BUF_DST_yLENGTH : %d\n", dwBufferWidth, dwBufferHeight);

  dwBufferStride = dwBufferWidth; /* Convert width (in pixels) to stride (in bytes).*/
  
  PBL_BufferParameterSet( shkBuffer1, PBL_BUFF_HEIGHT, &dwBufferHeight ); /* If we assume Y8 capture format Set the height and stride required. */
  PBL_BufferParameterSet( shkBuffer1, PBL_BUFF_STRIDE, &dwBufferStride );

  PBL_BufferParameterSet( shkBuffer2, PBL_BUFF_HEIGHT, &dwBufferHeight );
  PBL_BufferParameterSet( shkBuffer2, PBL_BUFF_STRIDE, &dwBufferStride );

  PBL_BufferInit( shkBuffer1 ); /* Initialise each buffer. This creates the buffers in system memory. */
  PBL_BufferInit( shkBuffer2 );

  PBL_BufferParameterGet( shkBuffer1, PBL_BUFF_ADDRESS, &dwParamValue );   /* Build our array of image buffers. PBL_BUFF_ADDRESS returns the address of the first pixel of image data. */
  pasImageBuffs[ 0 ].pvAddress = (void*)&dwParamValue;
  pasImageBuffs[ 0 ].pvContext = (void*)shkBuffer1;

  PBL_BufferParameterGet( shkBuffer2, PBL_BUFF_ADDRESS, &dwParamValue );
  pasImageBuffs[ 1 ].pvAddress = (void*)&dwParamValue;
  pasImageBuffs[ 1 ].pvContext = (void*)shkBuffer2;

  pasImageBuffs[ 2 ].pvAddress = NULL;
  pasImageBuffs[ 2 ].pvContext = NULL;

  dwAcqNumImages = 2;
  eStat = PHX_ParameterSet( shkCamera, PHX_ACQ_NUM_IMAGES, &dwAcqNumImages );
  if ( PHX_OK != eStat ) shkctrlC(0);

  dwAcqBufferStart = 1;
  eStat = PHX_ParameterSet( shkCamera, PHX_ACQ_BUFFER_START, &dwAcqBufferStart );
  if ( PHX_OK != eStat ) shkctrlC(0);


  eStat = PHX_ParameterSet( shkCamera, PHX_DST_PTRS_VIRT, (void *) pasImageBuffs ); /* Instruct Phoenix to use the user supplied Virtual Buffers */
  if ( PHX_OK != eStat ) shkctrlC(0);

  eParamValue = PHX_DST_PTR_USER_VIRT;
  eStat = PHX_ParameterSet( shkCamera, (etParam)( PHX_DST_PTR_TYPE | PHX_CACHE_FLUSH | PHX_FORCE_REWRITE ), (void *) &eParamValue );
  if ( PHX_OK != eStat ) shkctrlC(0);

  eStat = PHX_ParameterSet( shkCamera, PHX_EVENT_CONTEXT, (void *) &aContext ); /* Setup our own event context */
  if ( PHX_OK != eStat ) shkctrlC(0);

  printf("capturing for xx s\n");

  /* -------------------- Now start our capture -------------------- */
  eStat = PHX_StreamRead( shkCamera, PHX_START, (void*)phxlyotwfs_callback );
  if ( PHX_OK != eStat ) shkctrlC(0);

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
