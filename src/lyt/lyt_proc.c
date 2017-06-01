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
#include "../phx/include/phx_api.h"
#include "../phx/include/pbl_api.h"
#include "../phx/config.h"

/* LYT config */
#define LYT_BOARD_NUMBER PHX_BOARD_NUMBER_1
#define LYT_CONFIG_FILE "phx_config/lyt.cfg"

/* Process File Descriptor */
int lyt_shmfd;


/* Global Variables */
tHandle lytCamera = 0; /* Camera Handle   */
tPHX lytBuffer1, lytBuffer2;
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
  
  if (lytBuffer1) PBL_BufferDestroy( &lytBuffer1 );
  if (lytBuffer2) PBL_BufferDestroy( &lytBuffer1 );

  if ( lytCamera ) { /* Release the Phoenix board */
    PHX_Close( &lytCamera ); /* Close the Phoenix board */
    PHX_Destroy( &lytCamera ); /* Destroy the Phoenix handle */
  }
  exit(sig);
}

typedef struct { /* Define an application specific structure to hold user information */
  volatile ui32 nCurrentEventCount; /* Event counter   */
  volatile double coefficients[15]; /* Event counter   */
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
      //lyt_process_image(&stBuffer,aContext->sm_p,lyt_frame_count);

      //Check in with watchdog
      
      //Increment frame counter
      lyt_frame_count++;
    }
    PHX_StreamRead( lytCamera, PHX_BUFFER_RELEASE, NULL );
  }
}

/* Main Process */
int lyt_proc(void){
  etParamValue eBoardNumber = LYT_BOARD_NUMBER;
  char *configFileName = LYT_CONFIG_FILE;
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
  ui64 dwParamValue;

  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&lyt_shmfd)) == NULL){
    printf("openshm fail: lyt_proc\n");
    lytctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, lytctrlC);	/* usually ^C */

  /* Set up context for callback */
  memset( &aContext, 0, sizeof( tContext ) ); /* Initialise the user defined Event context structure */
  aContext.nCurrentEventCount = 0;
  aContext.sm_p = sm_p;
  
  eStat = PHX_Create( &lytCamera, PHX_ErrHandlerDefault ); /* Create a Phoenix handle */
  if ( PHX_OK != eStat ){
    printf("LYT: Error PHX_Create\n");
    lytctrlC(0);
  }

  eStat = PHX_ParameterSet( lytCamera, PHX_BOARD_NUMBER, &eBoardNumber ); /* Set the board number */
  if ( PHX_OK != eStat ){
    printf("LYT: Error PHX_ParameterSet\n");
    lytctrlC(0);
  }

  eStat = PHX_Open( lytCamera ); /* Open the Phoenix board using the above configuration file */
  if ( PHX_OK != eStat ){
    printf("LYT: Error PHX_Open\n");
    lytctrlC(0);
  }
  
  eStat = CONFIG_RunFile( lytCamera, &configFileName );  /* set the settings from the compound (BOBCAT and PHX) configuration file */
  if ( PHX_OK != eStat ){
    printf("LYT: Error CONFIG_RunFile\n");
    lytctrlC(0);
  }

  PBL_BufferCreate( &lytBuffer1, PBL_BUFF_SYSTEM_MEM_DIRECT, 0, lytCamera, PHX_ErrHandlerDefault );
  PBL_BufferCreate( &lytBuffer2, PBL_BUFF_SYSTEM_MEM_DIRECT, 0, lytCamera, PHX_ErrHandlerDefault );

  eStat = PHX_ParameterGet( lytCamera, PHX_BUF_DST_XLENGTH, &dwBufferWidth );
  if ( PHX_OK != eStat ){
    printf("LYT: Error PHX_ParameterGet --> PHX_BUF_DST_XLENGTH\n");
    lytctrlC(0);
  }
  

  eStat = PHX_ParameterGet( lytCamera, PHX_BUF_DST_YLENGTH, &dwBufferHeight );
  if ( PHX_OK != eStat ){
    printf("LYT: Error PHX_ParameterGet --> PHX_BUF_DST_YLENGTH\n");
    lytctrlC(0);
  }
  

  if(LYT_DEBUG) printf("PHX_BUF_DST_XLENGTH : %d\nPHX_BUF_DST_yLENGTH : %d\n", dwBufferWidth, dwBufferHeight);

  dwBufferStride = dwBufferWidth; /* Convert width (in pixels) to stride (in bytes).*/
  
  PBL_BufferParameterSet( lytBuffer1, PBL_BUFF_HEIGHT, &dwBufferHeight ); /* If we assume Y8 capture format Set the height and stride required. */
  PBL_BufferParameterSet( lytBuffer1, PBL_BUFF_STRIDE, &dwBufferStride );

  PBL_BufferParameterSet( lytBuffer2, PBL_BUFF_HEIGHT, &dwBufferHeight );
  PBL_BufferParameterSet( lytBuffer2, PBL_BUFF_STRIDE, &dwBufferStride );
  
  PBL_BufferInit( lytBuffer1 ); /* Initialise each buffer. This creates the buffers in system memory. */
  PBL_BufferInit( lytBuffer2 );

  PBL_BufferParameterGet( lytBuffer1, PBL_BUFF_ADDRESS, &dwParamValue );   /* Build our array of image buffers. PBL_BUFF_ADDRESS returns the address of the first pixel of image data. */
  pasImageBuffs[ 0 ].pvAddress = (void*)dwParamValue;
  pasImageBuffs[ 0 ].pvContext = (void*)lytBuffer1;

  PBL_BufferParameterGet( lytBuffer2, PBL_BUFF_ADDRESS, &dwParamValue );
  pasImageBuffs[ 1 ].pvAddress = (void*)dwParamValue;
  pasImageBuffs[ 1 ].pvContext = (void*)lytBuffer2;

  pasImageBuffs[ 2 ].pvAddress = NULL;
  pasImageBuffs[ 2 ].pvContext = NULL;

  dwAcqNumImages = 2;
  eStat = PHX_ParameterSet( lytCamera, PHX_ACQ_NUM_IMAGES, &dwAcqNumImages );
  if ( PHX_OK != eStat ){
    printf("LYT: Error PHX_ParameterSet --> PHX_ACQ_NUM_IMAGES\n");
    lytctrlC(0);
  }


  dwAcqBufferStart = 1;
  eStat = PHX_ParameterSet( lytCamera, PHX_ACQ_BUFFER_START, &dwAcqBufferStart );
  if ( PHX_OK != eStat ){
    printf("LYT: Error PHX_ParameterSet --> PHX_ACQ_BUFFER_START\n");
    lytctrlC(0);
  }



  eStat = PHX_ParameterSet( lytCamera, PHX_DST_PTRS_VIRT, (void *) pasImageBuffs ); /* Instruct Phoenix to use the user supplied Virtual Buffers */
  if ( PHX_OK != eStat ){
    printf("LYT: Error PHX_ParameterSet --> PHX_DST_PTRS_VIRT\n");
    lytctrlC(0);
  }
 

  eParamValue = PHX_DST_PTR_USER_VIRT;
  eStat = PHX_ParameterSet( lytCamera, (etParam)( PHX_DST_PTR_TYPE | PHX_CACHE_FLUSH | PHX_FORCE_REWRITE ), (void *) &eParamValue );
  if ( PHX_OK != eStat ){
    printf("LYT: Error PHX_ParameterSet --> PHX_DST_PTR_TYPE | PHX_CACHE_FLUSH | PHX_FORCE_REWRITE\n");
    lytctrlC(0);
  }
 

  eStat = PHX_ParameterSet( lytCamera, PHX_EVENT_CONTEXT, (void *) &aContext ); /* Setup our own event context */
  if ( PHX_OK != eStat ){
    printf("LYT: Error PHX_ParameterSet --> PHX_EVENT_CONTEXT\n");
    lytctrlC(0);
  }
 
  /* -------------------- Now start our capture -------------------- */
  eStat = PHX_StreamRead( lytCamera, PHX_START, (void*)lyt_callback );
  if ( PHX_OK != eStat ){
    printf("LYT: PHX_StreamRead --> PHX_START\n");
    lytctrlC(0);
  }


  /* -------------------- Enter Waiting Loop -------------------- */
  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[LYTID].die)
      lytctrlC(0);

    /* Check in with the watchdog */
    checkin(sm_p,LYTID);
    
    /* Sleep */
    sleep(sm_p->w[LYTID].per);

  }

  lytctrlC(0);
  return 0;
}
