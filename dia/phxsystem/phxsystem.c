/* Application & library headers */
#include <phx_api.h> /* Main Phoenix library */
#include <pbl_api.h> /* Phoenix Buffer library */
#include <config.h>

static void phxsystem_callback( tHandle, ui32, void* );
int phxsystem( etParamValue, char* );

static void phxsystem_callback( tHandle hCamera, ui32 dwInterruptMask, void *pvParams ) {
  if ( dwInterruptMask & PHX_INTRPT_BUFFER_READY ) {
    (*(int*) pvParams)++;
    /* stImageBuff stBuffer;
    etStat eStat = PHX_StreamRead( hCamera, PHX_BUFFER_GET, &stBuffer );
    if ( PHX_OK == eStat ) {
      printf("EventCount = %d [Addr=%p, Context=%p]\n", *(int*)pvParams, stBuffer.pvAddress, stBuffer.pvContex) );
    }
    PHX_StreamRead( hCamera, PHX_BUFFER_RELEASE, NULL ); */
  }
}

int phxsystem( etParamValue eBoardNumber, char *configFileName ) {
  etStat eStat = PHX_OK; /* Status variable */
  etParamValue eParamValue; /* Parameter for use with PHX_ParameterSet/Get calls */
  tHandle hCamera = 0; /* Camera Handle   */
  volatile int nCurrentEventCount = 0; /* Event counter   */
  int nLastEventCount = 0;
  ui32 dwAcqNumImages = 0;
  ui32 dwAcqBufferStart = 0;
  ui32 dwBufferWidth = 0;
  ui32 dwBufferHeight = 0;
  ui32 dwBufferStride = 0;
  ui32 dwLoop;

  tPHX hBuffer1, hBuffer2;
  stImageBuff pasImageBuffs[3];
  ui32 dwParamValue;

  eStat = PHX_Create( &hCamera, PHX_ErrHandlerDefault ); /* Create a Phoenix handle */
  if ( PHX_OK != eStat ) goto Error;

  eStat = PHX_ParameterSet( hCamera, PHX_BOARD_NUMBER, &eBoardNumber ); /* Set the board number */
  if ( PHX_OK != eStat ) goto Error;

  eStat = PHX_Open( hCamera ); /* Open the Phoenix board using the above configuration file */
  if ( PHX_OK != eStat ) goto Error;

  eStat = CONFIG_RunFile( hCamera, &configFileName );  /* set the settings from the compound (BOBCAT and PHX) configuration file */
  if ( eStat != PHX_OK ) goto Error;






  /* manual buffer definitions
  eStat = PHX_ParameterGet( hCamera, PHX_BUF_DST_XLENGTH, &dwBufferWidth );
  if ( PHX_OK != eStat ) goto Error;

  eStat = PHX_ParameterGet( hCamera, PHX_BUF_DST_YLENGTH, &dwBufferHeight );
  if ( PHX_OK != eStat ) goto Error;

  printf("NumImages = %d (%d x %d) px Images\n", dwAcqNumImages, dwBufferWidth, dwBufferHeight);

  stImageBuff *pasImageBuffs = NULL;
  stImageBuff *psImageBuff = NULL;

  // dma buffer defined manually
  pasImageBuffs = (stImageBuff*) malloc( ( dwAcqNumImages + 1) * sizeof(stImageBuff) );
  if ( NULL == pasImageBuffs ) {
    printf("Failed to allocate memory for the array of stImage structures\n");
    goto Error;
  }
  psImageBuff = pasImageBuffs;
  for ( dwLoop = 1; dwLoop <= dwAcqNumImages; dwLoop++ ) {
    psImageBuff->pvAddress = (ui8*) malloc(dwBufferWidth*dwBufferHeight);
    if ( NULL == psImageBuff->pvAddress ) {
      printf("Failed to allocate memory for stImage structures #%d\n", dwLoop);
      goto Error;
    }
    psImageBuff->pvContext = (void*) dwLoop;
    psImageBuff++;
  }
  psImageBuff->pvAddress = NULL; // Terminate the list of stImage buffers
  psImageBuff->pvContext = NULL;

  dwAcqNumImages = 2;
  eStat = PHX_ParameterSet( hCamera, PHX_ACQ_NUM_IMAGES, &dwAcqNumImages );
  if ( PHX_OK != eStat ) goto Error;

  dwAcqBufferStart = 1;
  eStat = PHX_ParameterSet( hCamera, PHX_ACQ_BUFFER_START, &dwAcqBufferStart );
  if ( PHX_OK != eStat ) goto Error; */





  /* phoenix buffer library buffer definitions */
  PBL_BufferCreate( &hBuffer1, PBL_BUFF_SYSTEM_MEM_DIRECT, 0, hCamera, PHX_ErrHandlerDefault );
  PBL_BufferCreate( &hBuffer2, PBL_BUFF_SYSTEM_MEM_DIRECT, 0, hCamera, PHX_ErrHandlerDefault );

  eStat = PHX_ParameterGet( hCamera, PHX_ROI_XLENGTH_SCALED, &dwBufferWidth );
  if ( PHX_OK != eStat ) goto Error;

  eStat = PHX_ParameterGet( hCamera, PHX_ROI_YLENGTH_SCALED, &dwBufferHeight );
  if ( PHX_OK != eStat ) goto Error;

  dwBufferStride = dwBufferWidth; /* Convert width (in pixels) to stride (in bytes).*/
  
  PBL_BufferParameterSet( hBuffer1, PBL_BUFF_HEIGHT, &dwBufferHeight ); /* If we assume Y8 capture format Set the height and stride required. */
  PBL_BufferParameterSet( hBuffer1, PBL_BUFF_STRIDE, &dwBufferStride );

  PBL_BufferParameterSet( hBuffer2, PBL_BUFF_HEIGHT, &dwBufferHeight );
  PBL_BufferParameterSet( hBuffer2, PBL_BUFF_STRIDE, &dwBufferStride );

  PBL_BufferInit( hBuffer1 ); /* Initialise each buffer. This creates the buffers in system memory. */
  PBL_BufferInit( hBuffer2 );


  PBL_BufferParameterGet( hBuffer1, PBL_BUFF_ADDRESS, &dwParamValue );   /* Build our array of image buffers. PBL_BUFF_ADDRESS returns the address of the first pixel of image data. */
  pasImageBuffs[ 0 ].pvAddress = (void*)dwParamValue;
  pasImageBuffs[ 0 ].pvContext = (void*)hBuffer1;

  PBL_BufferParameterGet( hBuffer2, PBL_BUFF_ADDRESS, &dwParamValue );
  pasImageBuffs[ 1 ].pvAddress = (void*)dwParamValue;
  pasImageBuffs[ 1 ].pvContext = (void*)hBuffer2;

  pasImageBuffs[ 2 ].pvAddress = NULL;
  pasImageBuffs[ 2 ].pvContext = NULL;

  dwAcqNumImages = 2;
  eStat = PHX_ParameterSet( hCamera, PHX_ACQ_NUM_IMAGES, &dwAcqNumImages );
  if ( PHX_OK != eStat ) goto Error;

  dwAcqBufferStart = 1;
  eStat = PHX_ParameterSet( hCamera, PHX_ACQ_BUFFER_START, &dwAcqBufferStart );
  if ( PHX_OK != eStat ) goto Error;








  eStat = PHX_ParameterSet( hCamera, PHX_DST_PTRS_VIRT, (void *) pasImageBuffs ); /* Instruct Phoenix to use the user supplied Virtual Buffers */
  if ( PHX_OK != eStat ) goto Error;

  eParamValue = PHX_DST_PTR_USER_VIRT;
  eStat = PHX_ParameterSet( hCamera, (etParam)( PHX_DST_PTR_TYPE | PHX_CACHE_FLUSH | PHX_FORCE_REWRITE ), (void *) &eParamValue );
  if ( PHX_OK != eStat ) goto Error;

  eStat = PHX_ParameterSet( hCamera, PHX_EVENT_CONTEXT, (void *) &nCurrentEventCount ); /* Setup our own event context */
  if ( PHX_OK != eStat ) goto Error;

  printf("capturing for xx s\n");

  /* -------------------- Now start our capture -------------------- */
  eStat = PHX_StreamRead( hCamera, PHX_START, (void*)phxsystem_callback );
  if ( PHX_OK != eStat ) goto Error;

  _PHX_SleepMs(30000);

  /* 
  stImageBuff stBuffer;
  while(!PhxCommonKbHit()) {
    _PHX_SleepMs(100);
    if ( nCurrentEventCount != nLastEventCount ) {
      eStat = PHX_StreamRead( hCamera, PHX_BUFFER_GET, &stBuffer );
      if ( PHX_OK == eStat ) {
        // Add Processing here
        printf("EventCount = %d [Addr=%p, Context=%p]\n", nCurrentEventCount, stBuffer.pvAddress, stBuffer.pvContext );
      }
      PHX_StreamRead( hCamera, PHX_BUFFER_RELEASE, NULL );
      nLastEventCount = nCurrentEventCount;
    }
    (void) PhxCommonKbRead();
  }
  printf("\n");
  // ----------------------------------------
  */
  
  Error:
  if ( hCamera ) PHX_StreamRead( hCamera, PHX_ABORT, NULL ); /* Now cease all captures */
  
  /*
  psImageBuff = pasImageBuffs; // Free all the user allocated memory
  if ( NULL != pasImageBuffs ) {
    while ( NULL != psImageBuff->pvAddress ) {
      free( psImageBuff->pvAddress );
      psImageBuff++;
    }
    free( pasImageBuffs );
  } */

  if (hBuffer1) PBL_BufferDestroy( &hBuffer1 );
  if (hBuffer2) PBL_BufferDestroy( &hBuffer1 );

  if ( hCamera ) { /* Release the Phoenix board */
    PHX_Close( &hCamera ); /* Close the Phoenix board */
    PHX_Destroy( &hCamera ); /* Destroy the Phoenix handle */
  }

  printf("Exiting\n");
  return 0;
}



/* VxWorks requires a unique function name for each application */
int main( int argc, char* argv[] ) {

  tPhxCmd sPhxCmd;
  int nStatus;

  CONFIG_ParseCmd( argc, argv, &sPhxCmd );
  /* PhxCommonKbInit();*/
  nStatus = phxsystem( sPhxCmd.eBoardNumber, sPhxCmd.pszConfigFileName );
  /* PhxCommonKbClose();*/
  return nStatus;

}
