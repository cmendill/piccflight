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
#include "controller.h"
#include "common_functions.h"

/* Process File Descriptor */
int thm_shmfd;

/* CTRL-C Function */
void thmctrlC(int sig)
{
#if MSG_CTRLC
  printf("THM: ctrlC! exiting.\n");
#endif
  close(thm_shmfd);
  exit(sig);
}

/* Main Process */
void thm_proc(void){
  uint32 count = 0;
  BYTE result; // returned error code
  DSCB board1, board2, board3;  // handle used to refer to the board

  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&thm_shmfd)) == NULL){
    printf("openshm fail: thm_proc\n");
    thmctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, thmctrlC);	/* usually ^C */
 
  /* Define structures containing board settings */
  DSCCB
  dsccb1 = {
    .base_address = ADC1_BASE,
    .int_level = 5
  },
    dsccb2 = {
    .base_address = ADC2_BASE,
    .int_level = 5
  },
  dsccb3 = {
    .base_address = ADC3_BASE,
    .int_level = 5
  };

  /* Define structure containing A/D conversion settings (all boards set to the same settings) */
  DSCADSETTINGS dscadsettings = {
    .range = RANGE_5,
    .polarity = BIPOLAR,
    .gain = GAIN_8,
    .load_cal = (BYTE)TRUE,
    .current_channel = 0,
    .scan_interval = SCAN_INTERVAL_10 // allows the channel switching to be 10 microseconds
  };


  /* Define structure containing A/D scan settings (all boards set to the same scan settings) */
  DSCADSCAN dscadscan = {
    .low_channel = 13, 
    .high_channel = 15,
    .gain = dscadsettings.gain
  };

  /* Define structure containing auto-calibration settings */
  DSCAUTOCAL dscautocal = {
    .adrange = 3, // see page 42
    .boot_adrange = 3 // see page 42
  };

  ERRPARAMS errorParams; // structure for returning error code and error string
  int i, j; // miscellaneous counters

  int n_chnl; // number of channels
  n_chnl = dscadscan.high_channel - dscadscan.low_channel + 1;

  DSCSAMPLE *samples1, *samples2, *samples3; // sample readings
  samples1 = (DSCSAMPLE*)malloc( sizeof(DSCSAMPLE) * ( ADC1_NCHAN ) );
  samples2 = (DSCSAMPLE*)malloc( sizeof(DSCSAMPLE) * ( ADC2_NCHAN ) );
  samples3 = (DSCSAMPLE*)malloc( sizeof(DSCSAMPLE) * ( ADC3_NCHAN ) );

  DFLOAT voltage; //voltage value

  
  
  //=========================================================================
  // LIBRARY INITIALIZATION
  //=========================================================================
  if( dscInit( DSC_VERSION ) != DE_NONE ) {
    dscGetLastError(&errorParams);
    fprintf( stderr, "dscInit error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
    return 0;
  }
  
  //=========================================================================
  // BOARD 1 CONFIGURATION
  //=========================================================================
  printf( "Board 1\n" );
  printf( "\tinitialization\n" );
  if(dscInitBoard(DSC_DMM32X, &dsccb1, &board1)!= DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf( stderr, "\tdscInitBoard error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
    return 0;
  }
  printf( "\tAD settings\n" );
  if( ( result = dscADSetSettings( board1, &dscadsettings ) ) != DE_NONE ) {
    dscGetLastError(&errorParams);
    fprintf( stderr, "\tdscADSetSettings error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
    return 0;
  }
  printf( "\tAD auto-calibration...this will take a few seconds\n" );
  if( dscADAutoCal( board1, &dscautocal ) != DE_NONE ) {
      dscGetLastError(&errorParams);
      fprintf( stderr, "\tdscADAutoCal error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
      return 0;
  }
  printf( "\tCalibration Verification:\n" );  
  if( ( result = dscADCalVerify( board1, &dscautocal ) ) != DE_NONE ) {
    dscGetLastError(&errorParams);
    fprintf( stderr, "\tdscADCalVerify error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
    return 0;
  }
  printf( "\tConfiguration Mode: %d, Offset Error: %9.3f, Gain Error: %9.3f\n", 3, dscautocal.ad_offset, dscautocal.ad_gain );
  if ( fabs( dscautocal.ad_offset ) > 2 || fabs( dscautocal.ad_gain ) > 2 )
    printf( "\tValues for offset or gain exceeded specified tolerance\n" );
  else
    printf( "\tValues for offset and gain met specified tolerance\n" );
  
  //=========================================================================
  // BOARD 2 CONFIGURATION
  //=========================================================================
  printf( "Board 2\n" );
  printf( "\tinitialization\n" );
  if(dscInitBoard(DSC_DMM32X, &dsccb2, &board2)!= DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf( stderr, "\tdscInitBoard error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
    return 0;
  }
  printf( "\tAD settings\n" );
  if( ( result = dscADSetSettings( board2, &dscadsettings ) ) != DE_NONE ) {
    dscGetLastError(&errorParams);
    fprintf( stderr, "\tdscADSetSettings error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
    return 0;
  }
  printf( "\tAD auto-calibration...this will take a few seconds\n" );
  if( dscADAutoCal( board2, &dscautocal ) != DE_NONE ) {
      dscGetLastError(&errorParams);
      fprintf( stderr, "\tdscADAutoCal error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
      return 0;
  }
  printf( "\tCalibration Verification:\n" );  
  if( ( result = dscADCalVerify( board2, &dscautocal ) ) != DE_NONE ) {
    dscGetLastError(&errorParams);
    fprintf( stderr, "\tdscADCalVerify error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
    return 0;
  }
  printf( "\tConfiguration Mode: %d, Offset Error: %9.3f, Gain Error: %9.3f\n", 3, dscautocal.ad_offset, dscautocal.ad_gain );
  if ( fabs( dscautocal.ad_offset ) > 2 || fabs( dscautocal.ad_gain ) > 2 )
    printf( "\tValues for offset or gain exceeded specified tolerance\n" );
  else
    printf( "\tValues for offset and gain met specified tolerance\n" );


  //=========================================================================
  // BOARD 3 CONFIGURATION
  //=========================================================================
  printf( "Board 3\n" );
  printf( "\tinitialization\n" );
  if(dscInitBoard(DSC_DMM32X, &dsccb3, &board3)!= DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf( stderr, "\tdscInitBoard error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
    return 0;
  }
  printf( "\tAD settings\n" );
  if( ( result = dscADSetSettings( board3, &dscadsettings ) ) != DE_NONE ) {
    dscGetLastError(&errorParams);
    fprintf( stderr, "\tdscADSetSettings error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
    return 0;
  }
  printf( "\tAD auto-calibration...this will take a few seconds\n" );
  if( dscADAutoCal( board3, &dscautocal ) != DE_NONE ) {
      dscGetLastError(&errorParams);
      fprintf( stderr, "\tdscADAutoCal error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
      return 0;
  }
  printf( "\tCalibration Verification:\n" );  
  if( ( result = dscADCalVerify( board3, &dscautocal ) ) != DE_NONE ) {
    dscGetLastError(&errorParams);
    fprintf( stderr, "\tdscADCalVerify error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
    return 0;
  }
  printf( "\tConfiguration Mode: %d, Offset Error: %9.3f, Gain Error: %9.3f\n", 3, dscautocal.ad_offset, dscautocal.ad_gain );
  if ( fabs( dscautocal.ad_offset ) > 2 || fabs( dscautocal.ad_gain ) > 2 )
    printf( "\tValues for offset or gain exceeded specified tolerance\n" );
  else
    printf( "\tValues for offset and gain met specified tolerance\n" );


  
  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[THMID].die)
      thmctrlC(0);
    
    /* Check in with the watchdog */
    checkin(sm_p,THMID);
    
    //=========================================================================
    // SCANNING AND OUTPUT
    // Perform the actual sampling and then output the results.
    // To calculate the actual input voltages, we must convert the sample code (which must be cast to a short to get the correct code) and then plug it into one of the formulas located in the manual for your board (under "A/D Conversion Formulas"). 
    //=========================================================================
    //Board 1 ADC
    if( ( result = dscADScan( board1, &dscadscan, samples1 ) ) != DE_NONE ) {
      dscGetLastError(&errorParams);
      fprintf( stderr, "dscADScan error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
      free( samples1 ); // remember to deallocate malloc() memory
      return 0;
    }
    for( i = 0; i < (dscadscan.high_channel - dscadscan.low_channel)+ 1; i++) {
      if( dscADCodeToVoltage( board1, dscadsettings, dscadscan.sample_values[i], &voltage) != DE_NONE) {
	dscGetLastError(&errorParams);
	fprintf( stderr, "dscADCodeToVoltage error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
	free( samples1 ); // remember to deallocate malloc() memory
	return 0;
      }
    }
    //Board 2 ADC
    if( ( result = dscADScan( board2, &dscadscan, samples2 ) ) != DE_NONE ) {
      dscGetLastError(&errorParams);
      fprintf( stderr, "dscADScan error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
      free( samples2 ); // remember to deallocate malloc() memory
      return 0;
    }
    for( i = 0; i < (dscadscan.high_channel - dscadscan.low_channel)+ 1; i++) {
      if( dscADCodeToVoltage( board2, dscadsettings, dscadscan.sample_values[i], &voltage) != DE_NONE) {
	dscGetLastError(&errorParams);
	fprintf( stderr, "dscADCodeToVoltage error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
	free( samples2 ); // remember to deallocate malloc() memory
	return 0;
      }
    }
    //Board 3 ADC
    if( ( result = dscADScan( board3, &dscadscan, samples3 ) ) != DE_NONE ) {
      dscGetLastError(&errorParams);
      fprintf( stderr, "dscADScan error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
      free( samples3 ); // remember to deallocate malloc() memory
      return 0;
    }
    for( i = 0; i < (dscadscan.high_channel - dscadscan.low_channel)+ 1; i++) {
      if( dscADCodeToVoltage( board3, dscadsettings, dscadscan.sample_values[i], &voltage) != DE_NONE) {
	dscGetLastError(&errorParams);
	fprintf( stderr, "dscADCodeToVoltage error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring );
	free( samples3 ); // remember to deallocate malloc() memory
	return 0;
      }
    }

    /* Sleep */
    sleep(sm_p->w[THMID].per);
  }


  thmctrlC(0);
  return;
}
