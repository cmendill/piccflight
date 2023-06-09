#define _XOPEN_SOURCE 500
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <strings.h>
#include <fcntl.h>
#include <ctype.h>
#include <math.h>
#include <sys/io.h>
#include <sensors/sensors.h>
#include <sensors/error.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "dscud.h"
#include "../drivers/phxdrv/picc_dio.h"
#include <libhdc.h>

/* settings */
#define MAX_AD_OFFSET   2
#define MAX_AD_GAIN     2
#define AD_CONFIG_MODE  3
#define AD_RANGE_CODE   3    
#define HTR_NSTEPS      100  //Heater resolution (0-100%)
#define HTR_NCYCLES     10   //Number of heater output cycles
#define ADC_NAVG        10   //Number of temperature reads to average
#define ADC_VREF_SENSOR 15


/* temperature conversion */
#define VREF_DEFAULT    5.0     //volts
#define ADC1_R1         1000.0  //ohms
#define ADC2_R1         2000.0  //ohms
#define ADC3_R1         2000.0  //ohms
#define RTD_ALPHA       0.00385 //ohms/ohms/degC
#define RTD_OHMS        100.0   //ohms

/* Process File Descriptors */
int thm_shmfd;
int thm_humfd;

/* CTRL-C Function */
void thmctrlC(int sig)
{
  //turn off all heaters
  if(HTR_ENABLE){
    outb(0xFF,SSR_BASE+0);
    outb(0xFF,SSR_BASE+4);
  }
  //cleanup humidity sensors
  hdc_cleanup(thm_humfd);
  //cleanup cpu sensors
  sensors_cleanup();
  //close shared memory
  close(thm_shmfd);
#if MSG_CTRLC
  printf("THM: exiting\n");
#endif
  exit(sig);
}

/**************************************************************/
/* FIND_HUMIDITY                                              */
/* - Parse /sys/bus/i2c and return the humidity sensor device */
/**************************************************************/
int find_humidity(void){
  FILE *fd=NULL;
  char filename[256];
  char line[256];
  char dev[128];
  int i;

  sprintf(dev,"i2c-tiny-usb");
  
  //Loop over device files
  for(i=0;i<=9;i++){
    //Open file
    sprintf(filename,"/sys/bus/i2c/devices/i2c-%d/name",i);
    if((fd = fopen(filename,"r")) == NULL){
      perror("COM: geti2c fopen");
      return(-1);
    }
    //Read file
    if(fgets(line, sizeof(line), fd)){
      if(!strncasecmp(line,dev,strlen(dev))){
	fclose(fd);
	return i;
      }
    }
  }

  printf("COM: geti2c could not I2C device!\n");
  
  //Close file
  fclose(fd);

  //Return error
  return(-1);
  
}

/**************************************************************/
/* THM_PROC                                                   */
/*  - Main thermal control process                            */
/**************************************************************/
void thm_proc(void){
  static thmevent_t thmevent;  
  static struct timespec start, end;
  static int init = 0;
  static unsigned long count=0;
  double resistance;              // calculated RTD resistance
  double vref;                    // calculated using the reference resistor RTD_OHMS
  uint16_t htr_command;           // heater command word
  unsigned char htr_lsb=0,htr_msb=0;
  const long htr_sleep = ONE_MILLION / HTR_NSTEPS / HTR_NCYCLES; //us
  int i, j, k, iavg, dev;
  static int pulse_index[HTR_NSTEPS];
  int htr_pulse[SSR_NCHAN][HTR_NSTEPS];
  int state;
  char hum_device[128];
  double delta,tdir;
  double power[SSR_NCHAN]={0};
  double tint[SSR_NCHAN]={0};
  double delta_last[SSR_NCHAN]={0};

  
  //Humidity sensors
  hdc_config config;
  hdc_device_t hum[HUM_NSENSORS] = {HUM1_ADDR,HUM2_ADDR,HUM3_ADDR};
  int hum_ready[HUM_NSENSORS] = {0};

  //CPU temp sensors
  const sensors_chip_name *chip = NULL;
  const sensors_feature *feature = NULL;
  const sensors_subfeature *sub = NULL;
  int cpu_ready=0;
  int chip_nr=0;
  double cpuval;

  //DSCUD Variables
  BYTE result;                    // returned error code
  DSCB board1, board2, board3;    // handle used to refer to the board
  ERRPARAMS errorParams;          // structure for returning error code and error string
  DSCSAMPLE samples1[ADC1_NCHAN]; // digital readings
  DSCSAMPLE samples2[ADC2_NCHAN]; // digital readings
  DSCSAMPLE samples3[ADC3_NCHAN]; // digital readings
  DFLOAT voltage;                 // voltage value
  DFLOAT voltage1[ADC1_NCHAN];    // voltage values
  DFLOAT voltage2[ADC2_NCHAN];    // voltage values
  DFLOAT voltage3[ADC3_NCHAN];    // voltage values

  /* Initialize */
  if(!init){
    memset(&thmevent,0,sizeof(thmevent));
    memset(power,0,sizeof(power));
    memset(tint,0,sizeof(tint));
    memset(delta_last,0,sizeof(delta_last));
    count=0;
    //Get dither indicies for heater control
    if(ditherfill(pulse_index,HTR_NSTEPS)){
      printf("THM: ditherfill error. Using default.\n");
      for(i=0;i<HTR_NSTEPS;i++)
	pulse_index[i] = i;
    }
    //Set init flag
    init=1;
    if(THM_DEBUG) printf("THM: Initialized\n");
  }

  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&thm_shmfd)) == NULL){
    printf("openshm fail: thm_proc\n");
    thmctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, thmctrlC);	/* usually ^C */

  /* Init humidity sensors */
  if((dev = find_humidity()) >= 0){
    sprintf(hum_device,"/dev/i2c-%d",dev);
    printf("THM: found humidity sensors on %s\n",hum_device);
    if((thm_humfd=hdc_open(hum_device)) >= 0){
      for(i=0;i<HUM_NSENSORS;i++){
	if(hdc_init(thm_humfd, &hum[i]))
	  continue;
	if(hdc_write_config(thm_humfd, &hum[i], &config))
	  continue;
	if(THM_DEBUG) hdc_get_info(thm_humfd, &hum[i]);
	hum_ready[i]=1;
      }
    }else{
      printf("THM: hdc_open failed\n");
    }
  }else{
    printf("THM: could not find humidity sensors!\n");
  }
  for(i=0;i<HUM_NSENSORS;i++)
    printf("THM: Humidity sensor %d status: %d\n",i,hum_ready[i]); 
  
  /* Init CPU temp sensors */
  if(sensors_init(NULL) == 0){
    if((chip = sensors_get_detected_chips(NULL, &chip_nr))){
      cpu_ready=1;
      printf("THM: Initialized CPU sensors\n");
    }
  }
  
  /* Define structures containing board settings */
  DSCCB dsccb1 = {
    .base_address = ADC1_BASE,
    .int_level = 5
  };
  DSCCB dsccb2 = {
    .base_address = ADC2_BASE,
    .int_level = 5
  };
  DSCCB dsccb3 = {
    .base_address = ADC3_BASE,
    .int_level = 5
  };

  /* Define structure containing A/D conversion settings */
  DSCADSETTINGS dscadsettings1 = {
    .range = RANGE_5,
    .polarity = BIPOLAR,
    .gain = GAIN_8,
    .load_cal = (BYTE)TRUE,
    .current_channel = 0,
    .scan_interval = SCAN_INTERVAL_10 // allows the channel switching to be 10 microseconds
  };
  DSCADSETTINGS dscadsettings2 = {
    .range = RANGE_5,
    .polarity = BIPOLAR,
    .gain = GAIN_8,
    .load_cal = (BYTE)TRUE,
    .current_channel = 0,
    .scan_interval = SCAN_INTERVAL_10 // allows the channel switching to be 10 microseconds
  };
  DSCADSETTINGS dscadsettings3 = {
    .range = RANGE_5,
    .polarity = BIPOLAR,
    .gain = GAIN_8,
    .load_cal = (BYTE)TRUE,
    .current_channel = 0,
    .scan_interval = SCAN_INTERVAL_10 // allows the channel switching to be 10 microseconds
  };


  /* Define structure containing A/D scan settings */
  DSCADSCAN dscadscan1 = {
    .low_channel = 0, 
    .high_channel = ADC1_NCHAN-1,
    .gain = dscadsettings1.gain
  };
  DSCADSCAN dscadscan2 = {
    .low_channel = 0, 
    .high_channel = ADC2_NCHAN-1,
    .gain = dscadsettings2.gain
  };
  DSCADSCAN dscadscan3 = {
    .low_channel = 0, 
    .high_channel = ADC3_NCHAN-1,
    .gain = dscadsettings3.gain
  };

  
  /* Define structure containing auto-calibration settings */
  DSCAUTOCAL dscautocal1 = {
    .adrange = AD_RANGE_CODE, // see page 42
    .boot_adrange = AD_RANGE_CODE // see page 42
  };
  DSCAUTOCAL dscautocal2 = {
    .adrange = AD_RANGE_CODE, // see page 42
    .boot_adrange = AD_RANGE_CODE // see page 42
  };
  DSCAUTOCAL dscautocal3 = {
    .adrange = AD_RANGE_CODE, // see page 42
    .boot_adrange = AD_RANGE_CODE // see page 42
  };

  
  
  //=========================================================================
  // LIBRARY INITIALIZATION
  //=========================================================================
  if(dscInit(DSC_VERSION) != DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf(stderr, "dscInit error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
    thmctrlC(0);
  }
  
  //=========================================================================
  // BOARD 1 CONFIGURATION
  //=========================================================================
  #if PICC_DIO_ENABLE == 0
  //-- Init Board
  if(dscInitBoard(DSC_DMM32X, &dsccb1, &board1) != DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf(stderr, "THM: Board 1 dscInitBoard error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
    thmctrlC(0);
  }
  //-- ADC Settings
  if((result = dscADSetSettings(board1, &dscadsettings1)) != DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf(stderr, "THM: Board 1 dscADSetSettings error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
    thmctrlC(0);
  }
  //-- Auto Calibration Settings
  if(dscADAutoCal(board1, &dscautocal1) != DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf(stderr, "THM: Board 1 dscADAutoCal error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
    thmctrlC(0);
  }
  //-- Verify Auto Calibration
  if((result = dscADCalVerify(board1, &dscautocal1)) != DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf(stderr, "THM: Board 1 dscADCalVerify error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
    thmctrlC(0);
  }
  if((fabs(dscautocal1.ad_offset) > MAX_AD_OFFSET) || (fabs(dscautocal1.ad_gain) > MAX_AD_GAIN)){
    fprintf(stderr, "THM: Board 1 Configuration Mode: %d, Offset Error: %9.3f, Gain Error: %9.3f\n", AD_CONFIG_MODE, dscautocal1.ad_offset, dscautocal1.ad_gain);
    fprintf(stderr, "THM: Board 1 values for offset or gain exceeded specified tolerance\n" );
    thmctrlC(0);
  }
  //-- Check in with watchdog
  checkin(sm_p,THMID);
  #else
  printf("THM: ADC Board 1 disabled for PICC_DIO\n");
  #endif

  //=========================================================================
  // BOARD 2 CONFIGURATION
  //=========================================================================
  //-- Init Board
  if(dscInitBoard(DSC_DMM32X, &dsccb2, &board2) != DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf(stderr, "THM: Board 2 dscInitBoard error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
    thmctrlC(0);
  }
  //-- ADC Settings
  if((result = dscADSetSettings(board2, &dscadsettings2)) != DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf(stderr, "THM: Board 2 dscADSetSettings error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
    thmctrlC(0);
  }
  //-- Auto Calibration Settings
  if(dscADAutoCal(board2, &dscautocal2) != DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf(stderr, "THM: Board 2 dscADAutoCal error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
    thmctrlC(0);
  }
  //-- Verify Auto Calibration
  if((result = dscADCalVerify(board2, &dscautocal2)) != DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf(stderr, "THM: Board 2 dscADCalVerify error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
    thmctrlC(0);
  }
  if((fabs(dscautocal2.ad_offset) > MAX_AD_OFFSET) || (fabs(dscautocal2.ad_gain) > MAX_AD_GAIN)){
    fprintf(stderr, "THM: Board 2 Configuration Mode: %d, Offset Error: %9.3f, Gain Error: %9.3f\n", AD_CONFIG_MODE, dscautocal2.ad_offset, dscautocal2.ad_gain);
    fprintf(stderr, "THM: Board 2 values for offset or gain exceeded specified tolerance\n" );
    thmctrlC(0);
  }
  //-- Check in with watchdog
  checkin(sm_p,THMID);

  //=========================================================================
  // BOARD 3 CONFIGURATION
  //=========================================================================
  //-- Init Board
  if(dscInitBoard(DSC_DMM32X, &dsccb3, &board3) != DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf(stderr, "THM: Board 3 dscInitBoard error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
    thmctrlC(0);
  }
  //-- ADC Settings
  if((result = dscADSetSettings(board3, &dscadsettings3)) != DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf(stderr, "THM: Board 3 dscADSetSettings error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
    thmctrlC(0);
  }
  //-- Auto Calibration Settings
  if(dscADAutoCal(board3, &dscautocal3) != DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf(stderr, "THM: Board 3 dscADAutoCal error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
    thmctrlC(0);
  }
  //-- Verify Auto Calibration
  if((result = dscADCalVerify(board3, &dscautocal3)) != DE_NONE) {
    dscGetLastError(&errorParams);
    fprintf(stderr, "THM: Board 3 dscADCalVerify error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
    thmctrlC(0);
  }
  if((fabs(dscautocal3.ad_offset) > MAX_AD_OFFSET) || (fabs(dscautocal3.ad_gain) > MAX_AD_GAIN)){
    fprintf(stderr, "THM: Board 3 Configuration Mode: %d, Offset Error: %9.3f, Gain Error: %9.3f\n", AD_CONFIG_MODE, dscautocal3.ad_offset, dscautocal3.ad_gain);
    fprintf(stderr, "THM: Board 3 values for offset or gain exceeded specified tolerance\n" );
    thmctrlC(0);
  }
  //-- Check in with watchdog
  checkin(sm_p,THMID);
  printf("THM: 3 ADC Boards Initialized\n");


  //=========================================================================
  // BEGIN MAIN LOOP
  //=========================================================================
  while(1){
    //Get start timestamp
    clock_gettime(CLOCK_REALTIME,&start);

    //Get state
    state = sm_p->state;
    
    //Check if we've been asked to exit 
    if(sm_p->w[THMID].die)
      thmctrlC(0);
    
    //Check in with the watchdog
    checkin(sm_p,THMID);
    
    //Fill out event header
    thmevent.hed.version       = PICC_PKT_VERSION;
    thmevent.hed.type          = BUFFER_THMEVENT;
    thmevent.hed.frame_number  = count++;
    thmevent.hed.state         = state;
    thmevent.hed.alp_commander = sm_p->state_array[state].alp_commander;
    thmevent.hed.hex_commander = sm_p->state_array[state].hex_commander;
    thmevent.hed.bmc_commander = sm_p->state_array[state].bmc_commander;
    thmevent.hed.start_sec     = start.tv_sec;
    thmevent.hed.start_nsec    = start.tv_nsec;

    //Init VREF
    vref = VREF_DEFAULT;
    
    //=========================================================================
    // SCANNING AND OUTPUT
    // Perform the actual sampling and then output the results.
    // To calculate the voltages, convert the sample code (cast as short)
    // and then plug it into one of the formulas located in the manual for
    // your board (under "A/D Conversion Formulas"). 
    //=========================================================================

    //Run through averaging loop
    for(iavg=0;iavg<ADC_NAVG;iavg++){
      //Board ADC1
      #if PICC_DIO_ENABLE == 0
      if((result = dscADScan(board1, &dscadscan1, samples1 )) != DE_NONE){
	dscGetLastError(&errorParams);
	fprintf(stderr, "THM: Board ADC1 dscADScan error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
	thmctrlC(0);
      }       
      for(i = 0; i < ADC1_NCHAN; i++){
	if(dscADCodeToVoltage(board1, dscadsettings1, dscadscan1.sample_values[i], &voltage) != DE_NONE) {
	  dscGetLastError(&errorParams);
	  fprintf(stderr, "THM: Board ADC1 dscADCodeToVoltage error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
	  fprintf(stderr, "THM: Gain = %d\n",dscadsettings1.gain);
	  thmctrlC(0);
	}
	if(iavg == 0) voltage1[i] = 0; //reset voltages to zero for averaging
	voltage1[i] += voltage/ADC_NAVG;
      }
      //Get reference voltage
      if(sm_p->thm_enable_vref) vref = voltage1[ADC_VREF_SENSOR]*(ADC1_R1+RTD_OHMS)/RTD_OHMS;
      #endif
    
      //Board ADC2
      if((result = dscADScan(board2, &dscadscan2, samples2 )) != DE_NONE){
	dscGetLastError(&errorParams);
	fprintf(stderr, "THM: Board ADC2 dscADScan error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
	thmctrlC(0);
      }
      for(i = 0; i < ADC2_NCHAN; i++){
	if(dscADCodeToVoltage(board2, dscadsettings2, dscadscan2.sample_values[i], &voltage) != DE_NONE) {
	  dscGetLastError(&errorParams);
	  fprintf(stderr, "THM: Board ADC2 dscADCodeToVoltage error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
	  fprintf(stderr, "THM: Gain = %d\n",dscadsettings2.gain);
	  thmctrlC(0);
	}
	if(iavg == 0) voltage2[i] = 0; //reset voltages to zero for averaging
	voltage2[i] += voltage/ADC_NAVG;
      }

      //Board ADC3
      if((result = dscADScan(board3, &dscadscan3, samples3 )) != DE_NONE){
	dscGetLastError(&errorParams);
	fprintf(stderr, "THM: Board ADC3 dscADScan error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
	thmctrlC(0);
      }
      for(i = 0; i < ADC3_NCHAN; i++){
	if(dscADCodeToVoltage(board3, dscadsettings3, dscadscan3.sample_values[i], &voltage) != DE_NONE) {
	  dscGetLastError(&errorParams);
	  fprintf(stderr, "THM: Board ADC3 dscADCodeToVoltage error: %s %s\n", dscGetErrorString(errorParams.ErrCode), errorParams.errstring);
	  fprintf(stderr, "THM: Gain = %d\n",dscadsettings3.gain);
	  thmctrlC(0);
	}
	if(iavg == 0) voltage3[i] = 0; //reset voltages to zero for averaging
	voltage3[i] += voltage/ADC_NAVG;
      }
    }
    
    /* Convert voltage to temperature */
    for(i = 0; i < ADC1_NCHAN; i++){
      resistance = (voltage1[i] * ADC1_R1) / (vref - voltage1[i]);
      thmevent.adc1_temp[i] = (resistance - RTD_OHMS) / (RTD_ALPHA * RTD_OHMS);
      if(i==ADC_VREF_SENSOR) thmevent.adc1_temp[i] = vref;
    }
    for(i = 0; i < ADC2_NCHAN; i++){
      resistance = (voltage2[i] * ADC2_R1) / (vref - voltage2[i]);
      thmevent.adc2_temp[i] = (resistance - RTD_OHMS) / (RTD_ALPHA * RTD_OHMS);
    }
    for(i = 0; i < ADC3_NCHAN; i++){
      resistance = (voltage3[i] * ADC3_R1) / (vref - voltage3[i]);
      thmevent.adc3_temp[i] = (resistance - RTD_OHMS) / (RTD_ALPHA * RTD_OHMS);
    }
    

    /* Read humidity sensors */
    if(thm_humfd >= 0){
      for(i=0;i<HUM_NSENSORS;i++){
	if(hum_ready[i]){
	  hdc_get_t(thm_humfd, &hum[i], &thmevent.hum[i].temp);
	  hdc_get_rh(thm_humfd, &hum[i], &thmevent.hum[i].humidity);
	}
      }
    }

    /* Read CPU sensors */
    if(cpu_ready){
      //CPU 1
      i=0;
      feature = sensors_get_features(chip, &i);
      i=0;
      sub = sensors_get_all_subfeatures(chip, feature, &i);
      if(sub->flags & SENSORS_MODE_R){
        if(sensors_get_value(chip, sub->number, &cpuval))
	  cpuval=-99;
        else
          thmevent.cpu1_temp = cpuval;
      }
      //CPU 2
      i=1;
      feature = sensors_get_features(chip, &i);
      i=0;
      sub = sensors_get_all_subfeatures(chip, feature, &i);
      if(sub->flags & SENSORS_MODE_R){
        if(sensors_get_value(chip, sub->number, &cpuval))
	  cpuval=-99;
        else
          thmevent.cpu2_temp = cpuval;
      }
    }
    
    /* Set Control Parameters */
    for(i=0;i<SSR_NCHAN;i++){
      //Get controller settings from shared memory
      thmevent.htr[i].adc      = sm_p->htr[i].adc;
      thmevent.htr[i].ch       = sm_p->htr[i].ch;
      thmevent.htr[i].maxpower = sm_p->htr[i].maxpower;
      thmevent.htr[i].enable   = sm_p->htr[i].enable;
      thmevent.htr[i].setpoint = sm_p->htr[i].setpoint;
      thmevent.htr[i].deadband = sm_p->htr[i].deadband;
      thmevent.htr[i].override = sm_p->htr[i].override;
      thmevent.htr[i].gain     = sm_p->htr[i].gain;
      thmevent.htr[i].usepid   = sm_p->htr[i].usepid;
      thmevent.htr[i].kP       = sm_p->htr[i].kP;
      thmevent.htr[i].kI       = sm_p->htr[i].kI;
      thmevent.htr[i].kD       = sm_p->htr[i].kD;
      thmevent.htr[i].intmax   = sm_p->htr[i].intmax;
      //Copy temperatures from adc arrays
      if(thmevent.htr[i].adc == 1) thmevent.htr[i].temp = thmevent.adc1_temp[thmevent.htr[i].ch];
      if(thmevent.htr[i].adc == 2) thmevent.htr[i].temp = thmevent.adc2_temp[thmevent.htr[i].ch];
      if(thmevent.htr[i].adc == 3) thmevent.htr[i].temp = thmevent.adc3_temp[thmevent.htr[i].ch];
      //Copy name
      memcpy(thmevent.htr[i].name,(char *)sm_p->htr[i].name,sizeof(thmevent.htr[i].name));
    }
	  
    
    /* Run Temperature Control */
    for(i=0;i<SSR_NCHAN;i++){
      if(!thmevent.htr[i].override){
	//Get temperature error
	delta = thmevent.htr[i].setpoint - thmevent.htr[i].temp;
	//Choose controller type
	if(thmevent.htr[i].usepid){
	  //Run PID controller
	  tint[i] += delta;
	  if(tint[i] > thmevent.htr[i].intmax) tint[i] = thmevent.htr[i].intmax;
	  if(tint[i] < -1*thmevent.htr[i].intmax) tint[i] = -1*thmevent.htr[i].intmax;
	  if(thmevent.htr[i].kI == 0) tint[i] = 0; //reset integrator if gain is zero
	  tdir = delta - delta_last[i];
	  power[i] = delta * thmevent.htr[i].kP + tint[i] * thmevent.htr[i].kI + tdir * thmevent.htr[i].kD;
	  //Save controller values
	  thmevent.htr[i].Pval = delta;
	  thmevent.htr[i].Ival = tint[i];
	  thmevent.htr[i].Dval = tdir;
	}
	else{
	  //Run simple integral controller
	  if(fabs(delta) > thmevent.htr[i].deadband)
	    power[i] += delta * thmevent.htr[i].gain; //gain is % per degree C
	}
	//Save delta
	delta_last[i] = delta;
      }
      else{
	//User override command
	power[i] = sm_p->htr[i].overpower;
      }
    }
    
    /* Check heater enable */
    for(i=0;i<SSR_NCHAN;i++){
      if(!thmevent.htr[i].enable){
	power[i] = 0;
	tint[i] = 0;
	delta_last[i] = 0;
      }
    }

    /* Limit heater power and round to integer */
    for(i=0;i<SSR_NCHAN;i++){
      power[i] = power[i] > thmevent.htr[i].maxpower ? thmevent.htr[i].maxpower : power[i];
      power[i] = power[i] < 0 ? 0 : power[i];
      power[i] = power[i] > HTR_POWER_MAX ? HTR_POWER_MAX : power[i];
      thmevent.htr[i].power = lround(power[i]);
    }
    
    /* Fill out heater pulses */
    memset(htr_pulse,0,sizeof(htr_pulse));
    for(i=0;i<SSR_NCHAN;i++)
      for(j=0;j<thmevent.htr[i].power;j++)
	htr_pulse[i][pulse_index[j]] = 1;
	
    /* Command heaters */
    if(HTR_ENABLE){
      for(i=0;i<HTR_NCYCLES;i++){
	for(j=0;j<HTR_NSTEPS;j++){
	  htr_command = 0;
	  for(k=0;k<SSR_NCHAN;k++){
	    htr_command |= htr_pulse[k][j] << k;
	  }
	
	  //htr_command: 1 = ON,  0 = OFF
	  //SRR Board:   1 = OFF, 0 = ON
	  //Take ones complement of htr_command
	  //Seperate LSB and MSB
	  //Send to board
	  htr_lsb = ~htr_command & 0x00FF;
	  htr_msb = (~htr_command & 0xFF00) >> 8;
	  outb(htr_lsb,SSR_BASE+0);
	  outb(htr_msb,SSR_BASE+4);
	
	  //sleep
	  usleep(htr_sleep);
	}
      }
      if(THM_DEBUG) printf("THM: MSB: 0x%2.2x  LSB: 0x%2.2x\n",htr_msb,htr_lsb);
    }
    
    /* Copy values back to shared memory */
    for(i=0;i<SSR_NCHAN;i++){
      sm_p->htr[i].temp  = thmevent.htr[i].temp;
      sm_p->htr[i].power = thmevent.htr[i].power;
      sm_p->htr[i].Pval  = thmevent.htr[i].Pval;
      sm_p->htr[i].Ival  = thmevent.htr[i].Ival;
      sm_p->htr[i].Dval  = thmevent.htr[i].Dval;
   }
    
    /* Get end timestamp */
    clock_gettime(CLOCK_REALTIME,&end);
    thmevent.hed.end_sec = end.tv_sec;
    thmevent.hed.end_nsec = end.tv_nsec;
    
    /* Write data to circular buffer */
    if(sm_p->circbuf[BUFFER_THMEVENT].write) write_to_buffer(sm_p,&thmevent,BUFFER_THMEVENT);
  }


  thmctrlC(0);
  return;
}
