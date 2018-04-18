#include <QuickUSB.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <time.h>

/* piccflight headers */
#include "../common/controller.h"
#include "../common/common_functions.h"
#include "../common/numeric.h"

/**************************************************************/
/*                      IWC_INIT                             */
/**************************************************************/
void iwc_init(iwc_t *iwc){
  int i;
  for(i=0;i<IWC_NSPA;i++)
    iwc->spa[i] = IWC_SPA_BIAS;
  for(i=0;i<IWC_NTTP;i++)
    iwc->ttp[i] = 0;
}

/**************************************************************/
/*                      IWC_CALIBRATE                         */
/**************************************************************/
int iwc_calibrate(int calmode, iwc_t *iwc,int reset){
  int i,j,index;
  static struct timespec start,this,last,delta;
  static unsigned long int countA=0,countB=0,countF=0;
  static double zernike2spa[LOWFS_N_ZERNIKE*IWC_NSPA]={0};
  static double zernike_errors[LOWFS_N_ZERNIKE][ZERNIKE_ERRORS_NUMBER]={{0}};
  const double zernike_timestep = ZERNIKE_ERRORS_PERIOD;
  const int zuse[LOWFS_N_ZERNIKE]={0,0,0,1,1,1, 1,1,1,1,1,1, 0,0,0,0,0,0, 0,0,0,0,0,0};
  static int init=0;
  time_t t;
  FILE *fileptr=NULL;
  char filename[MAX_FILENAME];
  double dt=0,dt0=0,period=0;
  double zernikes[LOWFS_N_ZERNIKE]={0};
  double spa[IWC_NSPA];

  /* Reset */
  if(reset){
    countA=0;
    countB=0;
    countF=0;
    init=0;
    return calmode;
  }

  /* Initialize */
  if(!init){
    countA=0;
    countB=0;
    countF=0;
    clock_gettime(CLOCK_REALTIME, &start);

    /* Open ZERNIKE2SPA matrix file */
    //--setup filename
    sprintf(filename,ZERNIKE2SPA_FILE);
    //--open file
    if((fileptr = fopen(filename,"r")) == NULL){
      printf("Zernike2spa file\r\n");
      perror("fopen");
      goto endofinit;
    }
    //--check file size
    fseek(fileptr, 0L, SEEK_END);
    if(ftell(fileptr) != sizeof(zernike2spa)){
      printf("SHK: incorrect matrix file size %lu != %lu\n",ftell(fileptr),sizeof(zernike2spa));
      goto endofinit;
    }
    rewind(fileptr);
    //--read matrix
    if(fread(zernike2spa,sizeof(zernike2spa),1,fileptr) != 1){
      perror("fread");
      goto endofinit;
    }

    /* Open zernike errors file */
    //--setup filename
    sprintf(filename,ZERNIKE_ERRORS_FILE);
    //--open file
    if((fileptr = fopen(filename,"r")) == NULL){
      printf("Zernike Errors file\r\n");
      perror("fopen");
      goto endofinit;
    }
    //--check file size
    fseek(fileptr, 0L, SEEK_END);
    if(ftell(fileptr) != sizeof(zernike_errors)){
      printf("SHK: incorrect zernike_errors file size %lu != %lu\n",ftell(fileptr),sizeof(zernike_errors));
      goto endofinit;
    }
    rewind(fileptr);
    //--read matrix
    if(fread(zernike_errors,sizeof(zernike_errors),1,fileptr) != 1){
      perror("fread");
      goto endofinit;
    }

  endofinit:
    //--close file
    if(fileptr != NULL) fclose(fileptr);
    init=1;
  }

  /* Calculate times */
  clock_gettime(CLOCK_REALTIME, &this);
  if(timespec_subtract(&delta,&this,&start))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);

  /* CALMODE 1: Scan through acuators poking one at a time */
  if(calmode == 1){
    //Set all SPA actuators to bias
    for(i=0;i<IWC_NSPA;i++)
      iwc->spa[i]=IWC_SPA_BIAS;
    //poke one actuator
    iwc->spa[(countA/IWC_NCALIM) % IWC_NSPA] = IWC_SPA_BIAS+IWC_SPA_POKE;
    countA++;
    return calmode;
  }

  /* CALMODE 2: Scan through acuators poking one at a time.
   *            Set flat inbetween each poke
   */
  if(calmode == 2){
    if(countA >= 0 && countA < (2*IWC_NSPA*IWC_NCALIM)){
      //set all SPA actuators to bias
      for(i=0;i<IWC_NSPA;i++)
	iwc->spa[i]=IWC_SPA_BIAS;

      //poke one actuator
      if((countA/IWC_NCALIM) % 2 == 1){
	iwc->spa[(countB/IWC_NCALIM) % IWC_NSPA] = IWC_SPA_BIAS+IWC_SPA_POKE;
	countB++;
      }
      countA++;
    }
    else{
      //Turn off calibration
      printf("XIN: Stopping IWC calmode 2\n");
      calmode = 0;
      init = 0;
    }
    return calmode;
  }

  /* CALMODE 3: Set random pattern on IWC */
  if(calmode == 3){
    //Intializes random number generator
    srand((unsigned) time(&t));
    if(countA == 0){
      printf("XIN: Randomizing SPA\n");
      for(i=0;i<IWC_NSPA;i++)
	iwc->spa[i] = (uint16)(0.5*((rand() % 2*IWC_SPA_POKE) - IWC_SPA_POKE)) + IWC_SPA_BIAS;
    }
    countA++;
    return calmode;
  }

  /* CALMODE 4: Flight Simulator */
  if(calmode == 4){
    if(countF == 0)
      dt0 = dt;
    if(countF == 1)
      period = dt-dt0;
    //Set index
    index = (int)((dt-dt0)/zernike_timestep);
    //Get zernikes for this timestep
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      if(zuse[i])
	zernikes[i] = zernike_errors[i][index % ZERNIKE_ERRORS_NUMBER];
    //Convert to IWC DAC codes
    num_dgemv(zernike2spa, zernikes, spa, IWC_NSPA, LOWFS_N_ZERNIKE);
    //Add offsets to SPA position
    for(i=0;i<IWC_NSPA;i++)
      iwc->spa[i] += (uint16)spa[i];
    //Check if we are done
    if((index + (int)(1.1*period/zernike_timestep)) > ZERNIKE_ERRORS_NUMBER){
      //Turn off calibration
      printf("XIN: Stopping IWC calmode 4\n");
      calmode = 0;
      init = 0;
      return calmode;
    }
    countF++;
  }

  /* CALMODE 5: Hold current pattern on IWC */
  if(calmode == 5){
    for(i=0;i<IWC_NSPA;i++)
      iwc->spa[i] = iwc->spa[i];
  return calmode;
  }

    //Return calmode
  return calmode;
}



/**************************************************************/
/*                      IWC_CHECK                             */
/**************************************************************/
void iwc_check(iwc_t *iwc){
  int i;
  for(i=0;i<IWC_NSPA;i++){
    iwc->spa[i] = iwc->spa[i] > IWC_DMAX ? IWC_DMAX : iwc->spa[i];
    iwc->spa[i] = iwc->spa[i] < IWC_DMIN ? IWC_DMIN : iwc->spa[i];
  }
  for(i=0;i<IWC_NTTP;i++){
    iwc->ttp[i] = iwc->ttp[i] > IWC_DMAX ? IWC_DMAX : iwc->ttp[i];
    iwc->ttp[i] = iwc->ttp[i] < IWC_DMIN ? IWC_DMIN : iwc->ttp[i];
  }
}

/**************************************************************/
/*                      XIN_OPEN                              */
/**************************************************************/
int xin_open(void){
  signed long res;
  signed long listlen = 10;
  char uname[10];
  signed short hDevice;
  unsigned long error;
  int lastE;

  //Find Module
  res = QuickUsbFindModules(uname, listlen);
  if (res == 0){
    lastE = QuickUsbGetLastError(&error);
    printf("XIN: 'Find Module'  failed. Error #%lu\r\n", error);
    return -1;
  }else{
    printf("XIN: Device %s found.\r\n", uname);
  }

  //Open Device
  res = QuickUsbOpen(&hDevice, uname);
  if (res == 0){
    lastE = QuickUsbGetLastError(&error);
    printf("XIN: 'Open Device'  failed. Error #%lu\r\n", error);
    return -1;
  }else{
    printf("XIN: Device %s open.\r\n", uname);
  }

  //Return handle
  return hDevice;
}

/**************************************************************/
/*                      XIN_WRITEUSB                          */
/**************************************************************/
int xin_writeUsb(signed short hDevice, unsigned char* data, unsigned long length){
  signed long res;
  signed long lastE;
  unsigned long error;
  res = QuickUsbWriteData(hDevice, data, length);
  if (res == 0){
    lastE = QuickUsbGetLastError(&error);
    printf("XIN: 'Write Data'   failed. Error #%lu\r\n", error);
    return 1;
  }
  return 0;
}

/**************************************************************/
/*                      XIN_READUSB                           */
/**************************************************************/
int xin_readUsb(signed short hDevice, unsigned char* data, unsigned long length){
  signed long res;
  signed long lastE;
  unsigned long error;
  res = QuickUsbReadData(hDevice, data, &length);
  if (res == 0){
    lastE = QuickUsbGetLastError(&error);
    printf("XIN: 'Read Data'    failed. Error #%lu\r\n", error);
    return 1;
  }
  return 0;
}

/**************************************************************/
/*                      XIN_CLOSEDEV                          */
/**************************************************************/
int xin_closeDev(signed short hDevice){
  signed long res;
  signed long lastE;
  unsigned long error;
  res = QuickUsbClose(hDevice);
  if (res == 0){
    lastE = QuickUsbGetLastError(&error);
    printf("XIN: 'Close Device' failed. Error #%lu\r\n", error);
    return 1;
  }else{
    printf("XIN: Device closed.\r\n");
  }
  return 0;
}


/**************************************************************/
/*                      XIN_ZERO                              */
/**************************************************************/
int xin_zero(signed short hDevice){
  uint16 output[XIN_NCHANNELS]={0};
  if(xin_writeUsb(hDevice, (unsigned char *)output, sizeof(output))){
    printf("XIN: xin_writeUsb failed!\n");
    return 1;
  }
  return 0;
}

/**************************************************************/
/*                      XIN_WRITE                             */
/**************************************************************/
int xin_write(signed short hDevice, iwc_t *iwc, dm_t *dm, pez_t *pez){
  #include "iwc_map.h"
  int pez1_map[PEZ_NACT]={0};
  int pez2_map[PEZ_NACT]={0};
  int dm_map[DM_NACT]={0};
  uint16 output[XIN_NCHANNELS]={0};
  int i;

  if(hDevice >= 0){
#if IWC_ENABLE
    //Check IWC
    iwc_check(iwc);

    //Map IWC
    for(i=0;i<IWC_NSPA;i++)
      output[spa_map[i]] = iwc->spa[i];

    //Map TTP
    for(i=0;i<IWC_NTTP;i++)
      output[ttp_map[i]] = 0;//iwc->ttp[i]; set back when proper checks are in place
#endif

#if PEZ_ENABLE
    //Map Piezo Mirror #1
    for(i=0;i<PEZ_NACT;i++)
      output[pez1_map[i]] = pez->fm1[i];

    //Map Piezo Mirror #2
    for(i=0;i<PEZ_NACT;i++)
      output[pez2_map[i]] = pez->fm2[i];
#endif

#if DM_ENABLE
    //Map DM
    for(i=0;i<DM_NACT;i++)
      output[dm_map[i]] = dm->act[i];
#endif

    if(xin_writeUsb(hDevice, (unsigned char *)output, sizeof(output))){
      printf("XIN: xin_writeUsb failed!\n");
      return 1;
    }

  }
  return 0;
}
