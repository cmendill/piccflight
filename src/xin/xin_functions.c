#include <QuickUSB.h>
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

/**************************************************************/
/*                      IWC_CALIBRATE                         */
/**************************************************************/
void iwc_calibrate(uint16 calmode, iwc_t *iwc){
  int i;
  static unsigned long int count=0;
  
  //Flip through calibration modes
  if(calmode == 1){
    //set all SPA actuators to bias
    for(i=0;i<IWC_NSPA;i++)
      iwc->spa[i]=IWC_SPA_BIAS;
    //poke one actuator
    iwc->spa[(count/10) % IWC_NSPA] = IWC_SPA_BIAS+IWC_SPA_POKE;
    count++;
    return;
  }
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
