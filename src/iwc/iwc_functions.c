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
  static int count=0;
  
  //Flip through calibration modes
  if(calmode == 0)
    return;

  if(calmode == 1){
    //set all SPA actuators to bias
    for(i=0;i<IWC_NSPA;i++)
      iwc->spa[i]=IWC_SPA_BIAS;
    //poke one actuator
    iwc->spa[count/10 % IWC_NSPA] = IWC_SPA_BIAS+IWC_SPA_POKE;

    return;
  }
  
  
  
}
