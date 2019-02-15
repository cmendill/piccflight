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
#include <math.h>
#include <sys/io.h>

/* piccflight headers */
#include "controller.h"


/**************************************************************/
/* THM_INIT_HEATER                                            */
/*  - Initialize heater settings                              */
/**************************************************************/
void thm_init_heater(int h, htr_t *htr){
  //Zero out heater structure
  memset(htr,0,sizeof(htr_t));

  
  //HTR0: 
  if(h == 0){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR1: 
  if(h == 1){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR2: 
  if(h == 2){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR3: 
  if(h == 3){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR4: 
  if(h == 4){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR5: 
  if(h == 5){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR6: 
  if(h == 6){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR7: 
  if(h == 7){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR8: 
  if(h == 8){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR9: 
  if(h == 9){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR10: 
  if(h == 10){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR11: 
  if(h == 11){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR12: 
  if(h == 12){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR13: 
  if(h == 13){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR14: 
  if(h == 14){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }

  //HTR15: 
  if(h == 15){
    htr->adc      = 0;
    htr->ch       = 0;
    htr->maxpower = 0;
    htr->setpoint = 0;
    htr->deadband = 0;
  }



  
}


