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

  
  //HTR0: HEX1
  if(h == 0){
    sprintf(htr->name,"HEX1");
    htr->adc      = 1;
    htr->ch       = 1; //check
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }

  //HTR1: HEX2
  if(h == 1){
    sprintf(htr->name,"HEX2");
    htr->adc      = 1;
    htr->ch       = 3; //check
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }

  //HTR2: HEX3
  if(h == 2){
    sprintf(htr->name,"HEX3");
    htr->adc      = 1;
    htr->ch       = 5; //check
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }

  //HTR3: HEX4
  if(h == 3){
    sprintf(htr->name,"HEX4");
    htr->adc      = 1;
    htr->ch       = 7; //check
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }

  //HTR4: HEX5
  if(h == 4){
    sprintf(htr->name,"HEX5");
    htr->adc      = 1;
    htr->ch       = 9; //check
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }

  //HTR5: HEX6
  if(h == 5){
    sprintf(htr->name,"HEX6");
    htr->adc      = 1;
    htr->ch       = 11; //check
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }
  
  //HTR6: HXB1
  if(h == 6){
    sprintf(htr->name,"HXB1");
    htr->adc      = 3;
    htr->ch       = 31;
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }

  //HTR7: ALB1
  if(h == 7){
    sprintf(htr->name,"ALB1");
    htr->adc      = 3;
    htr->ch       = 27;
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }

  //HTR8: ALB2
  if(h == 8){
    sprintf(htr->name,"ALB2");
    htr->adc      = 3;
    htr->ch       = 28;
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }

  //HTR9: ALB3
  if(h == 9){
    sprintf(htr->name,"ALB3");
    htr->adc      = 3;
    htr->ch       = 29;
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }

  //HTR10: ALB4
  if(h == 10){
    sprintf(htr->name,"ALB4");
    htr->adc      = 3;
    htr->ch       = 30;
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }


  //HTR11: ALPC
  if(h == 11){
    sprintf(htr->name,"ALPC");
    htr->adc      = 1;
    htr->ch       = 2;
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }

  //HTR12: BMCC
  if(h == 12){
    sprintf(htr->name,"BMCC");
    htr->adc      = 1;
    htr->ch       = 0;
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }

  //HTR13: POL1
  if(h == 13){
    sprintf(htr->name,"POL1");
    htr->adc      = 1;
    htr->ch       = 4;
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }

  //HTR14: POL2
  if(h == 14){
    sprintf(htr->name,"POL2");
    htr->adc      = 1;
    htr->ch       = 6;
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }

  //HTR15: 5CCB
  if(h == 15){
    sprintf(htr->name,"5CCB");
    htr->adc      = 1;
    htr->ch       = 8;
    htr->maxpower = 50;
    htr->setpoint = 25;
    htr->deadband = 1;
  }

  
}


