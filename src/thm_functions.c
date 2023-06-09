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
  float kP_elec=20;
  float kI_elec=1;
  float kD_elec=0;
  float intmax_elec=25;
  float kP_hex=10;
  float kI_hex=0.2;
  float kD_hex=0;
  float intmax_hex=50;
  float kP_opt=10;
  float kI_opt=0.2;
  float kD_opt=0;
  float intmax_opt=50;
  float kP_m2=10;
  float kI_m2=0.2;
  float kD_m2=0;
  float intmax_m2=50;
 
  //HTR0: HEX1
  if(h == 0){
    sprintf(htr->name,"HEX1");
    htr->adc      = 1;
    htr->ch       = 5;
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_hex;
    htr->kI       = kI_hex;
    htr->kD       = kD_hex;
    htr->intmax   = intmax_hex;
    htr->override = 0;
    htr->enable   = 1;
  }

  //HTR1: HEX2
  if(h == 1){
    sprintf(htr->name,"HEX2");
    htr->adc      = 1;
    htr->ch       = 9; 
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_hex;
    htr->kI       = kI_hex;
    htr->kD       = kD_hex;
    htr->intmax   = intmax_hex;
    htr->override = 0;
    htr->enable   = 1;
  }

  //HTR2: HEX3
  if(h == 2){
    sprintf(htr->name,"HEX3");
    htr->adc      = 1;
    htr->ch       = 11;
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_hex;
    htr->kI       = kI_hex;
    htr->kD       = kD_hex;
    htr->intmax   = intmax_hex;
    htr->override = 0;
    htr->enable   = 1;
  }

  //HTR3: HEX4
  if(h == 3){
    sprintf(htr->name,"HEX4");
    htr->adc      = 1;
    htr->ch       = 3; 
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_hex;
    htr->kI       = kI_hex;
    htr->kD       = kD_hex;
    htr->intmax   = intmax_hex;
    htr->override = 0;
    htr->enable   = 1;
  }

  //HTR4: HEX5
  if(h == 4){
    sprintf(htr->name,"HEX5");
    htr->adc      = 1;
    htr->ch       = 7; 
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_hex;
    htr->kI       = kI_hex;
    htr->kD       = kD_hex;
    htr->intmax   = intmax_hex;
    htr->override = 0;
    htr->enable   = 1;
  }

  //HTR5: HEX6
  if(h == 5){
    sprintf(htr->name,"HEX6");
    htr->adc      = 1;
    htr->ch       = 13;
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_hex;
    htr->kI       = kI_hex;
    htr->kD       = kD_hex;
    htr->intmax   = intmax_hex;
    htr->override = 0;
    htr->enable   = 1;
  }
  
  //HTR6: ALB1
  if(h == 6){
    sprintf(htr->name,"ALB1");
    htr->adc      = 3;
    htr->ch       = 27;
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_elec;
    htr->kI       = kI_elec;
    htr->kD       = kD_elec;
    htr->intmax   = intmax_elec;
    htr->override = 0;
    htr->enable   = 1;
  }

  //HTR7: ALB2
  if(h == 7){
    sprintf(htr->name,"ALB2");
    htr->adc      = 3;
    htr->ch       = 28;
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_elec;
    htr->kI       = kI_elec;
    htr->kD       = kD_elec;
    htr->intmax   = intmax_elec;
    htr->override = 0;
    htr->enable   = 1;
  }

  //HTR8: ALB3
  if(h == 8){
    sprintf(htr->name,"ALB3");
    htr->adc      = 3;
    htr->ch       = 29;
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_elec;
    htr->kI       = kI_elec;
    htr->kD       = kD_elec;
    htr->intmax   = intmax_elec;
    htr->override = 0;
    htr->enable   = 1;
  }

  //HTR9: ALB4
  if(h == 9){
    sprintf(htr->name,"ALB4");
    htr->adc      = 3;
    htr->ch       = 30;
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_elec;
    htr->kI       = kI_elec;
    htr->kD       = kD_elec;
    htr->intmax   = intmax_elec;
    htr->override = 0;
    htr->enable   = 1;
  }

  //HTR10: HXB1
  if(h == 10){
    sprintf(htr->name,"HXB1");
    htr->adc      = 3;
    htr->ch       = 31;
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_elec;
    htr->kI       = kI_elec;
    htr->kD       = kD_elec;
    htr->intmax   = intmax_elec;
    htr->override = 0;
    htr->enable   = 1;
  }


  //HTR11: M2GL
  if(h == 11){
    sprintf(htr->name,"M2GL");
    htr->adc      = 1;
    htr->ch       = 1;
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_m2;
    htr->kI       = kI_m2;
    htr->kD       = kD_m2;
    htr->intmax   = intmax_m2;
    htr->override = 0;
    htr->enable   = 1;
  }

  //HTR12: BMCC
  if(h == 12){
    sprintf(htr->name,"BMCC");
    htr->adc      = 1;
    htr->ch       = 0;
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_opt;
    htr->kI       = kI_opt;
    htr->kD       = kD_opt;
    htr->intmax   = intmax_opt;
    htr->override = 0;
    htr->enable   = 1;
  }

  //HTR13: POL1
  if(h == 13){
    sprintf(htr->name,"POL1");
    htr->adc      = 1;
    htr->ch       = 4;
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_opt;
    htr->kI       = kI_opt;
    htr->kD       = kD_opt;
    htr->intmax   = intmax_opt;
    htr->override = 0;
    htr->enable   = 1;
  }

  //HTR14: POL2
  if(h == 14){
    sprintf(htr->name,"POL2");
    htr->adc      = 1;
    htr->ch       = 6;
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_opt;
    htr->kI       = kI_opt;
    htr->kD       = kD_opt;
    htr->intmax   = intmax_opt;
    htr->override = 0;
    htr->enable   = 1;
  }

  //HTR15: ALPC
  if(h == 15){
    sprintf(htr->name,"ALPC");
    htr->adc      = 1;
    htr->ch       = 2;
    htr->maxpower = 50;
    htr->setpoint = 20;
    htr->deadband = 0;
    htr->gain     = 10;
    htr->usepid   = 1;
    htr->kP       = kP_opt;
    htr->kI       = kI_opt;
    htr->kD       = kD_opt;
    htr->intmax   = intmax_opt;
    htr->override = 0;
    htr->enable   = 1;
  }


  
}


