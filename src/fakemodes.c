#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "controller.h"
#include "fakemodes.h"

/**************************************************************/
/* INIT_FAKEMODE                                           */
/*  - Initialize fakemode structure                        */
/**************************************************************/
void init_fakemode(int fakemode, calmode_t *fake){
  //FAKEMODE_NONE
  if(fakemode == FAKEMODE_NONE){
    sprintf(fake->name,"FAKEMODE_NONE");
    sprintf(fake->cmd,"none");
  }
  //FAKEMODE_GEN_IMAGE_TIMER_SYNC
  if(fakemode == FAKEMODE_GEN_IMAGE_TIMER_SYNC){
    sprintf(fake->name,"FAKEMODE_GEN_IMAGE_TIMER_SYNC");
    sprintf(fake->cmd,"gen image timer");
  }
  //FAKEMODE_READ_IMAGE_TIMER_SYNC
  if(fakemode == FAKEMODE_READ_IMAGE_TIMER_SYNC){
    sprintf(fake->name,"FAKEMODE_READ_IMAGE_TIMER_SYNC");
    sprintf(fake->cmd,"read image timer");
  }
  //FAKEMODE_GEN_IMAGE_CAMERA_SYNC
  if(fakemode == FAKEMODE_GEN_IMAGE_CAMERA_SYNC){
    sprintf(fake->name,"FAKEMODE_GEN_IMAGE_CAMERA_SYNC");
    sprintf(fake->cmd,"gen image camera");
  }
  //FAKEMODE_READ_IMAGE_CAMERA_SYNC
  if(fakemode == FAKEMODE_READ_IMAGE_CAMERA_SYNC){
    sprintf(fake->name,"FAKEMODE_READ_IMAGE_CAMERA_SYNC");
    sprintf(fake->cmd,"read image camera");
  }
  //FAKEMODE_TM_TEST_PATTERN
  if(fakemode == FAKEMODE_TM_TEST_PATTERN){
    sprintf(fake->name,"FAKEMODE_TM_TEST_PATTERN");
    sprintf(fake->cmd,"tm test pattern");
  }

}
