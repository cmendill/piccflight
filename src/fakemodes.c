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
  //FAKEMODE_TEST_PATTERN
  if(fakemode == FAKEMODE_TEST_PATTERN){
    sprintf(fake->name,"FAKEMODE_TEST_PATTERN");
    sprintf(fake->cmd,"pattern");
  }
  //FAKEMODE_LYT_REFIMG
  if(fakemode == FAKEMODE_LYT_REFIMG){
    sprintf(fake->name,"FAKEMODE_LYT_REFIMG");
    sprintf(fake->cmd,"lytref");
  }
  //FAKEMODE_TEST_PATTERN2
  if(fakemode == FAKEMODE_TEST_PATTERN2){
    sprintf(fake->name,"FAKEMODE_TEST_PATTERN2");
    sprintf(fake->cmd,"pat2tern");
  }
  //FAKEMODE_TEST_PATTERN3
  if(fakemode == FAKEMODE_TEST_PATTERN3){
    sprintf(fake->name,"FAKEMODE_TEST_PATTERN3");
    sprintf(fake->cmd,"pat3tern");
  }
  //FAKEMODE_SCI_MASK
  if(fakemode == FAKEMODE_SCI_MASK){
    sprintf(fake->name,"FAKEMODE_SCI_MASK");
    sprintf(fake->cmd,"scimask");
  }
}
