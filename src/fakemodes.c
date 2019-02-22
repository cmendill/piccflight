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
  //FAKEMODE_LYTPIX2ALPZER_REFIMG
  if(fakemode == FAKEMODE_LYTPIX2ALPZER_REFIMG){
    sprintf(fake->name,"FAKEMODE_LYTPIX2ALPZER_REFIMG");
    sprintf(fake->cmd,"lzref");
  }
}
