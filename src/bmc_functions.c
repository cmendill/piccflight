#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <time.h>
#include <libgen.h>
#include <sys/stat.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "numeric.h"
#include "bmc_functions.h"

/**************************************************************/
/* BMC_INIT_CALMODE                                           */
/*  - Initialize BMC calmode structure                        */
/**************************************************************/
void bmc_init_calmode(int calmode, calmode_t *bmc){
  //BMC_CALMODE_NONE
  if(calmode == BMC_CALMODE_NONE){
    sprintf(bmc->name,"BMC_CALMODE_NONE");
    sprintf(bmc->cmd,"none");
    bmc->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  }
}
