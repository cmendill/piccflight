#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>

/* piccflight headers */
#include "handle_command.h"
#include "controller.h"

/* Report Fake Modes */
void report_fake_modes(void){
  printf("************************************************\n");
  printf("*                   FAKE MODES                 *\n");
  printf("************************************************\n");
  printf("0: Disabled\n");
  printf("1: TM test pattern\n");
  printf("2: Generate fake Shack-Hartmann images\n");
  printf("\n");
}

/* Handle user commands*/
int handle_command(char *line, sm_t *sm_p){
  float ftemp;
  int   itemp;
  char  stemp[CMD_MAX_LENGTH];

  /****************************************
   * SYSTEM COMMANDS
   ***************************************/
  //exit: exit watchdog
  if(!strncasecmp(line,"exit",4)){
    return(PICC_EXIT_WATCHDOG);
  }

  
}

