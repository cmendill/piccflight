#include <phx_api.h> /* Main Phoenix library */
#include <config.h>

#include "phxlyotwfs.h"

/* VxWorks requires a unique function name for each application */
int main( int argc, char* argv[] ) {

  tPhxCmd sPhxCmd;
  int nStatus;

  CONFIG_ParseCmd( argc, argv, &sPhxCmd );
  /* PhxCommonKbInit();*/
  nStatus = phxlyotwfs( sPhxCmd.eBoardNumber, sPhxCmd.pszConfigFileName );
  /* PhxCommonKbClose();*/
  return nStatus;

}
