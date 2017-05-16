#include <phx_api.h> /* Main Phoenix library */
#include <config.h>
#include "phxshwfs.h"

int main(int argc, char *argv[]) {
  
  tPhxCmd sPhxCmd;
  int nStatus;

  CONFIG_ParseCmd( argc, argv, &sPhxCmd );
  /* PhxCommonKbInit();*/
  nStatus = phxshwfs( sPhxCmd.eBoardNumber, sPhxCmd.pszConfigFileName );
  /* PhxCommonKbClose();*/
  return nStatus;

}