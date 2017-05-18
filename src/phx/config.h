#ifndef _CONFIG
  #define _CONFIG

  #include "include/phx_api.h"
  #include "bobcat.h"

  #define PHX_MAX_FILE_LENGTH 128
  typedef struct {
    ui32 dwBoardNumber;
    etParamValue eBoardNumber;
    etCamConfigLoad eCamConfigLoad;
    char bConfigFileName[PHX_MAX_FILE_LENGTH];
    char bOutputFileName[PHX_MAX_FILE_LENGTH];
    char *pszConfigFileName;
    char *pszOutputFileName;
    ui32 dwBayerOption;
    ui32 dwGridSize;
    ui32 dwGammaOption;
    ui32 dwThresholdOption;
    ui32 dwTrackOption;
    ui32 dwFrameOption;
    ui32 dwTimeOption;
    ui32 dwSlowOption;
    ui32 dwServerPort;
  } tPhxCmd;

  #define DEFAULT_CFG_FILENAME "Imperx_Bobcat_0620CL-8bit-1tap.pcf" /* Application constants */
  
  etStat CONFIG_ParseCmd( int, char*[], tPhxCmd* );

  int CONFIG_str_to_region(char**, region*);
  etStat CONFIG_RunFile(tHandle, char**);

#endif /* _CONFIG */
