#include <string.h>

#include "config.h"
#include "phoenix.h"
#include "bobcat.h"
#include "phoenix_bobcat.h"

//#define _VERBOSE

/* PhxCommonParseCmd(ForBrief)
 * adapted from active silicons comand parser
 * Parse the command line parameters, and place results in a common structure
 * The command line parameters take the following form:
 * AppName -b<BoardNumber> -c<ConfigFileName> -o<OutputFileName>
 * -b<BoardNumber>    is an optional parameter which specifies which board to use.
 *                    The default value is board 1.
 * -c<ConfigFileName> is an optional parameter specifying the Phoenix Config File,
 *                    The default value is an OS specific path to "default.pcf" which
 *                    is in the root directory of the example suite.
 * -O<OutputFileName> is an optional parameter specifiying the root name of an output file.
 *                    The default setting is NULL, indicating no output file.
 * Whilst all parameters may be specified, each example application only uses appropriate
 * parameters, for example "OutputFileName" will be ignored by the phxinfo example.
 */
etStat CONFIG_ParseCmd( int argc, char* argv[], tPhxCmd* ptPhxCmd ) {
  etStat eStat = PHX_OK;

  /* Initialise the PhxCmd structure with default values */
  ptPhxCmd->dwBoardNumber = 1;
  ptPhxCmd->pszConfigFileName = NULL;
  ptPhxCmd->pszOutputFileName = NULL;
  ptPhxCmd->dwBayerOption = 11;
  ptPhxCmd->dwGridSize = 66;
  ptPhxCmd->dwGammaOption = 100;
  ptPhxCmd->dwFrameOption = 0;
  ptPhxCmd->dwTimeOption = 5;
  ptPhxCmd->dwThresholdOption = 40;
  ptPhxCmd->dwTrackOption = 50;
  ptPhxCmd->dwSlowOption = 10;
  ptPhxCmd->pszConfigFileName = DEFAULT_CFG_FILENAME;
  ptPhxCmd->dwServerPort = 8000;

  /* The first argument is always the function name itself */
  printf("\n*** %s ***\n", *argv );
  argc--;
  argv++;

  /* Parse the command options */
  while ( argc > 0 ) {
    if ( '-' == **argv ) {
      switch ( (int) *(*argv+1) ) {

        /* Board number */
        case 'b' : case 'B' :
          ptPhxCmd->dwBoardNumber = atoi(*argv+2);
          break;

        /* Config File */
        case 'c' : case 'C' :
          strcpy( ptPhxCmd->bConfigFileName, *argv+2 );
          if ( 0 != strlen(ptPhxCmd->bConfigFileName) )
            ptPhxCmd->pszConfigFileName = ptPhxCmd->bConfigFileName;
          else
            ptPhxCmd->pszConfigFileName = NULL;
          break;

        /* griD size */
        case 'd' : case 'D' :
          ptPhxCmd->dwGridSize = atoi(*argv+2);
          break;

        /* Gamma correction */
        case 'g' : case 'G' :
          ptPhxCmd->dwGammaOption = atoi(*argv+2);
          break;

        /* Threshold */
        case 'h' : case 'H' :
          ptPhxCmd->dwThresholdOption = atoi(*argv+2);
          break;

        /* Track */
        case 'k' : case 'K' :
          ptPhxCmd->dwTrackOption = atoi(*argv+2);
          break;

        /* Output File */
        case 'o' : case 'O' :
          strcpy( ptPhxCmd->bOutputFileName, *argv+2 );
          if ( 0 != strlen(ptPhxCmd->bOutputFileName) )
            ptPhxCmd->pszOutputFileName = ptPhxCmd->bOutputFileName;
          else
            ptPhxCmd->pszOutputFileName = NULL;
          break;

        /* baYer conversion */
        case 'y' : case 'Y' :
          ptPhxCmd->dwBayerOption = atoi(*argv+2);
          break;

        /* Frames */
        case 'f' : case 'F' :
          ptPhxCmd->dwFrameOption = atoi(*argv+2);
          break;

        /* Seconds */
        case 't' : case 'T' :
          ptPhxCmd->dwTimeOption = atoi(*argv+2);
          break;

        /* Slow motion subsample */
        case 's' : case 'S' :
          ptPhxCmd->dwSlowOption = atoi(*argv+2);
          break;

        /* Slow motion subsample */
        case 'p' : case 'P' :
          ptPhxCmd->dwServerPort = atoi(*argv+2);
          break;

        default :
          printf("Unrecognised parameter %c - Ignoring\n", *(*argv+1) );
          break;

      }
    }
    argc--;
    argv++;
  }

  printf("Using BoardNumber = %d\n",  ptPhxCmd->dwBoardNumber );
  printf("      Config File = ");
  if ( NULL == ptPhxCmd->pszConfigFileName ) printf("<None>\n"); else printf("%s\n", ptPhxCmd->pszConfigFileName );
  printf("      Output File = ");
  if ( NULL == ptPhxCmd->pszOutputFileName ) printf("<None>\n"); else printf("%s\n", ptPhxCmd->pszOutputFileName );
  printf("\n");

  /* Create an eCamConfigLoad parameter by OR'ing the board number with the board type */
  ptPhxCmd->eCamConfigLoad = (etCamConfigLoad) ( PHX_DIGITAL /*| PHX_NO_RECONFIGURE*/ | ptPhxCmd->dwBoardNumber );

  switch( ptPhxCmd->dwBoardNumber ) {
    case 0: ptPhxCmd->eBoardNumber = PHX_BOARD_NUMBER_AUTO; break;
    case 1: ptPhxCmd->eBoardNumber = PHX_BOARD_NUMBER_1; break;
    case 2: ptPhxCmd->eBoardNumber = PHX_BOARD_NUMBER_2; break;
    case 3: ptPhxCmd->eBoardNumber = PHX_BOARD_NUMBER_3; break;
    case 4: ptPhxCmd->eBoardNumber = PHX_BOARD_NUMBER_4; break;
    case 5: ptPhxCmd->eBoardNumber = PHX_BOARD_NUMBER_5; break;
    case 6: ptPhxCmd->eBoardNumber = PHX_BOARD_NUMBER_6; break;
    case 7: ptPhxCmd->eBoardNumber = PHX_BOARD_NUMBER_7; break;
    default: eStat = PHX_ERROR_BAD_PARAM_VALUE; goto Error; break;
  }

  Error:
  return eStat;
}


int CONFIG_str_to_region(char** str, region* proi) {
  char *token;
  char comma[2] = ",";
  token = strtok(*str, comma);
  if (token != NULL) {
    proi->x_offset = atol(token);
    token = strtok(NULL, comma);
  } else {
    return 0;
  }
  if (token != NULL) {
    proi->y_offset = atol(token);
    token = strtok(NULL, comma);
  } else {
    return 0;
  }
  if (token != NULL) {
    proi->x_length = atol(token);
    token = strtok(NULL, comma);
  } else {
    return 0;
  }
  if (token != NULL) {
    proi->y_length = atol(token);
    token = strtok(NULL, comma);
  } else {
    return 0;
  }
  if (token != NULL) {
    BOBCAT_str_to_bobcatParamValue(&token,&(proi->x_binning));
    token = strtok(NULL, comma);
  } else {
    return 0;
  }
  if (token != NULL) {
    BOBCAT_str_to_bobcatParamValue(&token,&(proi->y_binning));
  } else {
    return 0;
  }
  return 1;
}


etStat CONFIG_RunFile(tHandle handle, char** pszConfigFileName) {
  etStat eStat = PHX_OK;

  FILE* fp;
  char strLine[PHX_CONFIG_MAX_LINE];

  char delimit[] = "= \t\r\n\v\f";
  char fphx=0, fbobcat=0, fsystem=0;
  char strPhoenix[] = "[phoenix]";
  char strSystem[] = "[system]";
  char strBobcat[] = "[bobcat]";
  char *token, *strParam, *strParamValue;

  eStat = BOBCAT_LoadFromFactory(handle);
  printf("PHX: Opening config: %s\n",*pszConfigFileName);

  fp = fopen(*pszConfigFileName, "r");
  if(fp==NULL){
    printf("Config file\r\n");
    perror("fopen()");
  }else{
    printf("PHX: config file opened.\r\n");
  }

  while(fgets(strLine, PHX_CONFIG_MAX_LINE, (FILE*) fp)) {
    if(strstr(strLine, strPhoenix) != NULL) {
      #ifdef _VERBOSE
        printf("PHX: phoenix block found\n");
      #endif
      fphx=1; fbobcat=0; fsystem=0;
    } else if(strstr(strLine, strSystem) != NULL) {
      #ifdef _VERBOSE
        printf("PHX: system block found\n");
      #endif
      fphx=0; fbobcat=0; fsystem=1;
    } else if(strstr(strLine, strBobcat) != NULL) {
      #ifdef _VERBOSE
        printf("PHX: bobcat block found\n");
      #endif
      fphx=0; fbobcat=1; fsystem=0;
    } else if (strLine[0]=='\n') {
      #ifdef _VERBOSE
        printf("PHX: run PHX_ParameterSet( handle, (etParam)( PHX_DUMMY_PARAM | PHX_CACHE_FLUSH | PHX_FORCE_REWRITE ), NULL )\n");
      #endif
        eStat = PHX_ParameterSet( handle, (etParam)( PHX_DUMMY_PARAM | PHX_CACHE_FLUSH | PHX_FORCE_REWRITE ), NULL );
    } else {

      token = strtok(strLine, delimit);
      strParam = malloc(sizeof(char)*strlen(token)+1);
      strcpy(strParam, token);
      token = strtok(NULL, delimit);
      strParamValue = malloc(sizeof(char)*strlen(token)+1);
      strcpy(strParamValue, token);

      if (fsystem) {
        phxbobcatParam pbParam;
        if (PHX_BOBCAT_str_to_phxbobcatParam(&strParam, &pbParam)<0) {
          eStat = PHX_ERROR_BAD_PARAM;
        } else {
          if (pbParam==PHX_BOBCAT_ROI) {
            region roi;
            if (CONFIG_str_to_region(&strParamValue, &roi)<0) {
              eStat = PHX_ERROR_BAD_PARAM_VALUE;
            } else {
              #ifdef _VERBOSE
                printf("PHX: run eStat = PHX_BOBCAT_Configure(handle, %s, roi = [%d, %d, %d, %d, %s, %s])\n", strParam, roi.x_offset, roi.y_offset, roi.x_length, roi.y_length, roi.x_binning==BOBCAT_BINNING_1X?"BOBCAT_BINNING_1X":"er", roi.y_binning==BOBCAT_BINNING_1X?"BOBCAT_BINNING_1X":"er");
              #endif
                eStat = PHX_BOBCAT_Configure(handle, pbParam, &roi);
            }
          } else {
            phxbobcatParamValue pbParamValue;
            if (PHX_BOBCAT_str_to_phxbobcatParamValue(&strParamValue, &pbParamValue)<0) {
              eStat = PHX_ERROR_BAD_PARAM_VALUE;
            } else {
              #ifdef _VERBOSE
                printf("PHX: run eStat = PHX_BOBCAT_Configure(handle, %s, %s)\n", strParam, strParamValue);
              #endif
                eStat = PHX_BOBCAT_Configure(handle, pbParam, &pbParamValue);
            }
          }
        }
      } else if (fbobcat) {
        bobcatParam bParam;
        bobcatParamValue bParamValue;
        if (BOBCAT_str_to_bobcatParam(&strParam, &bParam)<0) {
          eStat = PHX_ERROR_BAD_PARAM;
        } else {
          if (BOBCAT_str_to_bobcatParamValues(&strParamValue, &bParamValue)<0) {
            eStat = PHX_ERROR_BAD_PARAM_VALUE;
          } else{
            #ifdef _VERBOSE
              printf("PHX: run eStat = BOBCAT_ParameterSet(handle, %s, %s)\n",strParam,strParamValue);
            #endif
            eStat = BOBCAT_ParameterSet(handle, bParam, &bParamValue);
          }
        }
      } else if (fphx) {
        etParam pParam;
        etParamValue pParamValue;
        if (PHX_str_to_etParam(&strParam, &pParam)<0) {
          eStat = PHX_ERROR_BAD_PARAM;
        } else {
          if (PHX_str_to_etParamValues(&strParamValue, &pParamValue)<0) {
            eStat = PHX_ERROR_BAD_PARAM_VALUE;
          } else {
            #ifdef _VERBOSE
              printf("PHX: run eStat = PHX_ParameterSet(handle, %s, %s)\n",strParam,strParamValue);
            #endif
            eStat = PHX_ParameterSet(handle, pParam, &pParamValue);
          }
        }
      }

      free(strParam);
      free(strParamValue);

    }
  }
  if (feof(fp)) {
    #ifdef _VERBOSE
      printf("PHX: run eStat = PHX_ParameterSet( handle, (etParam)( PHX_DUMMY_PARAM | PHX_CACHE_FLUSH | PHX_FORCE_REWRITE ), NULL )\n");
    #endif
    eStat = PHX_ParameterSet( handle, (etParam)( PHX_DUMMY_PARAM | PHX_CACHE_FLUSH | PHX_FORCE_REWRITE ), NULL );
  }

  fclose(fp);
  #ifdef _VERBOSE
    printf("PHX: config done. Returning %d\n",eStat);
  #endif
  return eStat;
}
