#include <phx_api.h>    /* Main Phoenix library */
#include "phx_bobcat.h"

//#define _VERBOSE
#define MAX_BUFFER_LENGTH 256

etStat BOBCAT_ControlRead(tHandle hCamera, ui8* msgBuffer, ui8* msgLength) {
  etStat eStat = PHX_OK;
  etParamValue eParamValue;
  
  eStat = PHX_ParameterGet( hCamera, PHX_COMMS_INCOMING, (void*)msgLength ); /* Check how many characters are waiting to be read */
  if ( PHX_OK != eStat ) return eStat;
  
  if ( msgLength != 0 ) {
    eParamValue = *msgLength;
    eStat = PHX_ControlRead( hCamera, PHX_COMMS_PORT, NULL, msgBuffer, (ui32*)&eParamValue, 500 );
    if ( eStat != PHX_OK ) return eStat;
    if ( eParamValue == *msgLength ) {
#if defined _VERBOSE
      int x;
      printf("rx : ");
      for ( x = 0; x < *msgLength; x++ ) printf("[%02X]", msgBuffer[x]);
      printf("\n");
#endif
      return PHX_OK;
    } else {
      return PHX_ERROR_NOT_IMPLEMENTED; /*Failed to read all characters on comm port read buffer*/
    }
  } else {
    return PHX_ERROR_NOT_IMPLEMENTED; /*camera is not returning any characters*/
  }

}


etStat BOBCAT_ControlWrite(tHandle hCamera, ui8* msgBuffer, ui8 msgLength) {
  etStat eStat = PHX_OK;
  etParamValue eParamValue = msgLength;

  eStat = PHX_ControlWrite( hCamera, PHX_COMMS_PORT, NULL, msgBuffer, (ui32*)&eParamValue, 500 );
  if ( eStat != PHX_OK ) return eStat;

  if (eParamValue == msgLength) {
#if defined _VERBOSE
    int x;
    printf("tx : ");
    for (x = 0; x < msgLength; x++) printf("[%02X]", msgBuffer[x]);
    printf("\n");
#endif
    return PHX_OK;
  } else {
    return PHX_ERROR_NOT_IMPLEMENTED; /*Failed to write all characters to comm port*/
  }
  
}


#define READ_CMD 0x52
#define WRITE_CMD 0x57

etStat BOBCAT_ParameterGet(tHandle hCamera, bobcatParam parameter, void* value) {
  etStat eStat = PHX_OK;
  ui8 rxMsgBuffer[MAX_BUFFER_LENGTH];
  ui8 rxMsgLength;
  ui8 txMsgBuffer[3];
  ui8 txMsgLength=3;

  //Setup command
  txMsgBuffer[0] = READ_CMD;
  txMsgBuffer[1] = ((ui8*)&parameter)[1];
  txMsgBuffer[2] = ((ui8*)&parameter)[0];
  
  eStat = BOBCAT_ControlWrite(hCamera, txMsgBuffer, txMsgLength);
  if ( PHX_OK != eStat ) goto Return;
  
  _PHX_SleepMs(100);
  
  eStat = BOBCAT_ControlRead(hCamera, rxMsgBuffer, &rxMsgLength);
  if ( PHX_OK != eStat ) goto Return;

  if (rxMsgBuffer[0] == 0x06){
    *(ui32*)value = (rxMsgBuffer[1]<<24)+(rxMsgBuffer[2]<<16)+(rxMsgBuffer[3]<<8)+(rxMsgBuffer[4]); /* all good */
  } else if (rxMsgBuffer[0] == 0x15) { /* camera returns an error code */
    int x;
    printf("Camera error code returned for command : BOBCAT_ParameterGet\n");
    for ( x = 0; x < txMsgLength; x++ ) printf("[%02X]", txMsgBuffer[x]);
    printf("\n");
    printf("address  0x%02X%02X\n", txMsgBuffer[1], txMsgBuffer[2]);
    printf("data     0x%02X%02X%02X%02X (%d)\n", rxMsgBuffer[3], rxMsgBuffer[4], rxMsgBuffer[5], rxMsgBuffer[6], *(ui32*)value);
    switch (rxMsgBuffer[1]) {
      case 0: printf("No error\n"); break;
      case 1: printf("Invalid command\n"); break;
      case 2: printf("Time-out\n"); break;
      case 3: printf("Checksum error\n"); break;
      case 4: printf("Value less then minimum\n"); break;
      case 5: printf("Value higher than maximum\n"); break;
      case 6: printf("AGC error\n"); break;
      case 7: printf("Supervisor mode error\n"); break;
      case 8: printf("Mode not supported error\n"); break;
    }
    eStat = PHX_ERROR_NOT_IMPLEMENTED;
    goto Return;
  } else {
    eStat = PHX_ERROR_NOT_IMPLEMENTED; 
    goto Return;
  }

  Return:
  return eStat;
}


etStat BOBCAT_ParameterSet(tHandle hCamera, bobcatParam parameter, void* value) {
  ui8 rxMsgBuffer[MAX_BUFFER_LENGTH];
  ui8 rxMsgLength;
  ui8 txMsgBuffer[7];
  ui8 txMsgLength = 7;
  etStat eStat = PHX_OK;

  //Setup command
  txMsgBuffer[0] = WRITE_CMD;
  txMsgBuffer[1] = ((ui8*)&parameter)[1];
  txMsgBuffer[2] = ((ui8*)&parameter)[0];
  txMsgBuffer[3] = ((ui8*)value)[3];
  txMsgBuffer[4] = ((ui8*)value)[2];
  txMsgBuffer[5] = ((ui8*)value)[1];
  txMsgBuffer[6] = ((ui8*)value)[0];
  
  eStat = BOBCAT_ControlWrite(hCamera, txMsgBuffer, txMsgLength);
  if ( PHX_OK != eStat ) goto Return;
  
  _PHX_SleepMs(100);
  
  eStat = BOBCAT_ControlRead(hCamera, rxMsgBuffer, &rxMsgLength);
  if ( PHX_OK != eStat ) goto Return;

  if (rxMsgBuffer[0] == 0x6){ /* all good */
  } else if (rxMsgBuffer[0] == 0x15) { /* camera returns an error code */
    int x;
    printf("Camera error code returned for command : BOBCAT_ParameterSet \n");
    for ( x = 0; x < txMsgLength; x++ ) printf("[%02X]", txMsgBuffer[x]);
    printf("\n");
    printf("address  0x%02X%02X\n", txMsgBuffer[1], txMsgBuffer[2]);
    printf("data     0x%02X%02X%02X%02X (%d)\n", txMsgBuffer[3], txMsgBuffer[4], txMsgBuffer[5], txMsgBuffer[6], *(ui32*)value);
    switch (rxMsgBuffer[1]) {
      case 0: printf("No error\n"); break;
      case 1: printf("Invalid command\n"); break;
      case 2: printf("Time-out\n"); break;
      case 3: printf("Checksum error\n"); break;
      case 4: printf("Value less then minimum\n"); break;
      case 5: printf("Value higher than maximum\n"); break;
      case 6: printf("AGC error\n"); break;
      case 7: printf("Supervisor mode error\n"); break;
      case 8: printf("Mode not supported error\n"); break;
    }
    eStat = PHX_ERROR_NOT_IMPLEMENTED;
    goto Return;
  } else { /* camera returns an error coder */
    eStat = PHX_ERROR_NOT_IMPLEMENTED;
    goto Return;
  }

  Return:
  return eStat;
}


etStat BOBCAT_SoftReset(tHandle hCamera) {
  bobcatParamValue value = 0xDEDBEEF;
  bobcatParam parameter = BOBCAT_SOFT_RESET;
  return BOBCAT_ParameterSet( hCamera, parameter, &value );
}


etStat BOBCAT_LoadFromFactory(tHandle hCamera) {
  bobcatParamValue value = 0x1;
  bobcatParam parameter = BOBCAT_LOAD_FROM_FACTORY;
  return BOBCAT_ParameterSet( hCamera, parameter, &value );
}


etStat BOBCAT_LoadFromUser1(tHandle hCamera) {
  bobcatParamValue value = 0x1;
  bobcatParam parameter = BOBCAT_LOAD_FROM_USR1;
  return BOBCAT_ParameterSet( hCamera, parameter, &value );
}


etStat BOBCAT_LoadFromUser2(tHandle hCamera) {
  bobcatParamValue value = 0x1;
  bobcatParam parameter = BOBCAT_LOAD_FROM_USR2;
  return BOBCAT_ParameterSet( hCamera, parameter, &value );
}


etStat BOBCAT_SaveToUser1(tHandle hCamera) {
  bobcatParamValue value = 0x1;
  bobcatParam parameter = BOBCAT_SAVE_TO_USR1;
  return BOBCAT_ParameterSet( hCamera, parameter, &value );
}


etStat BOBCAT_SaveToUser2(tHandle hCamera) {
  bobcatParamValue value = 0x1;
  bobcatParam parameter = BOBCAT_SAVE_TO_USR2;
  return BOBCAT_ParameterSet( hCamera, parameter, &value );
}


etStat BOBCAT_SoftwareTriggerStart(tHandle hCamera) {
  bobcatParamValue value = 0x1;
  bobcatParam parameter = BOBCAT_SW_TRIG_START;
  return BOBCAT_ParameterSet( hCamera, parameter, &value );
}


int BOBCAT_str_to_bobcatParam(char* str, bobcatParam* pbParam) {
  if (strcmp(str, "BOBCAT_INVALID_PARAM")==0) {
    *pbParam = BOBCAT_INVALID_PARAM;
  } else if (strcmp(str, "BOBCAT_INFO_TEST_REGISTER")==0) {
    *pbParam = BOBCAT_INFO_TEST_REGISTER;
  } else if (strcmp(str, "BOBCAT_SOFT_RESET")==0) {
    *pbParam = BOBCAT_SOFT_RESET;
  } else if (strcmp(str, "BOBCAT_INFO_FRM_NUM")==0) {
    *pbParam = BOBCAT_INFO_FRM_NUM;
  } else if (strcmp(str, "BOBCAT_BOOT_FROM")==0) {
    *pbParam = BOBCAT_BOOT_FROM;
  } else if (strcmp(str, "BOBCAT_LOAD_FROM_FACTORY")==0) {
    *pbParam = BOBCAT_LOAD_FROM_FACTORY;
  } else if (strcmp(str, "BOBCAT_LOAD_FROM_USR1")==0) {
    *pbParam = BOBCAT_LOAD_FROM_USR1;
  } else if (strcmp(str, "BOBCAT_LOAD_FROM_USR2")==0) {
    *pbParam = BOBCAT_LOAD_FROM_USR2;
  } else if (strcmp(str, "BOBCAT_LOAD_MFG_DEFAULT_GAIN")==0) {
    *pbParam = BOBCAT_LOAD_MFG_DEFAULT_GAIN;
  } else if (strcmp(str, "BOBCAT_SAVE_TO_USR1")==0) {
    *pbParam = BOBCAT_SAVE_TO_USR1;
  } else if (strcmp(str, "BOBCAT_SAVE_TO_USR2")==0) {
    *pbParam = BOBCAT_SAVE_TO_USR2;
  } else if (strcmp(str, "BOBCAT_BAUD_RATE_SELECT")==0) {
    *pbParam = BOBCAT_BAUD_RATE_SELECT;
  } else if (strcmp(str, "BOBCAT_MFG_FW_REV")==0) {
    *pbParam = BOBCAT_MFG_FW_REV;
  } else if (strcmp(str, "BOBCAT_MFG_FW_BUILD")==0) {
    *pbParam = BOBCAT_MFG_FW_BUILD;
  } else if (strcmp(str, "BOBCAT_MFG_PART_NUM0")==0) {
    *pbParam = BOBCAT_MFG_PART_NUM0;
  } else if (strcmp(str, "BOBCAT_MFG_PART_NUM1")==0) {
    *pbParam = BOBCAT_MFG_PART_NUM1;
  } else if (strcmp(str, "BOBCAT_MFG_PART_NUM2")==0) {
    *pbParam = BOBCAT_MFG_PART_NUM2;
  } else if (strcmp(str, "BOBCAT_MFG_PART_NUM3")==0) {
    *pbParam = BOBCAT_MFG_PART_NUM3;
  } else if (strcmp(str, "BOBCAT_MFG_CAM_SN0")==0) {
    *pbParam = BOBCAT_MFG_CAM_SN0;
  } else if (strcmp(str, "BOBCAT_MFG_CAM_SN1")==0) {
    *pbParam = BOBCAT_MFG_CAM_SN1;
  } else if (strcmp(str, "BOBCAT_MFG_CCD_SN0")==0) {
    *pbParam = BOBCAT_MFG_CCD_SN0;
  } else if (strcmp(str, "BOBCAT_MFG_CCD_SN1")==0) {
    *pbParam = BOBCAT_MFG_CCD_SN1;
  } else if (strcmp(str, "BOBCAT_MFG_DATE0")==0) {
    *pbParam = BOBCAT_MFG_DATE0;
  } else if (strcmp(str, "BOBCAT_MFG_DATE1")==0) {
    *pbParam = BOBCAT_MFG_DATE1;
  } else if (strcmp(str, "BOBCAT_MFG_CAM_TYPE0")==0) {
    *pbParam = BOBCAT_MFG_CAM_TYPE0;
  } else if (strcmp(str, "BOBCAT_MFG_CAM_TYPE1")==0) {
    *pbParam = BOBCAT_MFG_CAM_TYPE1;
  } else if (strcmp(str, "BOBCAT_MFG_CAM_TYPE2")==0) {
    *pbParam = BOBCAT_MFG_CAM_TYPE2;
  } else if (strcmp(str, "BOBCAT_MFG_CAM_TYPE3")==0) {
    *pbParam = BOBCAT_MFG_CAM_TYPE3;
  } else if (strcmp(str, "BOBCAT_INFO_MIN_MAX_XLENGTHS")==0) {
    *pbParam = BOBCAT_INFO_MIN_MAX_XLENGTHS;
  } else if (strcmp(str, "BOBCAT_INFO_MIN_MAX_YLENGTHS")==0) {
    *pbParam = BOBCAT_INFO_MIN_MAX_YLENGTHS;
  } else if (strcmp(str, "BOBCAT_INFO_XYLENGTHS")==0) {
    *pbParam = BOBCAT_INFO_XYLENGTHS;
  } else if (strcmp(str, "BOBCAT_INFO_MIN_FRM_TIME")==0) {
    *pbParam = BOBCAT_INFO_MIN_FRM_TIME;
  } else if (strcmp(str, "BOBCAT_INFO_FRM_TIME")==0) {
    *pbParam = BOBCAT_INFO_FRM_TIME;
  } else if (strcmp(str, "BOBCAT_INFO_MIN_LN_TIME")==0) {
    *pbParam = BOBCAT_INFO_MIN_LN_TIME;
  } else if (strcmp(str, "BOBCAT_INFO_MIN_EXP_TIME")==0) {
    *pbParam = BOBCAT_INFO_MIN_EXP_TIME;
  } else if (strcmp(str, "BOBCAT_INFO_EXP_TIME")==0) {
    *pbParam = BOBCAT_INFO_EXP_TIME;
  } else if (strcmp(str, "BOBCAT_INFO_MAX_EXP_TIME")==0) {
    *pbParam = BOBCAT_INFO_MAX_EXP_TIME;
  } else if (strcmp(str, "BOBCAT_INFO_AEC_AGC_STATUS")==0) {
    *pbParam = BOBCAT_INFO_AEC_AGC_STATUS;
  } else if (strcmp(str, "BOBCAT_INFO_CCD_TEMP")==0) {
    *pbParam = BOBCAT_INFO_CCD_TEMP;
  } else if (strcmp(str, "BOBCAT_SCAN_MODE")==0) {
    *pbParam = BOBCAT_SCAN_MODE;
  } else if (strcmp(str, "BOBCAT_YBINNING")==0) {
    *pbParam = BOBCAT_YBINNING;
  } else if (strcmp(str, "BOBCAT_XBINNING")==0) {
    *pbParam = BOBCAT_XBINNING;
  } else if (strcmp(str, "BOBCAT_MAOI_ENABLE")==0) {
    *pbParam = BOBCAT_MAOI_ENABLE;
  } else if (strcmp(str, "BOBCAT_AOI1_ENABLE")==0) {
    *pbParam = BOBCAT_AOI1_ENABLE;
  } else if (strcmp(str, "BOBCAT_AOI2_ENABLE")==0) {
    *pbParam = BOBCAT_AOI2_ENABLE;
  } else if (strcmp(str, "BOBCAT_AOI3_ENABLE")==0) {
    *pbParam = BOBCAT_AOI3_ENABLE;
  } else if (strcmp(str, "BOBCAT_AOI4_ENABLE")==0) {
    *pbParam = BOBCAT_AOI4_ENABLE;
  } else if (strcmp(str, "BOBCAT_AOI5_ENABLE")==0) {
    *pbParam = BOBCAT_AOI5_ENABLE;
  } else if (strcmp(str, "BOBCAT_AOI6_ENABLE")==0) {
    *pbParam = BOBCAT_AOI6_ENABLE;
  } else if (strcmp(str, "BOBCAT_MAOI_XLENGTH")==0) {
    *pbParam = BOBCAT_MAOI_XLENGTH;
  } else if (strcmp(str, "BOBCAT_AOI1_XLENGTH")==0) {
    *pbParam = BOBCAT_AOI1_XLENGTH;
  } else if (strcmp(str, "BOBCAT_AOI2_XLENGTH")==0) {
    *pbParam = BOBCAT_AOI2_XLENGTH;
  } else if (strcmp(str, "BOBCAT_AOI3_XLENGTH")==0) {
    *pbParam = BOBCAT_AOI3_XLENGTH;
  } else if (strcmp(str, "BOBCAT_AOI4_XLENGTH")==0) {
    *pbParam = BOBCAT_AOI4_XLENGTH;
  } else if (strcmp(str, "BOBCAT_AOI5_XLENGTH")==0) {
    *pbParam = BOBCAT_AOI5_XLENGTH;
  } else if (strcmp(str, "BOBCAT_AOI6_XLENGTH")==0) {
    *pbParam = BOBCAT_AOI6_XLENGTH;
  } else if (strcmp(str, "BOBCAT_MAOI_XOFFSET")==0) {
    *pbParam = BOBCAT_MAOI_XOFFSET;
  } else if (strcmp(str, "BOBCAT_AOI1_XOFFSET")==0) {
    *pbParam = BOBCAT_AOI1_XOFFSET;
  } else if (strcmp(str, "BOBCAT_AOI2_XOFFSET")==0) {
    *pbParam = BOBCAT_AOI2_XOFFSET;
  } else if (strcmp(str, "BOBCAT_AOI3_XOFFSET")==0) {
    *pbParam = BOBCAT_AOI3_XOFFSET;
  } else if (strcmp(str, "BOBCAT_AOI4_XOFFSET")==0) {
    *pbParam = BOBCAT_AOI4_XOFFSET;
  } else if (strcmp(str, "BOBCAT_AOI5_XOFFSET")==0) {
    *pbParam = BOBCAT_AOI5_XOFFSET;
  } else if (strcmp(str, "BOBCAT_AOI6_XOFFSET")==0) {
    *pbParam = BOBCAT_AOI6_XOFFSET;
  } else if (strcmp(str, "BOBCAT_MAOI_YLENGTH")==0) {
    *pbParam = BOBCAT_MAOI_YLENGTH;
  } else if (strcmp(str, "BOBCAT_AOI1_YLENGTH")==0) {
    *pbParam = BOBCAT_AOI1_YLENGTH;
  } else if (strcmp(str, "BOBCAT_AOI2_YLENGTH")==0) {
    *pbParam = BOBCAT_AOI2_YLENGTH;
  } else if (strcmp(str, "BOBCAT_AOI3_YLENGTH")==0) {
    *pbParam = BOBCAT_AOI3_YLENGTH;
  } else if (strcmp(str, "BOBCAT_AOI4_YLENGTH")==0) {
    *pbParam = BOBCAT_AOI4_YLENGTH;
  } else if (strcmp(str, "BOBCAT_AOI5_YLENGTH")==0) {
    *pbParam = BOBCAT_AOI5_YLENGTH;
  } else if (strcmp(str, "BOBCAT_AOI6_YLENGTH")==0) {
    *pbParam = BOBCAT_AOI6_YLENGTH;
  } else if (strcmp(str, "BOBCAT_MAOI_YOFFSET")==0) {
    *pbParam = BOBCAT_MAOI_YOFFSET;
  } else if (strcmp(str, "BOBCAT_AOI1_YOFFSET")==0) {
    *pbParam = BOBCAT_AOI1_YOFFSET;
  } else if (strcmp(str, "BOBCAT_AOI2_YOFFSET")==0) {
    *pbParam = BOBCAT_AOI2_YOFFSET;
  } else if (strcmp(str, "BOBCAT_AOI3_YOFFSET")==0) {
    *pbParam = BOBCAT_AOI3_YOFFSET;
  } else if (strcmp(str, "BOBCAT_AOI4_YOFFSET")==0) {
    *pbParam = BOBCAT_AOI4_YOFFSET;
  } else if (strcmp(str, "BOBCAT_AOI5_YOFFSET")==0) {
    *pbParam = BOBCAT_AOI5_YOFFSET;
  } else if (strcmp(str, "BOBCAT_AOI6_YOFFSET")==0) {
    *pbParam = BOBCAT_AOI6_YOFFSET;
  } else if (strcmp(str, "BOBCAT_CONST_FRM_RATE_ENABLE")==0) {
    *pbParam = BOBCAT_CONST_FRM_RATE_ENABLE;
  } else if (strcmp(str, "BOBCAT_CAM_LVAL_SIZE_MODE")==0) {
    *pbParam = BOBCAT_CAM_LVAL_SIZE_MODE;
  } else if (strcmp(str, "BOBCAT_CAM_FVAL_SIZE_MODE")==0) {
    *pbParam = BOBCAT_CAM_FVAL_SIZE_MODE;
  } else if (strcmp(str, "BOBCAT_EXP_CTRL_MODE")==0) {
    *pbParam = BOBCAT_EXP_CTRL_MODE;
  } else if (strcmp(str, "BOBCAT_EXP_TIME")==0) {
    *pbParam = BOBCAT_EXP_TIME;
  } else if (strcmp(str, "BOBCAT_PROG_LN_TIME_ENABLE")==0) {
    *pbParam = BOBCAT_PROG_LN_TIME_ENABLE;
  } else if (strcmp(str, "BOBCAT_PROG_FRM_TIME_ENABLE")==0) {
    *pbParam = BOBCAT_PROG_FRM_TIME_ENABLE;
  } else if (strcmp(str, "BOBCAT_LN_TIME")==0) {
    *pbParam = BOBCAT_LN_TIME;
  } else if (strcmp(str, "BOBCAT_FRM_TIME")==0) {
    *pbParam = BOBCAT_FRM_TIME;
  } else if (strcmp(str, "BOBCAT_SPEED")==0) {
    *pbParam = BOBCAT_SPEED;
  } else if (strcmp(str, "BOBCAT_EXP_CTRL_AUTO")==0) {
    *pbParam = BOBCAT_EXP_CTRL_AUTO;
  } else if (strcmp(str, "BOBCAT_EXP_MAX_LIMIT")==0) {
    *pbParam = BOBCAT_EXP_MAX_LIMIT;
  } else if (strcmp(str, "BOBCAT_EXP_CORR_SPEED")==0) {
    *pbParam = BOBCAT_EXP_CORR_SPEED;
  } else if (strcmp(str, "BOBCAT_AUTO_GAIN_CTRL")==0) {
    *pbParam = BOBCAT_AUTO_GAIN_CTRL;
  } else if (strcmp(str, "BOBCAT_GAIN_MAX_LIMIT")==0) {
    *pbParam = BOBCAT_GAIN_MAX_LIMIT;
  } else if (strcmp(str, "BOBCAT_GAIN_CORR_SPEED")==0) {
    *pbParam = BOBCAT_GAIN_CORR_SPEED;
  } else if (strcmp(str, "BOBCAT_LUMINANCE_LVL_THRES")==0) {
    *pbParam = BOBCAT_LUMINANCE_LVL_THRES;
  } else if (strcmp(str, "BOBCAT_LUMINANCE_TYPE")==0) {
    *pbParam = BOBCAT_LUMINANCE_TYPE;
  } else if (strcmp(str, "BOBCAT_AOI_XOFFSET")==0) {
    *pbParam = BOBCAT_AOI_XOFFSET;
  } else if (strcmp(str, "BOBCAT_AOI_XLENGTH")==0) {
    *pbParam = BOBCAT_AOI_XLENGTH;
  } else if (strcmp(str, "BOBCAT_AOI_YOFFSET")==0) {
    *pbParam = BOBCAT_AOI_YOFFSET;
  } else if (strcmp(str, "BOBCAT_AOI_YLENGTH")==0) {
    *pbParam = BOBCAT_AOI_YLENGTH;
  } else if (strcmp(str, "BOBCAT_AUTO_IRIS_CTRL")==0) {
    *pbParam = BOBCAT_AUTO_IRIS_CTRL;
  } else if (strcmp(str, "BOBCAT_PRE_AMP_GAIN_CH1")==0) {
    *pbParam = BOBCAT_PRE_AMP_GAIN_CH1;
  } else if (strcmp(str, "BOBCAT_ANALOG_GAIN_CH1")==0) {
    *pbParam = BOBCAT_ANALOG_GAIN_CH1;
  } else if (strcmp(str, "BOBCAT_ANALOG_OFFSET_CH1")==0) {
    *pbParam = BOBCAT_ANALOG_OFFSET_CH1;
  } else if (strcmp(str, "BOBCAT_PRE_AMP_GAIN_CH2")==0) {
    *pbParam = BOBCAT_PRE_AMP_GAIN_CH2;
  } else if (strcmp(str, "BOBCAT_ANALOG_GAIN_CH2")==0) {
    *pbParam = BOBCAT_ANALOG_GAIN_CH2;
  } else if (strcmp(str, "BOBCAT_ANALOG_OFFSET_CH2")==0) {
    *pbParam = BOBCAT_ANALOG_OFFSET_CH2;
  } else if (strcmp(str, "BOBCAT_PRE_AMP_GAIN_CH3")==0) {
    *pbParam = BOBCAT_PRE_AMP_GAIN_CH3;
  } else if (strcmp(str, "BOBCAT_ANALOG_GAIN_CH3")==0) {
    *pbParam = BOBCAT_ANALOG_GAIN_CH3;
  } else if (strcmp(str, "BOBCAT_ANALOG_OFFSET_CH3")==0) {
    *pbParam = BOBCAT_ANALOG_OFFSET_CH3;
  } else if (strcmp(str, "BOBCAT_PRE_AMP_GAIN_CH4")==0) {
    *pbParam = BOBCAT_PRE_AMP_GAIN_CH4;
  } else if (strcmp(str, "BOBCAT_ANALOG_GAIN_CH4")==0) {
    *pbParam = BOBCAT_ANALOG_GAIN_CH4;
  } else if (strcmp(str, "BOBCAT_ANALOG_OFFSET_CH4")==0) {
    *pbParam = BOBCAT_ANALOG_OFFSET_CH4;
  } else if (strcmp(str, "BOBCAT_DIGITAL_GAIN")==0) {
    *pbParam = BOBCAT_DIGITAL_GAIN;
  } else if (strcmp(str, "BOBCAT_DIGITAL_OFFSET")==0) {
    *pbParam = BOBCAT_DIGITAL_OFFSET;
  } else if (strcmp(str, "BOBCAT_BLCK_LVL_CORR")==0) {
    *pbParam = BOBCAT_BLCK_LVL_CORR;
  } else if (strcmp(str, "BOBCAT_TAP_BALANCE")==0) {
    *pbParam = BOBCAT_TAP_BALANCE;
  } else if (strcmp(str, "BOBCAT_TRIG_INPUT_SELECT")==0) {
    *pbParam = BOBCAT_TRIG_INPUT_SELECT;
  } else if (strcmp(str, "BOBCAT_TRIG_INPUT_MODE")==0) {
    *pbParam = BOBCAT_TRIG_INPUT_MODE;
  } else if (strcmp(str, "BOBCAT_SW_TRIG_START")==0) {
    *pbParam = BOBCAT_SW_TRIG_START;
  } else if (strcmp(str, "BOBCAT_TRIG_EDGE_SELECT")==0) {
    *pbParam = BOBCAT_TRIG_EDGE_SELECT;
  } else if (strcmp(str, "BOBCAT_DEBOUNCE_TIME_SELECT")==0) {
    *pbParam = BOBCAT_DEBOUNCE_TIME_SELECT;
  } else if (strcmp(str, "BOBCAT_TRIG_OVERLAP")==0) {
    *pbParam = BOBCAT_TRIG_OVERLAP;
  } else if (strcmp(str, "BOBCAT_TRIG_MODE_SELECT")==0) {
    *pbParam = BOBCAT_TRIG_MODE_SELECT;
  } else if (strcmp(str, "BOBCAT_NUM_OF_FRMS_CAPTURED")==0) {
    *pbParam = BOBCAT_NUM_OF_FRMS_CAPTURED;
  } else if (strcmp(str, "BOBCAT_NUM_OF_PULSES_USED")==0) {
    *pbParam = BOBCAT_NUM_OF_PULSES_USED;
  } else if (strcmp(str, "BOBCAT_TRIG_EXP_DELAY")==0) {
    *pbParam = BOBCAT_TRIG_EXP_DELAY;
  } else if (strcmp(str, "BOBCAT_TRIG_STROBE_ENABLE")==0) {
    *pbParam = BOBCAT_TRIG_STROBE_ENABLE;
  } else if (strcmp(str, "BOBCAT_TRIG_STROBE_DELAY")==0) {
    *pbParam = BOBCAT_TRIG_STROBE_DELAY;
  } else if (strcmp(str, "BOBCAT_TRIG_STROBE_DURATION")==0) {
    *pbParam = BOBCAT_TRIG_STROBE_DURATION;
  } else if (strcmp(str, "BOBCAT_PULSE_GEN_GRANULARITY")==0) {
    *pbParam = BOBCAT_PULSE_GEN_GRANULARITY;
  } else if (strcmp(str, "BOBCAT_PULSE_GEN_PULSE_WIDTH")==0) {
    *pbParam = BOBCAT_PULSE_GEN_PULSE_WIDTH;
  } else if (strcmp(str, "BOBCAT_PULSE_GEN_PULSE_PERIOD")==0) {
    *pbParam = BOBCAT_PULSE_GEN_PULSE_PERIOD;
  } else if (strcmp(str, "BOBCAT_PULSE_GEN_NO_OF_PULSE")==0) {
    *pbParam = BOBCAT_PULSE_GEN_NO_OF_PULSE;
  } else if (strcmp(str, "BOBCAT_PULSE_GEN_ENABLE")==0) {
    *pbParam = BOBCAT_PULSE_GEN_ENABLE;
  } else if (strcmp(str, "BOBCAT_TEST_MODE_SELECT")==0) {
    *pbParam = BOBCAT_TEST_MODE_SELECT;
  } else if (strcmp(str, "BOBCAT_H_V_LNS_OVERLAY")==0) {
    *pbParam = BOBCAT_H_V_LNS_OVERLAY;
  } else if (strcmp(str, "BOBCAT_CROSS_OVERLAY")==0) {
    *pbParam = BOBCAT_CROSS_OVERLAY;
  } else if (strcmp(str, "BOBCAT_H1_OVERLAY_LN_POSITION")==0) {
    *pbParam = BOBCAT_H1_OVERLAY_LN_POSITION;
  } else if (strcmp(str, "BOBCAT_H2_OVERLAY_LN_POSITION")==0) {
    *pbParam = BOBCAT_H2_OVERLAY_LN_POSITION;
  } else if (strcmp(str, "BOBCAT_V1_OVERLAY_LN_POSITION")==0) {
    *pbParam = BOBCAT_V1_OVERLAY_LN_POSITION;
  } else if (strcmp(str, "BOBCAT_V2_OVERLAY_LN_POSITION")==0) {
    *pbParam = BOBCAT_V2_OVERLAY_LN_POSITION;
  } else if (strcmp(str, "BOBCAT_OVERLAY_LNS_BRIGHTNESS")==0) {
    *pbParam = BOBCAT_OVERLAY_LNS_BRIGHTNESS;
  } else if (strcmp(str, "BOBCAT_CC1_POLARITY_SELECT")==0) {
    *pbParam = BOBCAT_CC1_POLARITY_SELECT;
  } else if (strcmp(str, "BOBCAT_CC1_INPUT_SELECT")==0) {
    *pbParam = BOBCAT_CC1_INPUT_SELECT;
  } else if (strcmp(str, "BOBCAT_CC2_POLARITY_SELECT")==0) {
    *pbParam = BOBCAT_CC2_POLARITY_SELECT;
  } else if (strcmp(str, "BOBCAT_CC2_INPUT_SELECT")==0) {
    *pbParam = BOBCAT_CC2_INPUT_SELECT;
  } else if (strcmp(str, "BOBCAT_IN1_POLARITY_SELECT")==0) {
    *pbParam = BOBCAT_IN1_POLARITY_SELECT;
  } else if (strcmp(str, "BOBCAT_IN1_INPUT_SELECT")==0) {
    *pbParam = BOBCAT_IN1_INPUT_SELECT;
  } else if (strcmp(str, "BOBCAT_IN2_POLARITY_SELECT")==0) {
    *pbParam = BOBCAT_IN2_POLARITY_SELECT;
  } else if (strcmp(str, "BOBCAT_IN2_INPUT_SELECT")==0) {
    *pbParam = BOBCAT_IN2_INPUT_SELECT;
  } else if (strcmp(str, "BOBCAT_OUT1_POLARITY_SELECT")==0) {
    *pbParam = BOBCAT_OUT1_POLARITY_SELECT;
  } else if (strcmp(str, "BOBCAT_OUT1_OUTPUT_SELECT")==0) {
    *pbParam = BOBCAT_OUT1_OUTPUT_SELECT;
  } else if (strcmp(str, "BOBCAT_OUT2_POLARITY_SELECT")==0) {
    *pbParam = BOBCAT_OUT2_POLARITY_SELECT;
  } else if (strcmp(str, "BOBCAT_OUT2_OUTPUT_SELECT")==0) {
    *pbParam = BOBCAT_OUT2_OUTPUT_SELECT;
  } else if (strcmp(str, "BOBCAT_CLSP_POLARITY_SELECT")==0) {
    *pbParam = BOBCAT_CLSP_POLARITY_SELECT;
  } else if (strcmp(str, "BOBCAT_CLSP_OUTPUT_SELECT")==0) {
    *pbParam = BOBCAT_CLSP_OUTPUT_SELECT;
  } else if (strcmp(str, "BOBCAT_STROBE1_MODE_SELECT")==0) {
    *pbParam = BOBCAT_STROBE1_MODE_SELECT;
  } else if (strcmp(str, "BOBCAT_STROBE2_MODE_SELECT")==0) {
    *pbParam = BOBCAT_STROBE2_MODE_SELECT;
  } else if (strcmp(str, "BOBCAT_STROBE1_DURATION")==0) {
    *pbParam = BOBCAT_STROBE1_DURATION;
  } else if (strcmp(str, "BOBCAT_STROBE2_DURATION")==0) {
    *pbParam = BOBCAT_STROBE2_DURATION;
  } else if (strcmp(str, "BOBCAT_STROBE1_POSITION")==0) {
    *pbParam = BOBCAT_STROBE1_POSITION;
  } else if (strcmp(str, "BOBCAT_STROBE2_POSITION")==0) {
    *pbParam = BOBCAT_STROBE2_POSITION;
  } else if (strcmp(str, "BOBCAT_BIT_DEPTH_SELECT")==0) {
    *pbParam = BOBCAT_BIT_DEPTH_SELECT;
  } else if (strcmp(str, "BOBCAT_BIT_SHIFT_SELECT")==0) {
    *pbParam = BOBCAT_BIT_SHIFT_SELECT;
  } else if (strcmp(str, "BOBCAT_TAP_MODE_SELECT")==0) {
    *pbParam = BOBCAT_TAP_MODE_SELECT;
  } else if (strcmp(str, "BOBCAT_DATA_FORMAT_SELECT")==0) {
    *pbParam = BOBCAT_DATA_FORMAT_SELECT;
  } else if (strcmp(str, "BOBCAT_NEG_IMG")==0) {
    *pbParam = BOBCAT_NEG_IMG;
  } else if (strcmp(str, "BOBCAT_WB_MODE")==0) {
    *pbParam = BOBCAT_WB_MODE;
  } else if (strcmp(str, "BOBCAT_WBC_RED")==0) {
    *pbParam = BOBCAT_WBC_RED;
  } else if (strcmp(str, "BOBCAT_WBC_GREEN")==0) {
    *pbParam = BOBCAT_WBC_GREEN;
  } else if (strcmp(str, "BOBCAT_WBC_BLUE")==0) {
    *pbParam = BOBCAT_WBC_BLUE;
  } else if (strcmp(str, "BOBCAT_GAIN_RED")==0) {
    *pbParam = BOBCAT_GAIN_RED;
  } else if (strcmp(str, "BOBCAT_GAIN_GREEN")==0) {
    *pbParam = BOBCAT_GAIN_GREEN;
  } else if (strcmp(str, "BOBCAT_GAIN_BLUE")==0) {
    *pbParam = BOBCAT_GAIN_BLUE;
  } else if (strcmp(str, "BOBCAT_OFFSET_RED")==0) {
    *pbParam = BOBCAT_OFFSET_RED;
  } else if (strcmp(str, "BOBCAT_OFFSET_GREEN")==0) {
    *pbParam = BOBCAT_OFFSET_GREEN;
  } else if (strcmp(str, "BOBCAT_OFFSET_BLUE")==0) {
    *pbParam = BOBCAT_OFFSET_BLUE;
  } else if (strcmp(str, "BOBCAT_LUT_SELECT")==0) {
    *pbParam = BOBCAT_LUT_SELECT;
  } else if (strcmp(str, "BOBCAT_LUT_ENABLE")==0) {
    *pbParam = BOBCAT_LUT_ENABLE;
  } else if (strcmp(str, "BOBCAT_DEFECTIVE_PIXEL_CORR")==0) {
    *pbParam = BOBCAT_DEFECTIVE_PIXEL_CORR;
  } else if (strcmp(str, "BOBCAT_HOT_PIXEL_CORR")==0) {
    *pbParam = BOBCAT_HOT_PIXEL_CORR;
  } else if (strcmp(str, "BOBCAT_FLAT_FIELD_CORR")==0) {
    *pbParam = BOBCAT_FLAT_FIELD_CORR;
  } else if (strcmp(str, "BOBCAT_ENHANCEMENT_MODE")==0) {
    *pbParam = BOBCAT_ENHANCEMENT_MODE;
  } else if (strcmp(str, "BOBCAT_POINT_X1_POSITION")==0) {
    *pbParam = BOBCAT_POINT_X1_POSITION;
  } else if (strcmp(str, "BOBCAT_POINT_X2_POSITION")==0) {
    *pbParam = BOBCAT_POINT_X2_POSITION;
  } else if (strcmp(str, "BOBCAT_POINT_Y1_POSITION")==0) {
    *pbParam = BOBCAT_POINT_Y1_POSITION;
  } else if (strcmp(str, "BOBCAT_POINT_Y2_POSITION")==0) {
    *pbParam = BOBCAT_POINT_Y2_POSITION;
  } else if (strcmp(str, "BOBCAT_PAOI_ENABLE")==0) {
    *pbParam = BOBCAT_PAOI_ENABLE;
  } else if (strcmp(str, "BOBCAT_PAOI_XLENGTH")==0) {
    *pbParam = BOBCAT_PAOI_XLENGTH;
  } else if (strcmp(str, "BOBCAT_PAOI_YLENGTH")==0) {
    *pbParam = BOBCAT_PAOI_YLENGTH;
  } else if (strcmp(str, "BOBCAT_PAOI_XOFFSET")==0) {
    *pbParam = BOBCAT_PAOI_XOFFSET;
  } else if (strcmp(str, "BOBCAT_PAOI_YOFFSET")==0) {
    *pbParam = BOBCAT_PAOI_YOFFSET;
  } else {
    return 0;
  }
  return 1;
}


int BOBCAT_str_to_bobcatParamValue(char* str, bobcatParamValue* pbParamValue) {
  if (strcmp(str, "BOBCAT_INVALID_PARAMVALUE")==0) {
    *pbParamValue = BOBCAT_INVALID_PARAMVALUE;
  } else if (strcmp(str, "BOBCAT_DISABLE")==0) {
    *pbParamValue = BOBCAT_DISABLE;
  } else if (strcmp(str, "BOBCAT_ENABLE")==0) {
    *pbParamValue = BOBCAT_ENABLE;
  } else if (strcmp(str, "BOBCAT_BOOT_FROM_FACTORY")==0) {
    *pbParamValue = BOBCAT_BOOT_FROM_FACTORY;
  } else if (strcmp(str, "BOBCAT_BOOT_FROM_USER1")==0) {
    *pbParamValue = BOBCAT_BOOT_FROM_USER1;
  } else if (strcmp(str, "BOBCAT_BOOT_FROM_USER2")==0) {
    *pbParamValue = BOBCAT_BOOT_FROM_USER2;
  } else if (strcmp(str, "BOBCAT_BAUD_RATE_9600")==0) {
    *pbParamValue = BOBCAT_BAUD_RATE_9600;
  } else if (strcmp(str, "BOBCAT_BAUD_RATE_19200")==0) {
    *pbParamValue = BOBCAT_BAUD_RATE_19200;
  } else if (strcmp(str, "BOBCAT_BAUD_RATE_38400")==0) {
    *pbParamValue = BOBCAT_BAUD_RATE_38400;
  } else if (strcmp(str, "BOBCAT_BAUD_RATE_57600")==0) {
    *pbParamValue = BOBCAT_BAUD_RATE_57600;
  } else if (strcmp(str, "BOBCAT_BAUD_RATE_115200")==0) {
    *pbParamValue = BOBCAT_BAUD_RATE_115200;
  } else if (strcmp(str, "BOBCAT_SCAN_MODE_NORMAL")==0) {
    *pbParamValue = BOBCAT_SCAN_MODE_NORMAL;
  } else if (strcmp(str, "BOBCAT_SCAN_MODE_CENTER")==0) {
    *pbParamValue = BOBCAT_SCAN_MODE_CENTER;
  } else if (strcmp(str, "BOBCAT_BINNING_1X")==0) {
    *pbParamValue = BOBCAT_BINNING_1X;
  } else if (strcmp(str, "BOBCAT_BINNING_2X")==0) {
    *pbParamValue = BOBCAT_BINNING_2X;
  } else if (strcmp(str, "BOBCAT_BINNING_3X")==0) {
    *pbParamValue = BOBCAT_BINNING_3X;
  } else if (strcmp(str, "BOBCAT_BINNING_4X")==0) {
    *pbParamValue = BOBCAT_BINNING_4X;
  } else if (strcmp(str, "BOBCAT_BINNING_8X")==0) {
    *pbParamValue = BOBCAT_BINNING_8X;
  } else if (strcmp(str, "BOBCAT_INCLUDE")==0) {
    *pbParamValue = BOBCAT_INCLUDE;
  } else if (strcmp(str, "BOBCAT_EXCLUDE")==0) {
    *pbParamValue = BOBCAT_EXCLUDE;
  } else if (strcmp(str, "BOBCAT_ACTIVE_ONLY")==0) {
    *pbParamValue = BOBCAT_ACTIVE_ONLY;
  } else if (strcmp(str, "BOBCAT_ALL_VISIBLE")==0) {
    *pbParamValue = BOBCAT_ALL_VISIBLE;
  } else if (strcmp(str, "BOBCAT_EXP_CTRL_PW")==0) {
    *pbParamValue = BOBCAT_EXP_CTRL_PW;
  } else if (strcmp(str, "BOBCAT_EXP_CTRL_INT")==0) {
    *pbParamValue = BOBCAT_EXP_CTRL_INT;
  } else if (strcmp(str, "BOBCAT_EXP_CTRL_EXT")==0) {
    *pbParamValue = BOBCAT_EXP_CTRL_EXT;
  } else if (strcmp(str, "BOBCAT_SPEED_NORM")==0) {
    *pbParamValue = BOBCAT_SPEED_NORM;
  } else if (strcmp(str, "BOBCAT_SPEED_FAST")==0) {
    *pbParamValue = BOBCAT_SPEED_FAST;
  } else if (strcmp(str, "BOBCAT_SPEED_1X")==0) {
    *pbParamValue = BOBCAT_SPEED_1X;
  } else if (strcmp(str, "BOBCAT_SPEED_2X")==0) {
    *pbParamValue = BOBCAT_SPEED_2X;
  } else if (strcmp(str, "BOBCAT_SPEED_3X")==0) {
    *pbParamValue = BOBCAT_SPEED_3X;
  } else if (strcmp(str, "BOBCAT_SPEED_4X")==0) {
    *pbParamValue = BOBCAT_SPEED_4X;
  } else if (strcmp(str, "BOBCAT_LUMINANCE_AVG")==0) {
    *pbParamValue = BOBCAT_LUMINANCE_AVG;
  } else if (strcmp(str, "BOBCAT_LUMINANCE_PK")==0) {
    *pbParamValue = BOBCAT_LUMINANCE_PK;
  } else if (strcmp(str, "BOBCAT_GAIN_NEG3DB")==0) {
    *pbParamValue = BOBCAT_GAIN_NEG3DB;
  } else if (strcmp(str, "BOBCAT_GAIN_0DB")==0) {
    *pbParamValue = BOBCAT_GAIN_0DB;
  } else if (strcmp(str, "BOBCAT_GAIN_POS3DB")==0) {
    *pbParamValue = BOBCAT_GAIN_POS3DB;
  } else if (strcmp(str, "BOBCAT_GAIN_POS6DB")==0) {
    *pbParamValue = BOBCAT_GAIN_POS6DB;
  } else if (strcmp(str, "BOBCAT_DYN_TAP_BALANCE_AUTO")==0) {
    *pbParamValue = BOBCAT_DYN_TAP_BALANCE_AUTO;
  } else if (strcmp(str, "BOBCAT_DYN_BALANCE_ONCE")==0) {
    *pbParamValue = BOBCAT_DYN_BALANCE_ONCE;
  } else if (strcmp(str, "BOBCAT_STAT_TAP_BALANCE")==0) {
    *pbParamValue = BOBCAT_STAT_TAP_BALANCE;
  } else if (strcmp(str, "BOBCAT_TRIG_EXT")==0) {
    *pbParamValue = BOBCAT_TRIG_EXT;
  } else if (strcmp(str, "BOBCAT_TRIG_PROG")==0) {
    *pbParamValue = BOBCAT_TRIG_PROG;
  } else if (strcmp(str, "BOBCAT_TRIG_COMP")==0) {
    *pbParamValue = BOBCAT_TRIG_COMP;
  } else if (strcmp(str, "BOBCAT_TRIG_SW")==0) {
    *pbParamValue = BOBCAT_TRIG_SW;
  } else if (strcmp(str, "BOBCAT_EDGE_RISING")==0) {
    *pbParamValue = BOBCAT_EDGE_RISING;
  } else if (strcmp(str, "BOBCAT_EDGE_FALLING")==0) {
    *pbParamValue = BOBCAT_EDGE_FALLING;
  } else if (strcmp(str, "BOBCAT_DEBOUNCE_100US")==0) {
    *pbParamValue = BOBCAT_DEBOUNCE_100US;
  } else if (strcmp(str, "BOBCAT_DEBOUNCE_1000US")==0) {
    *pbParamValue = BOBCAT_DEBOUNCE_1000US;
  } else if (strcmp(str, "BOBCAT_DEBOUNCE_10000US")==0) {
    *pbParamValue = BOBCAT_DEBOUNCE_10000US;
  } else if (strcmp(str, "BOBCAT_DEBOUNCE_10US")==0) {
    *pbParamValue = BOBCAT_DEBOUNCE_10US;
  } else if (strcmp(str, "BOBCAT_DEBOUNCE_50US")==0) {
    *pbParamValue = BOBCAT_DEBOUNCE_50US;
  } else if (strcmp(str, "BOBCAT_DEBOUNCE_500US")==0) {
    *pbParamValue = BOBCAT_DEBOUNCE_500US;
  } else if (strcmp(str, "BOBCAT_DEBOUNCE_5000US")==0) {
    *pbParamValue = BOBCAT_DEBOUNCE_5000US;
  } else if (strcmp(str, "BOBCAT_IGNORE_NEXT_TRIG")==0) {
    *pbParamValue = BOBCAT_IGNORE_NEXT_TRIG;
  } else if (strcmp(str, "BOBCAT_ACCEPT_AFTER_COMPLETED")==0) {
    *pbParamValue = BOBCAT_ACCEPT_AFTER_COMPLETED;
  } else if (strcmp(str, "BOBCAT_ACCEPT_ANY_TIME")==0) {
    *pbParamValue = BOBCAT_ACCEPT_ANY_TIME;
  } else if (strcmp(str, "BOBCAT_TRIG_STD")==0) {
    *pbParamValue = BOBCAT_TRIG_STD;
  } else if (strcmp(str, "BOBCAT_TRIG_FAST")==0) {
    *pbParamValue = BOBCAT_TRIG_FAST;
  } else if (strcmp(str, "BOBCAT_TRIG_DBL")==0) {
    *pbParamValue = BOBCAT_TRIG_DBL;
  } else if (strcmp(str, "BOBCAT_TRIG_FRM_ACCUM")==0) {
    *pbParamValue = BOBCAT_TRIG_FRM_ACCUM;
  } else if (strcmp(str, "BOBCAT_TRIG_ASYNC")==0) {
    *pbParamValue = BOBCAT_TRIG_ASYNC;
  } else if (strcmp(str, "BOBCAT_TRIG_STROBE1")==0) {
    *pbParamValue = BOBCAT_TRIG_STROBE1;
  } else if (strcmp(str, "BOBCAT_TRIG_STROBE2")==0) {
    *pbParamValue = BOBCAT_TRIG_STROBE2;
  } else if (strcmp(str, "BOBCAT_PULSE_GEN_GRANULAR_1X")==0) {
    *pbParamValue = BOBCAT_PULSE_GEN_GRANULAR_1X;
  } else if (strcmp(str, "BOBCAT_PULSE_GEN_GRANULAR_10X")==0) {
    *pbParamValue = BOBCAT_PULSE_GEN_GRANULAR_10X;
  } else if (strcmp(str, "BOBCAT_PULSE_GEN_GRANULAR_100X")==0) {
    *pbParamValue = BOBCAT_PULSE_GEN_GRANULAR_100X;
  } else if (strcmp(str, "BOBCAT_PULSE_GEN_GRANULAR_1000X")==0) {
    *pbParamValue = BOBCAT_PULSE_GEN_GRANULAR_1000X;
  } else if (strcmp(str, "BOBCAT_TEST_IMG_BLACK")==0) {
    *pbParamValue = BOBCAT_TEST_IMG_BLACK;
  } else if (strcmp(str, "BOBCAT_TEST_IMG_GRAY")==0) {
    *pbParamValue = BOBCAT_TEST_IMG_GRAY;
  } else if (strcmp(str, "BOBCAT_TEST_IMG_WHITE")==0) {
    *pbParamValue = BOBCAT_TEST_IMG_WHITE;
  } else if (strcmp(str, "BOBCAT_TEST_IMG_HRAMP")==0) {
    *pbParamValue = BOBCAT_TEST_IMG_HRAMP;
  } else if (strcmp(str, "BOBCAT_TEST_IMG_VRAMP")==0) {
    *pbParamValue = BOBCAT_TEST_IMG_VRAMP;
  } else if (strcmp(str, "BOBCAT_TEST_IMG_MOVING_HRAMP")==0) {
    *pbParamValue = BOBCAT_TEST_IMG_MOVING_HRAMP;
  } else if (strcmp(str, "BOBCAT_TEST_IMG_MOVING_VRAMP")==0) {
    *pbParamValue = BOBCAT_TEST_IMG_MOVING_VRAMP;
  } else if (strcmp(str, "BOBCAT_TEST_IMG_8BARS")==0) {
    *pbParamValue = BOBCAT_TEST_IMG_8BARS;
  } else if (strcmp(str, "BOBCAT_TEST_IMG_HVLNS_OVER_LIVE")==0) {
    *pbParamValue = BOBCAT_TEST_IMG_HVLNS_OVER_LIVE;
  } else if (strcmp(str, "BOBCAT_ACTIVE_LOW")==0) {
    *pbParamValue = BOBCAT_ACTIVE_LOW;
  } else if (strcmp(str, "BOBCAT_ACTIVE_HIGH")==0) {
    *pbParamValue = BOBCAT_ACTIVE_HIGH;
  } else if (strcmp(str, "BOBCAT_OUTPUT_EXP_START")==0) {
    *pbParamValue = BOBCAT_OUTPUT_EXP_START;
  } else if (strcmp(str, "BOBCAT_OUTPUT_EXP_END")==0) {
    *pbParamValue = BOBCAT_OUTPUT_EXP_END;
  } else if (strcmp(str, "BOBCAT_OUTPUT_EXP_MID")==0) {
    *pbParamValue = BOBCAT_OUTPUT_EXP_MID;
  } else if (strcmp(str, "BOBCAT_OUTPUT_EXP_ACTIVE_WIN")==0) {
    *pbParamValue = BOBCAT_OUTPUT_EXP_ACTIVE_WIN;
  } else if (strcmp(str, "BOBCAT_OUTPUT_H_SYNC")==0) {
    *pbParamValue = BOBCAT_OUTPUT_H_SYNC;
  } else if (strcmp(str, "BOBCAT_OUTPUT_V_SYNC")==0) {
    *pbParamValue = BOBCAT_OUTPUT_V_SYNC;
  } else if (strcmp(str, "BOBCAT_OUTPUT_ODD_EVEN_FRM_FLAG")==0) {
    *pbParamValue = BOBCAT_OUTPUT_ODD_EVEN_FRM_FLAG;
  } else if (strcmp(str, "BOBCAT_OUTPUT_TRIG_PULSE_ACTUAL")==0) {
    *pbParamValue = BOBCAT_OUTPUT_TRIG_PULSE_ACTUAL;
  } else if (strcmp(str, "BOBCAT_OUTPUT_TRIG_PULSE_DELAYED")==0) {
    *pbParamValue = BOBCAT_OUTPUT_TRIG_PULSE_DELAYED;
  } else if (strcmp(str, "BOBCAT_OUTPUT_CAM_READY")==0) {
    *pbParamValue = BOBCAT_OUTPUT_CAM_READY;
  } else if (strcmp(str, "BOBCAT_OUTPUT_PULSE_GEN")==0) {
    *pbParamValue = BOBCAT_OUTPUT_PULSE_GEN;
  } else if (strcmp(str, "BOBCAT_OUTPUT_STROBE1")==0) {
    *pbParamValue = BOBCAT_OUTPUT_STROBE1;
  } else if (strcmp(str, "BOBCAT_OUTPUT_STROBE2")==0) {
    *pbParamValue = BOBCAT_OUTPUT_STROBE2;
  } else if (strcmp(str, "BOBCAT_OUTPUT_SW_BIT_TOGGLE")==0) {
    *pbParamValue = BOBCAT_OUTPUT_SW_BIT_TOGGLE;
  } else if (strcmp(str, "BOBCAT_INPUT_COM_TRIG")==0) {
    *pbParamValue = BOBCAT_INPUT_COM_TRIG;
  } else if (strcmp(str, "BOBCAT_INPUT_EXT_TRIG")==0) {
    *pbParamValue = BOBCAT_INPUT_EXT_TRIG;
  } else if (strcmp(str, "BOBCAT_INPUT_EXP_CTRL")==0) {
    *pbParamValue = BOBCAT_INPUT_EXP_CTRL;
  } else if (strcmp(str, "BOBCAT_INPUT_H_SYNC")==0) {
    *pbParamValue = BOBCAT_INPUT_H_SYNC;
  } else if (strcmp(str, "BOBCAT_INPUT_V_SYNC")==0) {
    *pbParamValue = BOBCAT_INPUT_V_SYNC;
  } else if (strcmp(str, "BOBCAT_STROBE_EACH_FRAME")==0) {
    *pbParamValue = BOBCAT_STROBE_EACH_FRAME;
  } else if (strcmp(str, "BOBCAT_STROBE_ODD_FRAMES")==0) {
    *pbParamValue = BOBCAT_STROBE_ODD_FRAMES;
  } else if (strcmp(str, "BOBCAT_STROBE_EVEN_FRAMES")==0) {
    *pbParamValue = BOBCAT_STROBE_EVEN_FRAMES;
  } else if (strcmp(str, "BOBCAT_8BIT")==0) {
    *pbParamValue = BOBCAT_8BIT;
  } else if (strcmp(str, "BOBCAT_10BIT")==0) {
    *pbParamValue = BOBCAT_10BIT;
  } else if (strcmp(str, "BOBCAT_12BIT")==0) {
    *pbParamValue = BOBCAT_12BIT;
  } else if (strcmp(str, "BOBCAT_14BIT")==0) {
    *pbParamValue = BOBCAT_14BIT;
  } else if (strcmp(str, "BOBCAT_3X8BIT")==0) {
    *pbParamValue = BOBCAT_3X8BIT;
  } else if (strcmp(str, "BOBCAT_SINGLE_TAP")==0) {
    *pbParamValue = BOBCAT_SINGLE_TAP;
  } else if (strcmp(str, "BOBCAT_DOUBLE_TAP")==0) {
    *pbParamValue = BOBCAT_DOUBLE_TAP;
  } else if (strcmp(str, "BOBCAT_SINGLE_TAP_SINGLE")==0) {
    *pbParamValue = BOBCAT_SINGLE_TAP_SINGLE;
  } else if (strcmp(str, "BOBCAT_DOUBLE_TAP_INTERLEAVED")==0) {
    *pbParamValue = BOBCAT_DOUBLE_TAP_INTERLEAVED;
  } else if (strcmp(str, "BOBCAT_DOUBLE_TAP_SEQUENTIAL")==0) {
    *pbParamValue = BOBCAT_DOUBLE_TAP_SEQUENTIAL;
  } else if (strcmp(str, "BOBCAT_WB_ONCE")==0) {
    *pbParamValue = BOBCAT_WB_ONCE;
  } else if (strcmp(str, "BOBCAT_WB_AUTO")==0) {
    *pbParamValue = BOBCAT_WB_AUTO;
  } else if (strcmp(str, "BOBCAT_WB_MANUAL")==0) {
    *pbParamValue = BOBCAT_WB_MANUAL;
  } else if (strcmp(str, "BOBCAT_LUT1")==0) {
    *pbParamValue = BOBCAT_LUT1;
  } else if (strcmp(str, "BOBCAT_LUT2")==0) {
    *pbParamValue = BOBCAT_LUT2;
  } else if (strcmp(str, "BOBCAT_STATIC_PIXEL_CORR")==0) {
    *pbParamValue = BOBCAT_STATIC_PIXEL_CORR;
  } else if (strcmp(str, "BOBCAT_DYNAMIC_PIXEL_CORR")==0) {
    *pbParamValue = BOBCAT_DYNAMIC_PIXEL_CORR;
  } else if (strcmp(str, "BOBCAT_SINGLE_BINARY")==0) {
    *pbParamValue = BOBCAT_SINGLE_BINARY;
  } else if (strcmp(str, "BOBCAT_DOUBLE_BINARY")==0) {
    *pbParamValue = BOBCAT_DOUBLE_BINARY;
  } else if (strcmp(str, "BOBCAT_DOUBLE_GRAYSCALE")==0) {
    *pbParamValue = BOBCAT_DOUBLE_GRAYSCALE;
  } else if (strcmp(str, "BOBCAT_DOUBLE_CONTRAST")==0) {
    *pbParamValue = BOBCAT_DOUBLE_CONTRAST;
  } else if (strcmp(str, "BOBCAT_SINGLE_POINT")==0) {
    *pbParamValue = BOBCAT_SINGLE_POINT;
  } else if (strcmp(str, "BOBCAT_MULTI_POINT")==0) {
    *pbParamValue = BOBCAT_MULTI_POINT;
  } else {
    *pbParamValue = atol(str);
    return 0;
  }
  return 1;
}


int BOBCAT_str_to_bobcatParamValues(char* str, bobcatParamValue* pbParamValue) {
  char *token;
  bobcatParamValue temp_bParamValue;
  char char_pipe[2] = "|";
  *pbParamValue = 0;
#if defined _VERBOSE
  printf("[");
#endif
  token = strtok(str, char_pipe);
  while( token != NULL ) { /* cycle through other tokens */
#if defined _VERBOSE
    printf("%s:",token);
#endif
    if (BOBCAT_str_to_bobcatParamValue(token, &temp_bParamValue)<0) {
      return 0;
    } else {
      *pbParamValue |= temp_bParamValue;
#if defined _VERBOSE
      printf("%d:%d ,", temp_bParamValue, *pbParamValue);
#endif
    }
    token = strtok(NULL, char_pipe);
  }
#if defined _VERBOSE
  printf("\b]\n");
#endif
  return 1;
}


float BOBCAT_GetTemp(tHandle hCamera) {
  bobcatParamValue bParamValue;
  int temp_value,temp_sign;
  float temp;
  etStat eStat = PHX_OK;
  eStat = BOBCAT_ParameterGet( hCamera, BOBCAT_INFO_CCD_TEMP, &bParamValue );
  if ( PHX_OK != eStat ){
    printf("PHX: Error BOBCAT_GetTemp\n");
  }else{
    temp_value = bParamValue & 0x1FF;
    temp_sign  = bParamValue & 0x200;
    temp = (float)temp_value * 0.25;
    if(temp_sign) temp*=-1;
    return temp;
  }
  return 0;
}
