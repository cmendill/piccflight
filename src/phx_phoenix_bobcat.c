#include <phx_api.h>

#include "phx_phoenix_bobcat.h"
#include "phx_phoenix.h"
#include "phx_bobcat.h"

/* function to handle parameters that require simultaneous configuration of both the bobcat and the phoenix
  PHX_BOBCAT_BIT_DEPTH = PHX_BOBCAT_8BIT | PHX_BOBCAT_10BIT | PHX_BOBCAT_12BIT
  PHX_BOBCAT_TAPS = PHX_BOBCAT_DOUBLE_TAP | PHX_BOBCAT_SINGLE_TAP
  PHX_BOBCAT_ROI ~ 0,0,400,400,BOBCAT_BINNING_1X,BOBCAT_BINNING_1X
*/
etStat PHX_BOBCAT_Configure(tHandle hpb, phxbobcatParam parameter, void* value) {
  etStat eStat = PHX_OK;
  etParamValue eParamValue; /* Parameter for use with PHX_ParameterSet/Get calls */
  bobcatParamValue bParamValue; /* Parameter for use with BOBCAT_ParameterSet/Get calls */
  phxbobcatParamValue* ppbParamValue;
  region* proi;

  switch(parameter) {
    case PHX_BOBCAT_BIT_DEPTH:

      eParamValue = PHX_CAM_SRC_MONO;
      eStat = PHX_ParameterSet( hpb, PHX_CAM_SRC_COL, (etParamValue*) &(eParamValue) );
      if ( eStat != PHX_OK ) goto Error;

      ppbParamValue = value;
      switch(*ppbParamValue) {

        case PHX_BOBCAT_SINGLE_TAP:

          eStat = PHX_ERROR_BAD_PARAM_VALUE;
          goto Error;
          break;

        case PHX_BOBCAT_DOUBLE_TAP:

          eStat = PHX_ERROR_BAD_PARAM_VALUE;
          goto Error;
          break;

        case PHX_BOBCAT_8BIT:  /* 8bit format */

          bParamValue = BOBCAT_8BIT;
          eStat = BOBCAT_ParameterSet( hpb, BOBCAT_BIT_DEPTH_SELECT, (bobcatParamValue*) &bParamValue);
          if ( eStat != PHX_OK ) goto Error;
          eParamValue = 8;
          eStat = PHX_ParameterSet( hpb, PHX_CAM_SRC_DEPTH, (etParamValue*) &(eParamValue) );
          if ( eStat != PHX_OK ) goto Error;
          eParamValue = PHX_DST_FORMAT_Y8;
          eStat = PHX_ParameterSet( hpb, PHX_CAPTURE_FORMAT, (etParamValue*) &(eParamValue) );
          if ( eStat != PHX_OK ) goto Error;
          break;

        case PHX_BOBCAT_10BIT:  /* 10bit format */

          bParamValue = BOBCAT_10BIT;
          eStat = BOBCAT_ParameterSet( hpb, BOBCAT_BIT_DEPTH_SELECT, (bobcatParamValue*) &bParamValue);
          if ( eStat != PHX_OK ) goto Error;
          eParamValue = 10;
          eStat = PHX_ParameterSet( hpb, PHX_CAM_SRC_DEPTH, (etParamValue*) &(eParamValue) );
          if ( eStat != PHX_OK ) goto Error;
          eParamValue = PHX_DST_FORMAT_Y10;
          eStat = PHX_ParameterSet( hpb, PHX_CAPTURE_FORMAT, (etParamValue*) &(eParamValue) );
          if ( eStat != PHX_OK ) goto Error;
          break;

        case PHX_BOBCAT_12BIT:  /* 12bit format */

          bParamValue = BOBCAT_12BIT;
          eStat = BOBCAT_ParameterSet( hpb, BOBCAT_BIT_DEPTH_SELECT, (bobcatParamValue*) &bParamValue);
          if ( eStat != PHX_OK ) goto Error;
          eParamValue = 12;
          eStat = PHX_ParameterSet( hpb, PHX_CAM_SRC_DEPTH, (etParamValue*) &(eParamValue) );
          if ( eStat != PHX_OK ) goto Error;
          eParamValue = PHX_DST_FORMAT_Y12;
          eStat = PHX_ParameterSet( hpb, PHX_CAPTURE_FORMAT, (etParamValue*) &(eParamValue) );
          if ( eStat != PHX_OK ) goto Error;
          break;

        }
      break;

    case PHX_BOBCAT_TAPS:

      eParamValue = PHX_CAM_HTAP_LEFT;
      eStat = PHX_ParameterSet( hpb, PHX_CAM_HTAP_DIR, (etParamValue*) &(eParamValue) );
      if ( eStat != PHX_OK ) goto Error;
      eParamValue = PHX_CAM_HTAP_LINEAR;
      eStat = PHX_ParameterSet( hpb, PHX_CAM_HTAP_TYPE, (etParamValue*) &(eParamValue) );
      if ( eStat != PHX_OK ) goto Error;
      eParamValue = PHX_CAM_HTAP_ASCENDING;
      eStat = PHX_ParameterSet( hpb, PHX_CAM_HTAP_ORDER, (etParamValue*) &(eParamValue) );
      if ( eStat != PHX_OK ) goto Error;
      eParamValue = 1;
      eStat = PHX_ParameterSet( hpb, PHX_CAM_VTAP_NUM, (etParamValue*) &(eParamValue) );
      if ( eStat != PHX_OK ) goto Error;
      eParamValue = PHX_CAM_VTAP_TOP;
      eStat = PHX_ParameterSet( hpb, PHX_CAM_VTAP_DIR, (etParamValue*) &(eParamValue) );
      if ( eStat != PHX_OK ) goto Error;
      eParamValue = PHX_CAM_VTAP_LINEAR;
      eStat = PHX_ParameterSet( hpb, PHX_CAM_VTAP_TYPE, (etParamValue*) &(eParamValue) );
      if ( eStat != PHX_OK ) goto Error;
      eParamValue = PHX_CAM_VTAP_ASCENDING;
      eStat = PHX_ParameterSet( hpb, PHX_CAM_VTAP_ORDER, (etParamValue*) &(eParamValue) );
      if ( eStat != PHX_OK ) goto Error;

      ppbParamValue = value;
      switch(*ppbParamValue) {

        case PHX_BOBCAT_8BIT:
          eStat = PHX_ERROR_BAD_PARAM_VALUE;

          goto Error;
          break;

        case PHX_BOBCAT_10BIT:

          eStat = PHX_ERROR_BAD_PARAM_VALUE;
          goto Error;
          break;

        case PHX_BOBCAT_12BIT:

          eStat = PHX_ERROR_BAD_PARAM_VALUE;
          goto Error;
          break;

        case PHX_BOBCAT_DOUBLE_TAP:

          eParamValue = 2;
          eStat = PHX_ParameterSet( hpb, PHX_CAM_HTAP_NUM, (etParamValue*) &(eParamValue) );
          if ( eStat != PHX_OK ) goto Error;
          bParamValue = BOBCAT_DOUBLE_TAP;
          eStat = BOBCAT_ParameterSet( hpb, BOBCAT_TAP_MODE_SELECT, (bobcatParamValue*) &bParamValue );
          if ( eStat != PHX_OK ) goto Error;
          bParamValue = BOBCAT_DOUBLE_TAP_SEQUENTIAL;
          eStat = BOBCAT_ParameterSet( hpb, BOBCAT_DATA_FORMAT_SELECT, (bobcatParamValue*) &bParamValue );
          if ( eStat != PHX_OK ) goto Error;
          break;

        case PHX_BOBCAT_SINGLE_TAP:

          eParamValue = 1;
          eStat = PHX_ParameterSet( hpb, PHX_CAM_HTAP_NUM, (etParamValue*) &(eParamValue) );
          if ( eStat != PHX_OK ) goto Error;
          bParamValue = BOBCAT_SINGLE_TAP;
          eStat = BOBCAT_ParameterSet( hpb, BOBCAT_TAP_MODE_SELECT, (bobcatParamValue*) &bParamValue );
          if ( eStat != PHX_OK ) goto Error;
          break;

        }
      break;

    case PHX_BOBCAT_ROI:

      proi = value;

      eParamValue = 1;
      eStat = PHX_ParameterSet( hpb, PHX_CAM_XBINNING, (etParamValue*) &(eParamValue) );
      if ( eStat != PHX_OK ) goto Error;
      eStat = PHX_ParameterSet( hpb, PHX_CAM_YBINNING, (etParamValue*) &(eParamValue) );
      if ( eStat != PHX_OK ) goto Error;
      eStat = PHX_ParameterSet( hpb, PHX_CAM_ACTIVE_XLENGTH, (etParamValue*) &(proi->x_length) );
      if ( eStat != PHX_OK ) goto Error;
      eStat = PHX_ParameterSet( hpb, PHX_CAM_ACTIVE_YLENGTH, (etParamValue*) &(proi->y_length) );
      if ( eStat != PHX_OK ) goto Error;
      eStat = PHX_ParameterSet( hpb, PHX_ROI_XLENGTH, (etParamValue*) &(proi->x_length) );
      if ( eStat != PHX_OK ) goto Error;
      eStat = PHX_ParameterSet( hpb, PHX_ROI_YLENGTH, (etParamValue*) &(proi->y_length) );
      if ( eStat != PHX_OK ) goto Error;
      eParamValue = 0;
      eStat = PHX_ParameterSet( hpb, PHX_CAM_ACTIVE_XOFFSET, (etParamValue*) &(proi->x_offset) );
      if ( eStat != PHX_OK ) goto Error;
      eStat = PHX_ParameterSet( hpb, PHX_CAM_ACTIVE_YOFFSET, (etParamValue*) &(eParamValue) );
      if ( eStat != PHX_OK ) goto Error;
      eStat = PHX_ParameterSet( hpb, PHX_ROI_SRC_XOFFSET, (etParamValue*) &(eParamValue) );
      if ( eStat != PHX_OK ) goto Error;
      eStat = PHX_ParameterSet( hpb, PHX_ROI_SRC_YOFFSET, (etParamValue*) &(eParamValue) );
      if ( eStat != PHX_OK ) goto Error;
      eStat = BOBCAT_ParameterSet( hpb, BOBCAT_XBINNING, (bobcatParamValue*) &(proi->x_binning) );
      if ( eStat != PHX_OK ) goto Error;
      eStat = BOBCAT_ParameterSet( hpb, BOBCAT_YBINNING, (bobcatParamValue*) &(proi->y_binning) );
      if ( eStat != PHX_OK ) goto Error;
      bParamValue = BOBCAT_ENABLE;
      eStat = BOBCAT_ParameterSet( hpb, BOBCAT_MAOI_ENABLE, (bobcatParamValue*) &bParamValue );
      if ( eStat != PHX_OK ) goto Error;
      eStat = BOBCAT_ParameterSet( hpb, BOBCAT_MAOI_XLENGTH, (bobcatParamValue*) &(proi->x_length) );
      if ( eStat != PHX_OK ) goto Error;
      eStat = BOBCAT_ParameterSet( hpb, BOBCAT_MAOI_YLENGTH, (bobcatParamValue*) &(proi->y_length) );
      if ( eStat != PHX_OK ) goto Error;
      eStat = BOBCAT_ParameterSet( hpb, BOBCAT_MAOI_XOFFSET, (bobcatParamValue*) &(proi->x_offset) );
      if ( eStat != PHX_OK ) goto Error;
      eStat = BOBCAT_ParameterSet( hpb, BOBCAT_MAOI_YOFFSET, (bobcatParamValue*) &(proi->y_offset) );
      if ( eStat != PHX_OK ) goto Error;
      break;

  }

  Error:
  return eStat;
}


int PHX_BOBCAT_str_to_phxbobcatParam(char** str, phxbobcatParam* ppbParam) {
  if (strcmp(*str, "PHX_BOBCAT_BIT_DEPTH")==0) {
    *ppbParam = PHX_BOBCAT_BIT_DEPTH;
  } else if (strcmp(*str, "PHX_BOBCAT_TAPS")==0) {
    *ppbParam = PHX_BOBCAT_TAPS;
  } else if (strcmp(*str, "PHX_BOBCAT_ROI")==0) {
    *ppbParam = PHX_BOBCAT_ROI;
  } else {
    return 0;
  }
  return 1;
}


int PHX_BOBCAT_str_to_phxbobcatParamValue(char** str, phxbobcatParamValue* ppbParamValue) {
  if (strcmp(*str, "PHX_BOBCAT_8BIT")==0) {
    *ppbParamValue = PHX_BOBCAT_8BIT;
  } else if (strcmp(*str, "PHX_BOBCAT_10BIT")==0) {
    *ppbParamValue = PHX_BOBCAT_10BIT;
  } else if (strcmp(*str, "PHX_BOBCAT_12BIT")==0) {
    *ppbParamValue = PHX_BOBCAT_12BIT;
  } else if (strcmp(*str, "PHX_BOBCAT_DOUBLE_TAP")==0) {
    *ppbParamValue = PHX_BOBCAT_DOUBLE_TAP;
  } else if (strcmp(*str, "PHX_BOBCAT_SINGLE_TAP")==0) {
    *ppbParamValue = PHX_BOBCAT_SINGLE_TAP;
  } else {
    return 0;
  }
  return 1;
}
