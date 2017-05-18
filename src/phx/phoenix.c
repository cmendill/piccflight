#include "phoenix.h"

int PHX_str_to_etParam(char** str, etParam* ppParam) {
  if (strcmp(*str, "PHX_PARAM_MASK")==0) {
    *ppParam = PHX_PARAM_MASK;
  } else if (strcmp(*str, "PHX_INVALID_PARAM")==0) {
    *ppParam = PHX_INVALID_PARAM;
  } else if (strcmp(*str, "PHX_CACHE_FLUSH")==0) {
    *ppParam = PHX_CACHE_FLUSH;
  } else if (strcmp(*str, "PHX_FORCE_REWRITE")==0) {
    *ppParam = PHX_FORCE_REWRITE;
  } else if (strcmp(*str, "PHX_ACQ_CONTINUOUS")==0) {
    *ppParam = PHX_ACQ_CONTINUOUS;
  } else if (strcmp(*str, "PHX_ACQ_NUM_IMAGES")==0) {
    *ppParam = PHX_ACQ_NUM_IMAGES;
  } else if (strcmp(*str, "PHX_ACQ_SKIP")==0) {
    *ppParam = PHX_ACQ_SKIP;
  } else if (strcmp(*str, "PHX_ACQTRIG_SRC")==0) {
    *ppParam = PHX_ACQTRIG_SRC;
  } else if (strcmp(*str, "PHX_ACQTRIG_TYPE")==0) {
    *ppParam = PHX_ACQTRIG_TYPE;
  } else if (strcmp(*str, "PHX_ACQ_TYPE")==0) {
    *ppParam = PHX_ACQ_TYPE;
  } else if (strcmp(*str, "PHX_ACQ_XSUB")==0) {
    *ppParam = PHX_ACQ_XSUB;
  } else if (strcmp(*str, "PHX_ACQ_YSUB")==0) {
    *ppParam = PHX_ACQ_YSUB;
  } else if (strcmp(*str, "PHX_CAM_ACTIVE_XLENGTH")==0) {
    *ppParam = PHX_CAM_ACTIVE_XLENGTH;
  } else if (strcmp(*str, "PHX_CAM_ACTIVE_YLENGTH")==0) {
    *ppParam = PHX_CAM_ACTIVE_YLENGTH;
  } else if (strcmp(*str, "PHX_CAM_ACTIVE_XOFFSET")==0) {
    *ppParam = PHX_CAM_ACTIVE_XOFFSET;
  } else if (strcmp(*str, "PHX_CAM_ACTIVE_YOFFSET")==0) {
    *ppParam = PHX_CAM_ACTIVE_YOFFSET;
  } else if (strcmp(*str, "PHX_CAM_CLOCK_POLARITY")==0) {
    *ppParam = PHX_CAM_CLOCK_POLARITY;
  } else if (strcmp(*str, "PHX_CAM_FORMAT")==0) {
    *ppParam = PHX_CAM_FORMAT;
  } else if (strcmp(*str, "PHX_CAM_NUM_TAPS")==0) {
    *ppParam = PHX_CAM_NUM_TAPS;
  } else if (strcmp(*str, "PHX_CAM_SRC_DEPTH")==0) {
    *ppParam = PHX_CAM_SRC_DEPTH;
  } else if (strcmp(*str, "PHX_CAM_SRC_COL")==0) {
    *ppParam = PHX_CAM_SRC_COL;
  } else if (strcmp(*str, "PHX_CAM_HTAP_DIR")==0) {
    *ppParam = PHX_CAM_HTAP_DIR;
  } else if (strcmp(*str, "PHX_CAM_HTAP_TYPE")==0) {
    *ppParam = PHX_CAM_HTAP_TYPE;
  } else if (strcmp(*str, "PHX_CAM_HTAP_NUM")==0) {
    *ppParam = PHX_CAM_HTAP_NUM;
  } else if (strcmp(*str, "PHX_CAM_VTAP_DIR")==0) {
    *ppParam = PHX_CAM_VTAP_DIR;
  } else if (strcmp(*str, "PHX_CAM_VTAP_TYPE")==0) {
    *ppParam = PHX_CAM_VTAP_TYPE;
  } else if (strcmp(*str, "PHX_CAM_VTAP_NUM")==0) {
    *ppParam = PHX_CAM_VTAP_NUM;
  } else if (strcmp(*str, "PHX_CAM_TYPE")==0) {
    *ppParam = PHX_CAM_TYPE;
  } else if (strcmp(*str, "PHX_CAM_XBINNING")==0) {
    *ppParam = PHX_CAM_XBINNING;
  } else if (strcmp(*str, "PHX_CAM_YBINNING")==0) {
    *ppParam = PHX_CAM_YBINNING;
  } else if (strcmp(*str, "PHX_COMMS_DATA")==0) {
    *ppParam = PHX_COMMS_DATA;
  } else if (strcmp(*str, "PHX_COMMS_FLOW")==0) {
    *ppParam = PHX_COMMS_FLOW;
  } else if (strcmp(*str, "PHX_COMMS_INCOMING")==0) {
    *ppParam = PHX_COMMS_INCOMING;
  } else if (strcmp(*str, "PHX_COMMS_OUTGOING")==0) {
    *ppParam = PHX_COMMS_OUTGOING;
  } else if (strcmp(*str, "PHX_COMMS_PARITY")==0) {
    *ppParam = PHX_COMMS_PARITY;
  } else if (strcmp(*str, "PHX_COMMS_SPEED")==0) {
    *ppParam = PHX_COMMS_SPEED;
  } else if (strcmp(*str, "PHX_COMMS_STANDARD")==0) {
    *ppParam = PHX_COMMS_STANDARD;
  } else if (strcmp(*str, "PHX_COMMS_STOP")==0) {
    *ppParam = PHX_COMMS_STOP;
  } else if (strcmp(*str, "PHX_DATASRC")==0) {
    *ppParam = PHX_DATASRC;
  } else if (strcmp(*str, "PHX_DATASTREAM_VALID")==0) {
    *ppParam = PHX_DATASTREAM_VALID;
  } else if (strcmp(*str, "PHX_CAPTURE_FORMAT")==0) {
    *ppParam = PHX_CAPTURE_FORMAT;
  } else if (strcmp(*str, "PHX_DST_FORMAT")==0) {
    *ppParam = PHX_DST_FORMAT;
  } else if (strcmp(*str, "PHX_DST_PTR_TYPE")==0) {
    *ppParam = PHX_DST_PTR_TYPE;
  } else if (strcmp(*str, "PHX_DST_PTRS_VIRT")==0) {
    *ppParam = PHX_DST_PTRS_VIRT;
  } else if (strcmp(*str, "PHX_DUMMY_PARAM")==0) {
    *ppParam = PHX_DUMMY_PARAM;
  } else if (strcmp(*str, "PHX_ERROR_FIRST_ERRNUM")==0) {
    *ppParam = PHX_ERROR_FIRST_ERRNUM;
  } else if (strcmp(*str, "PHX_ERROR_FIRST_ERRSTRING")==0) {
    *ppParam = PHX_ERROR_FIRST_ERRSTRING;
  } else if (strcmp(*str, "PHX_ERROR_HANDLER")==0) {
    *ppParam = PHX_ERROR_HANDLER;
  } else if (strcmp(*str, "PHX_ERROR_LAST_ERRNUM")==0) {
    *ppParam = PHX_ERROR_LAST_ERRNUM;
  } else if (strcmp(*str, "PHX_ERROR_LAST_ERRSTRING")==0) {
    *ppParam = PHX_ERROR_LAST_ERRSTRING;
  } else if (strcmp(*str, "PHX_EVENTCOUNT")==0) {
    *ppParam = PHX_EVENTCOUNT;
  } else if (strcmp(*str, "PHX_NUM_BOARDS")==0) {
    *ppParam = PHX_NUM_BOARDS;
  } else if (strcmp(*str, "PHX_IO_CCIO_A")==0) {
    *ppParam = PHX_IO_CCIO_A;
  } else if (strcmp(*str, "PHX_IO_CCIO_A_OUT")==0) {
    *ppParam = PHX_IO_CCIO_A_OUT;
  } else if (strcmp(*str, "PHX_IO_CCIO_B")==0) {
    *ppParam = PHX_IO_CCIO_B;
  } else if (strcmp(*str, "PHX_IO_CCIO_B_OUT")==0) {
    *ppParam = PHX_IO_CCIO_B_OUT;
  } else if (strcmp(*str, "PHX_IO_CCOUT_A")==0) {
    *ppParam = PHX_IO_CCOUT_A;
  } else if (strcmp(*str, "PHX_IO_CCOUT_B")==0) {
    *ppParam = PHX_IO_CCOUT_B;
  } else if (strcmp(*str, "PHX_IO_OPTO_SET")==0) {
    *ppParam = PHX_IO_OPTO_SET;
  } else if (strcmp(*str, "PHX_IO_OPTO_CLR")==0) {
    *ppParam = PHX_IO_OPTO_CLR;
  } else if (strcmp(*str, "PHX_IO_OPTO")==0) {
    *ppParam = PHX_IO_OPTO;
  } else if (strcmp(*str, "PHX_IO_TTL_A")==0) {
    *ppParam = PHX_IO_TTL_A;
  } else if (strcmp(*str, "PHX_IO_TTL_A_OUT")==0) {
    *ppParam = PHX_IO_TTL_A_OUT;
  } else if (strcmp(*str, "PHX_IO_TTL_B")==0) {
    *ppParam = PHX_IO_TTL_B;
  } else if (strcmp(*str, "PHX_IO_TTL_B_OUT")==0) {
    *ppParam = PHX_IO_TTL_B_OUT;
  } else if (strcmp(*str, "PHX_TIMEOUT_CAPTURE")==0) {
    *ppParam = PHX_TIMEOUT_CAPTURE;
  } else if (strcmp(*str, "PHX_TIMEOUT_DMA")==0) {
    *ppParam = PHX_TIMEOUT_DMA;
  } else if (strcmp(*str, "PHX_TIMEOUT_TRIGGER")==0) {
    *ppParam = PHX_TIMEOUT_TRIGGER;
  } else if (strcmp(*str, "PHX_INTRPT_SET")==0) {
    *ppParam = PHX_INTRPT_SET;
  } else if (strcmp(*str, "PHX_INTRPT_CLR")==0) {
    *ppParam = PHX_INTRPT_CLR;
  } else if (strcmp(*str, "PHX_REV_HW")==0) {
    *ppParam = PHX_REV_HW;
  } else if (strcmp(*str, "PHX_REV_HW_MAJOR")==0) {
    *ppParam = PHX_REV_HW_MAJOR;
  } else if (strcmp(*str, "PHX_REV_HW_MINOR")==0) {
    *ppParam = PHX_REV_HW_MINOR;
  } else if (strcmp(*str, "PHX_REV_SW")==0) {
    *ppParam = PHX_REV_SW;
  } else if (strcmp(*str, "PHX_REV_SW_MAJOR")==0) {
    *ppParam = PHX_REV_SW_MAJOR;
  } else if (strcmp(*str, "PHX_REV_SW_MINOR")==0) {
    *ppParam = PHX_REV_SW_MINOR;
  } else if (strcmp(*str, "PHX_REV_SW_SUBMINOR")==0) {
    *ppParam = PHX_REV_SW_SUBMINOR;
  } else if (strcmp(*str, "PHX_ROI_DST_XOFFSET")==0) {
    *ppParam = PHX_ROI_DST_XOFFSET;
  } else if (strcmp(*str, "PHX_ROI_DST_YOFFSET")==0) {
    *ppParam = PHX_ROI_DST_YOFFSET;
  } else if (strcmp(*str, "PHX_ROI_SRC_XOFFSET")==0) {
    *ppParam = PHX_ROI_SRC_XOFFSET;
  } else if (strcmp(*str, "PHX_ROI_SRC_YOFFSET")==0) {
    *ppParam = PHX_ROI_SRC_YOFFSET;
  } else if (strcmp(*str, "PHX_ROI_XLENGTH")==0) {
    *ppParam = PHX_ROI_XLENGTH;
  } else if (strcmp(*str, "PHX_ROI_YLENGTH")==0) {
    *ppParam = PHX_ROI_YLENGTH;
  } else if (strcmp(*str, "PHX_BUF_DST_XLENGTH")==0) {
    *ppParam = PHX_BUF_DST_XLENGTH;
  } else if (strcmp(*str, "PHX_BUF_DST_YLENGTH")==0) {
    *ppParam = PHX_BUF_DST_YLENGTH;
  } else if (strcmp(*str, "PHX_STATUS")==0) {
    *ppParam = PHX_STATUS;
  } else if (strcmp(*str, "PHX_BOARD_PROPERTIES")==0) {
    *ppParam = PHX_BOARD_PROPERTIES;
  } else if (strcmp(*str, "PHX_ROI_XLENGTH_SCALED")==0) {
    *ppParam = PHX_ROI_XLENGTH_SCALED;
  } else if (strcmp(*str, "PHX_ROI_YLENGTH_SCALED")==0) {
    *ppParam = PHX_ROI_YLENGTH_SCALED;
  } else if (strcmp(*str, "PHX_BUF_SET")==0) {
    *ppParam = PHX_BUF_SET;
  } else if (strcmp(*str, "PHX_BUF_SET_COLOUR")==0) {
    *ppParam = PHX_BUF_SET_COLOUR;
  } else if (strcmp(*str, "PHX_LUT_COUNT")==0) {
    *ppParam = PHX_LUT_COUNT;
  } else if (strcmp(*str, "PHX_LUT_INFO")==0) {
    *ppParam = PHX_LUT_INFO;
  } else if (strcmp(*str, "PHX_REV_HW_SUBMINOR")==0) {
    *ppParam = PHX_REV_HW_SUBMINOR;
  } else if (strcmp(*str, "PHX_LUT_CORRECT")==0) {
    *ppParam = PHX_LUT_CORRECT;
  } else if (strcmp(*str, "PHX_LINETRIG_SRC")==0) {
    *ppParam = PHX_LINETRIG_SRC;
  } else if (strcmp(*str, "PHX_LINETRIG_TIMER_CTRL")==0) {
    *ppParam = PHX_LINETRIG_TIMER_CTRL;
  } else if (strcmp(*str, "PHX_LINETRIG_TIMER_PERIOD")==0) {
    *ppParam = PHX_LINETRIG_TIMER_PERIOD;
  } else if (strcmp(*str, "PHX_EXPTRIG_SRC")==0) {
    *ppParam = PHX_EXPTRIG_SRC;
  } else if (strcmp(*str, "PHX_EXP_CTRLIO_1")==0) {
    *ppParam = PHX_EXP_CTRLIO_1;
  } else if (strcmp(*str, "PHX_IO_TIMER_1_PERIOD")==0) {
    *ppParam = PHX_IO_TIMER_1_PERIOD;
  } else if (strcmp(*str, "PHX_EXP_CTRLIO_2")==0) {
    *ppParam = PHX_EXP_CTRLIO_2;
  } else if (strcmp(*str, "PHX_IO_TIMER_2_PERIOD")==0) {
    *ppParam = PHX_IO_TIMER_2_PERIOD;
  } else if (strcmp(*str, "PHX_EXP_LINESTART")==0) {
    *ppParam = PHX_EXP_LINESTART;
  } else if (strcmp(*str, "PHX_ACQTRIG_DELAY_TYPE")==0) {
    *ppParam = PHX_ACQTRIG_DELAY_TYPE;
  } else if (strcmp(*str, "PHX_ACQTRIG_DELAY")==0) {
    *ppParam = PHX_ACQTRIG_DELAY;
  } else if (strcmp(*str, "PHX_EVENTCOUNT_SRC")==0) {
    *ppParam = PHX_EVENTCOUNT_SRC;
  } else if (strcmp(*str, "PHX_EVENTGATE_SRC")==0) {
    *ppParam = PHX_EVENTGATE_SRC;
  } else if (strcmp(*str, "PHX_CAM_HTAP_ORDER")==0) {
    *ppParam = PHX_CAM_HTAP_ORDER;
  } else if (strcmp(*str, "PHX_CAM_VTAP_ORDER")==0) {
    *ppParam = PHX_CAM_VTAP_ORDER;
  } else if (strcmp(*str, "PHX_EVENT_CONTEXT")==0) {
    *ppParam = PHX_EVENT_CONTEXT;
  } else if (strcmp(*str, "PHX_CAM_DATA_VALID")==0) {
    *ppParam = PHX_CAM_DATA_VALID;
  } else if (strcmp(*str, "PHX_BUFFER_READY_COUNT")==0) {
    *ppParam = PHX_BUFFER_READY_COUNT;
  } else if (strcmp(*str, "PHX_BUFFER_READY_COUNTER")==0) {
    *ppParam = PHX_BUFFER_READY_COUNTER;
  } else if (strcmp(*str, "PHX_LUT_SHIFT")==0) {
    *ppParam = PHX_LUT_SHIFT;
  } else if (strcmp(*str, "PHX_MASK_CCIO")==0) {
    *ppParam = PHX_MASK_CCIO;
  } else if (strcmp(*str, "PHX_MASK_CCOUT")==0) {
    *ppParam = PHX_MASK_CCOUT;
  } else if (strcmp(*str, "PHX_MASK_OPTO")==0) {
    *ppParam = PHX_MASK_OPTO;
  } else if (strcmp(*str, "PHX_IO_CCIO")==0) {
    *ppParam = PHX_IO_CCIO;
  } else if (strcmp(*str, "PHX_IO_CCOUT")==0) {
    *ppParam = PHX_IO_CCOUT;
  } else if (strcmp(*str, "PHX_IO_CCIO_OUT")==0) {
    *ppParam = PHX_IO_CCIO_OUT;
  } else if (strcmp(*str, "PHX_IO_TTL")==0) {
    *ppParam = PHX_IO_TTL;
  } else if (strcmp(*str, "PHX_IO_TTL_OUT")==0) {
    *ppParam = PHX_IO_TTL_OUT;
  } else if (strcmp(*str, "PHX_IO_OPTO_A")==0) {
    *ppParam = PHX_IO_OPTO_A;
  } else if (strcmp(*str, "PHX_IO_OPTO_B")==0) {
    *ppParam = PHX_IO_OPTO_B;
  } else if (strcmp(*str, "PHX_IO_TIMER_A1_PERIOD")==0) {
    *ppParam = PHX_IO_TIMER_A1_PERIOD;
  } else if (strcmp(*str, "PHX_IO_TIMER_A2_PERIOD")==0) {
    *ppParam = PHX_IO_TIMER_A2_PERIOD;
  } else if (strcmp(*str, "PHX_IO_TIMER_B1_PERIOD")==0) {
    *ppParam = PHX_IO_TIMER_B1_PERIOD;
  } else if (strcmp(*str, "PHX_IO_TIMER_B2_PERIOD")==0) {
    *ppParam = PHX_IO_TIMER_B2_PERIOD;
  } else if (strcmp(*str, "PHX_IO_OPTO_OUT")==0) {
    *ppParam = PHX_IO_OPTO_OUT;
  } else if (strcmp(*str, "PHX_IO_OPTO_A_OUT")==0) {
    *ppParam = PHX_IO_OPTO_A_OUT;
  } else if (strcmp(*str, "PHX_IO_OPTO_B_OUT")==0) {
    *ppParam = PHX_IO_OPTO_B_OUT;
  } else if (strcmp(*str, "PHX_ACQTRIG_ALIGN")==0) {
    *ppParam = PHX_ACQTRIG_ALIGN;
  } else if (strcmp(*str, "PHX_DST_ENDIAN")==0) {
    *ppParam = PHX_DST_ENDIAN;
  } else if (strcmp(*str, "PHX_ACQ_CHAIN")==0) {
    *ppParam = PHX_ACQ_CHAIN;
  } else if (strcmp(*str, "PHX_ACQ_BLOCKING")==0) {
    *ppParam = PHX_ACQ_BLOCKING;
  } else if (strcmp(*str, "PHX_DST_PTRS_PHYS")==0) {
    *ppParam = PHX_DST_PTRS_PHYS;
  } else if (strcmp(*str, "PHX_BOARD_INFO")==0) {
    *ppParam = PHX_BOARD_INFO;
  } else if (strcmp(*str, "PHX_DATARATE_TEST")==0) {
    *ppParam = PHX_DATARATE_TEST;
  } else if (strcmp(*str, "PHX_CAM_CLOCK_MAX")==0) {
    *ppParam = PHX_CAM_CLOCK_MAX;
  } else if (strcmp(*str, "PHX_EVENTCOUNT_AT_GATE")==0) {
    *ppParam = PHX_EVENTCOUNT_AT_GATE;
  } else if (strcmp(*str, "PHX_CHAN_SYNC_MODE")==0) {
    *ppParam = PHX_CHAN_SYNC_MODE;
  } else if (strcmp(*str, "PHX_ACQ_BUFFER_START")==0) {
    *ppParam = PHX_ACQ_BUFFER_START;
  } else if (strcmp(*str, "PHX_LUT_BYPASS")==0) {
    *ppParam = PHX_LUT_BYPASS;
  } else if (strcmp(*str, "PHX_COMMS_PORT_NAME")==0) {
    *ppParam = PHX_COMMS_PORT_NAME;
  } else if (strcmp(*str, "PHX_CVB_PARAM")==0) {
    *ppParam = PHX_CVB_PARAM;
  } else if (strcmp(*str, "PHX_USER_FORMAT")==0) {
    *ppParam = PHX_USER_FORMAT;
  } else if (strcmp(*str, "PHX_ACQ_AUTO_RESTART")==0) {
    *ppParam = PHX_ACQ_AUTO_RESTART;
  } else if (strcmp(*str, "PHX_ACQ_HSCALE")==0) {
    *ppParam = PHX_ACQ_HSCALE;
  } else if (strcmp(*str, "PHX_MERGE_CHAN")==0) {
    *ppParam = PHX_MERGE_CHAN;
  } else if (strcmp(*str, "PHX_MERGE_INTRPT_SET")==0) {
    *ppParam = PHX_MERGE_INTRPT_SET;
  } else if (strcmp(*str, "PHX_MERGE_INTRPT_CLR")==0) {
    *ppParam = PHX_MERGE_INTRPT_CLR;
  } else if (strcmp(*str, "PHX_CLSER_INDEX")==0) {
    *ppParam = PHX_CLSER_INDEX;
  } else if (strcmp(*str, "PHX_FIFO_BUFFER_COUNT")==0) {
    *ppParam = PHX_FIFO_BUFFER_COUNT;
  } else if (strcmp(*str, "PHX_REV_FLASH")==0) {
    *ppParam = PHX_REV_FLASH;
  } else if (strcmp(*str, "PHX_CONFIG_FILE")==0) {
    *ppParam = PHX_CONFIG_FILE;
  } else if (strcmp(*str, "PHX_BOARD_VARIANT")==0) {
    *ppParam = PHX_BOARD_VARIANT;
  } else if (strcmp(*str, "PHX_BOARD_NUMBER")==0) {
    *ppParam = PHX_BOARD_NUMBER;
  } else if (strcmp(*str, "PHX_CHANNEL_NUMBER")==0) {
    *ppParam = PHX_CHANNEL_NUMBER;
  } else if (strcmp(*str, "PHX_CONFIG_MODE")==0) {
    *ppParam = PHX_CONFIG_MODE;
  }	else {
    return 0;
  }
  return 1;
}


int PHX_str_to_etParamValue(char** str, etParamValue* ppParamValue) {
  if (strcmp(*str,"PHX_INVALID_PARAMVALUE")==0) {
    *ppParamValue = PHX_INVALID_PARAMVALUE;
  } else if (strcmp(*str,"PHX_ENABLE")==0) {
    *ppParamValue = PHX_ENABLE;
  } else if (strcmp(*str,"PHX_DISABLE")==0) {
    *ppParamValue = PHX_DISABLE;
  } else if (strcmp(*str,"PHX_COMMS_DATA_5")==0) {
    *ppParamValue = PHX_COMMS_DATA_5;
  } else if (strcmp(*str,"PHX_COMMS_DATA_6")==0) {
    *ppParamValue = PHX_COMMS_DATA_6;
  } else if (strcmp(*str,"PHX_COMMS_DATA_7")==0) {
    *ppParamValue = PHX_COMMS_DATA_7;
  } else if (strcmp(*str,"PHX_COMMS_DATA_8")==0) {
    *ppParamValue = PHX_COMMS_DATA_8;
  } else if (strcmp(*str,"PHX_COMMS_STOP_1")==0) {
    *ppParamValue = PHX_COMMS_STOP_1;
  } else if (strcmp(*str,"PHX_COMMS_STOP_1_5")==0) {
    *ppParamValue = PHX_COMMS_STOP_1_5;
  } else if (strcmp(*str,"PHX_COMMS_STOP_2")==0) {
    *ppParamValue = PHX_COMMS_STOP_2;
  } else if (strcmp(*str,"PHX_COMMS_PARITY_NONE")==0) {
    *ppParamValue = PHX_COMMS_PARITY_NONE;
  } else if (strcmp(*str,"PHX_COMMS_PARITY_EVEN")==0) {
    *ppParamValue = PHX_COMMS_PARITY_EVEN;
  } else if (strcmp(*str,"PHX_COMMS_PARITY_ODD")==0) {
    *ppParamValue = PHX_COMMS_PARITY_ODD;
  } else if (strcmp(*str,"PHX_COMMS_FLOW_NONE")==0) {
    *ppParamValue = PHX_COMMS_FLOW_NONE;
  } else if (strcmp(*str,"PHX_COMMS_FLOW_HW")==0) {
    *ppParamValue = PHX_COMMS_FLOW_HW;
  } else if (strcmp(*str,"PHX_COMMS_FLOW_SW")==0) {
    *ppParamValue = PHX_COMMS_FLOW_SW;
  } else if (strcmp(*str,"PHX_COMMS_STANDARD_RS232")==0) {
    *ppParamValue = PHX_COMMS_STANDARD_RS232;
  } else if (strcmp(*str,"PHX_COMMS_STANDARD_LVDS")==0) {
    *ppParamValue = PHX_COMMS_STANDARD_LVDS;
  } else if (strcmp(*str,"PHX_IO_OPTO_OUT1")==0) {
    *ppParamValue = PHX_IO_OPTO_OUT1;
  } else if (strcmp(*str,"PHX_IO_OPTO_OUT2")==0) {
    *ppParamValue = PHX_IO_OPTO_OUT2;
  } else if (strcmp(*str,"PHX_IO_OPTO_OUT3")==0) {
    *ppParamValue = PHX_IO_OPTO_OUT3;
  } else if (strcmp(*str,"PHX_IO_OPTO_OUT4")==0) {
    *ppParamValue = PHX_IO_OPTO_OUT4;
  } else if (strcmp(*str,"PHX_IO_OPTO1")==0) {
    *ppParamValue = PHX_IO_OPTO1;
  } else if (strcmp(*str,"PHX_IO_OPTO2")==0) {
    *ppParamValue = PHX_IO_OPTO2;
  } else if (strcmp(*str,"PHX_IO_OPTO3")==0) {
    *ppParamValue = PHX_IO_OPTO3;
  } else if (strcmp(*str,"PHX_IO_OPTO4")==0) {
    *ppParamValue = PHX_IO_OPTO4;
  } else if (strcmp(*str,"PHX_STATUS_IDLE")==0) {
    *ppParamValue = PHX_STATUS_IDLE;
  } else if (strcmp(*str,"PHX_STATUS_ACQ_IN_PROGRESS")==0) {
    *ppParamValue = PHX_STATUS_ACQ_IN_PROGRESS;
  } else if (strcmp(*str,"PHX_STATUS_WAITING_FOR_TRIGGER")==0) {
    *ppParamValue = PHX_STATUS_WAITING_FOR_TRIGGER;
  } else if (strcmp(*str,"PHX_CAM_LINESCAN_ROI")==0) {
    *ppParamValue = PHX_CAM_LINESCAN_ROI;
  } else if (strcmp(*str,"PHX_CAM_LINESCAN_NO_ROI")==0) {
    *ppParamValue = PHX_CAM_LINESCAN_NO_ROI;
  } else if (strcmp(*str,"PHX_CAM_AREASCAN_ROI")==0) {
    *ppParamValue = PHX_CAM_AREASCAN_ROI;
  } else if (strcmp(*str,"PHX_CAM_AREASCAN_NO_ROI")==0) {
    *ppParamValue = PHX_CAM_AREASCAN_NO_ROI;
  } else if (strcmp(*str,"PHX_CAM_INTERLACED")==0) {
    *ppParamValue = PHX_CAM_INTERLACED;
  } else if (strcmp(*str,"PHX_CAM_NON_INTERLACED")==0) {
    *ppParamValue = PHX_CAM_NON_INTERLACED;
  } else if (strcmp(*str,"PHX_CAM_SRC_MONO")==0) {
    *ppParamValue = PHX_CAM_SRC_MONO;
  } else if (strcmp(*str,"PHX_CAM_SRC_RGB")==0) {
    *ppParamValue = PHX_CAM_SRC_RGB;
  } else if (strcmp(*str,"PHX_CAM_SRC_BAY_RGGB")==0) {
    *ppParamValue = PHX_CAM_SRC_BAY_RGGB;
  } else if (strcmp(*str,"PHX_CAM_SRC_BAY_GRBG")==0) {
    *ppParamValue = PHX_CAM_SRC_BAY_GRBG;
  } else if (strcmp(*str,"PHX_CAM_SRC_BAY_GBRG")==0) {
    *ppParamValue = PHX_CAM_SRC_BAY_GBRG;
  } else if (strcmp(*str,"PHX_CAM_SRC_BAY_BGGR")==0) {
    *ppParamValue = PHX_CAM_SRC_BAY_BGGR;
  } else if (strcmp(*str,"PHX_CAM_SRC_YUV")==0) {
    *ppParamValue = PHX_CAM_SRC_YUV;
  } else if (strcmp(*str,"PHX_CAM_HTAP_LEFT")==0) {
    *ppParamValue = PHX_CAM_HTAP_LEFT;
  } else if (strcmp(*str,"PHX_CAM_HTAP_RIGHT")==0) {
    *ppParamValue = PHX_CAM_HTAP_RIGHT;
  } else if (strcmp(*str,"PHX_CAM_HTAP_CONVERGE")==0) {
    *ppParamValue = PHX_CAM_HTAP_CONVERGE;
  } else if (strcmp(*str,"PHX_CAM_HTAP_DIVERGE")==0) {
    *ppParamValue = PHX_CAM_HTAP_DIVERGE;
  } else if (strcmp(*str,"PHX_CAM_HTAP_BOTH")==0) {
    *ppParamValue = PHX_CAM_HTAP_BOTH;
  } else if (strcmp(*str,"PHX_CAM_HTAP_LINEAR")==0) {
    *ppParamValue = PHX_CAM_HTAP_LINEAR;
  } else if (strcmp(*str,"PHX_CAM_HTAP_OFFSET_1")==0) {
    *ppParamValue = PHX_CAM_HTAP_OFFSET_1;
  } else if (strcmp(*str,"PHX_CAM_HTAP_ALTERNATE")==0) {
    *ppParamValue = PHX_CAM_HTAP_ALTERNATE;
  } else if (strcmp(*str,"PHX_CAM_HTAP_OFFSET_2")==0) {
    *ppParamValue = PHX_CAM_HTAP_OFFSET_2;
  } else if (strcmp(*str,"PHX_CAM_HTAP_SPAN")==0) {
    *ppParamValue = PHX_CAM_HTAP_SPAN;
  } else if (strcmp(*str,"PHX_CAM_HTAP_OFFSET")==0) {
    *ppParamValue = PHX_CAM_HTAP_OFFSET;
  } else if (strcmp(*str,"PHX_CAM_HTAP_ASCENDING")==0) {
    *ppParamValue = PHX_CAM_HTAP_ASCENDING;
  } else if (strcmp(*str,"PHX_CAM_HTAP_DESCENDING")==0) {
    *ppParamValue = PHX_CAM_HTAP_DESCENDING;
  } else if (strcmp(*str,"PHX_CAM_VTAP_TOP")==0) {
    *ppParamValue = PHX_CAM_VTAP_TOP;
  } else if (strcmp(*str,"PHX_CAM_VTAP_BOTTOM")==0) {
    *ppParamValue = PHX_CAM_VTAP_BOTTOM;
  } else if (strcmp(*str,"PHX_CAM_VTAP_BOTH")==0) {
    *ppParamValue = PHX_CAM_VTAP_BOTH;
  } else if (strcmp(*str,"PHX_CAM_VTAP_LINEAR")==0) {
    *ppParamValue = PHX_CAM_VTAP_LINEAR;
  } else if (strcmp(*str,"PHX_CAM_VTAP_OFFSET")==0) {
    *ppParamValue = PHX_CAM_VTAP_OFFSET;
  } else if (strcmp(*str,"PHX_CAM_VTAP_ALTERNATE")==0) {
    *ppParamValue = PHX_CAM_VTAP_ALTERNATE;
  } else if (strcmp(*str,"PHX_CAM_VTAP_ASCENDING")==0) {
    *ppParamValue = PHX_CAM_VTAP_ASCENDING;
  } else if (strcmp(*str,"PHX_CAM_VTAP_DESCENDING")==0) {
    *ppParamValue = PHX_CAM_VTAP_DESCENDING;
  } else if (strcmp(*str,"PHX_CAM_CLOCK_POS")==0) {
    *ppParamValue = PHX_CAM_CLOCK_POS;
  } else if (strcmp(*str,"PHX_CAM_CLOCK_NEG")==0) {
    *ppParamValue = PHX_CAM_CLOCK_NEG;
  } else if (strcmp(*str,"PHX_CAM_CLOCK_MAX_DEFAULT")==0) {
    *ppParamValue = PHX_CAM_CLOCK_MAX_DEFAULT;
  } else if (strcmp(*str,"PHX_CAM_CLOCK_MAX_85MHZ")==0) {
    *ppParamValue = PHX_CAM_CLOCK_MAX_85MHZ;
  } else if (strcmp(*str,"PHX_ACQ_FRAME_12")==0) {
    *ppParamValue = PHX_ACQ_FRAME_12;
  } else if (strcmp(*str,"PHX_ACQ_FRAME_21")==0) {
    *ppParamValue = PHX_ACQ_FRAME_21;
  } else if (strcmp(*str,"PHX_ACQ_FIELD_12")==0) {
    *ppParamValue = PHX_ACQ_FIELD_12;
  } else if (strcmp(*str,"PHX_ACQ_FIELD_21")==0) {
    *ppParamValue = PHX_ACQ_FIELD_21;
  } else if (strcmp(*str,"PHX_ACQ_FIELD_1")==0) {
    *ppParamValue = PHX_ACQ_FIELD_1;
  } else if (strcmp(*str,"PHX_ACQ_FIELD_2")==0) {
    *ppParamValue = PHX_ACQ_FIELD_2;
  } else if (strcmp(*str,"PHX_ACQ_FIELD_NEXT")==0) {
    *ppParamValue = PHX_ACQ_FIELD_NEXT;
  } else if (strcmp(*str,"PHX_ACQ_LINE_DOUBLE_12")==0) {
    *ppParamValue = PHX_ACQ_LINE_DOUBLE_12;
  } else if (strcmp(*str,"PHX_ACQ_LINE_DOUBLE_21")==0) {
    *ppParamValue = PHX_ACQ_LINE_DOUBLE_21;
  } else if (strcmp(*str,"PHX_ACQ_LINE_DOUBLE_NEXT")==0) {
    *ppParamValue = PHX_ACQ_LINE_DOUBLE_NEXT;
  } else if (strcmp(*str,"PHX_ACQ_LINE_DOUBLE_1")==0) {
    *ppParamValue = PHX_ACQ_LINE_DOUBLE_1;
  } else if (strcmp(*str,"PHX_ACQ_LINE_DOUBLE_2")==0) {
    *ppParamValue = PHX_ACQ_LINE_DOUBLE_2;
  } else if (strcmp(*str,"PHX_ACQ_FRAME")==0) {
    *ppParamValue = PHX_ACQ_FRAME;
  } else if (strcmp(*str,"PHX_ACQ_X1")==0) {
    *ppParamValue = PHX_ACQ_X1;
  } else if (strcmp(*str,"PHX_ACQ_X2")==0) {
    *ppParamValue = PHX_ACQ_X2;
  } else if (strcmp(*str,"PHX_ACQ_X4")==0) {
    *ppParamValue = PHX_ACQ_X4;
  } else if (strcmp(*str,"PHX_ACQ_X8")==0) {
    *ppParamValue = PHX_ACQ_X8;
  } else if (strcmp(*str,"PHX_DST_PTR_INTERNAL")==0) {
    *ppParamValue = PHX_DST_PTR_INTERNAL;
  } else if (strcmp(*str,"PHX_DST_PTR_USER_VIRT")==0) {
    *ppParamValue = PHX_DST_PTR_USER_VIRT;
  } else if (strcmp(*str,"PHX_DST_PTR_USER_PHYS")==0) {
    *ppParamValue = PHX_DST_PTR_USER_PHYS;
  } else if (strcmp(*str,"PHX_DATASTREAM_ALWAYS")==0) {
    *ppParamValue = PHX_DATASTREAM_ALWAYS;
  } else if (strcmp(*str,"PHX_DATASTREAM_LINE_ONLY")==0) {
    *ppParamValue = PHX_DATASTREAM_LINE_ONLY;
  } else if (strcmp(*str,"PHX_DATASTREAM_FRAME_ONLY")==0) {
    *ppParamValue = PHX_DATASTREAM_FRAME_ONLY;
  } else if (strcmp(*str,"PHX_DATASTREAM_FRAME_AND_LINE")==0) {
    *ppParamValue = PHX_DATASTREAM_FRAME_AND_LINE;
  } else if (strcmp(*str,"PHX_DATASRC_CAMERA")==0) {
    *ppParamValue = PHX_DATASRC_CAMERA;
  } else if (strcmp(*str,"PHX_DATASRC_SIMULATOR_STATIC")==0) {
    *ppParamValue = PHX_DATASRC_SIMULATOR_STATIC;
  } else if (strcmp(*str,"PHX_DATASRC_SIMULATOR_ROLL")==0) {
    *ppParamValue = PHX_DATASRC_SIMULATOR_ROLL;
  } else if (strcmp(*str,"PHX_DST_FORMAT_Y8")==0) {
    *ppParamValue = PHX_DST_FORMAT_Y8;
  } else if (strcmp(*str,"PHX_DST_FORMAT_Y16")==0) {
    *ppParamValue = PHX_DST_FORMAT_Y16;
  } else if (strcmp(*str,"PHX_DST_FORMAT_Y32")==0) {
    *ppParamValue = PHX_DST_FORMAT_Y32;
  } else if (strcmp(*str,"PHX_DST_FORMAT_Y36")==0) {
    *ppParamValue = PHX_DST_FORMAT_Y36;
  } else if (strcmp(*str,"PHX_DST_FORMAT_RGB15")==0) {
    *ppParamValue = PHX_DST_FORMAT_RGB15;
  } else if (strcmp(*str,"PHX_DST_FORMAT_RGB16")==0) {
    *ppParamValue = PHX_DST_FORMAT_RGB16;
  } else if (strcmp(*str,"PHX_DST_XBGR32")==0) {
    *ppParamValue = PHX_DST_XBGR32;
  } else if (strcmp(*str,"PHX_DST_BGRX32")==0) {
    *ppParamValue = PHX_DST_BGRX32;
  } else if (strcmp(*str,"PHX_DST_FORMAT_RGB48")==0) {
    *ppParamValue = PHX_DST_FORMAT_RGB48;
  } else if (strcmp(*str,"PHX_DST_FORMAT_BGR15")==0) {
    *ppParamValue = PHX_DST_FORMAT_BGR15;
  } else if (strcmp(*str,"PHX_DST_FORMAT_BGR16")==0) {
    *ppParamValue = PHX_DST_FORMAT_BGR16;
  } else if (strcmp(*str,"PHX_DST_XRGB32")==0) {
    *ppParamValue = PHX_DST_XRGB32;
  } else if (strcmp(*str,"PHX_DST_RGBX32")==0) {
    *ppParamValue = PHX_DST_RGBX32;
  } else if (strcmp(*str,"PHX_DST_FORMAT_BGR48")==0) {
    *ppParamValue = PHX_DST_FORMAT_BGR48;
  } else if (strcmp(*str,"PHX_DST_FORMAT_RGB32")==0) {
    *ppParamValue = PHX_DST_FORMAT_RGB32;
  } else if (strcmp(*str,"PHX_DST_FORMAT_BGR32")==0) {
    *ppParamValue = PHX_DST_FORMAT_BGR32;
  } else if (strcmp(*str,"PHX_DST_FORMAT_RGB24")==0) {
    *ppParamValue = PHX_DST_FORMAT_RGB24;
  } else if (strcmp(*str,"PHX_DST_FORMAT_BGR24")==0) {
    *ppParamValue = PHX_DST_FORMAT_BGR24;
  } else if (strcmp(*str,"PHX_DST_FORMAT_Y10")==0) {
    *ppParamValue = PHX_DST_FORMAT_Y10;
  } else if (strcmp(*str,"PHX_DST_FORMAT_Y12")==0) {
    *ppParamValue = PHX_DST_FORMAT_Y12;
  } else if (strcmp(*str,"PHX_DST_FORMAT_Y14")==0) {
    *ppParamValue = PHX_DST_FORMAT_Y14;
  } else if (strcmp(*str,"PHX_DST_FORMAT_BAY8")==0) {
    *ppParamValue = PHX_DST_FORMAT_BAY8;
  } else if (strcmp(*str,"PHX_DST_FORMAT_BAY10")==0) {
    *ppParamValue = PHX_DST_FORMAT_BAY10;
  } else if (strcmp(*str,"PHX_DST_FORMAT_BAY12")==0) {
    *ppParamValue = PHX_DST_FORMAT_BAY12;
  } else if (strcmp(*str,"PHX_DST_FORMAT_BAY14")==0) {
    *ppParamValue = PHX_DST_FORMAT_BAY14;
  } else if (strcmp(*str,"PHX_DST_FORMAT_BAY16")==0) {
    *ppParamValue = PHX_DST_FORMAT_BAY16;
  } else if (strcmp(*str,"PHX_DST_FORMAT_2Y12")==0) {
    *ppParamValue = PHX_DST_FORMAT_2Y12;
  } else if (strcmp(*str,"PHX_DST_FORMAT_RGB36")==0) {
    *ppParamValue = PHX_DST_FORMAT_RGB36;
  } else if (strcmp(*str,"PHX_DST_FORMAT_BGR36")==0) {
    *ppParamValue = PHX_DST_FORMAT_BGR36;
  } else if (strcmp(*str,"PHX_DST_FORMAT_YUV422")==0) {
    *ppParamValue = PHX_DST_FORMAT_YUV422;
  } else if (strcmp(*str,"PHX_DST_FORMAT_Y12B")==0) {
    *ppParamValue = PHX_DST_FORMAT_Y12B;
  } else if (strcmp(*str,"PHX_DST_FORMAT_RGBX32")==0) {
    *ppParamValue = PHX_DST_FORMAT_RGBX32;
  } else if (strcmp(*str,"PHX_DST_FORMAT_XRGB32")==0) {
    *ppParamValue = PHX_DST_FORMAT_XRGB32;
  } else if (strcmp(*str,"PHX_DST_FORMAT_BGRX32")==0) {
    *ppParamValue = PHX_DST_FORMAT_BGRX32;
  } else if (strcmp(*str,"PHX_DST_FORMAT_XBGR32")==0) {
    *ppParamValue = PHX_DST_FORMAT_XBGR32;
  } else if (strcmp(*str,"PHX_USER_FORMAT_Y8")==0) {
    *ppParamValue = PHX_USER_FORMAT_Y8;
  } else if (strcmp(*str,"PHX_USER_FORMAT_Y16")==0) {
    *ppParamValue = PHX_USER_FORMAT_Y16;
  } else if (strcmp(*str,"PHX_USER_FORMAT_Y32")==0) {
    *ppParamValue = PHX_USER_FORMAT_Y32;
  } else if (strcmp(*str,"PHX_USER_FORMAT_Y36")==0) {
    *ppParamValue = PHX_USER_FORMAT_Y36;
  } else if (strcmp(*str,"PHX_USER_FORMAT_RGB15")==0) {
    *ppParamValue = PHX_USER_FORMAT_RGB15;
  } else if (strcmp(*str,"PHX_USER_FORMAT_RGB16")==0) {
    *ppParamValue = PHX_USER_FORMAT_RGB16;
  } else if (strcmp(*str,"PHX_USER_XBGR32")==0) {
    *ppParamValue = PHX_USER_XBGR32;
  } else if (strcmp(*str,"PHX_USER_BGRX32")==0) {
    *ppParamValue = PHX_USER_BGRX32;
  } else if (strcmp(*str,"PHX_USER_FORMAT_RGB48")==0) {
    *ppParamValue = PHX_USER_FORMAT_RGB48;
  } else if (strcmp(*str,"PHX_USER_FORMAT_BGR15")==0) {
    *ppParamValue = PHX_USER_FORMAT_BGR15;
  } else if (strcmp(*str,"PHX_USER_FORMAT_BGR16")==0) {
    *ppParamValue = PHX_USER_FORMAT_BGR16;
  } else if (strcmp(*str,"PHX_USER_XRGB32")==0) {
    *ppParamValue = PHX_USER_XRGB32;
  } else if (strcmp(*str,"PHX_USER_RGBX32")==0) {
    *ppParamValue = PHX_USER_RGBX32;
  } else if (strcmp(*str,"PHX_USER_FORMAT_BGR48")==0) {
    *ppParamValue = PHX_USER_FORMAT_BGR48;
  } else if (strcmp(*str,"PHX_USER_FORMAT_RGB32")==0) {
    *ppParamValue = PHX_USER_FORMAT_RGB32;
  } else if (strcmp(*str,"PHX_USER_FORMAT_BGR32")==0) {
    *ppParamValue = PHX_USER_FORMAT_BGR32;
  } else if (strcmp(*str,"PHX_USER_FORMAT_RGB24")==0) {
    *ppParamValue = PHX_USER_FORMAT_RGB24;
  } else if (strcmp(*str,"PHX_USER_FORMAT_BGR24")==0) {
    *ppParamValue = PHX_USER_FORMAT_BGR24;
  } else if (strcmp(*str,"PHX_USER_FORMAT_Y10")==0) {
    *ppParamValue = PHX_USER_FORMAT_Y10;
  } else if (strcmp(*str,"PHX_USER_FORMAT_Y12")==0) {
    *ppParamValue = PHX_USER_FORMAT_Y12;
  } else if (strcmp(*str,"PHX_USER_FORMAT_Y14")==0) {
    *ppParamValue = PHX_USER_FORMAT_Y14;
  } else if (strcmp(*str,"PHX_USER_FORMAT_BAY8")==0) {
    *ppParamValue = PHX_USER_FORMAT_BAY8;
  } else if (strcmp(*str,"PHX_USER_FORMAT_BAY10")==0) {
    *ppParamValue = PHX_USER_FORMAT_BAY10;
  } else if (strcmp(*str,"PHX_USER_FORMAT_BAY12")==0) {
    *ppParamValue = PHX_USER_FORMAT_BAY12;
  } else if (strcmp(*str,"PHX_USER_FORMAT_BAY14")==0) {
    *ppParamValue = PHX_USER_FORMAT_BAY14;
  } else if (strcmp(*str,"PHX_USER_FORMAT_BAY16")==0) {
    *ppParamValue = PHX_USER_FORMAT_BAY16;
  } else if (strcmp(*str,"PHX_USER_FORMAT_2Y12")==0) {
    *ppParamValue = PHX_USER_FORMAT_2Y12;
  } else if (strcmp(*str,"PHX_USER_FORMAT_RGB36")==0) {
    *ppParamValue = PHX_USER_FORMAT_RGB36;
  } else if (strcmp(*str,"PHX_USER_FORMAT_BGR36")==0) {
    *ppParamValue = PHX_USER_FORMAT_BGR36;
  } else if (strcmp(*str,"PHX_USER_FORMAT_YUV422")==0) {
    *ppParamValue = PHX_USER_FORMAT_YUV422;
  } else if (strcmp(*str,"PHX_USER_FORMAT_Y12B")==0) {
    *ppParamValue = PHX_USER_FORMAT_Y12B;
  } else if (strcmp(*str,"PHX_USER_FORMAT_RGBX32")==0) {
    *ppParamValue = PHX_USER_FORMAT_RGBX32;
  } else if (strcmp(*str,"PHX_USER_FORMAT_XRGB32")==0) {
    *ppParamValue = PHX_USER_FORMAT_XRGB32;
  } else if (strcmp(*str,"PHX_USER_FORMAT_BGRX32")==0) {
    *ppParamValue = PHX_USER_FORMAT_BGRX32;
  } else if (strcmp(*str,"PHX_USER_FORMAT_XBGR32")==0) {
    *ppParamValue = PHX_USER_FORMAT_XBGR32;
  } else if (strcmp(*str,"PHX_LINETRIG_NONE")==0) {
    *ppParamValue = PHX_LINETRIG_NONE;
  } else if (strcmp(*str,"PHX_LINETRIG_AUXIN_1_RISING")==0) {
    *ppParamValue = PHX_LINETRIG_AUXIN_1_RISING;
  } else if (strcmp(*str,"PHX_LINETRIG_AUXIN_1_FALLING")==0) {
    *ppParamValue = PHX_LINETRIG_AUXIN_1_FALLING;
  } else if (strcmp(*str,"PHX_LINETRIG_CTRLIN_2_RISING")==0) {
    *ppParamValue = PHX_LINETRIG_CTRLIN_2_RISING;
  } else if (strcmp(*str,"PHX_LINETRIG_CTRLIN_2_FALLING")==0) {
    *ppParamValue = PHX_LINETRIG_CTRLIN_2_FALLING;
  } else if (strcmp(*str,"PHX_LINETRIG_AUXIN_2_RISING")==0) {
    *ppParamValue = PHX_LINETRIG_AUXIN_2_RISING;
  } else if (strcmp(*str,"PHX_LINETRIG_AUXIN_2_FALLING")==0) {
    *ppParamValue = PHX_LINETRIG_AUXIN_2_FALLING;
  } else if (strcmp(*str,"PHX_LINETRIG_TIMER")==0) {
    *ppParamValue = PHX_LINETRIG_TIMER;
  } else if (strcmp(*str,"PHX_LINETRIG_AUXIN_A1_RISING")==0) {
    *ppParamValue = PHX_LINETRIG_AUXIN_A1_RISING;
  } else if (strcmp(*str,"PHX_LINETRIG_AUXIN_A1_FALLING")==0) {
    *ppParamValue = PHX_LINETRIG_AUXIN_A1_FALLING;
  } else if (strcmp(*str,"PHX_LINETRIG_AUXIN_A2_RISING")==0) {
    *ppParamValue = PHX_LINETRIG_AUXIN_A2_RISING;
  } else if (strcmp(*str,"PHX_LINETRIG_AUXIN_A2_FALLING")==0) {
    *ppParamValue = PHX_LINETRIG_AUXIN_A2_FALLING;
  } else if (strcmp(*str,"PHX_LINETRIG_AUXIN_B1_RISING")==0) {
    *ppParamValue = PHX_LINETRIG_AUXIN_B1_RISING;
  } else if (strcmp(*str,"PHX_LINETRIG_AUXIN_B1_FALLING")==0) {
    *ppParamValue = PHX_LINETRIG_AUXIN_B1_FALLING;
  } else if (strcmp(*str,"PHX_LINETRIG_AUXIN_B2_RISING")==0) {
    *ppParamValue = PHX_LINETRIG_AUXIN_B2_RISING;
  } else if (strcmp(*str,"PHX_LINETRIG_AUXIN_B2_FALLING")==0) {
    *ppParamValue = PHX_LINETRIG_AUXIN_B2_FALLING;
  } else if (strcmp(*str,"PHX_LINETRIG_CTRLIN_1_RISING")==0) {
    *ppParamValue = PHX_LINETRIG_CTRLIN_1_RISING;
  } else if (strcmp(*str,"PHX_LINETRIG_CTRLIN_1_FALLING")==0) {
    *ppParamValue = PHX_LINETRIG_CTRLIN_1_FALLING;
  } else if (strcmp(*str,"PHX_LINETRIG_CTRLIN_3_RISING")==0) {
    *ppParamValue = PHX_LINETRIG_CTRLIN_3_RISING;
  } else if (strcmp(*str,"PHX_LINETRIG_CTRLIN_3_FALLING")==0) {
    *ppParamValue = PHX_LINETRIG_CTRLIN_3_FALLING;
  } else if (strcmp(*str,"PHX_LINETRIG_TIMER_TIME")==0) {
    *ppParamValue = PHX_LINETRIG_TIMER_TIME;
  } else if (strcmp(*str,"PHX_LINETRIG_TIMER_DISABLE")==0) {
    *ppParamValue = PHX_LINETRIG_TIMER_DISABLE;
  } else if (strcmp(*str,"PHX_LINETRIG_TIMER_LINES")==0) {
    *ppParamValue = PHX_LINETRIG_TIMER_LINES;
  } else if (strcmp(*str,"PHX_LINETRIG_TIMER_START")==0) {
    *ppParamValue = PHX_LINETRIG_TIMER_START;
  } else if (strcmp(*str,"PHX_LINETRIG_TIMER_STOP")==0) {
    *ppParamValue = PHX_LINETRIG_TIMER_STOP;
  } else if (strcmp(*str,"PHX_EXPTRIG_LINETRIG")==0) {
    *ppParamValue = PHX_EXPTRIG_LINETRIG;
  } else if (strcmp(*str,"PHX_EXPTRIG_ACQTRIG")==0) {
    *ppParamValue = PHX_EXPTRIG_ACQTRIG;
  } else if (strcmp(*str,"PHX_EXPTRIG_NONE")==0) {
    *ppParamValue = PHX_EXPTRIG_NONE;
  } else if (strcmp(*str,"PHX_EXPTRIG_SWTRIG")==0) {
    *ppParamValue = PHX_EXPTRIG_SWTRIG;
  } else if (strcmp(*str,"PHX_EXPTRIG_AUXIN_1_RISING")==0) {
    *ppParamValue = PHX_EXPTRIG_AUXIN_1_RISING;
  } else if (strcmp(*str,"PHX_EXPTRIG_AUXIN_1_FALLING")==0) {
    *ppParamValue = PHX_EXPTRIG_AUXIN_1_FALLING;
  } else if (strcmp(*str,"PHX_EXPTRIG_AUXIN_2_RISING")==0) {
    *ppParamValue = PHX_EXPTRIG_AUXIN_2_RISING;
  } else if (strcmp(*str,"PHX_EXPTRIG_AUXIN_2_FALLING")==0) {
    *ppParamValue = PHX_EXPTRIG_AUXIN_2_FALLING;
  } else if (strcmp(*str,"PHX_EXPTRIG_TIMER")==0) {
    *ppParamValue = PHX_EXPTRIG_TIMER;
  } else if (strcmp(*str,"PHX_EXPTRIG_AUXIN_A1_RISING")==0) {
    *ppParamValue = PHX_EXPTRIG_AUXIN_A1_RISING;
  } else if (strcmp(*str,"PHX_EXPTRIG_AUXIN_A1_FALLING")==0) {
    *ppParamValue = PHX_EXPTRIG_AUXIN_A1_FALLING;
  } else if (strcmp(*str,"PHX_EXPTRIG_AUXIN_A2_RISING")==0) {
    *ppParamValue = PHX_EXPTRIG_AUXIN_A2_RISING;
  } else if (strcmp(*str,"PHX_EXPTRIG_AUXIN_A2_FALLING")==0) {
    *ppParamValue = PHX_EXPTRIG_AUXIN_A2_FALLING;
  } else if (strcmp(*str,"PHX_EXPTRIG_AUXIN_B1_RISING")==0) {
    *ppParamValue = PHX_EXPTRIG_AUXIN_B1_RISING;
  } else if (strcmp(*str,"PHX_EXPTRIG_AUXIN_B1_FALLING")==0) {
    *ppParamValue = PHX_EXPTRIG_AUXIN_B1_FALLING;
  } else if (strcmp(*str,"PHX_EXPTRIG_AUXIN_B2_RISING")==0) {
    *ppParamValue = PHX_EXPTRIG_AUXIN_B2_RISING;
  } else if (strcmp(*str,"PHX_EXPTRIG_AUXIN_B2_FALLING")==0) {
    *ppParamValue = PHX_EXPTRIG_AUXIN_B2_FALLING;
  } else if (strcmp(*str,"PHX_EXP_LINETRIG")==0) {
    *ppParamValue = PHX_EXP_LINETRIG;
  } else if (strcmp(*str,"PHX_EXP_ACQTRIG")==0) {
    *ppParamValue = PHX_EXP_ACQTRIG;
  } else if (strcmp(*str,"PHX_EXP_CTRLIO_1_HW_POS")==0) {
    *ppParamValue = PHX_EXP_CTRLIO_1_HW_POS;
  } else if (strcmp(*str,"PHX_EXP_CTRLIO_1_HW_NEG")==0) {
    *ppParamValue = PHX_EXP_CTRLIO_1_HW_NEG;
  } else if (strcmp(*str,"PHX_EXP_CTRLIO_1_SW_POS")==0) {
    *ppParamValue = PHX_EXP_CTRLIO_1_SW_POS;
  } else if (strcmp(*str,"PHX_EXP_CTRLIO_1_SW_NEG")==0) {
    *ppParamValue = PHX_EXP_CTRLIO_1_SW_NEG;
  } else if (strcmp(*str,"PHX_EXP_CTRLIO_2_HW_POS")==0) {
    *ppParamValue = PHX_EXP_CTRLIO_2_HW_POS;
  } else if (strcmp(*str,"PHX_EXP_CTRLIO_2_HW_NEG")==0) {
    *ppParamValue = PHX_EXP_CTRLIO_2_HW_NEG;
  } else if (strcmp(*str,"PHX_EXP_CTRLIO_2_SW_POS")==0) {
    *ppParamValue = PHX_EXP_CTRLIO_2_SW_POS;
  } else if (strcmp(*str,"PHX_EXP_CTRLIO_2_SW_NEG")==0) {
    *ppParamValue = PHX_EXP_CTRLIO_2_SW_NEG;
  } else if (strcmp(*str,"PHX_EXP_LINESTART_LINE")==0) {
    *ppParamValue = PHX_EXP_LINESTART_LINE;
  } else if (strcmp(*str,"PHX_EXP_LINESTART_CCIO_2")==0) {
    *ppParamValue = PHX_EXP_LINESTART_CCIO_2;
  } else if (strcmp(*str,"PHX_EXP_LINESTART_CCIO_A2")==0) {
    *ppParamValue = PHX_EXP_LINESTART_CCIO_A2;
  } else if (strcmp(*str,"PHX_EXP_LINESTART_CCIO_B2")==0) {
    *ppParamValue = PHX_EXP_LINESTART_CCIO_B2;
  } else if (strcmp(*str,"PHX_EXP_LINESTART_CTRLIO_2")==0) {
    *ppParamValue = PHX_EXP_LINESTART_CTRLIO_2;
  } else if (strcmp(*str,"PHX_ACQTRIG_OPTO_A1")==0) {
    *ppParamValue = PHX_ACQTRIG_OPTO_A1;
  } else if (strcmp(*str,"PHX_ACQTRIG_OPTO_A2")==0) {
    *ppParamValue = PHX_ACQTRIG_OPTO_A2;
  } else if (strcmp(*str,"PHX_ACQTRIG_OPTO_B1")==0) {
    *ppParamValue = PHX_ACQTRIG_OPTO_B1;
  } else if (strcmp(*str,"PHX_ACQTRIG_OPTO_B2")==0) {
    *ppParamValue = PHX_ACQTRIG_OPTO_B2;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRLIN_A1")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRLIN_A1;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRLIN_A2")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRLIN_A2;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRLIN_A3")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRLIN_A3;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRLIN_B1")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRLIN_B1;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRLIN_B2")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRLIN_B2;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRLIN_B3")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRLIN_B3;
  } else if (strcmp(*str,"PHX_ACQTRIG_CCIO_A1")==0) {
    *ppParamValue = PHX_ACQTRIG_CCIO_A1;
  } else if (strcmp(*str,"PHX_ACQTRIG_CCIO_A2")==0) {
    *ppParamValue = PHX_ACQTRIG_CCIO_A2;
  } else if (strcmp(*str,"PHX_ACQTRIG_CCIO_B1")==0) {
    *ppParamValue = PHX_ACQTRIG_CCIO_B1;
  } else if (strcmp(*str,"PHX_ACQTRIG_CCIO_B2")==0) {
    *ppParamValue = PHX_ACQTRIG_CCIO_B2;
  } else if (strcmp(*str,"PHX_ACQTRIG_AUXIN_A1")==0) {
    *ppParamValue = PHX_ACQTRIG_AUXIN_A1;
  } else if (strcmp(*str,"PHX_ACQTRIG_AUXIN_A2")==0) {
    *ppParamValue = PHX_ACQTRIG_AUXIN_A2;
  } else if (strcmp(*str,"PHX_ACQTRIG_AUXIN_B1")==0) {
    *ppParamValue = PHX_ACQTRIG_AUXIN_B1;
  } else if (strcmp(*str,"PHX_ACQTRIG_AUXIN_B2")==0) {
    *ppParamValue = PHX_ACQTRIG_AUXIN_B2;
  } else if (strcmp(*str,"PHX_ACQTRIG_OPTO_1")==0) {
    *ppParamValue = PHX_ACQTRIG_OPTO_1;
  } else if (strcmp(*str,"PHX_ACQTRIG_OPTO_2")==0) {
    *ppParamValue = PHX_ACQTRIG_OPTO_2;
  } else if (strcmp(*str,"PHX_ACQTRIG_AUXIN_1")==0) {
    *ppParamValue = PHX_ACQTRIG_AUXIN_1;
  } else if (strcmp(*str,"PHX_ACQTRIG_AUXIN_2")==0) {
    *ppParamValue = PHX_ACQTRIG_AUXIN_2;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRLIN_1")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRLIN_1;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRLIN_2")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRLIN_2;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRLIN_3")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRLIN_3;
  } else if (strcmp(*str,"PHX_ACQTRIG_CCIO_1")==0) {
    *ppParamValue = PHX_ACQTRIG_CCIO_1;
  } else if (strcmp(*str,"PHX_ACQTRIG_CCIO_2")==0) {
    *ppParamValue = PHX_ACQTRIG_CCIO_2;
  } else if (strcmp(*str,"PHX_ACQTRIG_TIMER")==0) {
    *ppParamValue = PHX_ACQTRIG_TIMER;
  } else if (strcmp(*str,"PHX_ACQTRIG_OPTO1")==0) {
    *ppParamValue = PHX_ACQTRIG_OPTO1;
  } else if (strcmp(*str,"PHX_ACQTRIG_OPTO2")==0) {
    *ppParamValue = PHX_ACQTRIG_OPTO2;
  } else if (strcmp(*str,"PHX_ACQTRIG_OPTO3")==0) {
    *ppParamValue = PHX_ACQTRIG_OPTO3;
  } else if (strcmp(*str,"PHX_ACQTRIG_OPTO4")==0) {
    *ppParamValue = PHX_ACQTRIG_OPTO4;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRL1IN_1")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRL1IN_1;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRL1IN_2")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRL1IN_2;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRL1IN_3")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRL1IN_3;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRL2IN_1")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRL2IN_1;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRL2IN_2")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRL2IN_2;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRL2IN_3")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRL2IN_3;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRLIO_1")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRLIO_1;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRLIO_2")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRLIO_2;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRLIO_3")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRLIO_3;
  } else if (strcmp(*str,"PHX_ACQTRIG_CTRLIO_4")==0) {
    *ppParamValue = PHX_ACQTRIG_CTRLIO_4;
  } else if (strcmp(*str,"PHX_ACQTRIG_NONE")==0) {
    *ppParamValue = PHX_ACQTRIG_NONE;
  } else if (strcmp(*str,"PHX_ACQTRIG_FIRST_POS_EDGE")==0) {
    *ppParamValue = PHX_ACQTRIG_FIRST_POS_EDGE;
  } else if (strcmp(*str,"PHX_ACQTRIG_FIRST_NEG_EDGE")==0) {
    *ppParamValue = PHX_ACQTRIG_FIRST_NEG_EDGE;
  } else if (strcmp(*str,"PHX_ACQTRIG_EACH_POS_EDGE")==0) {
    *ppParamValue = PHX_ACQTRIG_EACH_POS_EDGE;
  } else if (strcmp(*str,"PHX_ACQTRIG_EACH_NEG_EDGE")==0) {
    *ppParamValue = PHX_ACQTRIG_EACH_NEG_EDGE;
  } else if (strcmp(*str,"PHX_ACQTRIG_FIRST_POS_LEVEL")==0) {
    *ppParamValue = PHX_ACQTRIG_FIRST_POS_LEVEL;
  } else if (strcmp(*str,"PHX_ACQTRIG_FIRST_NEG_LEVEL")==0) {
    *ppParamValue = PHX_ACQTRIG_FIRST_NEG_LEVEL;
  } else if (strcmp(*str,"PHX_ACQTRIG_EACH_POS_LEVEL")==0) {
    *ppParamValue = PHX_ACQTRIG_EACH_POS_LEVEL;
  } else if (strcmp(*str,"PHX_ACQTRIG_EACH_NEG_LEVEL")==0) {
    *ppParamValue = PHX_ACQTRIG_EACH_NEG_LEVEL;
  } else if (strcmp(*str,"PHX_ACQTRIG_GATED_POS_LEVEL")==0) {
    *ppParamValue = PHX_ACQTRIG_GATED_POS_LEVEL;
  } else if (strcmp(*str,"PHX_ACQTRIG_GATED_NEG_LEVEL")==0) {
    *ppParamValue = PHX_ACQTRIG_GATED_NEG_LEVEL;
  } else if (strcmp(*str,"PHX_ACQTRIG_ALIGN_NONE")==0) {
    *ppParamValue = PHX_ACQTRIG_ALIGN_NONE;
  } else if (strcmp(*str,"PHX_ACQTRIG_ALIGN_TO_CLK")==0) {
    *ppParamValue = PHX_ACQTRIG_ALIGN_TO_CLK;
  } else if (strcmp(*str,"PHX_ACQTRIG_ALIGN_TO_LINE")==0) {
    *ppParamValue = PHX_ACQTRIG_ALIGN_TO_LINE;
  } else if (strcmp(*str,"PHX_ACQTRIG_ALIGN_TO_FRAME")==0) {
    *ppParamValue = PHX_ACQTRIG_ALIGN_TO_FRAME;
  } else if (strcmp(*str,"PHX_ACQTRIG_DELAY_NONE")==0) {
    *ppParamValue = PHX_ACQTRIG_DELAY_NONE;
  } else if (strcmp(*str,"PHX_ACQTRIG_DELAY_LINE")==0) {
    *ppParamValue = PHX_ACQTRIG_DELAY_LINE;
  } else if (strcmp(*str,"PHX_ACQTRIG_DELAY_TIMER")==0) {
    *ppParamValue = PHX_ACQTRIG_DELAY_TIMER;
  } else if (strcmp(*str,"PHX_EVENTCOUNT_LINE")==0) {
    *ppParamValue = PHX_EVENTCOUNT_LINE;
  } else if (strcmp(*str,"PHX_EVENTCOUNT_FRAME")==0) {
    *ppParamValue = PHX_EVENTCOUNT_FRAME;
  } else if (strcmp(*str,"PHX_EVENTCOUNT_TIME")==0) {
    *ppParamValue = PHX_EVENTCOUNT_TIME;
  } else if (strcmp(*str,"PHX_EVENTGATE_ACQTRIG")==0) {
    *ppParamValue = PHX_EVENTGATE_ACQTRIG;
  } else if (strcmp(*str,"PHX_EVENTGATE_FRAME")==0) {
    *ppParamValue = PHX_EVENTGATE_FRAME;
  } else if (strcmp(*str,"PHX_EVENTGATE_ACQ")==0) {
    *ppParamValue = PHX_EVENTGATE_ACQ;
  } else if (strcmp(*str,"PHX_EVENTGATE_LINE")==0) {
    *ppParamValue = PHX_EVENTGATE_LINE;
  } else if (strcmp(*str,"PHX_EVENTGATE_START")==0) {
    *ppParamValue = PHX_EVENTGATE_START;
  } else if (strcmp(*str,"PHX_DST_LITTLE_ENDIAN")==0) {
    *ppParamValue = PHX_DST_LITTLE_ENDIAN;
  } else if (strcmp(*str,"PHX_DST_BIG_ENDIAN")==0) {
    *ppParamValue = PHX_DST_BIG_ENDIAN;
  } else if (strcmp(*str,"PHX_CHAN_SYNC_NONE")==0) {
    *ppParamValue = PHX_CHAN_SYNC_NONE;
  } else if (strcmp(*str,"PHX_CHAN_SYNC_ACQEXPTRIG")==0) {
    *ppParamValue = PHX_CHAN_SYNC_ACQEXPTRIG;
  } else if (strcmp(*str,"PHX_CVB_WIDTH")==0) {
    *ppParamValue = PHX_CVB_WIDTH;
  } else if (strcmp(*str,"PHX_CVB_HEIGHT")==0) {
    *ppParamValue = PHX_CVB_HEIGHT;
  } else if (strcmp(*str,"PHX_CVB_PLANES")==0) {
    *ppParamValue = PHX_CVB_PLANES;
  } else if (strcmp(*str,"PHX_CVB_BIT_DEPTH")==0) {
    *ppParamValue = PHX_CVB_BIT_DEPTH;
  } else if (strcmp(*str,"PHX_CVB_BYTES_PER_PIXEL")==0) {
    *ppParamValue = PHX_CVB_BYTES_PER_PIXEL;
  } else if (strcmp(*str,"PHX_CVB_X_STEP")==0) {
    *ppParamValue = PHX_CVB_X_STEP;
  } else if (strcmp(*str,"PHX_CVB_Y_STEP")==0) {
    *ppParamValue = PHX_CVB_Y_STEP;
  } else if (strcmp(*str,"PHX_CVB_PLANE_STEP")==0) {
    *ppParamValue = PHX_CVB_PLANE_STEP;
  } else if (strcmp(*str,"PHX_CVB_MALLOC")==0) {
    *ppParamValue = PHX_CVB_MALLOC;
  } else if (strcmp(*str,"PHX_ACQ_AUTO_NONE")==0) {
    *ppParamValue = PHX_ACQ_AUTO_NONE;
  } else if (strcmp(*str,"PHX_ACQ_AUTO_SYNC_LOST")==0) {
    *ppParamValue = PHX_ACQ_AUTO_SYNC_LOST;
  } else if (strcmp(*str,"PHX_ACQ_AUTO_FIFO_OVERFLOW")==0) {
    *ppParamValue = PHX_ACQ_AUTO_FIFO_OVERFLOW;
  } else if (strcmp(*str,"PHX_BOARD_DIGITAL")==0) {
    *ppParamValue = PHX_BOARD_DIGITAL;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D24CL_PE1")==0) {
    *ppParamValue = PHX_BOARD_PHX_D24CL_PE1;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D48CL_PE1")==0) {
    *ppParamValue = PHX_BOARD_PHX_D48CL_PE1;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D48CL_PE4")==0) {
    *ppParamValue = PHX_BOARD_PHX_D48CL_PE4;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D64CL_PE4")==0) {
    *ppParamValue = PHX_BOARD_PHX_D64CL_PE4;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D24CL_PCI32")==0) {
    *ppParamValue = PHX_BOARD_PHX_D24CL_PCI32;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D48CL_PCI32")==0) {
    *ppParamValue = PHX_BOARD_PHX_D48CL_PCI32;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D48CL_PCI64")==0) {
    *ppParamValue = PHX_BOARD_PHX_D48CL_PCI64;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D48CL_PCI64U")==0) {
    *ppParamValue = PHX_BOARD_PHX_D48CL_PCI64U;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D10HDSDI_PE1")==0) {
    *ppParamValue = PHX_BOARD_PHX_D10HDSDI_PE1;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D20HDSDI_PE1")==0) {
    *ppParamValue = PHX_BOARD_PHX_D20HDSDI_PE1;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D10HDSDI_PE4")==0) {
    *ppParamValue = PHX_BOARD_PHX_D10HDSDI_PE4;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D20HDSDI_PE4")==0) {
    *ppParamValue = PHX_BOARD_PHX_D20HDSDI_PE4;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D36_PE1")==0) {
    *ppParamValue = PHX_BOARD_PHX_D36_PE1;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D36_PE4")==0) {
    *ppParamValue = PHX_BOARD_PHX_D36_PE4;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D32_PCI32")==0) {
    *ppParamValue = PHX_BOARD_PHX_D32_PCI32;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D36_PCI32")==0) {
    *ppParamValue = PHX_BOARD_PHX_D36_PCI32;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D36_PCI64")==0) {
    *ppParamValue = PHX_BOARD_PHX_D36_PCI64;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D36_PCI64U")==0) {
    *ppParamValue = PHX_BOARD_PHX_D36_PCI64U;
  } else if (strcmp(*str,"PHX_BOARD_PHX_D24AVDS_PE1")==0) {
    *ppParamValue = PHX_BOARD_PHX_D24AVDS_PE1;
  } else if (strcmp(*str,"PHX_BOARD_NUMBER_AUTO")==0) {
    *ppParamValue = PHX_BOARD_NUMBER_AUTO;
  } else if (strcmp(*str,"PHX_BOARD_NUMBER_1")==0) {
    *ppParamValue = PHX_BOARD_NUMBER_1;
  } else if (strcmp(*str,"PHX_BOARD_NUMBER_2")==0) {
    *ppParamValue = PHX_BOARD_NUMBER_2;
  } else if (strcmp(*str,"PHX_BOARD_NUMBER_3")==0) {
    *ppParamValue = PHX_BOARD_NUMBER_3;
  } else if (strcmp(*str,"PHX_BOARD_NUMBER_4")==0) {
    *ppParamValue = PHX_BOARD_NUMBER_4;
  } else if (strcmp(*str,"PHX_BOARD_NUMBER_5")==0) {
    *ppParamValue = PHX_BOARD_NUMBER_5;
  } else if (strcmp(*str,"PHX_BOARD_NUMBER_6")==0) {
    *ppParamValue = PHX_BOARD_NUMBER_6;
  } else if (strcmp(*str,"PHX_BOARD_NUMBER_7")==0) {
    *ppParamValue = PHX_BOARD_NUMBER_7;
  } else if (strcmp(*str,"PHX_CHANNEL_NUMBER_AUTO")==0) {
    *ppParamValue = PHX_CHANNEL_NUMBER_AUTO;
  } else if (strcmp(*str,"PHX_CHANNEL_NUMBER_1")==0) {
    *ppParamValue = PHX_CHANNEL_NUMBER_1;
  } else if (strcmp(*str,"PHX_CHANNEL_NUMBER_2")==0) {
    *ppParamValue = PHX_CHANNEL_NUMBER_2;
  } else if (strcmp(*str,"PHX_CONFIG_NORMAL")==0) {
    *ppParamValue = PHX_CONFIG_NORMAL;
  } else if (strcmp(*str,"PHX_CONFIG_COMMS_ONLY")==0) {
    *ppParamValue = PHX_CONFIG_COMMS_ONLY;
  } else if (strcmp(*str,"PHX_CONFIG_ACQ_ONLY")==0) {
    *ppParamValue = PHX_CONFIG_ACQ_ONLY;
  } else if (strcmp(*str,"PHX_INTRPT_TEST")==0) {
    *ppParamValue = PHX_INTRPT_TEST;
  } else if (strcmp(*str,"PHX_INTRPT_DMA")==0) {
    *ppParamValue = PHX_INTRPT_DMA;
  } else if (strcmp(*str,"PHX_INTRPT_BUFFER_READY")==0) {
    *ppParamValue = PHX_INTRPT_BUFFER_READY;
  } else if (strcmp(*str,"PHX_INTRPT_FIFO_OVERFLOW")==0) {
    *ppParamValue = PHX_INTRPT_FIFO_OVERFLOW;
  } else if (strcmp(*str,"PHX_INTRPT_FIFO_A_OVERFLOW")==0) {
    *ppParamValue = PHX_INTRPT_FIFO_A_OVERFLOW;
  } else if (strcmp(*str,"PHX_INTRPT_FRAME_LOST")==0) {
    *ppParamValue = PHX_INTRPT_FRAME_LOST;
  } else if (strcmp(*str,"PHX_INTRPT_CAPTURE_COMPLETE")==0) {
    *ppParamValue = PHX_INTRPT_CAPTURE_COMPLETE;
  } else if (strcmp(*str,"PHX_INTRPT_FRAME_START")==0) {
    *ppParamValue = PHX_INTRPT_FRAME_START;
  } else if (strcmp(*str,"PHX_INTRPT_FRAME_END")==0) {
    *ppParamValue = PHX_INTRPT_FRAME_END;
  } else if (strcmp(*str,"PHX_INTRPT_LINE_START")==0) {
    *ppParamValue = PHX_INTRPT_LINE_START;
  } else if (strcmp(*str,"PHX_INTRPT_LINE_END")==0) {
    *ppParamValue = PHX_INTRPT_LINE_END;
  } else if (strcmp(*str,"PHX_INTRPT_ACQ_TRIG_START")==0) {
    *ppParamValue = PHX_INTRPT_ACQ_TRIG_START;
  } else if (strcmp(*str,"PHX_INTRPT_ACQ_TRIG_END")==0) {
    *ppParamValue = PHX_INTRPT_ACQ_TRIG_END;
  } else if (strcmp(*str,"PHX_INTRPT_TIMEOUT")==0) {
    *ppParamValue = PHX_INTRPT_TIMEOUT;
  } else if (strcmp(*str,"PHX_INTRPT_SYNC_LOST")==0) {
    *ppParamValue = PHX_INTRPT_SYNC_LOST;
  } else if (strcmp(*str,"PHX_INTRPT_TIMER")==0) {
    *ppParamValue = PHX_INTRPT_TIMER;
  } else if (strcmp(*str,"PHX_INTRPT_GLOBAL_ENABLE")==0) {
    *ppParamValue = PHX_INTRPT_GLOBAL_ENABLE;
  } else {
    *ppParamValue = atol(*str);
    return 0;
  }
  return 1;
}


int PHX_str_to_etParamValues(char** str, etParamValue* ppParamValue) {
  char *token;
  etParamValue temp_pParamValue;
  char char_pipe[2] = "|";
  *ppParamValue = 0;
#if defined _VERBOSE
  printf("[");
#endif
  token = strtok(*str, char_pipe);
  while( token != NULL ) { /* cycle through other tokens */
#if defined _VERBOSE
    printf("%s:",token);
#endif
    if (PHX_str_to_etParamValue(&token, &temp_pParamValue)<0) {
      return 0;
    } else {
      *ppParamValue |= temp_pParamValue;
#if defined _VERBOSE
      printf("%d:%d ,", temp_pParamValue, *ppParamValue);
#endif
    }
    token = strtok(NULL, char_pipe);
  }
#if defined _VERBOSE
  printf("\b]\n");
#endif
  return 1;
}
