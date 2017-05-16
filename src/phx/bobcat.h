/*! \file bobcat.h
    \brief Bobcat camera controls.*/

#ifndef _BOBCAT
  #define _BOBCAT

  #include <phx_api.h>    /* Main Phoenix library */

  typedef enum{
    BOBCAT_INVALID_PARAM								= (ui16) 0,

    BOBCAT_INFO_TEST_REGISTER 					= (ui16) 0x600C,
    BOBCAT_SOFT_RESET 									= (ui16) 0x601C,
    BOBCAT_INFO_FRM_NUM 								= (ui16) 0x6098,

    /* Saving and restoring registers */
    BOBCAT_BOOT_FROM										= (ui16) 0x6000,
    BOBCAT_LOAD_FROM_FACTORY						= (ui16) 0x6060,
    BOBCAT_LOAD_FROM_USR1								= (ui16) 0x6064,
    BOBCAT_LOAD_FROM_USR2								= (ui16) 0x6068,
    BOBCAT_LOAD_MFG_DEFAULT_GAIN				= (ui16) 0x606C,
    BOBCAT_SAVE_TO_USR1									= (ui16) 0x6074,
    BOBCAT_SAVE_TO_USR2									= (ui16) 0x6078,
    BOBCAT_BAUD_RATE_SELECT							= (ui16) 0x0604,

    /* Camera Manufacturere data */
    BOBCAT_MFG_FW_REV 									= (ui16) 0x6004,
    BOBCAT_MFG_FW_BUILD 								= (ui16) 0x6008,
    BOBCAT_MFG_PART_NUM0 								= (ui16) 0x7004,
    BOBCAT_MFG_PART_NUM1 								= (ui16) 0x7008,
    BOBCAT_MFG_PART_NUM2 								= (ui16) 0x700C,
    BOBCAT_MFG_PART_NUM3 								= (ui16) 0x7010,
    BOBCAT_MFG_CAM_SN0									= (ui16) 0x7014,
    BOBCAT_MFG_CAM_SN1									= (ui16) 0x7018,
    BOBCAT_MFG_CCD_SN0									= (ui16) 0x701C,
    BOBCAT_MFG_CCD_SN1									= (ui16) 0x7020,
    BOBCAT_MFG_DATE0										= (ui16) 0x7024,
    BOBCAT_MFG_DATE1										= (ui16) 0x7028,
    BOBCAT_MFG_CAM_TYPE0								= (ui16) 0x702C,
    BOBCAT_MFG_CAM_TYPE1								= (ui16) 0x7030,
    BOBCAT_MFG_CAM_TYPE2								= (ui16) 0x7034,
    BOBCAT_MFG_CAM_TYPE3								= (ui16) 0x7038,

    /* Camera information registers */
    BOBCAT_INFO_MIN_MAX_XLENGTHS 				= (ui16) 0x6080,
    BOBCAT_INFO_MIN_MAX_YLENGTHS 				= (ui16) 0x6084,
    BOBCAT_INFO_XYLENGTHS 							= (ui16) 0x60B0,
    BOBCAT_INFO_MIN_FRM_TIME 						= (ui16) 0x6088,
    BOBCAT_INFO_FRM_TIME								= (ui16) 0x60A0,
    BOBCAT_INFO_MIN_LN_TIME 						= (ui16) 0x608C,
    BOBCAT_INFO_MIN_EXP_TIME 						= (ui16) 0x6094,
    BOBCAT_INFO_EXP_TIME 								= (ui16) 0x609C,
    BOBCAT_INFO_MAX_EXP_TIME 						= (ui16) 0x6090,
    BOBCAT_INFO_AEC_AGC_STATUS 					= (ui16) 0x60B4,
    BOBCAT_INFO_CCD_TEMP 								= (ui16) 0x6010,

    /* Image size (AOI) registers*/
    BOBCAT_SCAN_MODE 										= (ui16) 0x0504,
    BOBCAT_YBINNING 										= (ui16) 0x0500,
    BOBCAT_XBINNING 										= (ui16) 0x0204,
    BOBCAT_MAOI_ENABLE 									= (ui16) 0x0208,
    BOBCAT_AOI1_ENABLE 									= (ui16) 0x020C,
    BOBCAT_AOI2_ENABLE 									= (ui16) 0x0210,
    BOBCAT_AOI3_ENABLE 									= (ui16) 0x0214,
    BOBCAT_AOI4_ENABLE 									= (ui16) 0x0218,
    BOBCAT_AOI5_ENABLE 									= (ui16) 0x021C,
    BOBCAT_AOI6_ENABLE 									= (ui16) 0x0220,
    BOBCAT_MAOI_XLENGTH 								= (ui16) 0x0228,
    BOBCAT_AOI1_XLENGTH 								= (ui16) 0x022C,
    BOBCAT_AOI2_XLENGTH 								= (ui16) 0x0230,
    BOBCAT_AOI3_XLENGTH 								= (ui16) 0x0234,
    BOBCAT_AOI4_XLENGTH 								= (ui16) 0x0238,
    BOBCAT_AOI5_XLENGTH 								= (ui16) 0x023C,
    BOBCAT_AOI6_XLENGTH 								= (ui16) 0x0240,
    BOBCAT_MAOI_XOFFSET 								= (ui16) 0x0248,
    BOBCAT_AOI1_XOFFSET 								= (ui16) 0x024C,
    BOBCAT_AOI2_XOFFSET 								= (ui16) 0x0250,
    BOBCAT_AOI3_XOFFSET 								= (ui16) 0x0254,
    BOBCAT_AOI4_XOFFSET 								= (ui16) 0x0258,
    BOBCAT_AOI5_XOFFSET 								= (ui16) 0x025C,
    BOBCAT_AOI6_XOFFSET 								= (ui16) 0x0260,
    BOBCAT_MAOI_YLENGTH 								= (ui16) 0x0268,
    BOBCAT_AOI1_YLENGTH 								= (ui16) 0x026C,
    BOBCAT_AOI2_YLENGTH 								= (ui16) 0x0270,
    BOBCAT_AOI3_YLENGTH 								= (ui16) 0x0274,
    BOBCAT_AOI4_YLENGTH 								= (ui16) 0x0278,
    BOBCAT_AOI5_YLENGTH 								= (ui16) 0x027C,
    BOBCAT_AOI6_YLENGTH 								= (ui16) 0x0280,
    BOBCAT_MAOI_YOFFSET 								= (ui16) 0x0288,
    BOBCAT_AOI1_YOFFSET 								= (ui16) 0x028C,
    BOBCAT_AOI2_YOFFSET 								= (ui16) 0x0290,
    BOBCAT_AOI3_YOFFSET 								= (ui16) 0x0294,
    BOBCAT_AOI4_YOFFSET 								= (ui16) 0x0298,
    BOBCAT_AOI5_YOFFSET 								= (ui16) 0x029C,
    BOBCAT_AOI6_YOFFSET 								= (ui16) 0x02A0,
    BOBCAT_CONST_FRM_RATE_ENABLE				=	(ui16) 0x0200,
    BOBCAT_CAM_LVAL_SIZE_MODE 					= (ui16) 0x05A8,
    BOBCAT_CAM_FVAL_SIZE_MODE 					= (ui16) 0x05AC,

    /* Exposure control mode registers */
    BOBCAT_EXP_CTRL_MODE 								= (ui16) 0x0544,
    BOBCAT_EXP_TIME 										= (ui16) 0x0548,
    BOBCAT_PROG_LN_TIME_ENABLE 					= (ui16) 0x054C,
    BOBCAT_PROG_FRM_TIME_ENABLE 				= (ui16) 0x0550,
    BOBCAT_LN_TIME 											= (ui16) 0x0554,
    BOBCAT_FRM_TIME 										= (ui16) 0x0558,
    BOBCAT_SPEED 												= (ui16) 0x0608,

    /* AEC, AGC, AIC registers */
    BOBCAT_EXP_CTRL_AUTO 								= (ui16) 0x0150,
    BOBCAT_EXP_MAX_LIMIT 								= (ui16) 0x05B0,
    BOBCAT_EXP_CORR_SPEED 							= (ui16) 0x0174,
    BOBCAT_AUTO_GAIN_CTRL 							= (ui16) 0x0154,
    BOBCAT_GAIN_MAX_LIMIT 							= (ui16) 0x0160,
    BOBCAT_GAIN_CORR_SPEED 							= (ui16) 0x0178,
    BOBCAT_LUMINANCE_LVL_THRES					= (ui16) 0x0158,
    BOBCAT_LUMINANCE_TYPE								= (ui16) 0x017C,
    BOBCAT_AOI_XOFFSET 									= (ui16) 0x0168,
    BOBCAT_AOI_XLENGTH 									= (ui16) 0x0164,
    BOBCAT_AOI_YOFFSET 									= (ui16) 0x0170,
    BOBCAT_AOI_YLENGTH 									= (ui16) 0x016C,
    BOBCAT_AUTO_IRIS_CTRL 							= (ui16) 0x014C,

    /* Video registers */
    BOBCAT_PRE_AMP_GAIN_CH1 						= (ui16) 0x0000,
    BOBCAT_ANALOG_GAIN_CH1 							= (ui16) 0x0004,
    BOBCAT_ANALOG_OFFSET_CH1 						= (ui16) 0x0008,
    BOBCAT_PRE_AMP_GAIN_CH2 						= (ui16) 0x0010,
    BOBCAT_ANALOG_GAIN_CH2 							= (ui16) 0x0014,
    BOBCAT_ANALOG_OFFSET_CH2 						= (ui16) 0x0018,
    BOBCAT_PRE_AMP_GAIN_CH3 						= (ui16) 0x0020,
    BOBCAT_ANALOG_GAIN_CH3 							= (ui16) 0x0024,
    BOBCAT_ANALOG_OFFSET_CH3 						= (ui16) 0x0028,
    BOBCAT_PRE_AMP_GAIN_CH4 						= (ui16) 0x0030,
    BOBCAT_ANALOG_GAIN_CH4 							= (ui16) 0x0034,
    BOBCAT_ANALOG_OFFSET_CH4 						= (ui16) 0x0038,
    BOBCAT_DIGITAL_GAIN 								= (ui16) 0x0180,
    BOBCAT_DIGITAL_OFFSET 							= (ui16) 0x0184,
    BOBCAT_BLCK_LVL_CORR 								= (ui16) 0x0114,
    BOBCAT_TAP_BALANCE 									= (ui16) 0x0110,

    /* Trigger registers */
    BOBCAT_TRIG_INPUT_SELECT 						= (ui16) 0x0508,
    BOBCAT_TRIG_INPUT_MODE 							= (ui16) 0x05BC,
    BOBCAT_SW_TRIG_START 								= (ui16) 0x6030,
    BOBCAT_TRIG_EDGE_SELECT 						= (ui16) 0x050C,
    BOBCAT_DEBOUNCE_TIME_SELECT 				= (ui16) 0x0510,
    BOBCAT_TRIG_OVERLAP 								= (ui16) 0x0514,
    BOBCAT_TRIG_MODE_SELECT 						= (ui16) 0x0518,
    BOBCAT_NUM_OF_FRMS_CAPTURED 				= (ui16) 0x051C,
    BOBCAT_NUM_OF_PULSES_USED 					= (ui16) 0x0520,
    BOBCAT_TRIG_EXP_DELAY 							= (ui16) 0x0528,
    BOBCAT_TRIG_STROBE_ENABLE 					= (ui16) 0x0524,
    BOBCAT_TRIG_STROBE_DELAY 						= (ui16) 0x052C,
    BOBCAT_TRIG_STROBE_DURATION 				= (ui16) 0x05B8,

    /* Pulse generator registers */
    BOBCAT_PULSE_GEN_GRANULARITY 				= (ui16) 0x0530,
    BOBCAT_PULSE_GEN_PULSE_WIDTH 				= (ui16) 0x0534,
    BOBCAT_PULSE_GEN_PULSE_PERIOD 			= (ui16) 0x0538,
    BOBCAT_PULSE_GEN_NO_OF_PULSE 				= (ui16) 0x053C,
    BOBCAT_PULSE_GEN_ENABLE 						= (ui16) 0x0540,

    /* Test pattern registers */
    BOBCAT_TEST_MODE_SELECT 						= (ui16) 0x012C,
    BOBCAT_H_V_LNS_OVERLAY 							= (ui16) 0x0130,
    BOBCAT_CROSS_OVERLAY 								= (ui16) 0x0134,
    BOBCAT_H1_OVERLAY_LN_POSITION 			= (ui16) 0x0138,
    BOBCAT_H2_OVERLAY_LN_POSITION 			= (ui16) 0x013C,
    BOBCAT_V1_OVERLAY_LN_POSITION 			= (ui16) 0x0140,
    BOBCAT_V2_OVERLAY_LN_POSITION 			= (ui16) 0x0144,
    BOBCAT_OVERLAY_LNS_BRIGHTNESS 			= (ui16) 0x0148,

    /* Input output registers */
    BOBCAT_CC1_POLARITY_SELECT 					= (ui16) 0x0570,
    BOBCAT_CC1_INPUT_SELECT 						= (ui16) 0x0574,
    BOBCAT_CC2_POLARITY_SELECT 					= (ui16) 0x0578,
    BOBCAT_CC2_INPUT_SELECT 						= (ui16) 0x057C,
    BOBCAT_IN1_POLARITY_SELECT 					= (ui16) 0x0580,
    BOBCAT_IN1_INPUT_SELECT 						= (ui16) 0x0584,
    BOBCAT_IN2_POLARITY_SELECT 					= (ui16) 0x0588,
    BOBCAT_IN2_INPUT_SELECT 						= (ui16) 0x058C,
    BOBCAT_OUT1_POLARITY_SELECT 				= (ui16) 0x0590,
    BOBCAT_OUT1_OUTPUT_SELECT 					= (ui16) 0x0594,
    BOBCAT_OUT2_POLARITY_SELECT 				= (ui16) 0x0598,
    BOBCAT_OUT2_OUTPUT_SELECT 					= (ui16) 0x059C,
    BOBCAT_CLSP_POLARITY_SELECT 				= (ui16) 0x05A0,
    BOBCAT_CLSP_OUTPUT_SELECT 					= (ui16) 0x05A4,
    BOBCAT_STROBE1_MODE_SELECT 					= (ui16) 0x055C,
    BOBCAT_STROBE2_MODE_SELECT 					= (ui16) 0x0560,
    BOBCAT_STROBE1_DURATION 						= (ui16) 0x0564,
    BOBCAT_STROBE2_DURATION 						= (ui16) 0x05B4,
    BOBCAT_STROBE1_POSITION 						= (ui16) 0x0568,
    BOBCAT_STROBE2_POSITION 						= (ui16) 0x056C,

    /* Output data format registers */
    BOBCAT_BIT_DEPTH_SELECT 						= (ui16) 0x0100,
    BOBCAT_BIT_SHIFT_SELECT 						= (ui16) 0x0104,
    BOBCAT_TAP_MODE_SELECT 							= (ui16) 0x0108,
    BOBCAT_DATA_FORMAT_SELECT 					= (ui16) 0x010C,
    BOBCAT_NEG_IMG 											= (ui16) 0x0188,

    /* White Balance registers */
    BOBCAT_WB_MODE 											= (ui16) 0x0300,
    BOBCAT_WBC_RED 											= (ui16) 0x0304,
    BOBCAT_WBC_GREEN 										= (ui16) 0x0308,
    BOBCAT_WBC_BLUE 										= (ui16) 0x030C,

    /* color correction registers*/
    BOBCAT_GAIN_RED 										= (ui16) 0x0310,
    BOBCAT_GAIN_GREEN 									= (ui16) 0x0314,
    BOBCAT_GAIN_BLUE 										= (ui16) 0x0318,
    BOBCAT_OFFSET_RED 									= (ui16) 0x031C,
    BOBCAT_OFFSET_GREEN 								= (ui16) 0x0320,
    BOBCAT_OFFSET_BLUE 									= (ui16) 0x0324,

    /* Data correction registers */
    BOBCAT_LUT_SELECT 									= (ui16) 0x0118,
    BOBCAT_LUT_ENABLE 									= (ui16) 0x011C,
    BOBCAT_DEFECTIVE_PIXEL_CORR					= (ui16) 0x0120,
    BOBCAT_HOT_PIXEL_CORR								= (ui16) 0x0124,
    BOBCAT_FLAT_FIELD_CORR							= (ui16) 0x0128,

    /* Processing registers */
    BOBCAT_ENHANCEMENT_MODE 						= (ui16) 0x0400,
    BOBCAT_POINT_X1_POSITION 						= (ui16) 0x0404,
    BOBCAT_POINT_X2_POSITION 						= (ui16) 0x0408,
    BOBCAT_POINT_Y1_POSITION 						= (ui16) 0x040C,
    BOBCAT_POINT_Y2_POSITION 						= (ui16) 0x0410,
    BOBCAT_PAOI_ENABLE 									= (ui16) 0x0224,
    BOBCAT_PAOI_XLENGTH 								= (ui16) 0x0244,
    BOBCAT_PAOI_YLENGTH 								= (ui16) 0x0284,
    BOBCAT_PAOI_XOFFSET 								= (ui16) 0x0264,
    BOBCAT_PAOI_YOFFSET 								= (ui16) 0x02A4

  } bobcatParam;

  typedef enum{
    BOBCAT_INVALID_PARAMVALUE						= (ui32) 0,

    /*BOBCAT_MAOI_ENABLE, BOBCAT_CONST_FRAMERATE_ENABLE, BOBCAT_PROG_LN_TIME_ENABLE,
    BOBCAT_PROG_FRM_TIME_ENABLE, BOBCAT_AUTO_IRIS_CTRL, BOBCAT_BLCK_LVL_CORR,
    BOBCAT_TRIG_INPUT_MODE, BOBCAT_PULSE_GEN_ENABLE, BOBCAT_H_V_LNS_OVERLAY,
    BOBCAT_CROSS_OVERLAY, BOBCAT_NEG_IMG, BOBCAT_LUT_ENABLE, BOBCAT_FLAT_FIELD_CORR*/
    BOBCAT_DISABLE											= (ui32) 0,
    BOBCAT_ENABLE												= (ui32) 1,

    /* Saving and restoring registers */
    /*BOBCAT_BOOT_FROM*/
    BOBCAT_BOOT_FROM_FACTORY						= (ui32) 0x0,
    BOBCAT_BOOT_FROM_USER1							= (ui32) 0x1,
    BOBCAT_BOOT_FROM_USER2							= (ui32) 0x2,
    /*BOBCAT_BAUD_RATE_SELECT*/
    BOBCAT_BAUD_RATE_9600								= (ui32) 0x0,
    BOBCAT_BAUD_RATE_19200							= (ui32) 0x1,
    BOBCAT_BAUD_RATE_38400							= (ui32) 0x2,
    BOBCAT_BAUD_RATE_57600							= (ui32) 0x3,
    BOBCAT_BAUD_RATE_115200							= (ui32) 0x4,

    /* Image size (AOI) registers*/
    /*BOBCAT_SCAN_MODE*/
    BOBCAT_SCAN_MODE_NORMAL							= (ui32) 0x0,
    BOBCAT_SCAN_MODE_CENTER							= (ui32) 0x1,
    /*BOBCAT_YBINNING, BOBCAT_XBINNING*/
    BOBCAT_BINNING_1X 									= (ui32) 0x0,
    BOBCAT_BINNING_2X 									= (ui32) 0x1,
    BOBCAT_BINNING_3X 									= (ui32) 0x2,
    BOBCAT_BINNING_4X 									= (ui32) 0x3,
    BOBCAT_BINNING_8X 									= (ui32) 0x4,
    /*BOBCAT_AOI1_ENABLE, BOBCAT_AOI2_ENABLE, BOBCAT_AOI3_ENABLE, BOBCAT_AOI4_ENABLE, BOBCAT_AOI5_ENABLE, BOBCAT_AOI6_ENABLE*/
    BOBCAT_INCLUDE											= (ui32) 0x1,
    BOBCAT_EXCLUDE											= (ui32) 0x2,
    /*BOBCAT_CAM_LVAL_SIZE_MODE, BOBCAT_CAM_FVAL_SIZE_MODE*/
    BOBCAT_ACTIVE_ONLY									= (ui32) 0x0,
    BOBCAT_ALL_VISIBLE									= (ui32) 0x1,

    /* Exposure control mode registers */
    /*BOBCAT_EXP_CTRL_MODE*/
    BOBCAT_EXP_CTRL_PW 									= (ui32) 0x1,
    BOBCAT_EXP_CTRL_INT 								= (ui32) 0x2,
    BOBCAT_EXP_CTRL_EXT 								= (ui32) 0x3,
    /*BOBCAT_SPEED*/
    BOBCAT_SPEED_NORM 									= (ui32) 0x0,
    BOBCAT_SPEED_FAST										= (ui32) 0x1,

    /* AEC, AGC, AIC registers */
    /*BOBCAT_EXP_CORR_SPEED, BOBCAT_GAIN_CORR_SPEED*/
    BOBCAT_SPEED_1X 										= (ui32) 0x0,
    BOBCAT_SPEED_2X 										= (ui32) 0x1,
    BOBCAT_SPEED_3X 										= (ui32) 0x2,
    BOBCAT_SPEED_4X 										= (ui32) 0x3,
    /*BOBCAT_LUMINANCE_TYPE*/
    BOBCAT_LUMINANCE_AVG 								= (ui32) 0x0,
    BOBCAT_LUMINANCE_PK 								= (ui32) 0x1,

    /* Video registers */
    /*BOBCAT_PRE_AMP_GAIN_CH1, BOBCAT_PRE_AMP_GAIN_CH2, BOBCAT_PRE_AMP_GAIN_CH3, BOBCAT_PRE_AMP_GAIN_CH4*/
    BOBCAT_GAIN_NEG3DB 									= (ui32) 0x0,
    BOBCAT_GAIN_0DB 										= (ui32) 0x1,
    BOBCAT_GAIN_POS3DB 									= (ui32) 0x2,
    BOBCAT_GAIN_POS6DB 									= (ui32) 0x3,
    /*BOBCAT_TAP_BALANCE*/
    BOBCAT_DYN_TAP_BALANCE_AUTO 				= (ui32) 0x1,
    BOBCAT_DYN_BALANCE_ONCE 						= (ui32) 0x2,
    BOBCAT_STAT_TAP_BALANCE 						= (ui32) 0x3,

    /* Trigger registers */
    /*BOBCAT_TRIG_INPUT_SELECT*/
    BOBCAT_TRIG_EXT 										= (ui32) 0x1,
    BOBCAT_TRIG_PROG 										= (ui32) 0x2,
    BOBCAT_TRIG_COMP 										= (ui32) 0x3,
    BOBCAT_TRIG_SW 											= (ui32) 0x4,
    /*BOBCAT_TRIG_EDGE_SELECT*/
    BOBCAT_EDGE_RISING									=	(ui32) 0x0,
    BOBCAT_EDGE_FALLING									=	(ui32) 0x1,
    /*BOBCAT_DEBOUNCE_TIME_SELECT*/
    BOBCAT_DEBOUNCE_100US 							= (ui32) 0x1,
    BOBCAT_DEBOUNCE_1000US 							= (ui32) 0x2,
    BOBCAT_DEBOUNCE_10000US 						= (ui32) 0x3,
    BOBCAT_DEBOUNCE_10US 								= (ui32) 0x4,
    BOBCAT_DEBOUNCE_50US 								= (ui32) 0x5,
    BOBCAT_DEBOUNCE_500US 							= (ui32) 0x6,
    BOBCAT_DEBOUNCE_5000US 							= (ui32) 0x7,
    /*BOBCAT_TRIG_OVERLAP*/
    BOBCAT_IGNORE_NEXT_TRIG 						= (ui32) 0x0,
    BOBCAT_ACCEPT_AFTER_COMPLETED 			= (ui32) 0x1,
    BOBCAT_ACCEPT_ANY_TIME 							= (ui32) 0x2,
    /*BOBCAT_TRIGING_MODE_SELECT*/
    BOBCAT_TRIG_STD 										= (ui32) 0x0,
    BOBCAT_TRIG_FAST 										= (ui32) 0x1,
    BOBCAT_TRIG_DBL 										= (ui32) 0x2,
    BOBCAT_TRIG_FRM_ACCUM								= (ui32) 0x3,
    BOBCAT_TRIG_ASYNC 									= (ui32) 0x4,
    /*BOBCAT_TRIG_STROBE_ENABLE*/
    BOBCAT_TRIG_STROBE1 								= (ui32) 0x1,
    BOBCAT_TRIG_STROBE2 								= (ui32) 0x2,
    /*to enable both do BOBCAT_TRIG_STROBE1&&BOBCAT_TRIG_STROBE2*/

    /* Pulse generator registers */
    /*BOBCAT_PULSE_GEN_GRANULARITY*/
    BOBCAT_PULSE_GEN_GRANULAR_1X 				= (ui32) 0x0,
    BOBCAT_PULSE_GEN_GRANULAR_10X 			= (ui32) 0x1,
    BOBCAT_PULSE_GEN_GRANULAR_100X 			= (ui32) 0x2,
    BOBCAT_PULSE_GEN_GRANULAR_1000X 		= (ui32) 0x3,

    /* Test pattern registers */
    /*BOBCAT_TEST_MODE_SELECT*/
    BOBCAT_TEST_IMG_BLACK 							= (ui32) 0x1,
    BOBCAT_TEST_IMG_GRAY 								= (ui32) 0x2,
    BOBCAT_TEST_IMG_WHITE 							= (ui32) 0x3,
    BOBCAT_TEST_IMG_HRAMP 							= (ui32) 0x4,
    BOBCAT_TEST_IMG_VRAMP 							= (ui32) 0x5,
    BOBCAT_TEST_IMG_MOVING_HRAMP 				= (ui32) 0x6,
    BOBCAT_TEST_IMG_MOVING_VRAMP 				= (ui32) 0x7,
    BOBCAT_TEST_IMG_8BARS 							= (ui32) 0x8,
    BOBCAT_TEST_IMG_HVLNS_OVER_LIVE			= (ui32) 0x9,

    /* Input output registers */
    /*BOBCAT_CC1_POLARITY_SELECT, BOBCAT_CC2_POLARITY_SELECT, BOBCAT_IN1_POLARITY_SELECT, BOBCAT_IN2_POLARITY_SELECT, BOBCAT_OUT1_POLARITY_SELECT, BOBCAT_OUT2_POLARITY_SELECT, BOBCAT_CLSP_POLARITY_SELECT*/
    BOBCAT_ACTIVE_LOW 									= (ui32) 0x0,
    BOBCAT_ACTIVE_HIGH 									= (ui32) 0x1,
    /*BOBCAT_OUT1_OUTPUT_SELECT, BOBCAT_OUT2_OUTPUT_SELECT, BOBCAT_CLSP_OUTPUT_SELECT*/
    BOBCAT_OUTPUT_EXP_START 						= (ui32) 0x1,
    BOBCAT_OUTPUT_EXP_END 							= (ui32) 0x2,
    BOBCAT_OUTPUT_EXP_MID 							= (ui32) 0x3,
    BOBCAT_OUTPUT_EXP_ACTIVE_WIN 				= (ui32) 0x4,
    BOBCAT_OUTPUT_H_SYNC 								= (ui32) 0x5,
    BOBCAT_OUTPUT_V_SYNC 								= (ui32) 0x6,
    BOBCAT_OUTPUT_ODD_EVEN_FRM_FLAG 		= (ui32) 0x7,
    BOBCAT_OUTPUT_TRIG_PULSE_ACTUAL 		= (ui32) 0x8,
    BOBCAT_OUTPUT_TRIG_PULSE_DELAYED 		= (ui32) 0x9,
    BOBCAT_OUTPUT_CAM_READY 						= (ui32) 0xA,
    BOBCAT_OUTPUT_PULSE_GEN 						= (ui32) 0xB,
    BOBCAT_OUTPUT_STROBE1 							= (ui32) 0xC,
    BOBCAT_OUTPUT_STROBE2 							= (ui32) 0xD,
    BOBCAT_OUTPUT_SW_BIT_TOGGLE 				= (ui32) 0xE,
    /*BOBCAT_CC1_INPUT_SELECT, BOBCAT_CC2_INPUT_SELECT, BOBCAT_IN1_INPUT_SELECT, BOBCAT_IN2_INPUT_SELECT*/
    BOBCAT_INPUT_COM_TRIG 							= (ui32) 0x1,
    BOBCAT_INPUT_EXT_TRIG 							= (ui32) 0x2,
    BOBCAT_INPUT_EXP_CTRL 							= (ui32) 0x3,
    BOBCAT_INPUT_H_SYNC 								= (ui32) 0x4,
    BOBCAT_INPUT_V_SYNC 								= (ui32) 0x5,
    /*BOBCAT_STROBE1_MODE_SELECT, BOBCAT_STROBE2_MODE_SELECT*/
    BOBCAT_STROBE_EACH_FRAME 						= (ui32) 0x1,
    BOBCAT_STROBE_ODD_FRAMES 						= (ui32) 0x2,
    BOBCAT_STROBE_EVEN_FRAMES 					= (ui32) 0x3,

    /* Output data format registers */
    /*BOBCAT_BIT_DEPTH_SELECT*/
    BOBCAT_8BIT 												= (ui32) 0x0,
    BOBCAT_10BIT 												= (ui32) 0x1,
    BOBCAT_12BIT 												= (ui32) 0x2,
    BOBCAT_14BIT 												= (ui32) 0x3,
    BOBCAT_3X8BIT 											= (ui32) 0x4,
    /*BOBCAT_TAP_MODE_SELECT*/
    BOBCAT_SINGLE_TAP 									= (ui32) 0x0,
    BOBCAT_DOUBLE_TAP 									= (ui32) 0x1,
    /*BOBCAT_DATA_FORMAT_SELECT*/
    BOBCAT_SINGLE_TAP_SINGLE						= (ui32) 0x0,
    BOBCAT_DOUBLE_TAP_INTERLEAVED 			= (ui32) 0x1,
    BOBCAT_DOUBLE_TAP_SEQUENTIAL 				= (ui32) 0x2,
    /*BOBCAT_BIT_SHIFT_SELECT*/
    /* see p. 244 */

    /* White Balance registers */
    /*BOBCAT_WB_MODE*/
    BOBCAT_WB_ONCE 											= (ui16) 0x1,
    BOBCAT_WB_AUTO 											= (ui16) 0x2,
    BOBCAT_WB_MANUAL 										= (ui16) 0x3,

    /* Data correction registers */
    /*BOBCAT_LUT_SELECT*/
    BOBCAT_LUT1 												= (ui32) 0x0,
    BOBCAT_LUT2 												= (ui32) 0x1,
    /*BOBCAT_DEFECTIVE_PIXEL_CORR, BOBCAT_HOT_PIXEL_CORR*/
    BOBCAT_STATIC_PIXEL_CORR 						= (ui32) 0x1,
    BOBCAT_DYNAMIC_PIXEL_CORR 					= (ui32) 0x2,
    /*to enable both do BOBCAT_STATIC_PIXEL_CORR&&BOBCAT_DYNAMIC_PIXEL_CORR*/

    /* Processing registers */
    /*BOBCAT_ENHANCEMENT_MODE*/
    BOBCAT_SINGLE_BINARY								= (ui32) 0x1,
    BOBCAT_DOUBLE_BINARY								= (ui32) 0x2,
    BOBCAT_DOUBLE_GRAYSCALE							= (ui32) 0x3,
    BOBCAT_DOUBLE_CONTRAST							= (ui32) 0x4,
    BOBCAT_SINGLE_POINT									= (ui32) 0x5,
    BOBCAT_MULTI_POINT									= (ui32) 0x6
    /*BOBCAT_PAOI_ENABLE*/
    /*see p 249*/

  } bobcatParamValue;

  typedef ui32 bobcatParamNumber;

  enum bStat {
    BOBCAT_OK = 0,
    BOBCAT_ERROR_BAD_HANDLE
  };
  typedef enum bStat bobcatStat;

  /*!	\typedef
    \struct region
    \brief A region structure.
    \details Used to define the region on the CCD in the detector coordinate space.*/
  typedef struct {
    ui32 x_offset; /**< x offset */
    ui32 y_offset; /**< y offset */
    ui32 x_length; /**< x length or width */
    ui32 y_length; /**< y length or height */
    bobcatParamValue x_binning; /**< x binning, BOBCAT_BINNING_*X */
    bobcatParamValue y_binning; /**< y binning, BOBCAT_BINNING_*X */
  } region;

  /* Function prototypes */
  etStat BOBCAT_ControlRead(tHandle, ui8**, ui8*);
  etStat BOBCAT_ControlWrite(tHandle, ui8**, ui8*);
  etStat BOBCAT_ParameterGet(tHandle, bobcatParam, void*);
  etStat BOBCAT_ParameterSet(tHandle, bobcatParam, void*);
  etStat BOBCAT_SoftReset(tHandle);
  etStat BOBCAT_LoadFromFactory(tHandle);
  etStat BOBCAT_LoadFromUser1(tHandle);
  etStat BOBCAT_SaveToUser1(tHandle);
  etStat BOBCAT_LoadFromUser2(tHandle);
  etStat BOBCAT_SaveToUser2(tHandle);
  etStat BOBCAT_SoftwareTriggerStart(tHandle);

  int BOBCAT_str_to_bobcatParam(char**, bobcatParam*);
  int BOBCAT_str_to_bobcatParamValue(char**, bobcatParamValue*);
  int BOBCAT_str_to_bobcatParamValues(char**, bobcatParamValue*);

#endif   /* _BOBCAT */
