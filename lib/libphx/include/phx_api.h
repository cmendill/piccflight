/****************************************************************************
 *
 * ACTIVE SILICON LIMITED
 *
 * File name   : phx_api.h
 * Function    : User API for all Phoenix products
 * Project     : AP06
 * Authors     : Martin Bone, Richard Brown
 * Systems     : ANSI C
 * Version     : 1.0
 * Release Date: 1-Oct-00
 *
 * Copyright (c) 2000-2010 Active Silicon Ltd.
 ****************************************************************************
 * Comments:
 * --------
 * This file is the only include file a user (or higher level library) needs 
 * to include in their application in order to use the PHX library.
 *
 ****************************************************************************
 */
#ifndef _PHX_API_H_
#define _PHX_API_H_


/* Include all the OS dependent definitions */
/* Need to use #import if using native MacOS X with frameworks */
#if defined _ASL_MACOSX_FMWK
#import <ASLphoenix/phx_os.h>

#else
#include "phx_os.h"

#endif

/* API & Library function parameter definitions
 * ============================================
_FnTypes(ForBrief)
*/
typedef const enum {
   FNTYPE_EMASK         = (int) 0xF0000000,

   FNTYPE_PHA_API       = (int) 0x00000000,
   FNTYPE_PHA_LIB       = (int) 0x10000000,
   FNTYPE_PHC_API       = (int) 0x20000000,
   FNTYPE_PHC_LIB       = (int) 0x30000000,
   FNTYPE_PHD_API       = (int) 0x40000000,
   FNTYPE_PHD_LIB       = (int) 0x50000000,
   FNTYPE_PDL_API       = (int) 0x60000000,
   FNTYPE_PDL_LIB       = (int) 0x70000000,
   FNTYPE_PCC_API       = (int) 0x80000000,
   FNTYPE_PCC_LIB       = (int) 0x90000000,

   FNTYPE_PHX_API       = (int) 0xC0000000,
   FNTYPE_PHX_LIB       = (int) 0xD0000000
} etFnTypes;


/* PHX_Function Definitions
 * ========================
 * These enums are used as magic numbers for all parameters passed to the specific functions,
 * ie any parameter passed to the Acquisition functions will have the top 8 bits set to 3.
 * This is used to confirm that the parameter passed is valid for the function.
 */
typedef const enum {
   PHX_EMASK_FN          = (int) ((ui32) FNTYPE_EMASK   | (ui32) 0x000F0000),
   PHX_CAMERACONFIG_LOAD = (int) ((ui32) FNTYPE_PHX_API | (ui32) 0x00010000),
   PHX_SETANDGET         = (int) ((ui32) FNTYPE_PHX_API | (ui32) 0x00020000), /* For parameters after PHX_Open has been called */
   PHX_ACQUIRE           = (int) ((ui32) FNTYPE_PHX_API | (ui32) 0x00030000),
   PHX_CAMERACONFIG_SAVE = (int) ((ui32) FNTYPE_PHX_API | (ui32) 0x00040000),
   PHX_TEST              = (int) ((ui32) FNTYPE_PHX_API | (ui32) 0x00050000),
   PHX_COM               = (int) ((ui32) FNTYPE_PHX_API | (ui32) 0x00060000),
   PHX_PRE_OPEN          = (int) ((ui32) FNTYPE_PHX_API | (ui32) 0x00070000), /* For parameters before PHX_Open has been called */
   PHX_CONTROL           = (int) ((ui32) FNTYPE_PHX_API | (ui32) 0x00080000)
} etPhxFn ;


/* PHX_CameraConfigLoad Definitions
 * ================================
 */
typedef enum {
   PHX_EMASK_BOARD    = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0007),
   PHX_BOARD_AUTO     = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0000),   /* Auto detect the first available board */
   PHX_BOARD1         = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0001),   /* 1st board in the scan order */
   PHX_BOARD2         = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0002),   /* 2nd board in the scan order */
   PHX_BOARD3         = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0003),   /* 3rd board in the scan order */
   PHX_BOARD4         = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0004),   /* 4th board in the scan order */
   PHX_BOARD5         = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0005),   /* 5th board in the scan order */
   PHX_BOARD6         = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0006),   /* 6th board in the scan order */
   PHX_BOARD7         = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0007),   /* 7th board in the scan order */
   PHX_BOARD_MAX      = (int) 7,                                                /* Maximum board number supported */

   PHX_EMASK_CHANNEL  = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0030),
   PHX_CHANNEL_AUTO   = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0000),   /* Auto detect the first available channel */
   PHX_CHANNEL_A      = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0010),   /* 1st available channel */
   PHX_CHANNEL1       = (int) PHX_CHANNEL_A,                                    /* For Compatibility, do not use */
   PHX_CHANNEL_B      = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0020),   /* 2nd available channel */
   PHX_CHANNEL2       = (int) PHX_CHANNEL_B,                                    /* For Compatibility, do not use */
   PHX_CHANNEL_MAX    = (int) 2,                                                /* Maximum channel number supported */
   PHX_CHANNEL_ONLY   = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0040),   /* i.e. don't use other channel's resources   */

   PHX_EMASK_MODE     = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0280),
   PHX_MODE_NORMAL    = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0000),   /* i.e. both acquisition engine and com ports */
   PHX_COMMS_ONLY     = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0080),   /* i.e. no control of acquisition engine      */
   PHX_ACQ_ONLY       = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0200),   /* i.e. no control of com ports               */

   PHX_EMASK_UPDATE   = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0100),
   PHX_UPDATE_FIRMWARE= (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0100),   /* Reprogram all firmware (to flash) */
   PHX_NO_RECONFIGURE = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0400),   /* Don't automatically reconfigure the board
                                                                                   if the firmware is already up to date */

   PHX_EMASK_TYPE     = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0xC000),
   PHX_EMASK_VARIANT  = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x3808),
   PHX_TYPE_AUTO      = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x0000),
   PHX_DIGITAL        = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x4000),   /* Any digital interface board  */

   PHX_D24CL_PCI32    = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x4800),   /* AS-PHX-D24CL-PCI32   variants */
   PHX_D24CL_PE1      = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x4808),   /* AS-PHX-D24CL-PE1     variants */
   PHX_D24AVDS_PE1    = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x5008),   /* AS-PHX-D24AVDS-PE1   variants */

   PHX_D32_PCI32      = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x5000),   /* AS-PHX-D32-PCI32     variants */

   PHX_D36_PCI32      = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x5800),   /* AS-PHX-D36-PCI32     variants */
   PHX_D36_PCI64      = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x6000),   /* AS-PHX-D36-PCI64     variants */
   PHX_D36_PCI64U     = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x6800),   /* AS-PHX-D36-PCI64U    variants */
   PHX_D36_PE1        = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x5808),   /* AS-PHX-D36-PE1       variants */

   PHX_D10HDSDI_PE1   = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x6808),   /* AS-PHX-D10HDSDI-PE1  variants */
   PHX_D20HDSDI_PE1   = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x6008),   /* AS-PHX-D20HDSDI-PE1  variants */

   PHX_D48CL_PCI32    = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x4008),   /* AS-PHX-D48CL-PCI32   variants */
   PHX_D48CL_PCI64    = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x7000),   /* AS-PHX-D48CL-PCI64   variants */
   PHX_D48CL_PCI64U   = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x7800),   /* AS-PHX-D48CL-PCI64U  variants */
   PHX_D48CL_PE1      = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x7808),   /* AS-PHX-D48CL-PE1     variants */

   PHX_ANALOGUE       = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0x8000),   /* Any analogue interface board */

   PHX_DIGITAL2       = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0xC000),   /* Any other digital interface board */
   PHX_D36_PE4        = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0xD808),   /* AS-PHX-D36-PE4       variants */
   PHX_D10HDSDI_PE4   = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0xE808),   /* AS-PHX-D10HDSDI-PE4  variants */
   PHX_D20HDSDI_PE4   = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0xE008),   /* AS-PHX-D20HDSDI-PE4  variants */
   PHX_D48CL_PE4      = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0xF008),   /* AS-PHX-D48CL-PE4     variants */
   PHX_D64CL_PE4      = (int) ((ui32) PHX_CAMERACONFIG_LOAD | (ui32) 0xD008)    /* AS-PHX-D64CL-PE4     variants */
} etCamConfigLoad ;

/* PHX_CameraConfigSave Definitions
 * ================================
 */
typedef const enum {
   PHX_EMASK_SAVE     = (int) ((ui32) PHX_CAMERACONFIG_SAVE | (ui32) 0x0007),
   PHX_SAVE_CAM       = (int) ((ui32) PHX_CAMERACONFIG_SAVE | (ui32) 0x0001),   /* Camera specific parameters      */
   PHX_SAVE_SYS       = (int) ((ui32) PHX_CAMERACONFIG_SAVE | (ui32) 0x0002),   /* system specific parameters      */
   PHX_SAVE_APP       = (int) ((ui32) PHX_CAMERACONFIG_SAVE | (ui32) 0x0004),   /* Application specific parameters */
   PHX_SAVE_ALL       = (int) ((ui32) PHX_CAMERACONFIG_SAVE | (ui32) 0x0007)    /* Save all parameters             */
} etCamConfigSave ;


/* GetAndSet Parameter Definitions
 * ===============================
 * These define the parameters that can be read or modified via the PHX_ParameterGet & PHX_ParameterSet
 * interface.  Each value is a unique number coded as follows:
 * bits 31 to 24 are the GetAndSet function identifier,
 * bits 23 to 16 are the specific parameter identifier
 * They are implemented as enums to make them visible within the debugger.
 */


/* Note - Because we have changed the API on a couple of occasions, we have tried to preserve the
 * numbering of the unaffected enums. Therefore there are a number of gaps in the following list where
 * some parameters have been removed.
 */
typedef enum {
   PHX_PARAM_MASK                = (int) 0xffffff00,
   PHX_INVALID_PARAM             = 0,
   PHX_CACHE_FLUSH               = 1,
   PHX_FORCE_REWRITE             = 2,

   PHX_ACQ_CONTINUOUS            = (int) ((ui32) PHX_SETANDGET | ((ui32)  1 << 8 )),
   PHX_ACQ_NUM_IMAGES            = (int) ((ui32) PHX_SETANDGET | ((ui32)  2 << 8 )),
   PHX_ACQ_SKIP                  = (int) ((ui32) PHX_SETANDGET | ((ui32)  3 << 8 )),
   PHX_ACQTRIG_SRC               = (int) ((ui32) PHX_SETANDGET | ((ui32)  4 << 8 )),
   PHX_ACQTRIG_TYPE              = (int) ((ui32) PHX_SETANDGET | ((ui32)  5 << 8 )),
   PHX_ACQ_TYPE                  = (int) ((ui32) PHX_SETANDGET | ((ui32)  6 << 8 )),
   PHX_ACQ_XSUB                  = (int) ((ui32) PHX_SETANDGET | ((ui32)  7 << 8 )),
   PHX_ACQ_YSUB                  = (int) ((ui32) PHX_SETANDGET | ((ui32)  8 << 8 )),


   PHX_CAM_ACTIVE_XLENGTH        = (int) ((ui32) PHX_SETANDGET | ((ui32) 10 << 8 )),
   PHX_CAM_ACTIVE_YLENGTH        = (int) ((ui32) PHX_SETANDGET | ((ui32) 11 << 8 )),
   PHX_CAM_ACTIVE_XOFFSET        = (int) ((ui32) PHX_SETANDGET | ((ui32) 12 << 8 )),
   PHX_CAM_ACTIVE_YOFFSET        = (int) ((ui32) PHX_SETANDGET | ((ui32) 13 << 8 )),
   PHX_CAM_CLOCK_POLARITY        = (int) ((ui32) PHX_SETANDGET | ((ui32) 14 << 8 )),
   PHX_CAM_FORMAT                = (int) ((ui32) PHX_SETANDGET | ((ui32) 15 << 8 )),
   PHX_CAM_NUM_TAPS              = (int) ((ui32) PHX_SETANDGET | ((ui32) 16 << 8 )),
   PHX_CAM_SRC_DEPTH             = (int) ((ui32) PHX_SETANDGET | ((ui32) 17 << 8 )),
   PHX_CAM_SRC_COL               = (int) ((ui32) PHX_SETANDGET | ((ui32) 18 << 8 )),
   PHX_CAM_HTAP_DIR              = (int) ((ui32) PHX_SETANDGET | ((ui32) 19 << 8 )),
   PHX_CAM_HTAP_TYPE             = (int) ((ui32) PHX_SETANDGET | ((ui32) 20 << 8 )),
   PHX_CAM_HTAP_NUM              = (int) ((ui32) PHX_SETANDGET | ((ui32) 21 << 8 )),
   PHX_CAM_VTAP_DIR              = (int) ((ui32) PHX_SETANDGET | ((ui32) 22 << 8 )),
   PHX_CAM_VTAP_TYPE             = (int) ((ui32) PHX_SETANDGET | ((ui32) 23 << 8 )),
   PHX_CAM_VTAP_NUM              = (int) ((ui32) PHX_SETANDGET | ((ui32) 24 << 8 )),
   PHX_CAM_TYPE                  = (int) ((ui32) PHX_SETANDGET | ((ui32) 25 << 8 )),
   PHX_CAM_XBINNING              = (int) ((ui32) PHX_SETANDGET | ((ui32) 26 << 8 )),
   PHX_CAM_YBINNING              = (int) ((ui32) PHX_SETANDGET | ((ui32) 27 << 8 )),

   PHX_COMMS_DATA                = (int) ((ui32) PHX_SETANDGET | ((ui32) 28 << 8 )),
   PHX_COMMS_FLOW                = (int) ((ui32) PHX_SETANDGET | ((ui32) 29 << 8 )),
   PHX_COMMS_INCOMING            = (int) ((ui32) PHX_SETANDGET | ((ui32) 30 << 8 )),
   PHX_COMMS_OUTGOING            = (int) ((ui32) PHX_SETANDGET | ((ui32) 31 << 8 )),
   PHX_COMMS_PARITY              = (int) ((ui32) PHX_SETANDGET | ((ui32) 32 << 8 )),
   PHX_COMMS_SPEED               = (int) ((ui32) PHX_SETANDGET | ((ui32) 33 << 8 )),
   PHX_COMMS_STANDARD            = (int) ((ui32) PHX_SETANDGET | ((ui32) 34 << 8 )),
   PHX_COMMS_STOP                = (int) ((ui32) PHX_SETANDGET | ((ui32) 35 << 8 )),

   PHX_DATASRC                   = (int) ((ui32) PHX_SETANDGET | ((ui32) 36 << 8 )),
   PHX_DATASTREAM_VALID          = (int) ((ui32) PHX_SETANDGET | ((ui32) 37 << 8 )),

   PHX_CAPTURE_FORMAT            = (int) ((ui32) PHX_SETANDGET | ((ui32) 38 << 8 )),
   PHX_DST_FORMAT                = PHX_CAPTURE_FORMAT,

   PHX_DST_PTR_TYPE              = (int) ((ui32) PHX_SETANDGET | ((ui32) 39 << 8 )),
   PHX_DST_PTRS_VIRT             = (int) ((ui32) PHX_SETANDGET | ((ui32) 40 << 8 )),

   PHX_DUMMY_PARAM               = (int) ((ui32) PHX_SETANDGET | ((ui32) 41 << 8 )),

   PHX_ERROR_FIRST_ERRNUM        = (int) ((ui32) PHX_SETANDGET | ((ui32) 42 << 8 )),
   PHX_ERROR_FIRST_ERRSTRING     = (int) ((ui32) PHX_SETANDGET | ((ui32) 43 << 8 )),
   PHX_ERROR_HANDLER             = (int) ((ui32) PHX_SETANDGET | ((ui32) 44 << 8 )),
   PHX_ERROR_LAST_ERRNUM         = (int) ((ui32) PHX_SETANDGET | ((ui32) 45 << 8 )),
   PHX_ERROR_LAST_ERRSTRING      = (int) ((ui32) PHX_SETANDGET | ((ui32) 46 << 8 )),

   PHX_EVENTCOUNT                = (int) ((ui32) PHX_SETANDGET | ((ui32) 47 << 8 )),


   PHX_NUM_BOARDS                = (int) ((ui32) PHX_SETANDGET | ((ui32) 61 << 8 )),

   PHX_IO_CCIO_A                 = (int) ((ui32) PHX_SETANDGET | ((ui32) 62 << 8 )),  /* Absolute */
   PHX_IO_CCIO_A_OUT             = (int) ((ui32) PHX_SETANDGET | ((ui32) 63 << 8 )),
   PHX_IO_CCIO_B                 = (int) ((ui32) PHX_SETANDGET | ((ui32) 64 << 8 )),
   PHX_IO_CCIO_B_OUT             = (int) ((ui32) PHX_SETANDGET | ((ui32) 65 << 8 )),

   PHX_IO_CCOUT_A                = PHX_IO_CCIO_A,
   PHX_IO_CCOUT_B                = PHX_IO_CCIO_B,

   PHX_IO_OPTO_SET               = (int) ((ui32) PHX_SETANDGET | ((ui32) 68 << 8 )),
   PHX_IO_OPTO_CLR               = (int) ((ui32) PHX_SETANDGET | ((ui32) 69 << 8 )),
   PHX_IO_OPTO                   = (int) ((ui32) PHX_SETANDGET | ((ui32) 70 << 8 )),

   PHX_IO_TTL_A                  = (int) ((ui32) PHX_SETANDGET | ((ui32) 71 << 8 )),
   PHX_IO_TTL_A_OUT              = (int) ((ui32) PHX_SETANDGET | ((ui32) 72 << 8 )),
   PHX_IO_TTL_B                  = (int) ((ui32) PHX_SETANDGET | ((ui32) 73 << 8 )),
   PHX_IO_TTL_B_OUT              = (int) ((ui32) PHX_SETANDGET | ((ui32) 74 << 8 )),

   PHX_TIMEOUT_CAPTURE           = (int) ((ui32) PHX_SETANDGET | ((ui32) 75 << 8 )),
   PHX_TIMEOUT_DMA               = (int) ((ui32) PHX_SETANDGET | ((ui32) 76 << 8 )),
   PHX_TIMEOUT_TRIGGER           = (int) ((ui32) PHX_SETANDGET | ((ui32) 77 << 8 )),

   PHX_INTRPT_SET                = (int) ((ui32) PHX_SETANDGET | ((ui32) 78 << 8 )),
   PHX_INTRPT_CLR                = (int) ((ui32) PHX_SETANDGET | ((ui32) 79 << 8 )),

   PHX_REV_HW                    = (int) ((ui32) PHX_SETANDGET | ((ui32) 80 << 8 )),
   PHX_REV_HW_MAJOR              = (int) ((ui32) PHX_SETANDGET | ((ui32) 81 << 8 )),
   PHX_REV_HW_MINOR              = (int) ((ui32) PHX_SETANDGET | ((ui32) 82 << 8 )),
   PHX_REV_SW                    = (int) ((ui32) PHX_SETANDGET | ((ui32) 83 << 8 )),
   PHX_REV_SW_MAJOR              = (int) ((ui32) PHX_SETANDGET | ((ui32) 84 << 8 )),
   PHX_REV_SW_MINOR              = (int) ((ui32) PHX_SETANDGET | ((ui32) 85 << 8 )),
   PHX_REV_SW_SUBMINOR           = (int) ((ui32) PHX_SETANDGET | ((ui32) 86 << 8 )),

   PHX_ROI_DST_XOFFSET           = (int) ((ui32) PHX_SETANDGET | ((ui32) 87 << 8 )),
   PHX_ROI_DST_YOFFSET           = (int) ((ui32) PHX_SETANDGET | ((ui32) 88 << 8 )),
   PHX_ROI_SRC_XOFFSET           = (int) ((ui32) PHX_SETANDGET | ((ui32) 89 << 8 )),
   PHX_ROI_SRC_YOFFSET           = (int) ((ui32) PHX_SETANDGET | ((ui32) 90 << 8 )),
   PHX_ROI_XLENGTH               = (int) ((ui32) PHX_SETANDGET | ((ui32) 91 << 8 )),
   PHX_ROI_YLENGTH               = (int) ((ui32) PHX_SETANDGET | ((ui32) 92 << 8 )),

   PHX_BUF_DST_XLENGTH           = (int) ((ui32) PHX_SETANDGET | ((ui32) 93 << 8 )),
   PHX_BUF_DST_YLENGTH           = (int) ((ui32) PHX_SETANDGET | ((ui32) 94 << 8 )),

   PHX_STATUS                    = (int) ((ui32) PHX_SETANDGET | ((ui32) 95 << 8 )),

   PHX_BOARD_PROPERTIES          = (int) ((ui32) PHX_SETANDGET | ((ui32) 102 << 8 )),

   PHX_ROI_XLENGTH_SCALED        = (int) ((ui32) PHX_SETANDGET | ((ui32) 103 << 8 )),
   PHX_ROI_YLENGTH_SCALED        = (int) ((ui32) PHX_SETANDGET | ((ui32) 104 << 8 )),

   PHX_BUF_SET                   = (int) ((ui32) PHX_SETANDGET | ((ui32) 105 << 8 )),
   PHX_BUF_SET_COLOUR            = (int) ((ui32) PHX_SETANDGET | ((ui32) 106 << 8 )),

   PHX_LUT_COUNT                 = (int) ((ui32) PHX_SETANDGET | ((ui32) 107 << 8 )),
   PHX_LUT_INFO                  = (int) ((ui32) PHX_SETANDGET | ((ui32) 108 << 8 )),

   PHX_REV_HW_SUBMINOR           = (int) ((ui32) PHX_SETANDGET | ((ui32) 109 << 8 )),

   PHX_LUT_CORRECT               = (int) ((ui32) PHX_SETANDGET | ((ui32) 110 << 8 )),

   PHX_LINETRIG_SRC              = (int) ((ui32) PHX_SETANDGET | ((ui32) 111 << 8 )),
   PHX_LINETRIG_TIMER_CTRL       = (int) ((ui32) PHX_SETANDGET | ((ui32) 112 << 8 )),
   PHX_LINETRIG_TIMER_PERIOD     = (int) ((ui32) PHX_SETANDGET | ((ui32) 113 << 8 )),
   PHX_EXPTRIG_SRC               = (int) ((ui32) PHX_SETANDGET | ((ui32) 114 << 8 )),
   PHX_EXP_CTRLIO_1              = (int) ((ui32) PHX_SETANDGET | ((ui32) 115 << 8 )),
   PHX_IO_TIMER_1_PERIOD         = (int) ((ui32) PHX_SETANDGET | ((ui32) 116 << 8 )),
   PHX_EXP_CTRLIO_2              = (int) ((ui32) PHX_SETANDGET | ((ui32) 117 << 8 )),
   PHX_IO_TIMER_2_PERIOD         = (int) ((ui32) PHX_SETANDGET | ((ui32) 118 << 8 )),
   PHX_EXP_LINESTART             = (int) ((ui32) PHX_SETANDGET | ((ui32) 119 << 8 )),
   PHX_ACQTRIG_DELAY_TYPE        = (int) ((ui32) PHX_SETANDGET | ((ui32) 120 << 8 )),
   PHX_ACQTRIG_DELAY             = (int) ((ui32) PHX_SETANDGET | ((ui32) 121 << 8 )),
   PHX_EVENTCOUNT_SRC            = (int) ((ui32) PHX_SETANDGET | ((ui32) 122 << 8 )),
   PHX_EVENTGATE_SRC             = (int) ((ui32) PHX_SETANDGET | ((ui32) 123 << 8 )),

   PHX_CAM_HTAP_ORDER            = (int) ((ui32) PHX_SETANDGET | ((ui32) 124 << 8 )),
   PHX_CAM_VTAP_ORDER            = (int) ((ui32) PHX_SETANDGET | ((ui32) 125 << 8 )),

   PHX_EVENT_CONTEXT             = (int) ((ui32) PHX_SETANDGET | ((ui32) 126 << 8 )),

   PHX_CAM_DATA_VALID            = (int) ((ui32) PHX_SETANDGET | ((ui32) 127 << 8 )),

   PHX_BUFFER_READY_COUNT        = (int) ((ui32) PHX_SETANDGET | ((ui32) 128 << 8 )),
   PHX_BUFFER_READY_COUNTER      = (int) ((ui32) PHX_SETANDGET | ((ui32) 129 << 8 )),

   PHX_LUT_SHIFT                 = (int) ((ui32) PHX_SETANDGET | ((ui32) 130 << 8 )),

   PHX_MASK_CCIO                 = (int) 0x00000003,
   PHX_MASK_CCOUT                = (int) 0x0000000f,
   PHX_MASK_OPTO                 = (int) 0x0000000f,
   PHX_IO_CCIO                   = (int) ((ui32) PHX_SETANDGET | ((ui32) 131 << 8 )),  /* Relative */
   PHX_IO_CCOUT                  = PHX_IO_CCIO,
   PHX_IO_CCIO_OUT               = (int) ((ui32) PHX_SETANDGET | ((ui32) 132 << 8 )),

   PHX_IO_TTL                    = (int) ((ui32) PHX_SETANDGET | ((ui32) 133 << 8 )),  /* Relative */
   PHX_IO_TTL_OUT                = (int) ((ui32) PHX_SETANDGET | ((ui32) 134 << 8 )),

   PHX_IO_OPTO_A                 = (int) ((ui32) PHX_SETANDGET | ((ui32) 135 << 8 )),  /* Absolute */
   PHX_IO_OPTO_B                 = (int) ((ui32) PHX_SETANDGET | ((ui32) 136 << 8 )),

   PHX_IO_TIMER_A1_PERIOD        = (int) ((ui32) PHX_SETANDGET | ((ui32) 137 << 8 )),  /* Absolute */
   PHX_IO_TIMER_A2_PERIOD        = (int) ((ui32) PHX_SETANDGET | ((ui32) 138 << 8 )),
   PHX_IO_TIMER_B1_PERIOD        = (int) ((ui32) PHX_SETANDGET | ((ui32) 139 << 8 )),  /* Absolute */
   PHX_IO_TIMER_B2_PERIOD        = (int) ((ui32) PHX_SETANDGET | ((ui32) 140 << 8 )),

   PHX_IO_OPTO_OUT               = (int) ((ui32) PHX_SETANDGET | ((ui32) 141 << 8 )),
   PHX_IO_OPTO_A_OUT             = (int) ((ui32) PHX_SETANDGET | ((ui32) 142 << 8 )),
   PHX_IO_OPTO_B_OUT             = (int) ((ui32) PHX_SETANDGET | ((ui32) 143 << 8 )),

   PHX_ACQTRIG_ALIGN             = (int) ((ui32) PHX_SETANDGET | ((ui32) 144 << 8 )),

   PHX_DST_ENDIAN                = (int) ((ui32) PHX_SETANDGET | ((ui32) 145 << 8 )),

   PHX_ACQ_CHAIN                 = (int) ((ui32) PHX_SETANDGET | ((ui32) 146 << 8 )),
   PHX_ACQ_BLOCKING              = (int) ((ui32) PHX_SETANDGET | ((ui32) 147 << 8 )),

   PHX_DST_PTRS_PHYS             = (int) ((ui32) PHX_SETANDGET | ((ui32) 148 << 8 )),

   PHX_BOARD_INFO                = (int) ((ui32) PHX_SETANDGET | ((ui32) 149 << 8 )),
   PHX_DATARATE_TEST             = (int) ((ui32) PHX_SETANDGET | ((ui32) 150 << 8 )),

   PHX_CAM_CLOCK_MAX             = (int) ((ui32) PHX_SETANDGET | ((ui32) 151 << 8 )),

   PHX_EVENTCOUNT_AT_GATE        = (int) ((ui32) PHX_SETANDGET | ((ui32) 152 << 8 )),

   PHX_CHAN_SYNC_MODE            = (int) ((ui32) PHX_SETANDGET | ((ui32) 153 << 8 )),

   PHX_ACQ_BUFFER_START          = (int) ((ui32) PHX_SETANDGET | ((ui32) 154 << 8 )),

   PHX_LUT_BYPASS                = (int) ((ui32) PHX_SETANDGET | ((ui32) 155 << 8 )),

   PHX_COMMS_PORT_NAME           = (int) ((ui32) PHX_SETANDGET | ((ui32) 156 << 8 )),

   PHX_CVB_PARAM                 = (int) ((ui32) PHX_SETANDGET | ((ui32) 157 << 8 )),
   PHX_USER_FORMAT               = (int) ((ui32) PHX_SETANDGET | ((ui32) 158 << 8 )),
   
   PHX_ACQ_AUTO_RESTART          = (int) ((ui32) PHX_SETANDGET | ((ui32) 159 << 8 )),

   PHX_ACQ_HSCALE                = (int) ((ui32) PHX_SETANDGET | ((ui32) 160 << 8 )),

   PHX_MERGE_CHAN                = (int) ((ui32) PHX_SETANDGET | ((ui32) 161 << 8 )),
   PHX_MERGE_INTRPT_SET          = (int) ((ui32) PHX_SETANDGET | ((ui32) 162 << 8 )),
   PHX_MERGE_INTRPT_CLR          = (int) ((ui32) PHX_SETANDGET | ((ui32) 163 << 8 )),

   PHX_CLSER_INDEX               = (int) ((ui32) PHX_SETANDGET | ((ui32) 164 << 8 )),

   PHX_FIFO_BUFFER_COUNT         = (int) ((ui32) PHX_SETANDGET | ((ui32) 165 << 8 )),

   PHX_REV_FLASH                 = (int) ((ui32) PHX_SETANDGET | ((ui32) 166 << 8 )),

   PHX_CONFIG_FILE               = (int) ((ui32) PHX_PRE_OPEN | ((ui32)   1 << 8 )),
   PHX_BOARD_VARIANT             = (int) ((ui32) PHX_PRE_OPEN | ((ui32)   2 << 8 )),
   PHX_BOARD_NUMBER              = (int) ((ui32) PHX_PRE_OPEN | ((ui32)   3 << 8 )),
   PHX_CHANNEL_NUMBER            = (int) ((ui32) PHX_PRE_OPEN | ((ui32)   4 << 8 )),
   PHX_CONFIG_MODE               = (int) ((ui32) PHX_PRE_OPEN | ((ui32)   5 << 8 ))
} etParam;   


/* PHX_ControlRead/Write Definitions
 * =================================
 */
typedef enum {
   PHX_COMMS_PORT                = (int) ((ui32) PHX_CONTROL | (ui32) 0x0000),
   PHX_REGISTER_HOST             = (int) ((ui32) PHX_CONTROL | (ui32) 0x0001),
   PHX_REGISTER_DEVICE           = (int) ((ui32) PHX_CONTROL | (ui32) 0x0002)
} etControlPort;   


/* PHX_ComParameterGet Definitions
 * ===============================
 */
typedef enum {
   PHX_COM_NUM_PORTS             = (int) ((ui32) PHX_COM | (ui32) 0x0000),   /* return the number of Phoenix com ports available  */
   PHX_COM_CLSER_CONFIG          = (int) ((ui32) PHX_COM | (ui32) 0x0001),   /* convert clser index to config magic number        */
   PHX_COM_WIN_CONFIG            = (int) ((ui32) PHX_COM | (ui32) 0x0002)    /* convert windows comN index to config magic number */
} etComParam;   


/* Retained for backwards compatibility with previous software releases.
 * Do not use for new applications
 */
typedef enum {
   PHX_DST_PTRS                  = PHX_DST_PTRS_VIRT,
   PHX_IO_LVDS_A                 = PHX_IO_CCIO_A,
   PHX_IO_LVDS_A_OUT             = PHX_IO_CCIO_A_OUT,
   PHX_IO_LVDS_B                 = PHX_IO_CCIO_B,
   PHX_IO_LVDS_B_OUT             = PHX_IO_CCIO_B_OUT,
   PHX_EXP_CTRLIO_1_DELAY        = PHX_IO_TIMER_1_PERIOD,
   PHX_EXP_CTRLIO_2_DELAY        = PHX_IO_TIMER_2_PERIOD,
   PHX_EXP_SRC                   = PHX_EXPTRIG_SRC,
   PHX_EVENTCOUNT_TYPE           = PHX_EVENTCOUNT_SRC,
   PHX_EVENTGATE_TYPE            = PHX_EVENTGATE_SRC,
   PHX_EVENT_COUNT               = PHX_EVENTCOUNT,
   PHX_CAM_SRC_BANDS             = PHX_CAM_SRC_COL
} etParamCompatibility;

   
/* GetAndSet Parameter Values   
 * ===========================
 * These define the values passed for a given parameter within the PHX_ParameterGet & PHX_ParameterSet
 * interface.  Each value consists of a unique incremental number.  However each value contains
 * the parameter value coded within the top 8 bits, which is used as an error check that the correct
 * value is being used for the parameter.
 * They are implemented as enums to make them visible within the debugger.
 */
typedef enum {
   PHX_INVALID_PARAMVALUE = 0,

   /* Boolean flags */
   PHX_ENABLE = 1,
   PHX_DISABLE,

   /* PHX_COMMS_DATA */
   PHX_COMMS_DATA_5 = (int) ((ui32) PHX_COMMS_DATA + 1),
   PHX_COMMS_DATA_6,
   PHX_COMMS_DATA_7,
   PHX_COMMS_DATA_8,

   /* PHX_COMMS_STOP */
   PHX_COMMS_STOP_1 = (int) ((ui32) PHX_COMMS_STOP + 1),
   PHX_COMMS_STOP_1_5,
   PHX_COMMS_STOP_2,

   /* PHX_COMMS_PARITY */
   PHX_COMMS_PARITY_NONE = (int) ((ui32) PHX_COMMS_PARITY + 1),
   PHX_COMMS_PARITY_EVEN,
   PHX_COMMS_PARITY_ODD,

   /* PHX_COMMS_FLOW */
   PHX_COMMS_FLOW_NONE = (int) ((ui32) PHX_COMMS_FLOW + 1),
   PHX_COMMS_FLOW_HW,
   PHX_COMMS_FLOW_SW,

   /* PHX_COMMS_STANDARD */
   PHX_COMMS_STANDARD_RS232 = (int) ((ui32) PHX_COMMS_STANDARD + 1),
   PHX_COMMS_STANDARD_LVDS,

   /* PHX_IO_OPTO_OUT_SET/CLR */
   PHX_IO_OPTO_OUT1                 = 0x00000001,
   PHX_IO_OPTO_OUT2                 = 0x00000002,
   PHX_IO_OPTO_OUT3                 = 0x00000004,
   PHX_IO_OPTO_OUT4                 = 0x00000008,

   /* PHX_IO_OPTO_SET/CLR */
   PHX_IO_OPTO1                     = 0x00000001,
   PHX_IO_OPTO2                     = 0x00000002,
   PHX_IO_OPTO3                     = 0x00000004,
   PHX_IO_OPTO4                     = 0x00000008,

   /* PHX_STATUS */
   PHX_STATUS_IDLE = (int) ((ui32) PHX_STATUS + 1),
   PHX_STATUS_ACQ_IN_PROGRESS,
   PHX_STATUS_WAITING_FOR_TRIGGER,

   /* PHX_CAM_TYPE */
   PHX_CAM_LINESCAN_ROI = (int) ((ui32) PHX_CAM_TYPE + 1),
   PHX_CAM_LINESCAN_NO_ROI,
   PHX_CAM_AREASCAN_ROI,
   PHX_CAM_AREASCAN_NO_ROI,

   /* PHX_CAM_FORMAT */
   PHX_CAM_INTERLACED = (int) ((ui32) PHX_CAM_FORMAT + 1),
   PHX_CAM_NON_INTERLACED,

   /* PHX_CAM_SRC_COL */
   PHX_CAM_SRC_MONO                 = 0x00000001,
   PHX_CAM_SRC_RGB                  = 0x00000003,
   PHX_CAM_SRC_BAY_RGGB = (int) ((ui32) PHX_CAM_SRC_COL + 1),
   PHX_CAM_SRC_BAY_GRBG,
   PHX_CAM_SRC_BAY_GBRG,
   PHX_CAM_SRC_BAY_BGGR,
   PHX_CAM_SRC_YUV,

   /* PHX_CAM_HTAP_DIR */
   PHX_CAM_HTAP_LEFT = (int) ((ui32) PHX_CAM_HTAP_DIR + 1),
   PHX_CAM_HTAP_RIGHT,
   PHX_CAM_HTAP_CONVERGE,
   PHX_CAM_HTAP_DIVERGE,
   PHX_CAM_HTAP_BOTH = PHX_CAM_HTAP_CONVERGE,      /* For Backwards compatibility from 2.10.1 */

   /* PHX_CAM_HTAP_TYPE */
   PHX_CAM_HTAP_LINEAR = (int) ((ui32) PHX_CAM_HTAP_TYPE + 1),
   PHX_CAM_HTAP_OFFSET_1,
   PHX_CAM_HTAP_ALTERNATE,
   PHX_CAM_HTAP_OFFSET_2,
   PHX_CAM_HTAP_SPAN,
   /* Backward Compatibility prior to 4.14 */
   PHX_CAM_HTAP_OFFSET =    PHX_CAM_HTAP_OFFSET_1,

   /* PHX_CAM_HTAP_ORDER */
   PHX_CAM_HTAP_ASCENDING = (int) ((ui32) PHX_CAM_HTAP_ORDER + 1),
   PHX_CAM_HTAP_DESCENDING,

   /* PHX_CAM_VTAP_DIR */
   PHX_CAM_VTAP_TOP = (int) ((ui32) PHX_CAM_VTAP_DIR + 1),
   PHX_CAM_VTAP_BOTTOM,
   PHX_CAM_VTAP_BOTH,

   /* PHX_CAM_VTAP_TYPE */
   PHX_CAM_VTAP_LINEAR = (int) ((ui32) PHX_CAM_VTAP_TYPE + 1),
   PHX_CAM_VTAP_OFFSET,
   PHX_CAM_VTAP_ALTERNATE,

   /* PHX_CAM_VTAP_ORDER */
   PHX_CAM_VTAP_ASCENDING = (int) ((ui32) PHX_CAM_VTAP_ORDER + 1),
   PHX_CAM_VTAP_DESCENDING,

   /* PHX_CAM_CLOCK_POLARITY */
   PHX_CAM_CLOCK_POS = (int) ((ui32) PHX_CAM_CLOCK_POLARITY + 1),
   PHX_CAM_CLOCK_NEG,

   /* PHX_CAM_CLOCK_MAX */
   PHX_CAM_CLOCK_MAX_DEFAULT = (int) ((ui32) PHX_CAM_CLOCK_MAX + 1),
   PHX_CAM_CLOCK_MAX_85MHZ,

   /* PHX_ACQ_TYPE */
   PHX_ACQ_FRAME_12 = (int) ((ui32) PHX_ACQ_TYPE + 1),
   PHX_ACQ_FRAME_21,
   PHX_ACQ_FIELD_12,
   PHX_ACQ_FIELD_21,
   PHX_ACQ_FIELD_1,
   PHX_ACQ_FIELD_2,
   PHX_ACQ_FIELD_NEXT,
   PHX_ACQ_LINE_DOUBLE_12,
   PHX_ACQ_LINE_DOUBLE_21,
   PHX_ACQ_LINE_DOUBLE_NEXT,
   PHX_ACQ_LINE_DOUBLE_1,
   PHX_ACQ_LINE_DOUBLE_2,
   PHX_ACQ_FRAME    = PHX_ACQ_FRAME_12,   /* For Backwards compatibility from 2.8.2 */

   /* PHX_ACQ_XSUB */
   PHX_ACQ_X1 = (int) ((ui32) PHX_ACQ_XSUB + 1),
   PHX_ACQ_X2,
   PHX_ACQ_X4,
   PHX_ACQ_X8,

   /* PHX_DST_PTR_TYPE */
   PHX_DST_PTR_INTERNAL = (int) ((ui32) PHX_DST_PTR_TYPE + 1),
   PHX_DST_PTR_USER_VIRT,
   PHX_DST_PTR_USER_PHYS,

   /* PHX_DATASTREAM_VALID */
   PHX_DATASTREAM_ALWAYS = (int) ((ui32) PHX_DATASTREAM_VALID + 1),
   PHX_DATASTREAM_LINE_ONLY,
   PHX_DATASTREAM_FRAME_ONLY,
   PHX_DATASTREAM_FRAME_AND_LINE,

   /* PHX_DATASRC */
   PHX_DATASRC_CAMERA = (int) ((ui32) PHX_DATASRC + 1),
   PHX_DATASRC_SIMULATOR_STATIC,
   PHX_DATASRC_SIMULATOR_ROLL,

   /* PHX_DST_FORMAT */
   PHX_DST_FORMAT_Y8  = (int) ((ui32) PHX_DST_FORMAT + 1),
   PHX_DST_FORMAT_Y16,
   PHX_DST_FORMAT_Y32,
   PHX_DST_FORMAT_Y36,     /* No longer supported in version 2 release */
   PHX_DST_FORMAT_RGB15,
   PHX_DST_FORMAT_RGB16,
   PHX_DST_XBGR32,         /* PHX_DST_FORMAT_RGBX32, */
   PHX_DST_BGRX32,         /* PHX_DST_FORMAT_XRGB32, */
   PHX_DST_FORMAT_RGB48,
   PHX_DST_FORMAT_BGR15,
   PHX_DST_FORMAT_BGR16,
   PHX_DST_XRGB32,         /* PHX_DST_FORMAT_BGRX32, */
   PHX_DST_RGBX32,         /* PHX_DST_FORMAT_XBGR32, */
   PHX_DST_FORMAT_BGR48,
   PHX_DST_FORMAT_RGB32,   /* Added in version 2 release */
   PHX_DST_FORMAT_BGR32,
   PHX_DST_FORMAT_RGB24,   /* Added in version 2.25 release */
   PHX_DST_FORMAT_BGR24,
   PHX_DST_FORMAT_Y10,
   PHX_DST_FORMAT_Y12,
   PHX_DST_FORMAT_Y14,
   PHX_DST_FORMAT_BAY8,
   PHX_DST_FORMAT_BAY10,
   PHX_DST_FORMAT_BAY12,
   PHX_DST_FORMAT_BAY14,
   PHX_DST_FORMAT_BAY16,
   PHX_DST_FORMAT_2Y12,
   PHX_DST_FORMAT_RGB36,
   PHX_DST_FORMAT_BGR36,
   PHX_DST_FORMAT_YUV422,
   PHX_DST_FORMAT_Y12B,
   /* Backward Compatibility prior to 5.54, New names are consistent with TMG */
   PHX_DST_FORMAT_RGBX32 = PHX_DST_XBGR32,
   PHX_DST_FORMAT_XRGB32 = PHX_DST_BGRX32,
   PHX_DST_FORMAT_BGRX32 = PHX_DST_XRGB32,
   PHX_DST_FORMAT_XBGR32 = PHX_DST_RGBX32,

   /* PHX_USER_FORMAT */
   PHX_USER_FORMAT_Y8  = (int) ((ui32) PHX_USER_FORMAT + 1),
   PHX_USER_FORMAT_Y16,
   PHX_USER_FORMAT_Y32,
   PHX_USER_FORMAT_Y36,     /* No longer supported in version 2 release */
   PHX_USER_FORMAT_RGB15,
   PHX_USER_FORMAT_RGB16,
   PHX_USER_XBGR32,        /* PHX_USER_FORMAT_RGBX32, */
   PHX_USER_BGRX32,        /* PHX_USER_FORMAT_XRGB32, */
   PHX_USER_FORMAT_RGB48,
   PHX_USER_FORMAT_BGR15,
   PHX_USER_FORMAT_BGR16,
   PHX_USER_XRGB32,        /* PHX_USER_FORMAT_BGRX32, */
   PHX_USER_RGBX32,        /* PHX_USER_FORMAT_XBGR32, */
   PHX_USER_FORMAT_BGR48,
   PHX_USER_FORMAT_RGB32,   /* Added in version 2 release */
   PHX_USER_FORMAT_BGR32,
   PHX_USER_FORMAT_RGB24,   /* Added in version 2.25 release */
   PHX_USER_FORMAT_BGR24,
   PHX_USER_FORMAT_Y10,
   PHX_USER_FORMAT_Y12,
   PHX_USER_FORMAT_Y14,
   PHX_USER_FORMAT_BAY8,
   PHX_USER_FORMAT_BAY10,
   PHX_USER_FORMAT_BAY12,
   PHX_USER_FORMAT_BAY14,
   PHX_USER_FORMAT_BAY16,
   PHX_USER_FORMAT_2Y12,
   PHX_USER_FORMAT_RGB36,
   PHX_USER_FORMAT_BGR36,
   PHX_USER_FORMAT_YUV422,
   PHX_USER_FORMAT_Y12B,
   /* Backward Compatibility prior to 5.54, New names are consistent with TMG */
   PHX_USER_FORMAT_RGBX32 = PHX_USER_XBGR32,
   PHX_USER_FORMAT_XRGB32 = PHX_USER_BGRX32,
   PHX_USER_FORMAT_BGRX32 = PHX_USER_XRGB32,
   PHX_USER_FORMAT_XBGR32 = PHX_USER_RGBX32,

   /* PHX_LINETRIG_SRC */
   PHX_LINETRIG_NONE  = (int) ((ui32) PHX_LINETRIG_SRC + 1),
   PHX_LINETRIG_AUXIN_1_RISING,        /* Relative */
   PHX_LINETRIG_AUXIN_1_FALLING,
   PHX_LINETRIG_CTRLIN_2_RISING,       /* No longer supported; retained for enum backward compatibility */
   PHX_LINETRIG_CTRLIN_2_FALLING,      /* No longer supported; retained for enum backward compatibility */
   PHX_LINETRIG_AUXIN_2_RISING,
   PHX_LINETRIG_AUXIN_2_FALLING,
   PHX_LINETRIG_TIMER,
   PHX_LINETRIG_AUXIN_A1_RISING,       /* Absolute */
   PHX_LINETRIG_AUXIN_A1_FALLING,
   PHX_LINETRIG_AUXIN_A2_RISING,
   PHX_LINETRIG_AUXIN_A2_FALLING,
   PHX_LINETRIG_AUXIN_B1_RISING,
   PHX_LINETRIG_AUXIN_B1_FALLING,
   PHX_LINETRIG_AUXIN_B2_RISING,
   PHX_LINETRIG_AUXIN_B2_FALLING,
   /* Backward Compatibility prior to 2.4.0 */
   PHX_LINETRIG_CTRLIN_1_RISING  = PHX_LINETRIG_AUXIN_1_RISING,
   PHX_LINETRIG_CTRLIN_1_FALLING = PHX_LINETRIG_AUXIN_1_FALLING,
   PHX_LINETRIG_CTRLIN_3_RISING  = PHX_LINETRIG_AUXIN_2_RISING,
   PHX_LINETRIG_CTRLIN_3_FALLING = PHX_LINETRIG_AUXIN_2_FALLING,

   /* PHX_LINETRIG_TIMER_CTRL */
   PHX_LINETRIG_TIMER_TIME = (int) ((ui32) PHX_LINETRIG_TIMER_CTRL + 1),
   PHX_LINETRIG_TIMER_DISABLE,
   PHX_LINETRIG_TIMER_LINES,
   /* Backward Compatibility prior to 3.53.0 */
   PHX_LINETRIG_TIMER_START = PHX_LINETRIG_TIMER_TIME,
   PHX_LINETRIG_TIMER_STOP  = PHX_LINETRIG_TIMER_DISABLE,

   /* PHX_EXPTRIG_SRC */
   PHX_EXPTRIG_LINETRIG = (int) ((ui32) PHX_EXPTRIG_SRC + 1),
   PHX_EXPTRIG_ACQTRIG,
   PHX_EXPTRIG_NONE,
   PHX_EXPTRIG_SWTRIG,
   PHX_EXPTRIG_AUXIN_1_RISING,        /* Relative */
   PHX_EXPTRIG_AUXIN_1_FALLING,
   PHX_EXPTRIG_AUXIN_2_RISING,
   PHX_EXPTRIG_AUXIN_2_FALLING,
   PHX_EXPTRIG_TIMER,
   PHX_EXPTRIG_AUXIN_A1_RISING,       /* Absolute */
   PHX_EXPTRIG_AUXIN_A1_FALLING,
   PHX_EXPTRIG_AUXIN_A2_RISING,
   PHX_EXPTRIG_AUXIN_A2_FALLING,
   PHX_EXPTRIG_AUXIN_B1_RISING,
   PHX_EXPTRIG_AUXIN_B1_FALLING,
   PHX_EXPTRIG_AUXIN_B2_RISING,
   PHX_EXPTRIG_AUXIN_B2_FALLING,
   /* Backward compatibility prior to 3.5.0 */
   PHX_EXP_LINETRIG = PHX_EXPTRIG_LINETRIG,
   PHX_EXP_ACQTRIG  = PHX_EXPTRIG_ACQTRIG,

   /* PHX_EXP_CTRLIO_1 */
   PHX_EXP_CTRLIO_1_HW_POS = (int) ((ui32) PHX_EXP_CTRLIO_1 + 1),
   PHX_EXP_CTRLIO_1_HW_NEG,
   PHX_EXP_CTRLIO_1_SW_POS,
   PHX_EXP_CTRLIO_1_SW_NEG,

   /* PHX_EXP_CTRLIO_2 */
   PHX_EXP_CTRLIO_2_HW_POS = (int) ((ui32) PHX_EXP_CTRLIO_2 + 1),
   PHX_EXP_CTRLIO_2_HW_NEG,
   PHX_EXP_CTRLIO_2_SW_POS,
   PHX_EXP_CTRLIO_2_SW_NEG,

   /* PHX_EXP_LINESTART */
   PHX_EXP_LINESTART_LINE = (int) ((ui32) PHX_EXP_LINESTART + 1),
   PHX_EXP_LINESTART_CCIO_2,     /* Relative */
   PHX_EXP_LINESTART_CCIO_A2,    /* Absolute */
   PHX_EXP_LINESTART_CCIO_B2,
   /* Backward Compatibility prior to 2.4.0 */
   PHX_EXP_LINESTART_CTRLIO_2 = PHX_EXP_LINESTART_CCIO_2,

   /* PHX_ACQTRIG_SRC */
   PHX_ACQTRIG_OPTO_A1    = (int) ((ui32) PHX_ACQTRIG_SRC + 1), /* Absolute */
   PHX_ACQTRIG_OPTO_A2,
   PHX_ACQTRIG_OPTO_B1,
   PHX_ACQTRIG_OPTO_B2,
   PHX_ACQTRIG_CTRLIN_A1,
   PHX_ACQTRIG_CTRLIN_A2,
   PHX_ACQTRIG_CTRLIN_A3,
   PHX_ACQTRIG_CTRLIN_B1,
   PHX_ACQTRIG_CTRLIN_B2,
   PHX_ACQTRIG_CTRLIN_B3,
   PHX_ACQTRIG_CCIO_A1,
   PHX_ACQTRIG_CCIO_A2,
   PHX_ACQTRIG_CCIO_B1,
   PHX_ACQTRIG_CCIO_B2,
   PHX_ACQTRIG_AUXIN_A1,
   PHX_ACQTRIG_AUXIN_A2,
   PHX_ACQTRIG_AUXIN_B1,
   PHX_ACQTRIG_AUXIN_B2,
   PHX_ACQTRIG_OPTO_1,        /* Relative */
   PHX_ACQTRIG_OPTO_2,
   PHX_ACQTRIG_AUXIN_1,
   PHX_ACQTRIG_AUXIN_2,
   PHX_ACQTRIG_CTRLIN_1,
   PHX_ACQTRIG_CTRLIN_2,
   PHX_ACQTRIG_CTRLIN_3,
   PHX_ACQTRIG_CCIO_1,
   PHX_ACQTRIG_CCIO_2,
   PHX_ACQTRIG_TIMER,
   /* Backward Compatibility prior to 2.4.0 */
   PHX_ACQTRIG_OPTO1     = PHX_ACQTRIG_OPTO_A1,
   PHX_ACQTRIG_OPTO2     = PHX_ACQTRIG_OPTO_A2,
   PHX_ACQTRIG_OPTO3     = PHX_ACQTRIG_OPTO_B1,
   PHX_ACQTRIG_OPTO4     = PHX_ACQTRIG_OPTO_B2,
   PHX_ACQTRIG_CTRL1IN_1 = PHX_ACQTRIG_CTRLIN_A1,
   PHX_ACQTRIG_CTRL1IN_2 = PHX_ACQTRIG_CTRLIN_A2,
   PHX_ACQTRIG_CTRL1IN_3 = PHX_ACQTRIG_CTRLIN_A3,
   PHX_ACQTRIG_CTRL2IN_1 = PHX_ACQTRIG_CTRLIN_B1,
   PHX_ACQTRIG_CTRL2IN_2 = PHX_ACQTRIG_CTRLIN_B2,
   PHX_ACQTRIG_CTRL2IN_3 = PHX_ACQTRIG_CTRLIN_B3,
   PHX_ACQTRIG_CTRLIO_1  = PHX_ACQTRIG_CCIO_A1,
   PHX_ACQTRIG_CTRLIO_2  = PHX_ACQTRIG_CCIO_A2,
   PHX_ACQTRIG_CTRLIO_3  = PHX_ACQTRIG_CCIO_B1,
   PHX_ACQTRIG_CTRLIO_4  = PHX_ACQTRIG_CCIO_B2,

   /* PHX_ACQTRIG_TYPE */
   PHX_ACQTRIG_NONE = (int) ((ui32) PHX_ACQTRIG_TYPE + 1),
   PHX_ACQTRIG_FIRST_POS_EDGE,
   PHX_ACQTRIG_FIRST_NEG_EDGE,
   PHX_ACQTRIG_EACH_POS_EDGE,
   PHX_ACQTRIG_EACH_NEG_EDGE,
   PHX_ACQTRIG_FIRST_POS_LEVEL,
   PHX_ACQTRIG_FIRST_NEG_LEVEL,
   PHX_ACQTRIG_EACH_POS_LEVEL,
   PHX_ACQTRIG_EACH_NEG_LEVEL,
   PHX_ACQTRIG_GATED_POS_LEVEL,
   PHX_ACQTRIG_GATED_NEG_LEVEL,

   /* PHX_ACQTRIG_ALIGN */
   PHX_ACQTRIG_ALIGN_NONE = (int) ((ui32) PHX_ACQTRIG_ALIGN + 1),
   PHX_ACQTRIG_ALIGN_TO_CLK,
   PHX_ACQTRIG_ALIGN_TO_LINE,
   PHX_ACQTRIG_ALIGN_TO_FRAME,

   /* PHX_ACQTRIG_DELAY_TYPE*/
   PHX_ACQTRIG_DELAY_NONE = (int) ((ui32) PHX_ACQTRIG_DELAY_TYPE + 1),
   PHX_ACQTRIG_DELAY_LINE,
   PHX_ACQTRIG_DELAY_TIMER,

   /* PHX_EVENTCOUNT_SRC */
   PHX_EVENTCOUNT_LINE = (int) ((ui32) PHX_EVENTCOUNT_SRC + 1),
   PHX_EVENTCOUNT_FRAME,
   PHX_EVENTCOUNT_TIME,

   /* PHX_EVENTGATE_SRC */
   PHX_EVENTGATE_ACQTRIG = (int) ((ui32) PHX_EVENTGATE_SRC + 1),
   PHX_EVENTGATE_FRAME,
   PHX_EVENTGATE_ACQ,
   PHX_EVENTGATE_LINE,
   PHX_EVENTGATE_START = PHX_EVENTGATE_ACQ,     /* Retained for backwards compatibility */

   /* PHX_DST_ENDIAN */
   PHX_DST_LITTLE_ENDIAN = (int) ((ui32) PHX_DST_ENDIAN + 1),
   PHX_DST_BIG_ENDIAN,

   /* PHX_CHAN_SYNC_MODE */
   PHX_CHAN_SYNC_NONE = (int) ((ui32) PHX_CHAN_SYNC_MODE + 1),
   PHX_CHAN_SYNC_ACQEXPTRIG,

   /* PHX_CVB_PARAM */
   PHX_CVB_WIDTH = (int) ((ui32) PHX_CVB_PARAM + 1),
   PHX_CVB_HEIGHT,
   PHX_CVB_PLANES,
   PHX_CVB_BIT_DEPTH,   /* per plane */
   PHX_CVB_BYTES_PER_PIXEL,
   PHX_CVB_X_STEP,
   PHX_CVB_Y_STEP,
   PHX_CVB_PLANE_STEP,
   PHX_CVB_MALLOC,

   /* PHX_ACQ_AUTO_RESTART, these parameter values are 'OR'able together */
   PHX_ACQ_AUTO_NONE          = (int) ((ui32) PHX_ACQ_AUTO_RESTART + 0x01),
   PHX_ACQ_AUTO_SYNC_LOST     = (int) ((ui32) PHX_ACQ_AUTO_RESTART + 0x02),
   PHX_ACQ_AUTO_FIFO_OVERFLOW = (int) ((ui32) PHX_ACQ_AUTO_RESTART + 0x04),

   /* PHX_BOARD_VARIANT */
   PHX_BOARD_DIGITAL          = (int) ((ui32) PHX_DIGITAL),
   PHX_BOARD_PHX_D24CL_PE1    = (int) ((ui32) PHX_D24CL_PE1),
   PHX_BOARD_PHX_D48CL_PE1    = (int) ((ui32) PHX_D48CL_PE1),
   PHX_BOARD_PHX_D48CL_PE4    = (int) ((ui32) PHX_D48CL_PE4),
   PHX_BOARD_PHX_D64CL_PE4    = (int) ((ui32) PHX_D64CL_PE4),
   PHX_BOARD_PHX_D24CL_PCI32  = (int) ((ui32) PHX_D24CL_PCI32),
   PHX_BOARD_PHX_D48CL_PCI32  = (int) ((ui32) PHX_D48CL_PCI32),
   PHX_BOARD_PHX_D48CL_PCI64  = (int) ((ui32) PHX_D48CL_PCI64),
   PHX_BOARD_PHX_D48CL_PCI64U = (int) ((ui32) PHX_D48CL_PCI64U),
   PHX_BOARD_PHX_D10HDSDI_PE1 = (int) ((ui32) PHX_D10HDSDI_PE1),
   PHX_BOARD_PHX_D20HDSDI_PE1 = (int) ((ui32) PHX_D20HDSDI_PE1),
   PHX_BOARD_PHX_D10HDSDI_PE4 = (int) ((ui32) PHX_D10HDSDI_PE4),
   PHX_BOARD_PHX_D20HDSDI_PE4 = (int) ((ui32) PHX_D20HDSDI_PE4),
   PHX_BOARD_PHX_D36_PE1      = (int) ((ui32) PHX_D36_PE1),
   PHX_BOARD_PHX_D36_PE4      = (int) ((ui32) PHX_D36_PE4),
   PHX_BOARD_PHX_D32_PCI32    = (int) ((ui32) PHX_D32_PCI32),
   PHX_BOARD_PHX_D36_PCI32    = (int) ((ui32) PHX_D36_PCI32),
   PHX_BOARD_PHX_D36_PCI64    = (int) ((ui32) PHX_D36_PCI64),
   PHX_BOARD_PHX_D36_PCI64U   = (int) ((ui32) PHX_D36_PCI64U),
   PHX_BOARD_PHX_D24AVDS_PE1  = (int) ((ui32) PHX_D24AVDS_PE1),

   /* PHX_BOARD_NUMBER */
   PHX_BOARD_NUMBER_AUTO      = (int) ((ui32) PHX_BOARD_AUTO),
   PHX_BOARD_NUMBER_1         = (int) ((ui32) PHX_BOARD1),
   PHX_BOARD_NUMBER_2         = (int) ((ui32) PHX_BOARD2),
   PHX_BOARD_NUMBER_3         = (int) ((ui32) PHX_BOARD3),
   PHX_BOARD_NUMBER_4         = (int) ((ui32) PHX_BOARD4),
   PHX_BOARD_NUMBER_5         = (int) ((ui32) PHX_BOARD5),
   PHX_BOARD_NUMBER_6         = (int) ((ui32) PHX_BOARD6),
   PHX_BOARD_NUMBER_7         = (int) ((ui32) PHX_BOARD7),  /* Add or reduce these, if PHX_BOARD_MAX changes */

   /* PHX_CHANNEL_NUMBER */
   PHX_CHANNEL_NUMBER_AUTO    = (int) ((ui32) PHX_CHANNEL_AUTO),
   PHX_CHANNEL_NUMBER_1       = (int) ((ui32) PHX_CHANNEL_A),
   PHX_CHANNEL_NUMBER_2       = (int) ((ui32) PHX_CHANNEL_B),
/* PHX_CHANNEL_NUMBER_3,
   PHX_CHANNEL_NUMBER_4, */

   /* PHX_CONFIG_MODE */
   PHX_CONFIG_NORMAL          = (int) ((ui32) PHX_MODE_NORMAL),
   PHX_CONFIG_COMMS_ONLY      = (int) ((ui32) PHX_COMMS_ONLY),
   PHX_CONFIG_ACQ_ONLY        = (int) ((ui32) PHX_ACQ_ONLY),


   /* PHX_INTRPT_CONDITIONS */
   PHX_INTRPT_TEST               = 0x00000001,
   PHX_INTRPT_DMA                = 0x00000002,  /* Retained for backwards compatibility */
   PHX_INTRPT_BUFFER_READY       = 0x00000002,
   PHX_INTRPT_FIFO_OVERFLOW      = 0x00000004,
   PHX_INTRPT_FIFO_A_OVERFLOW    = 0x00000004,  /* Retained for backwards compatibility */
   PHX_INTRPT_FRAME_LOST         = 0x00000008,
   PHX_INTRPT_CAPTURE_COMPLETE   = 0x00000010,
   PHX_INTRPT_FRAME_START        = 0x00000020,
   PHX_INTRPT_FRAME_END          = 0x00000040,
   PHX_INTRPT_LINE_START         = 0x00000080,
   PHX_INTRPT_LINE_END           = 0x00000100,
   PHX_INTRPT_ACQ_TRIG_START     = 0x00000200,
   PHX_INTRPT_ACQ_TRIG_END       = 0x00000400,
   PHX_INTRPT_TIMEOUT            = 0x00000800,
   PHX_INTRPT_SYNC_LOST          = 0x00001000,
   PHX_INTRPT_TIMER              = 0x00002000,
   PHX_INTRPT_GLOBAL_ENABLE      = (int) 0x80000000
/* PHX_INTRPT_FIFO_B_OVERFLOW    = 0x00000008,   * Removed in v3.0 of SDK */
} etParamValue ;


/*
PHX_IO_METHOD(ForBrief)
*/
typedef enum {
   PHX_EMASK_IO_METHOD              = 0x0F000000,
   PHX_IO_METHOD_WRITE              = 0x00000000,
   PHX_IO_METHOD_READ               = 0x00000000,
   PHX_IO_METHOD_BIT_SET            = 0x01000000,
   PHX_IO_METHOD_BIT_CLR            = 0x02000000,
   PHX_IO_METHOD_BIT_HW_POS         = 0x04000000,     /* Retained for backwards compatibility */
   PHX_IO_METHOD_BIT_TIMER_POS      = 0x04000000,
   PHX_IO_METHOD_BIT_ACQTRIG_POS    = 0x05000000,
   PHX_IO_METHOD_BIT_FIFO_WARN_POS  = 0x06000000,
   PHX_IO_METHOD_BIT_HW_NEG         = 0x08000000,     /* Retained for backwards compatibility */
   PHX_IO_METHOD_BIT_TIMER_NEG      = 0x08000000,
   PHX_IO_METHOD_BIT_ACQTRIG_NEG    = 0x09000000,
   PHX_IO_METHOD_BIT_FIFO_WARN_NEG  = 0x0A000000
} etPhxIoMethod;   

/*
PHX_BOARD_INFO(ForBrief)
*/
typedef enum {
   PHX_BOARD_INFO_PCI_3V            = 0x00000001,     /* 3V PCI Interface      */
   PHX_BOARD_INFO_PCI_5V            = 0x00000002,     /* 5V PCI Interface      */
   PHX_BOARD_INFO_PCI_33M           = 0x00000004,     /* 33MHz PCI Interface   */
   PHX_BOARD_INFO_PCI_66M           = 0x00000008,     /* 66MHz PCI Interface   */
   PHX_BOARD_INFO_PCI_32B           = 0x00000010,     /* 32bit PCI Interface   */
   PHX_BOARD_INFO_PCI_64B           = 0x00000020,     /* 64bit PCI Interface   */
   PHX_BOARD_INFO_LVDS              = 0x00000040,     /* LVDS Camera Interface */
   PHX_BOARD_INFO_CL                = 0x00000080,     /* Camera Link Interface */
   PHX_BOARD_INFO_CHAIN_MASTER      = 0x00000100,     /* Master Chaining */
   PHX_BOARD_INFO_CHAIN_SLAVE       = 0x00000200,     /* Slave Chaining  */
   PHX_BOARD_INFO_PCI_EXPRESS       = 0x00000400,     /* PCI Express interface  */
   PHX_BOARD_INFO_CL_BASE           = 0x00000800,     /* Camera Link Base only  */
   PHX_BOARD_INFO_CL_MEDIUM         = 0x00001000,     /* Camera Link Medium     */
   PHX_BOARD_INFO_CL_FULL           = 0x00002000,     /* Camera Link Full       */
   PHX_BOARD_INFO_BOARD_3V          = 0x00010000,     /* Board is 3V compatible */
   PHX_BOARD_INFO_BOARD_5V          = 0x00020000,     /* Board is 5V compatible */
   PHX_BOARD_INFO_BOARD_33M         = 0x00040000,     /* Board is 33MHz compatible */
   PHX_BOARD_INFO_BOARD_66M         = 0x00080000,     /* Board is 66MHz compatible */
   PHX_BOARD_INFO_BOARD_32B         = 0x00100000,     /* Board is 32bit compatible */
   PHX_BOARD_INFO_BOARD_64B         = 0x00200000      /* Board is 64bit compatible */
} etBoardInfo;



/* Acquire Function Definitions
 * ============================
 * These define the operations that can be performed by the 
 * PHX_Acquire() function.
 * They are implemented as enums to make them visible within the debugger.
 */
typedef enum {
   PHX_START                         = (int) ((ui32) PHX_ACQUIRE | (ui32) ( 1 << 8 )),
   PHX_CHECK_AND_WAIT                = (int) ((ui32) PHX_ACQUIRE | (ui32) ( 2 << 8 )),
   PHX_CHECK_AND_RETURN              = (int) ((ui32) PHX_ACQUIRE | (ui32) ( 3 << 8 )),
   PHX_STOP                          = (int) ((ui32) PHX_ACQUIRE | (ui32) ( 4 << 8 )),
   PHX_ABORT                         = (int) ((ui32) PHX_ACQUIRE | (ui32) ( 5 << 8 )),
   PHX_BUFFER_GET                    = (int) ((ui32) PHX_ACQUIRE | (ui32) ( 6 << 8 )),
   PHX_BUFFER_RELEASE                = (int) ((ui32) PHX_ACQUIRE | (ui32) ( 7 << 8 )),
   PHX_BUFFER_ABORT                  = (int) ((ui32) PHX_ACQUIRE | (ui32) ( 8 << 8 )),
   PHX_EVENT_HANDLER                 = (int) ((ui32) PHX_ACQUIRE | (ui32) ( 9 << 8 )),
   PHX_START_IMMEDIATELY             = (int) ((ui32) PHX_ACQUIRE | (ui32) (10 << 8 )),
   PHX_EXPOSE                        = (int) ((ui32) PHX_ACQUIRE | (ui32) (11 << 8 )),
   PHX_UNLOCK                        = (int) ((ui32) PHX_ACQUIRE | (ui32) (12 << 8 )),
   PHX_AUTO_WHITE_BALANCE            = (int) ((ui32) PHX_ACQUIRE | (ui32) (13 << 8 )),
   PHX_AUTO_RESTART                  = (int) ((ui32) PHX_ACQUIRE | (ui32) (14 << 8 )),
   PHX_BUFFER_GET_MERGED             = (int) ((ui32) PHX_ACQUIRE | (ui32) (15 << 8 )),
   PHX_USER_LOCK                     = (int) ((ui32) PHX_ACQUIRE | (ui32) (16 << 8 )),
   PHX_USER_UNLOCK                   = (int) ((ui32) PHX_ACQUIRE | (ui32) (17 << 8 ))
} etAcq;   
 
 
/* This structure is used to specify the address and context of a block of
 * memory (used to specify user allocated image buffers). It can be used
 * either for virtual or physical addresses.
 */
typedef struct {
   void *pvAddress;
   void *pvContext;
} stImageBuff;

/* This structure is used to specify the address, size and context of a
 * block of memory (used to specify user locked image buffers).
 */
typedef struct {
   void *pvAddress;
   ui64  qwSizeBytes;
   void *pvContext;
} stUserBuff;

/*
 * NOTE
 * If you change the following values you ***MUST*** change the
 * appropriate number of initializers for the global array.
 */
/* The maximum allowable number of image buffers. */
#define PHX_MAX_BUFFS         0x00007FFF
#define PHX_BUFFS_MASK        0x00007FFF

/* Timeout Definitions
 * ===================
 */
enum eTimeouts {
   PHX_TIMEOUT_IMMEDIATE = 0,
   PHX_TIMEOUT_INFINITE  = -1
};

/* Status Definitions
 * ==================
 */
enum eStat {
   PHX_OK = 0,
   PHX_ERROR_BAD_HANDLE,
   PHX_ERROR_BAD_PARAM,
   PHX_ERROR_BAD_PARAM_VALUE,
   PHX_ERROR_READ_ONLY_PARAM,
   PHX_ERROR_OPEN_FAILED,
   PHX_ERROR_INCOMPATIBLE,
   PHX_ERROR_HANDSHAKE,
   PHX_ERROR_INTERNAL_ERROR,
   PHX_ERROR_OVERFLOW,
   PHX_ERROR_NOT_IMPLEMENTED,
   PHX_ERROR_HW_PROBLEM,
   PHX_ERROR_NOT_SUPPORTED,
   PHX_ERROR_OUT_OF_RANGE,
   PHX_ERROR_MALLOC_FAILED,
   PHX_ERROR_SYSTEM_CALL_FAILED,
   PHX_ERROR_FILE_OPEN_FAILED,
   PHX_ERROR_FILE_CLOSE_FAILED,
   PHX_ERROR_FILE_INVALID,
   PHX_ERROR_BAD_MEMBER,
   PHX_ERROR_HW_NOT_CONFIGURED,
   PHX_ERROR_INVALID_FLASH_PROPERTIES,
   PHX_ERROR_ACQUISITION_STARTED,
   PHX_ERROR_INVALID_POINTER,
   PHX_ERROR_LIB_INCOMPATIBLE,
   PHX_ERROR_SLAVE_MODE,

   /* Phoenix display library errors */
   PHX_ERROR_DISPLAY_CREATE_FAILED,
   PHX_ERROR_DISPLAY_DESTROY_FAILED,
   PHX_ERROR_DDRAW_INIT_FAILED,
   PHX_ERROR_DISPLAY_BUFF_CREATE_FAILED,
   PHX_ERROR_DISPLAY_BUFF_DESTROY_FAILED,
   PHX_ERROR_DDRAW_OPERATION_FAILED,

   /* Registry errors */
   PHX_ERROR_WIN32_REGISTRY_ERROR,

   PHX_ERROR_INVALID_FLASH_PROPERITES = PHX_ERROR_INVALID_FLASH_PROPERTIES, /* Backwards compatibility */

   PHX_WARNING                = 0x8000,
   PHX_WARNING_TIMEOUT        = ( PHX_WARNING | 0x00 ),
   PHX_WARNING_FLASH_RECONFIG,
   PHX_WARNING_ZBT_RECONFIG,
   PHX_WARNING_NOT_PHX_COM,
   PHX_WARNING_NO_PHX_BOARD_REGISTERED 
};
typedef enum eStat etStat;

/* This structure is used to specify a colour by its individual components.
 */
typedef struct {
   ui8 bRed;
   ui8 bGreen;
   ui8 bBlue;
} tColour;

/* This structure holds the details of a logical LUT
 */
struct tLutInfo{
   ui32 dwLut;
   ui32 dwColour;
   ui32 dwTap;
   ui32 dwBrightness;
   ui32 dwContrast;
   ui32 dwGamma;
   ui32 dwFloor;
   ui32 dwCeiling;
   ui16 *pwLutData;
   ui32 dwSize;
};

/* Default settings for the Luts */
enum eLutCtrl {
   PHX_LUT_DEFAULT_BRIGHTNESS    = 100,
   PHX_LUT_DEFAULT_CONTRAST      = 100,
   PHX_LUT_DEFAULT_GAMMA         = 100,
   PHX_LUT_MAX_LUTS              = 256,  /* Maximum number of LUTs across a line */
   PHX_LUT_MAX_COLS              = 3,    /* Maximum number of LUT components */
   PHX_LUT_MAX_TAPS              = 2     /* Maximum number of camera taps */
};


/* Prototype Definitions
 * =====================
 */

#if defined __cplusplus
extern "C" {
#endif
etStat PHX_EXPORT_FN PHX_Create          ( tHandle*, void (*) (const char*, etStat, const char*) );
etStat PHX_EXPORT_FN PHX_Open            ( tHandle );
etStat PHX_EXPORT_FN PHX_Close           ( tHandle* );
etStat PHX_EXPORT_FN PHX_Destroy         ( tHandle* );
etStat PHX_EXPORT_FN PHX_CameraConfigSave( tHandle,  char*, etCamConfigSave );
etStat PHX_EXPORT_FN PHX_ParameterGet    ( tHandle, etParam, void* );
etStat PHX_EXPORT_FN PHX_ParameterSet    ( tHandle, etParam, void* );
etStat PHX_EXPORT_FN PHX_StreamRead      ( tHandle, etAcq,   void* );
etStat PHX_EXPORT_FN PHX_ControlRead     ( tHandle, etControlPort, void*, ui8*, ui32*, ui32 );
etStat PHX_EXPORT_FN PHX_ControlWrite    ( tHandle, etControlPort, void*, ui8*, ui32*, ui32 );

void   PHX_EXPORT_FN PHX_ErrHandlerDefault ( const char*, etStat, const char* );
void   PHX_EXPORT_FN PHX_ErrCodeDecode     ( char*,       etStat );
void   PHX_EXPORT_FN PHX_DebugDefaultTraceHandler( ui8 * );

etStat PHX_EXPORT_FN PHX_MutexCreate(  tHandle hCamera, tPHX *phPhxMutex, char *szMutexName );
etStat PHX_EXPORT_FN PHX_MutexDestroy( tHandle hCamera, tPHX *phPhxMutex );
etStat PHX_EXPORT_FN PHX_MutexAcquire( tHandle hCamera, tPHX   hPhxMutex, ui32 dwTimeout );
etStat PHX_EXPORT_FN PHX_MutexRelease( tHandle hCamera, tPHX   hPhxMutex );
etStat PHX_EXPORT_FN PHX_SemaphoreCreate(  tHandle hCamera, tPHX *phPhxSem, ui32 dwInitialCount, ui32 dwMaximumCount );
etStat PHX_EXPORT_FN PHX_SemaphoreDestroy( tHandle hCamera, tPHX *phPhxSem );
etStat PHX_EXPORT_FN PHX_SemaphoreSignal(  tHandle hCamera, tPHX   hPhxSem, ui32 dwCount );
etStat PHX_EXPORT_FN PHX_SemaphoreWaitWithTimeout( tHandle hCamera, tPHX hPhxSem, ui32 dwWait );
etStat PHX_EXPORT_FN PHX_ComParameterGet( ui32, etComParam, void* );
etStat PHX_EXPORT_FN PHX_MemoryAlloc      ( tHandle, void **, ui32, ui32, ui32 );
void   PHX_EXPORT_FN PHX_MemoryFreeAndNull( tHandle, void ** );

/* Legacy Section
 * ===============================
 */
etStat PHX_EXPORT_FN PHX_Acquire         ( tHandle, etAcq,   void* );
etStat PHX_EXPORT_FN PHX_CameraConfigLoad( tHandle*, char*, etCamConfigLoad, void (*) (const char*, etStat, const char*) );
etStat PHX_EXPORT_FN PHX_CameraRelease   ( tHandle* );
etStat PHX_EXPORT_FN PHX_CommsReceive    ( tHandle, ui8*, ui32*, ui32 );
etStat PHX_EXPORT_FN PHX_CommsTransmit   ( tHandle, ui8*, ui32*, ui32 );

#if defined __cplusplus
};
#endif

/* Backwards Compatibility Section
 * ===============================
 */
#define etCamConfig            etCamConfigLoad  /* etCamConfig was used in PHX releases prior to version 2.2.10 */
#define PHX_MAX_IMAGE_BUFFERS  PHX_MAX_BUFFS    /* Prior to v3.04 */

#endif  /* _PHX_API_H_ */

