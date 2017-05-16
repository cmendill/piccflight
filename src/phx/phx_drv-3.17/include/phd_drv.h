/****************************************************************************
 *
 * ACTIVE SILICON LIMITED
 *
 * File name   : phd_drv.h
 * Function    : Hardware definitions required by the kernel mode driver
 * Project     : AP06
 * Authors     : Martin Bone
 * Systems     : ANSI C, Win32 (MSVC), 32 bit DOS (Symantec).
 * Version     : See phd_api.h for overall library release information.
 * Release Date: See phd_api.h for overall library release information.
 *
 * Copyright (c) 2000 Active Silicon Ltd.
 ****************************************************************************
 * Comments:
 * --------
 *
 * File History
 * ------------
 * $Log: phd_drv.h,v $
 * Revision 1.21  2010/02/01 12:02:31  richard
 * Added support for PHX_INTRPT_FRAME_LOST
 *
 * Revision 1.20  2009/05/18 15:52:26  richard
 * Added D24AVDS-PE1
 *
 * Revision 1.19  2008/10/02 15:18:25  richard
 * Changed the order of the HDSDI boards.
 *
 * Revision 1.18  2008/07/15 09:00:20  richard
 * Added HDSDI-PE4 boards.
 *
 * Revision 1.17  2008/07/01 16:19:43  richard
 * Changed D48CL_PE4
 * Added D64CL_PE4
 * Moved PHD_HBIT_MISC_UART_RESET
 * Moved PHD_ADDR_REG_MISC_CTRL
 *
 * Revision 1.16  2008/04/09 13:37:16  richard
 * Added PHX_INTRPT_SYNC_LOST
 *
 * Revision 1.15  2008/03/04 16:57:19  richard
 * Replaced SDI with D10HDSDH;
 * Added D20HDSDI.
 *
 * Revision 1.14  2008/02/01 10:42:27  richard
 * Added SDI-PE1
 *
 * Revision 1.13  2007/12/12 15:56:24  richard
 * Added d48cl-pe4
 *
 * Revision 1.12  2007/10/12 13:46:40  richard
 * Added support for D48CL-PCI32
 *
 * Revision 1.11  2007/02/19 15:14:52  richard
 * Added D36_PE1_ID and DNC_PE1_ID
 *
 * Revision 1.10  2007/02/16 09:13:48  richard
 * Changed PHD_HMASK_INTRPT_CH* to 7FFF
 *
 * Revision 1.9  2006/10/13 15:14:51  richard
 * Added D48CL-PE1 support
 *
 * Revision 1.8  2006/06/28 14:19:34  richard
 * Added PHX-D24CL-PE1 board support
 *
 * Revision 1.7  2005/07/11 08:24:04  richard
 * etPhdHwDrvBit : added PHD_*_INTRPT_TIMEOUT.
 *
 * Revision 1.6  2005/05/04 14:48:35  martin
 * Merging changes from branch 1.5.2.1
 *
 * Revision 1.5.2.1  2005/05/03 11:33:55  martin
 * Added Timer Interrupt control settings.
 *
 * Revision 1.5  2004/10/18 09:27:07  richard
 * Moved interrupt control register bits from phd_hw.h.
 *
 * Revision 1.4  2004/07/22 10:34:20  martin
 * Added <CR><LF> at end of file to stop VxWorks compiler crashing.
 *
 * Revision 1.3  2004/05/27 13:12:49  martin
 * Added PCI SubSystem & SubVendor definitions for all Phoenix variants.
 *
 * Revision 1.2  2004/04/08 14:55:39  martin
 * Added PHD_SUBSYSTEM_ID definition for backward compatability with LFG CDA driver build.
 *
 * Revision 1.1  2004/04/02 11:20:41  martin
 * File creation
 *
 *
 ****************************************************************************
 */


#ifndef _PHD_DRV_H_
#define _PHD_DRV_H_

/* Phoenix DIG36 PCI Constants
 * ---------------------------
 */
typedef enum {
   PHD_VENDOR_ID                   = 0x10EE,  /* Vendor ID    - Xilinx          */
   PHD_DEVICE_ID                   = 0x0300,  /* Device ID    - LogiCore        */

   PHD_SUBVENDOR_ID                = 0x4153,  /* SubVendor ID - "AS" (ie ASL)   */

                                              /* SubSystem IDs                  */
   PHD_SUBSYSTEM_GENERIC_ID        = 0x0001,  /* A generic Phoenix !            */
   PHD_SUBSYSTEM_DNC_PE1_ID        = 0x01FF,  /* AS-PHX-DNC-PE1                 */
   PHD_SUBSYSTEM_DNC_PE4_ID        = 0x03FF,  /* AS-PHX-DNC-PE4                 */
   PHD_SUBSYSTEM_D24CL_PCI32_ID    = 0x0080,  /* AS-PHX-D24CL-PCI32             */
   PHD_SUBSYSTEM_D24CL_PE1_ID      = 0x0180,  /* AS-PHX-D24CL-PE1               */
   PHD_SUBSYSTEM_D24AVDS_PE1_ID    = 0x0120,  /* AS-PHX-D24AVDS-PE1             */
   PHD_SUBSYSTEM_D32_PCI32_ID      = 0x0048,  /* AS-PHX-D32-PCI32               */
   PHD_SUBSYSTEM_D36_PCI32_ID      = 0x0040,  /* AS-PHX-D36-PCI32               */
   PHD_SUBSYSTEM_D36_PCI64_ID      = 0x0070,  /* AS-PHX-D36-PCI64               */
   PHD_SUBSYSTEM_D36_PCI64U_ID     = 0x0050,  /* AS-PHX-D36-PCI64U              */
   PHD_SUBSYSTEM_D36_PE1_ID        = 0x0140,  /* AS-PHX-D36-PE1                 */
   PHD_SUBSYSTEM_D10HDSDI_PE1_ID   = 0x0110,  /* AS-PHX-D10HDSDI-PE1            */
   PHD_SUBSYSTEM_D10HDSDI_PE4_ID   = 0x0310,  /* AS-PHX-D10HDSDI-PE4            */
   PHD_SUBSYSTEM_D20HDSDI_PE1_ID   = 0x0150,  /* AS-PHX-D20HDSDI-PE1            */
   PHD_SUBSYSTEM_D20HDSDI_PE4_ID   = 0x0350,  /* AS-PHX-D20HDSDI-PE4            */
   PHD_SUBSYSTEM_D48CL_PCI32_ID    = 0x00C0,  /* AS-PHX-D48CL-PCI32             */
   PHD_SUBSYSTEM_D48CL_PCI64_ID    = 0x00F0,  /* AS-PHX-D48CL-PCI64             */
   PHD_SUBSYSTEM_D48CL_PCI64U_ID   = 0x00D0,  /* AS-PHX-D48CL-PCI64U            */
   PHD_SUBSYSTEM_D48CL85_PCI64U_ID = 0x00D8,  /* AS-PHX-D48CL85-PCI64U          */
   PHD_SUBSYSTEM_D48CL_PE1_ID      = 0x01D0,  /* AS-PHX-D48CL-PE1               */
   PHD_SUBSYSTEM_D48CL_PE4_ID      = 0x03D0,  /* AS-PHX-D48CL-PE4               */
   PHD_SUBSYSTEM_D64CL_PE4_ID      = 0x03F0,  /* AS-PHX-D64CL-PE4               */

   PHD_SUBSYSTEM_ID                = PHD_SUBSYSTEM_GENERIC_ID, /* Backwards compatibility */
   PHD_FUNCTION_NUMBER             = 0x0000   /* Function number - only 1 function */
} etPhdPci;



/* Register Addresses
 * ------------------
Register_Addresses(ForBrief)
 */
typedef enum {
   PHD_ADDR_REG_INTRPT_CTRL       = 0x000000,      /* dwRegIntrptCtrl      */
   PHD_ADDR_REG_INTRPT_CLR        = 0x000004,      /* dwRegIntrptClr       */
   PHD_ADDR_REG_MISC_CTRL         = 0x000038,      /* dwRegMiscCtrl        */
   PHD_ADDR_REG_RISC_CTRL         = 0x280004,      /* dwRegRiscCtrl        */
   PHD_ADDR_STAT_INTRPT           = PHD_ADDR_REG_INTRPT_CTRL
} etPhdHwDrvAddr ;


/* Register Bit Definitions
 * ------------------------
Register_bits(ForBrief)
 */
typedef enum {
   /* Interrupt Control
    * =================
    */
   PHD_HMASK_INTRPT_TEST               = (int) 0x00000001,   /* Test interrupt control */
   PHD_HBIT_INTRPT_TEST_EN             = (int) 0x00000001,
   PHD_HBIT_INTRPT_TEST_DIS            = (int) 0x00000000,

   PHD_HMASK_INTRPT_RISC               = (int) 0x00000002,   /* RISC machine interrupt control */
   PHD_HBIT_INTRPT_RISC_EN             = (int) 0x00000002,
   PHD_HBIT_INTRPT_RISC_DIS            = (int) 0x00000000,

   PHD_HMASK_INTRPT_FIFO_OVERFLOW      = (int) 0x00000004,   /* FIFO full interrupt control */
   PHD_HBIT_INTRPT_FIFO_OVERFLOW_EN    = (int) 0x00000004,
   PHD_HBIT_INTRPT_FIFO_OVERFLOW_DIS   = (int) 0x00000000,

   PHD_HMASK_INTRPT_FRAME_LOST         = (int) 0x00000008,   /* Frame Lost interrupt control */
   PHD_HBIT_INTRPT_FRAME_LOST_EN       = (int) 0x00000008,
   PHD_HBIT_INTRPT_FRAME_LOST_DIS      = (int) 0x00000000,

   PHD_HMASK_INTRPT_FRAME_START        = (int) 0x00000020,   /* Start of frame interrupt control */
   PHD_HBIT_INTRPT_FRAME_START_EN      = (int) 0x00000020,
   PHD_HBIT_INTRPT_FRAME_START_DIS     = (int) 0x00000000,

   PHD_HMASK_INTRPT_FRAME_END          = (int) 0x00000040,   /* End of frame interrupt control */
   PHD_HBIT_INTRPT_FRAME_END_EN        = (int) 0x00000040,
   PHD_HBIT_INTRPT_FRAME_END_DIS       = (int) 0x00000000,

   PHD_HMASK_INTRPT_LINE_START         = (int) 0x00000080,   /* Start of line interrupt control */
   PHD_HBIT_INTRPT_LINE_START_EN       = (int) 0x00000080,
   PHD_HBIT_INTRPT_LINE_START_DIS      = (int) 0x00000000,

   PHD_HMASK_INTRPT_LINE_END           = (int) 0x00000100,   /* End of line interrupt control */
   PHD_HBIT_INTRPT_LINE_END_EN         = (int) 0x00000100,
   PHD_HBIT_INTRPT_LINE_END_DIS        = (int) 0x00000000,

   PHD_HMASK_INTRPT_ACQ_TRIG_START     = (int) 0x00000200,   /* Start of acquisition trigger interrupt control */
   PHD_HBIT_INTRPT_ACQ_TRIG_START_EN   = (int) 0x00000200,
   PHD_HBIT_INTRPT_ACQ_TRIG_START_DIS  = (int) 0x00000000,

   PHD_HMASK_INTRPT_ACQ_TRIG_END       = (int) 0x00000400,   /* End of acquisition trigger interrupt control */
   PHD_HBIT_INTRPT_ACQ_TRIG_END_EN     = (int) 0x00000400,
   PHD_HBIT_INTRPT_ACQ_TRIG_END_DIS    = (int) 0x00000000,

   PHD_HMASK_INTRPT_TIMEOUT            = (int) 0x00000800,   /* Timeout interrupt control */
   PHD_HBIT_INTRPT_TIMEOUT_EN          = (int) 0x00000800,
   PHD_HBIT_INTRPT_TIMEOUT_DIS         = (int) 0x00000000,

   PHD_HMASK_INTRPT_SYNC_LOST          = (int) 0x00001000,   /* Sync Lost interrupt control */
   PHD_HBIT_INTRPT_SYNC_LOST_EN        = (int) 0x00001000,
   PHD_HBIT_INTRPT_SYNC_LOST_DIS       = (int) 0x00000000,

   PHD_HMASK_INTRPT_TIMER              = (int) 0x00002000,   /* Timer interrupt control */
   PHD_HBIT_INTRPT_TIMER_EN            = (int) 0x00002000,
   PHD_HBIT_INTRPT_TIMER_DIS           = (int) 0x00000000,

   PHD_HMASK_INTRPT_UART               = (int) 0x00004000,   /* UART interrupt control */
   PHD_HBIT_INTRPT_UART_EN             = (int) 0x00004000,
   PHD_HBIT_INTRPT_UART_DIS            = (int) 0x00000000,

   PHD_HMASK_INTRPT_GLOBAL             = (int) 0x00008000,   /* Global interrupt control */
   PHD_HBIT_INTRPT_GLOBAL_EN           = (int) 0x00008000,
   PHD_HBIT_INTRPT_GLOBAL_DIS          = (int) 0x00000000,

   PHD_HMASK_INTRPT_CH1                = (int) 0x00007FFF,
   PHD_HMASK_INTRPT_CH2                = (int) 0x7FFF0000,

   PHD_HBIT_MISC_UART_RESET            = (int) 0x00000010    /* Reset the external UARTs */
} etPhdHwDrvBit ;


#endif /* _PHD_DRV_H_ */


