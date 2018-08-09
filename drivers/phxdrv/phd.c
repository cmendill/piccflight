/****************************************************************************
 *
 * ACTIVE SILICON LIMITED
 *
 * File name   : Phd.c
 * Function    : PHD specific code for the Linux driver for CDA library.
 * Project     : AP06
 * Authors     : Lawrence Rust
 * Systems     : Linux
 * Version     : See cda_api.h for overall library release information.
 * Release Date: See cda_api.h for overall library release information.
 *
 * Copyright (c) 2000-2004 Active Silicon Ltd.
 ****************************************************************************
 * File History
 * ------------
 * See also cda_api.h for overall library release information.
 *
 * Comments:
 * --------
 * This file contains PHD specific code for the CDA linux driver functions.
 * 
 ****************************************************************************/

/* Exports */

/* Imports */

#include <linux/module.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/bitops.h>
#include <asm/io.h> 
#include <linux/sched.h>
/*#include <linux/smp_lock.h>*/
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0) )
#include <linux/wrapper.h>
#endif
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <asm/uaccess.h>

#include "linuxdrv.h"
#include "cdadrv.h"
#include "cdapci.h"

#include <cda_lib.h>			/* For data definition used in Phoenix code */
#include <phd_drv.h>                    /* For register addresses                   */


/*
 * Local data types
 */


/*
 * Prototypes
 */
static PCI_FReset PHD_Reset;
static PCI_FIrqHandler PHD_IrqHandler;

/*
 * Module data
 */
/* PCI device info */
static PCI_SDevInfo const sPciInfo_AsPhxGeneric =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_GENERIC_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD10PE1 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D10HDSDI_PE1_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD10PE4 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D10HDSDI_PE4_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD20PE1 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D20HDSDI_PE1_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD20PE4 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D20HDSDI_PE4_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD24P32 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D24CL_PCI32_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD24PE1 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D24CL_PE1_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD32P32 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D32_PCI32_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD36P32 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D36_PCI32_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD36P64 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D36_PCI64_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD36P64U =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D36_PCI64U_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD36PE1 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D36_PE1_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD48P32 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D48CL_PCI32_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD48P64 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D48CL_PCI64_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD48P64U =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D48CL_PCI64U_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD48PE1 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D48CL_PE1_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD48PE4 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D48CL_PE4_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };

static PCI_SDevInfo const sPciInfo_AsPhxD64PE4 =
  {
  PHD_VENDOR_ID,
  PHD_DEVICE_ID,
  PHD_SUBVENDOR_ID,
  PHD_SUBSYSTEM_D64CL_PE4_ID,
  0x1000000,                            /* memory mapped register space */
  0,                                    /* Initialise */
  0,                                    /* Finalise */
  &PHD_Reset,
  &PHD_IrqHandler,
  };


/*
 * Enumerate, probe, and register all supported cards
 */
void PHD_Probe_AsPhxGeneric( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxGeneric );
}

void PHD_Probe_AsPhxD10PE1( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD10PE1 );
}

void PHD_Probe_AsPhxD10PE4( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD10PE4 );
}

void PHD_Probe_AsPhxD20PE1( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD20PE1 );
}

void PHD_Probe_AsPhxD20PE4( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD20PE4 );
}

void PHD_Probe_AsPhxD24P32( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD24P32 );
}

void PHD_Probe_AsPhxD24PE1( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD24PE1 );
}

void PHD_Probe_AsPhxD32P32( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD32P32 );
}

void PHD_Probe_AsPhxD36P32( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD36P32 );
}

void PHD_Probe_AsPhxD36P64( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD36P64 );
}

void PHD_Probe_AsPhxD36P64U( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD36P64U );
}

void PHD_Probe_AsPhxD36PE1( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD36PE1 );
}

void PHD_Probe_AsPhxD48P32( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD48P32 );
}

void PHD_Probe_AsPhxD48P64( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD48P64 );
}

void PHD_Probe_AsPhxD48P64U( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD48P64U );
}

void PHD_Probe_AsPhxD48PE1( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD48PE1 );
}

void PHD_Probe_AsPhxD48PE4( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD48PE4 );
}

void PHD_Probe_AsPhxD64PE4( void )
{
   /* Find this PCI device */
   CDA_PCI_Probe( &sPciInfo_AsPhxD64PE4 );
}


/*
 * Reset the device, disable interrupts & stop DMA
 */
static int PHD_Reset(
  void* pvBase
) {
  _DRV_Write_ui32( pvBase, PHD_ADDR_REG_RISC_CTRL,   0);           /* Disable the RISC engine */
  _DRV_Write_ui32( pvBase, PHD_ADDR_REG_INTRPT_CTRL, 0);           /* Disable all interrupts */
  _DRV_Write_ui32( pvBase, PHD_ADDR_REG_INTRPT_CLR,  0xFFFFFFFF);  /* Clear any pending interrupts */

  return 1; /* Device reset OK */
  }


/*
 * IRQ handler
 */
static int PHD_IrqHandler(
  void* pvBase,
  ui32* pulEvent,
  ui32* pulData
) {

  int bRet;
  ui32 status;

  /* Check if card is asserting INT */
  status = _DRV_Read_ui32( pvBase, PHD_ADDR_STAT_INTRPT );
  if ( status)
    {
    /* Clear the IRQ */
    _DRV_Write_ui32( pvBase, PHD_ADDR_REG_INTRPT_CLR, status );
    /* Return event info */
    *pulEvent = status;
    *pulData = 0;

    bRet = 1;
    }
  else
    {
    /* Not our IRQ */
    bRet = 0;
    }

  return bRet;
  }

/* End of file */
