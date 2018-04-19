/****************************************************************************
 *
 * ACTIVE SILICON LIMITED
 *
 * File name   : cdapci.h
 * Function    : Linux CDA driver. Kernel mode PCI specific code
 * Project     : AP04
 * Authors     : Lawrence Rust
 * Systems     : ANSI C.
 * Version     : See cdadrv.h, "#define CDA_DRV_VERSION".
 * Release Date: See cdadrv.h, "#define CDA_DRV_VERSION".
 *
 * Copyright (c) 2000-2004 Active Silicon Ltd.
 ****************************************************************************
 *
 * Comments:
 * --------
 * This file contains the CDA Linux driver PCI specific code
 *
 ****************************************************************************
 */

#ifndef CDAPCI_H
#define CDAPCI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
#include <asm/io.h>             /* IO address mapping */
#include "cdadrv.h"


/* Memory mapped register access */
#if defined NDEBUG
#define _DRV_Write_ui8( dwBasePhysAddr, dwAddrOffset, data ) \
         do { writeb( data, (ui8*)(dwBasePhysAddr) + (dwAddrOffset) ); mb(); } while(0)

#define _DRV_Write_ui16( dwBasePhysAddr, dwAddrOffset, data ) \
         do { writew( data, (ui8*)(dwBasePhysAddr) + (dwAddrOffset) ); mb(); } while(0)

#define _DRV_Write_ui32( dwBasePhysAddr, dwAddrOffset, data ) \
         do { writel( data, (ui8*)(dwBasePhysAddr) + (dwAddrOffset) ); mb(); } while(0)
#else
#define _DRV_Write_ui8( dwBasePhysAddr, dwAddrOffset, data ) \
         do {/*printk("_DRV_Write_ui8(%p, 0x%08X, 0x%02X)\n", dwBasePhysAddr, dwAddrOffset, data);*/ writeb( data, (ui8*)(dwBasePhysAddr) + (dwAddrOffset) ); mb(); } while(0)

#define _DRV_Write_ui16( dwBasePhysAddr, dwAddrOffset, data ) \
         do {/*printk("_DRV_Write_ui16(%p, 0x%08X, 0x%04X)\n", dwBasePhysAddr, dwAddrOffset, data);*/ writew( data, (ui8*)(dwBasePhysAddr) + (dwAddrOffset) ); mb(); } while(0)

#define _DRV_Write_ui32( dwBasePhysAddr, dwAddrOffset, data ) \
         do {/*printk("_DRV_Write_ui32(%p, 0x%08X, 0x%08X)\n", dwBasePhysAddr, dwAddrOffset, data);*/ writel( data, (ui8*)(dwBasePhysAddr) + (dwAddrOffset) ); mb(); } while(0)
#endif
#define _DRV_Read_ui8( dwBasePhysAddr, dwAddrOffset ) \
         readb( (ui8*)(dwBasePhysAddr) + (dwAddrOffset))

#define _DRV_Read_ui16( dwBasePhysAddr, dwAddrOffset ) \
         readw( (ui8*)(dwBasePhysAddr) + (dwAddrOffset))

#define _DRV_Read_ui32( dwBasePhysAddr, dwAddrOffset ) \
         readl( (ui8*)(dwBasePhysAddr) + (dwAddrOffset))


/* Types */
struct pci_dev;
typedef int PCI_FInitialise( struct pci_dev *, void*);
typedef int PCI_FFinalise( struct pci_dev *);
typedef int PCI_FReset( void*);
typedef int PCI_FIrqHandler( void*, ui32* pulEvent, ui32* pulData);

/* Registration data */
typedef struct PCI_SDevInfo
  {
  ui16 vendorID;                        /* Required */
  ui16 deviceID;                        /* Required */
  ui16 subsystemVendorID;               /* Optional, 0= none */
  ui16 subsystemID;                     /* */
  ui32 size;                            /* Register block size */
  PCI_FInitialise* pfnInitialise;       /* Initialise the device, disable IRQs & DMA */
  PCI_FFinalise* pfnFinalise;           /* Finalise the device */
  PCI_FReset* pfnReset;                 /* Reset the device, disable IRQs & DMA */
  PCI_FIrqHandler* pfnIrqHandler;       /* !0 if IRQ's required */
  } PCI_SDevInfo;


/* Functions */
extern void CDA_PCI_Probe( PCI_SDevInfo const*);

#if LINUX_VERSION_CODE >= 0x20600
/* PCI device instance */
typedef struct SPciInstance
{
  u16 subId, subVendor;
  struct pci_dev * pDev;
  PCI_SDevInfo info;                    /* Card info */
  int bIrqInstalled;
  u32 shadowConfig[ 64];                /* PCI config space snapshot */

  /* Registers */
  int iMapping;                         /* PCI address mapping# */
  void* pvBase;                         /* Card virtal base address */

  CDA_SInstance* pInst;                 /* Context for CDA_DeviceEvent */
} SPciInstance;
#endif

#ifdef __cplusplus
}
#endif

#endif /* ndef CDAPCI_H */
/* End of file */
