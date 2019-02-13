/****************************************************************************
 *
 * ACTIVE SILICON LIMITED
 *
 * File name   : cdapci.c
 * Function    : PCI specific code for the Linux CDA library driver.
 * Project     : AP04
 * Authors     : Lawrence Rust
 * Systems     : ANSI C.
 * Version     : See cdadrv.h, "#define CDA_DRV_VERSION".
 * Release Date: See cdadrv.h, "#define CDA_DRV_VERSION".
 *
 * Copyright (c) 1997-2004 Active Silicon Ltd.
 ****************************************************************************
 *
 * Comments:
 * --------
 * This file contains PCI specific code for the CDA Linux driver functions.
 *
 ****************************************************************************
 */

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
#else
#include <linux/interrupt.h>
#endif
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include "linuxdrv.h"           /* For CDA_SIoctlDeviceInfo */

#include "cdapci.h"
#include "debug.h"
#include "picc_dio.h"


/*
 * Macros & constants
 */
#ifndef lengthof
#define lengthof(_a) ( sizeof(_a) / sizeof((_a)[0]) )
#endif

extern int CDA_DeviceIdle( CDA_SInstance* pInst);
extern int CDA_GetDeviceNum(CDA_SInstance* pInst);

/*
 * Local data types
 */

#if LINUX_VERSION_CODE < 0x20600
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

/*
 * Prototypes
 */
static CDA_FStart    PCI_Start;
static CDA_FStop     PCI_Stop;
static CDA_FGetInfo  PCI_GetInfo;
static CDA_FOpen     PCI_Open;
static CDA_FClose    PCI_Close;
static CDA_FRead     PCI_Read;
static CDA_FWrite    PCI_Write;
static CDA_FFinal    PCI_Final;
static CDA_FReconfig PCI_Reconfig;
static CDA_FPutEvent PCI_PutEvent;
#if LINUX_VERSION_CODE >= 0x20613   /* 2.6.19 */
static irqreturn_t PCI_IrqHandler( int irq, void * pv);
#elif ( LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0) )
static irqreturn_t PCI_IrqHandler( int irq, void * pv, struct pt_regs * regs);
#else /* < 2.6.0 */
static void PCI_IrqHandler( int irq, void * pv, struct pt_regs * regs);
#endif

/*
 * Module data
 */

/* The dispatch table */
static CDA_SVtable const PCI_vtable =
{
  &PCI_Start,
  &PCI_Stop,
  &PCI_GetInfo,
  &PCI_Open,
  &PCI_Close,
  &PCI_Read,
  &PCI_Write,
  &PCI_Final,
  &PCI_Reconfig,
  &PCI_PutEvent
};


/*
 * Enumerate, probe, initialise and register all supported cards
 */
void CDA_PCI_Probe( PCI_SDevInfo const* pInfo)
{
  struct pci_dev * pDev;

  TRACE( 9, ( "%s(%p)\n", __FUNCTION__, pInfo));

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0) )
  if ( !pci_present())
    {
    TRACE( 1, ("*** %s: No PCI\n", __FUNCTION__));
    return;
    }
#endif

  /* Enumerate & register all supported devices */
  pDev = NULL;
  do
    {
    pDev = pci_get_device( pInfo->vendorID, pInfo->deviceID, pDev);
    if ( NULL != pDev)
      {
      /* Card found */
      SPciInstance* pPci;
      u16 subId, subVendor;

      /* Get subsystem ID & vendor */
      if ( pci_read_config_word( pDev, PCI_SUBSYSTEM_VENDOR_ID, &subVendor) < 0)
        subVendor = 0xffff;
      if ( pci_read_config_word( pDev, PCI_SUBSYSTEM_ID, &subId) < 0)
        subId = 0xffff;

      if ( pInfo->subsystemVendorID)
        {
        /* Check subsystem ID's */
        if ( pInfo->subsystemVendorID != subVendor
          || pInfo->subsystemID != subId
        ) {
          /* No match */
          continue;
          }
        }

      TRACE( 2, ("%s: vendor %04x, device %04x, subSysId %04x, subSysVendor %04x\n"
        "   bus %u, device %u, function %u, bus %s, irq %u\n"
        "   base0 %08llx, base1 %08llx, base2 %08llx\n",
        __FUNCTION__, pDev->vendor, pDev->device, subId, subVendor,
        pDev->bus->number, PCI_SLOT( pDev->devfn), PCI_FUNC( pDev->devfn),
        /* pDev->master*/ (1==1) ? "master" : "slave", pDev->irq,
        pci_resource_start(pDev, 0), pci_resource_start(pDev, 1), pci_resource_start(pDev, 2)
      ));

      /* Allocate device instance data */
      pPci = (SPciInstance*) kmalloc( sizeof( SPciInstance), GFP_KERNEL);
      if ( NULL == pPci)
        {
        TRACE( 0, ("!!! %s: Out of memory\n", __FUNCTION__));
        break;
        }

      /* Init instance data */
      memset( pPci, 0, sizeof( *pPci));
      pPci->pDev = pDev;
      pPci->subId = subId;
      pPci->subVendor = subVendor;
      pPci->info = *pInfo;

      /* Register the device */
      if ( 0 != CDA_RegisterDevice( &PCI_vtable, (CDA_SDevice*)pPci))
        {
        kfree(pPci);
        continue;
        }
      }
    }
  while ( NULL != pDev);
}


/*
 * Destroy device instance
 */
static int PCI_Final( CDA_SDevice* pDevice )
{
  SPciInstance* const pPci = (SPciInstance*)pDevice;
  
  TRACE( 9, ( "%s(%p)\n", __FUNCTION__, pDevice));

  /* Ensure device disabled */
  /* (void) PCI_Stop( pDevice); */
  /* this was leading to kernel paging request errors and segfault on unload,
   * because PCI_Stop was already being called in the master cda_unload function.
   * By calling it a second time, we were trying to free memory maps that no
   * longer existed. Hence page fault and seg fault.
   */

  /* Free instance data */
  kfree(pPci);

  return 0;
}


/*
 * Allocate device resources
 */
static int PCI_Start( CDA_SDevice* pDevice,
                      CDA_SInstance* pInst,
                      const char* pszDeviceName )
{
  SPciInstance* const pPci = (SPciInstance*)pDevice;
  int i, iRet = -EFAULT;
  
  TRACE( 9, ( "%s(%p,%p,%s)\n", __FUNCTION__, pDevice, pInst, pszDeviceName));
  ASSERT( NULL != pDevice);
  ASSERT( NULL != pszDeviceName);

  ASSERT( NULL != pPci->pDev);
  ASSERT( NULL == pPci->pvBase);

  /* Save the caller's context for CDA_DeviceEvent */
  pPci->pInst = pInst;

  /* Scan base addresses looking for a memory mapping */
  for ( i = 0; i < 6; ++i)
    {
    unsigned long bar_start;
    unsigned long bar_end;
    unsigned long bar_flags;
    unsigned int cmd;

    bar_flags = pci_resource_flags(pPci->pDev, i);
    bar_start = pci_resource_start(pPci->pDev, i);
    bar_end = pci_resource_end(pPci->pDev, i);

    
    if ( ( bar_flags & IORESOURCE_MEM ) == IORESOURCE_MEM
    /* If this is an Active Silicon Card */
    ) {

/*
      printk("  cda card hardware flags read out as 0x%x\n", (int)bar_flags );
      printk("  cda card hardware start address is  0x%x\n", (int)bar_start );
      printk("  cda card hardware finish address is 0x%x\n", (int)bar_end );
*/
    
      /* on OpenFirmware machines (PowerMac at least), PCI memory cycle 
       * response on cards with no firmware is not enabled by OpenFirmware.
       * This would mean that the card is not addressable, bus-mastering
       * will be off and so forth.
       *
       * We will therefore explicitly set ON the
       *   response to Memory
       *   enable bus mastering
       * register values on the card.
       *
       * iX86 BIOS will onormally have preset these values on boot for us,
       * but explicitly re-setting them will not hurt.
       *
       * This wastes a few cycles, but this only happens once when the device
       * driver module is loaded.
       *
       */

      /* Step 1 - make the board respond in memory space
       *  Read the control register and rewrite it with an OR of PCI_COMMAND.
       */
      pci_read_config_dword(pPci->pDev, PCI_COMMAND, &cmd);
      cmd = (cmd | PCI_COMMAND_MEMORY );
      pci_write_config_dword(pPci->pDev, PCI_COMMAND, cmd);

      /* Step 2 - make the board Bus Mastering capable
       *  Read the control register and rewrite it with an OR of PCI_COMMAND_MASTER.
       */
      pci_read_config_dword(pPci->pDev, PCI_COMMAND, &cmd);
      cmd = (cmd | PCI_COMMAND_MASTER );
      pci_write_config_dword(pPci->pDev, PCI_COMMAND, cmd);
    
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 10) )
      if( pci_enable_device(pPci->pDev) )
      {
            TRACE(1, ("%s: pci_enable_device failed", __FUNCTION__));
            iRet = -ENODEV; break;
      }
#endif
#ifndef DMA_32BIT_MASK
#define DMA_32BIT_MASK 0xffffffffULL
#endif
      if (pci_set_dma_mask(pPci->pDev, DMA_32BIT_MASK))
      {
            TRACE(1, ("%s: pci_set_dma_mask failed", __FUNCTION__));
            iRet = -EIO; break;
      }
      if (!request_mem_region(bar_start, bar_end - bar_start, "phx"))
      {
            TRACE(1, ("%s: request_mem_region failed", __FUNCTION__));
            iRet = -EBUSY; break;
      }
#if defined _PHX_CDA
      /* Phoenix uses a memory BAR for the main acquisition engine, plus an I/O BAR for the serial
       * port under Windows.  The iX86 BIOS appears to enable the I/O BAR by default.
       * Therefore disable the I/O area by default.
       */
      /* Step 3 - disable response in I/O space (in case iX86 BIOS has preset it!)
       *  Read the control register and rewrite it with an AND of ~PCI_COMMAND_IO.
       */
      pci_read_config_dword(pPci->pDev, PCI_COMMAND, &cmd);
      cmd = (cmd & ~PCI_COMMAND_IO );
      pci_write_config_dword(pPci->pDev, PCI_COMMAND, cmd);
#endif

      /* Note - the range of prefeined register settings we could use are : 
       *  PCI_COMMAND_IO            Enable response in I/O space
       *  PCI_COMMAND_MEMORY        Enable response in Memory space
       *  PCI_COMMAND_MASTER        Enable bus mastering
       *  PCI_COMMAND_SPECIAL       Enable response to special cycles
       *  PCI_COMMAND_INVALIDATE    Use memory write and invalidate
       *  PCI_COMMAND_VGA_PALETTE   Enable palette snooping
       *  PCI_COMMAND_PARITY        Enable parity checking
       *  PCI_COMMAND_WAIT          Enable address/data stepping
       */
    
      /* Get a non-cacheable logical mapping - ignore PCI prefetchable flag */
      pPci->pvBase = ioremap_nocache( bar_start, bar_end - bar_start); 

      /* printk("  cda card remapped start address is  %x\n", (int)pPci->pvBase ); */

      /* The following line can be useful when debugging to read the contents of a known register on the target board */
      /* printk("  cda card offset 0x064 memory holds  %x\n", readl(pPci->pvBase + 0x064) ); */

      TRACE( 3, ("%s: mapped bus address %08lx => virtual address %p\n",
        __FUNCTION__, bar_start, pPci->pvBase
      ));

      pPci->iMapping = i;
      iRet = 0;
      break;
      }
    }

  if ( iRet)
    {
    TRACE( 0, ("!!! %s: No memory mapping found\n", __FUNCTION__));
    return iRet;
    }
  ASSERT( NULL != pPci->pvBase);

  /* Init the device, ensure interrupts are off & DMA disabled */
  if ( (pPci->info.pfnInitialise
      && !(*pPci->info.pfnInitialise)( pPci->pDev, pPci->pvBase))
    || (pPci->info.pfnReset
      && !(*pPci->info.pfnReset)( pPci->pvBase) )
  ) {
    TRACE( 1, ("*** %s() device failed to initialise\n", __FUNCTION__));
    iRet = -EFAULT;
    }
  else if ( pPci->info.pfnIrqHandler
    && pPci->pDev->irq > 0
  ) {
    /* Register the interrupt handler */
    TRACE( 3, ("%s() installing IRQ handler\n", __FUNCTION__));
    iRet = request_irq(
      pPci->pDev->irq,    /* The IRQ number */ 
      &PCI_IrqHandler,    /* our handler */
#if LINUX_VERSION_CODE >= 0x20612   /* 2.6.18 */
      IRQF_SHARED,        /* IRQF_SHARED means we're willing to have other handlers on this IRQ */
#else
      SA_SHIRQ,           /* SA_SHIRQ means we're willing to have other handlers on this IRQ */
#endif
      pszDeviceName,      /* Name for /proc/interrupts */
      pPci                /* Device id */
    );
    if ( 0 == iRet)
      {
      pPci->bIrqInstalled = 1;
      }
    else
      {
      TRACE( 1, ("*** %s: request_irq failed %d\n", __FUNCTION__, iRet));
      }
    }

  if ( 0 == iRet)
    {
    /* Enable bus mastering & init latency timer */
    pci_set_master( pPci->pDev);

    /* Save shadow copy of PCI config space */
    for ( i = 0; i < lengthof( pPci->shadowConfig); ++i)
      pci_read_config_dword( pPci->pDev, i, &pPci->shadowConfig[i] );
    }
  else
    {
    PCI_Stop( pDevice);
    }

  return iRet;
}


/*
 * Deallocate device resources
 */
static int PCI_Stop( CDA_SDevice* pDevice )
{
  SPciInstance* const pPci = (SPciInstance*)pDevice;
  u16 wData;
  unsigned long bar_start;
  unsigned long bar_end;
  
  TRACE( 9, ( "%s(%p)\n", __FUNCTION__, pDevice));
  ASSERT( NULL != pDevice);
  ASSERT( NULL != pPci->pDev);

  /* Reset the device, ensure interrupts are off & DMA disabled */
  if ( pPci->info.pfnReset)
    {
    TRACE( 3, ( "%s() resetting device\n", __FUNCTION__ ));
    (*pPci->info.pfnReset)( pPci->pvBase);
    }

  if ( pPci->info.pfnFinalise)
    {
    TRACE( 3, ( "%s() finalising device\n", __FUNCTION__ ));
    (*pPci->info.pfnFinalise)( pPci->pvBase);
    }

  /* Disable bus mastering */
  if ( 0 == pci_read_config_word( pPci->pDev, PCI_COMMAND, &wData))
    {
    TRACE( 3, ( "%s() disabling bus mastering\n", __FUNCTION__ ));
    wData &= ~PCI_COMMAND_MASTER;
    VERIFY( !pci_write_config_word( pPci->pDev, PCI_COMMAND, wData));
    }
  else
    {
    TRACE( 0, ( "!!! %s: pci_read_config_word failed\n", __FUNCTION__));
    }

  /* Remove IRQ handler */
  if ( pPci->bIrqInstalled)
    {
    TRACE( 3, ( "%s() removing IRQ handler\n", __FUNCTION__ ));
    pPci->bIrqInstalled = 0;
    free_irq( pPci->pDev->irq, pPci);
    }

  /* Free register mapping */
  if ( NULL != pPci->pvBase)
    {
    TRACE( 3, ( "%s() releasing register mapping\n", __FUNCTION__ ));
    iounmap( pPci->pvBase);
    pPci->pvBase = NULL;
    }

  /* Release the allocated i/o memory region */
  bar_start = pci_resource_start(pPci->pDev, pPci->iMapping);
  bar_end   = pci_resource_end  (pPci->pDev, pPci->iMapping);
  release_mem_region(bar_start, bar_end - bar_start);

  return 0;
}


/*
 * Get device info
 */
static int PCI_GetInfo( CDA_SDevice*                 pDevice,
                        struct CDA_SIoctlDeviceInfo* pInfo )
{
  SPciInstance* const pPci = (SPciInstance*)pDevice;
  
  TRACE( 9, ( "%s(%p,%p)\n", __FUNCTION__, pDevice, pInfo));
  ASSERT( NULL != pDevice);
  ASSERT( NULL != pInfo);
  ASSERT( NULL != pPci->pDev);

  pInfo->type = CDA_kDevicePci;

  pInfo->wDeviceID = pPci->pDev->device;
  pInfo->wVendorID = pPci->pDev->vendor;
  pInfo->wPciSubVendorID = pPci->subVendor;
  pInfo->wPciSubSystemID = pPci->subId;

  /*  pInfo->wDeviceNumber = PCI_SLOT( pPci->pDev->devfn); */
  pInfo->wFunctionNumber =  PCI_FUNC( pPci->pDev->devfn);

  ASSERT( pPci->iMapping < 6 );
  pInfo->dwBase = pci_resource_start(pPci->pDev, pPci->iMapping);
  pInfo->dwInterrupt = pPci->pDev->irq;

  return 0;
}


/*
 * Device open
 */
static int PCI_Open( CDA_SDevice*  pDevice,
                     struct inode* pNode,
                     struct file*  pFile,
                     int           iOpens )
{
  (void)pDevice;
  (void)pNode;
  (void)pFile;
  (void)iOpens;

  TRACE( 9, ( "%s(%p,%p,%p,%d)\n", __FUNCTION__, pDevice, pNode, pFile, iOpens));
  ASSERT( NULL != pDevice);

  return 0;
} 


/*
 * Device close
 */
static int PCI_Close( CDA_SDevice*  pDevice,
                      struct inode* pNode,
                      struct file*  pFile,
                      int           iOpens )
{
  SPciInstance* const pPci = (SPciInstance*)pDevice;
  (void)pNode;

  TRACE( 9, ( "%s(%p,%p,%p,%d)\n", __FUNCTION__, pDevice, pNode, pFile, iOpens));
  ASSERT( NULL != pDevice);
  ASSERT( NULL != pPci->pDev);

  if ( iOpens <= 1)
    {
    /* Init the device, ensure interrupts are off & DMA disabled */
    if ( pPci->info.pfnReset)
      {
      TRACE( 3, ( "%s() resetting device\n", __FUNCTION__ ));
      (*pPci->info.pfnReset)( pPci->pvBase);
      }
    }

  return 0;
} 


/*
 * Device read/write
 */
static int PCI_Read( CDA_SDevice* pDevice,
                     struct file* pFile,
                     ui32*        pData,
                     ui32         offset,
                     int          bits )
{
  SPciInstance* const pPci = (SPciInstance*)pDevice;
  (void)pFile;

  TRACE( 9, ( "%s(%p,%p,%p,%u,%d)\n", __FUNCTION__, pDevice, pFile, pData, offset, bits));
  ASSERT( NULL != pDevice);
  ASSERT( NULL != pData);
  ASSERT( NULL != pPci->pDev);
  
  if ( offset > pPci->info.size)
    {
    TRACE( 1, ( "*** %s() offset 0x%x out of range\n", __FUNCTION__, offset));
    return -EINVAL;
    }

  switch ( bits)
    {
  case 32:
    *pData = _DRV_Read_ui32( pPci->pvBase, offset );
    break;

  case 16:
    *pData = _DRV_Read_ui16( pPci->pvBase, offset );
    break;

  case 8:
    *pData = _DRV_Read_ui8( pPci->pvBase, offset );
    break;

  default:
    TRACE( 0, ("!!! %s: invalid data size %u\n", __FUNCTION__, bits));
    return -EINVAL;
    }

  return 0;
}


static int PCI_Write( CDA_SDevice* pDevice,
                      struct file* pFile,
                      ui32         data,
                      ui32         offset,
                      int          bits )
{
  SPciInstance* const pPci = (SPciInstance*)pDevice;
  (void)pFile;

  TRACE( 9, ( "%s(%p,%p,%u,%u,%d)\n", __FUNCTION__, pDevice, pFile, data, offset, bits));

  ASSERT( NULL != pDevice);
  ASSERT( NULL != pPci->pDev);
  
  if ( offset > pPci->info.size)
    {
    TRACE( 1, ( "*** %s() offset 0x%x out of range\n", __FUNCTION__, offset));
    return -EINVAL;
    }

  switch ( bits)
    {
  case 32:
    _DRV_Write_ui32( pPci->pvBase, offset, data );
    break;

  case 16:
    _DRV_Write_ui16( pPci->pvBase, offset, data );
    break;

  case 8:
    _DRV_Write_ui8( pPci->pvBase, offset, data );
    break;

  default:
    TRACE( 0, ("!!! %s: invalid data size %u\n", __FUNCTION__, bits));
    return -EINVAL;
    }

  return 0;
}


/*
 * IRQ handler
 */
#if LINUX_VERSION_CODE >= 0x20613   /* 2.6.19 */
static irqreturn_t PCI_IrqHandler( int const             irq,
                                   void*                 pv )
#elif ( LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0) )
static irqreturn_t PCI_IrqHandler( int const             irq,
                                   void*                 pv,
                                   struct pt_regs* const regs )
#else /* < 2.6.0 */
static void PCI_IrqHandler( int const             irq,
                            void*                 pv,
                            struct pt_regs* const regs )
#endif
{
  SPciInstance* const pPci = (SPciInstance*)pv;
  ui32 ulEvent, ulData;
  int isInterrupt;
  (void)irq;
#if LINUX_VERSION_CODE < 0x20613   /* 2.6.19 */
  (void)regs;
#endif

  #if PICC_DIO_ENABLE
  if(CDA_GetDeviceNum(pPci->pInst) == PICC_SHK_DEVNUM) outb_p(0x01,PICC_DIO_BASE+PICC_DIO_PORTC); //SET DIO board PORTC bit C0
  if(CDA_GetDeviceNum(pPci->pInst) == PICC_LYT_DEVNUM) outb_p(0x01,PICC_DIO_BASE+PICC_DIO_PORTB); //SET DIO board PORTB bit B0 
  #endif
  
  TRACE( 10, ( "%s(%d,%p)\n", __FUNCTION__, irq, pv));
  ASSERT( NULL != pPci);
  ASSERT( irq == pPci->pDev->irq);
  
  /* Check if IRQ is from the device */
  ASSERT( pPci->info.pfnIrqHandler);
  /*This is the top of the top half of the IRQ handler. It clears the interrupt.*/
  /*The function is: PHD_IrqHandler*/
  isInterrupt = (*pPci->info.pfnIrqHandler)( pPci->pvBase, &ulEvent, &ulData);
  /*This is the bottom of the top half.*/ 

  if ( isInterrupt)
    {
    /* Signal an event */
    TRACE( 3, ("%s: PCI interrupt 0x%08X 0x%08X\n", __FUNCTION__, ulEvent, ulData));
    CDA_DeviceEvent( pPci->pInst, ulEvent, ulData);
    }
  else
    {
    u8 pci_status = 0;

    /* Check PCI status reg */
    pci_read_config_byte( pPci->pDev, PCI_STATUS + 1, &pci_status);
    if ( pci_status & 0xf9)
      {
      /* Clear the IRQ */
      pci_write_config_byte( pPci->pDev, PCI_STATUS + 1, pci_status);
      TRACE( 1, ("*** %s: PCI status IRQ 0x%02x\n", __FUNCTION__, pci_status));
      }
    }
  
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0) )
  return (IRQ_RETVAL(isInterrupt));
#endif
}


/*
 * Restore PCI config space
 */
static int PCI_Reconfig( CDA_SDevice* pDevice )
{
  SPciInstance* const pPci = (SPciInstance*)pDevice;
  int i;

  TRACE( 2, ( "%s(%p)\n", __FUNCTION__, pDevice));
  ASSERT( NULL != pDevice);
  ASSERT( NULL != pPci->pDev);

  /* Restore PCI config space */
  for ( i = 0; i < lengthof( pPci->shadowConfig); ++i)
    pci_write_config_dword( pPci->pDev, i, pPci->shadowConfig[i] );

  return 0;
}

  
/*
 * Insert an event
 */
static int PCI_PutEvent( CDA_SDevice*            pDevice, 
                         struct CDA_SIoctlEvent* pEvent )
{
  SPciInstance* const pPci = (SPciInstance*)pDevice;

  TRACE( 9, ( "%s(%p,%p)\n", __FUNCTION__, pDevice, pEvent));
  ASSERT( NULL != pDevice);
  ASSERT( NULL != pEvent);
  ASSERT( NULL != pPci->pDev);

  /* insert an event */
  TRACE( 3, ("%s: PCI put event 0x%08X 0x%08X\n", __FUNCTION__, pEvent->event, pEvent->data));
  CDA_DeviceEvent( pPci->pInst, pEvent->event, pEvent->data);

  return 0;
}

  
/*
 * Overrides for Emacs to follow the author's tabbing style.
 * Emacs will notice this stuff at the end of the file and automatically
 * adjust the settings for this buffer only.  This must remain at the end
 * of the file.
 * ---------------------------------------------------------------------------
 *
 * Local variables:
 *  c-indent-level: 2
 *  c-basic-offset: 2
 *  tab-width: 8
 *  c-brace-imaginary-offset: 0
 *  c-brace-offset: -2
 *  c-argdecl-indent: 2
 *  c-label-offset: -2
 *  c-continued-statement-offset: 2
 *  c-continued-brace-offset: 0
 *  indent-tabs-mode: nil
 * End:
 */
