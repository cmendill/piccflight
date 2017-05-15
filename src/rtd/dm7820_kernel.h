/**
    @file

    @brief
        Kernel compatibility issues between 2.4 and 2.6 kernels

    @verbatim
    --------------------------------------------------------------------------
    This file and its contents are copyright (C) RTD Embedded Technologies,
    Inc.  All Rights Reserved.

    This software is licensed as described in the RTD End-User Software License
    Agreement.  For a copy of this agreement, refer to the file LICENSE.TXT
    (which should be included with this software) or contact RTD Embedded
    Technologies, Inc.
    --------------------------------------------------------------------------
    @endverbatim

    $Id: dm7820_kernel.h 56181 2011-10-20 20:53:06Z rgroner $
*/

#ifndef __dm7820_kernel_h__
#define __dm7820_kernel_h__

#include <asm/ptrace.h>
#include <linux/version.h>

/**
 * @defgroup DM7820_Kernel_Header DM7820 kernel compatibility header file
 * @{
 */

/*=============================================================================
Sanity checking on RTD kernel version macro set in make file
 =============================================================================*/

/*
 * Make sure RTD_2_4_KERNEL and RTD_2_6_KERNEL are not defined at the same time
 */

#if (defined(RTD_2_4_KERNEL) && defined(RTD_2_6_KERNEL))

#error "RTD_2_4_KERNEL and RTD_2_6_KERNEL cannot be defined at the same time!"

#endif /* defined(RTD_2_4_KERNEL) && defined(RTD_2_6_KERNEL) */

/*
 * Make sure one of RTD_2_4_KERNEL and RTD_2_6_KERNEL is defined
 */

#if (! defined(RTD_2_4_KERNEL) && ! defined(RTD_2_6_KERNEL))

#error "One of RTD_2_4_KERNEL and RTD_2_6_KERNEL must be defined!"

#endif /* ! defined(RTD_2_4_KERNEL) && ! defined(RTD_2_6_KERNEL) */

/*=============================================================================
Interrupt header files
 =============================================================================*/

/*
 * Interrupt-specific information (including declarations for free_irq() and
 * request_irq() functions) moved to new header file in 2.6 kernel
 */

#if defined(RTD_2_6_KERNEL)

#include <linux/interrupt.h>

#endif /* RTD_2_6_KERNEL */

/*
 * SA_SHIRQ has become deprecated in versions after 2.6.18
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
#define IRQF_SHARED     SA_SHIRQ
#endif

/*=============================================================================
Character device header files
 =============================================================================*/

/*
 * 2.6 kernel character device definitions
 */

#if defined(RTD_2_6_KERNEL)

#include <linux/cdev.h>

#endif /* RTD_2_6_KERNEL */

/*=============================================================================
DMA mapping header files
 =============================================================================*/

/*
 * 2.6 kernel DMA mapping definitions
 */

#if defined(RTD_2_6_KERNEL)

#include <linux/dma-mapping.h>

#endif /* RTD_2_6_KERNEL */

/*=============================================================================
Module reference counting
 =============================================================================*/

/**
 * @defgroup DM7820_Kernel_Module_Reference_Counting_Macros DM7820 kernel compatibility module reference counting macros
 * @{
 */

/**
 * @def DECREMENT_MODULE_USAGE
 * @brief
 *      Entity which decrements a module's usage count.
 */

/**
 * @def INCREMENT_MODULE_USAGE
 * @brief
 *      Entity which increments a module's usage count.
 */

/*
 * Module reference counting is handled automatically in 2.6 kernel but must be
 * done manually in 2.4 kernel
 */

#if defined(RTD_2_4_KERNEL)

#define DECREMENT_MODULE_USAGE  MOD_DEC_USE_COUNT
#define INCREMENT_MODULE_USAGE  MOD_INC_USE_COUNT

#else

#define DECREMENT_MODULE_USAGE
#define INCREMENT_MODULE_USAGE

#endif /* RTD_2_4_KERNEL */

/**
 * @} DM7820_Kernel_Module_Reference_Counting_Macros
 */

/*=============================================================================
Module symbols
 =============================================================================*/

/**
 * @defgroup DM7820_Kernel_Module_Symbol_Macros DM7820 kernel compatibility module symbol macros
 * @{
 */

/**
 * @def DONT_EXPORT_SYMBOLS
 * @brief
 *      Assembler directive to prevent module symbols from being exported to
 *      other kernel code.
 */

/*
 * 2.6 kernel by default exports no module symbols whereas 2.4 kernel by default
 * exports module symbols
 */

#if defined(RTD_2_4_KERNEL)

#define DONT_EXPORT_SYMBOLS EXPORT_NO_SYMBOLS

#else

#define DONT_EXPORT_SYMBOLS

#endif /* RTD_2_4_KERNEL */

/**
 * @} DM7820_Kernel_Module_Symbol_Macros
 */

/*=============================================================================
Device major and minor numbers
 =============================================================================*/

/**
 * @defgroup DM7820_Kernel_Major_Minor_Number_Macros DM7820 kernel compatibility major/minor number macros
 * @{
 */

/**
 * @def GET_INODE_MAJOR_NUM(inode)
 * @brief
 *      Entity which obtains the device major number from inode \b inode.
 */

/**
 * @def GET_INODE_MINOR_NUM(inode)
 * @brief
 *      Entity which obtains the device minor number from inode \b inode.
 */

/*
 * 2.4 and 2.6 kernels differ in the way a device major and minor numbers are
 * obtained from an inode
 */

#if defined(RTD_2_4_KERNEL)

#define GET_INODE_MAJOR_NUM(inode)      MAJOR((inode)->i_rdev)
#define GET_INODE_MINOR_NUM(inode)      MINOR((inode)->i_rdev)

#else

#define GET_INODE_MAJOR_NUM(inode)      imajor((inode))
#define GET_INODE_MINOR_NUM(inode)      iminor((inode))

#endif /* RTD_2_4_KERNEL */

/**
 * @} DM7820_Kernel_Module_Major_Minor_Number_Macros
 */

/*=============================================================================
Interrupt handlers
 =============================================================================*/

/**
 * @defgroup DM7820_Kernel_Interrupt_Handler_Macros DM7820 kernel compatibility interrupt handler macros
 * @{
 */

/**
 * @def INTERRUPT_HANDLED
 * @brief
 *      Interrupt handler return value for successful interrupt processing.
 */

/**
 * @def INTERRUPT_HANDLER_TYPE
 * @brief
 *      Type returned by interrupt handler.
 */

/**
 * @def INTERRUPT_NOT_HANDLED
 * @brief
 *      Interrupt handler return value for unsuccessful interrupt processing
 *      either because an error occurred or the interrupt came from another
 *      device sharing the interrupt.
 */

/**
 * @typedef dm7820_handler_t
 * @brief
 *      Type definition for interrupt handling function.
 */

/*
 * 2.6 kernel interrupt handlers return a value indicating whether or not the
 * interrupt was able to be processed.  2.4 kernel interrupt handlers do not
 * return a value.
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)

#define INTERRUPT_HANDLED
#define INTERRUPT_HANDLER_TYPE  static void
#define INTERRUPT_NOT_HANDLED
typedef void (*dm7820_handler_t) (int, void *, struct pt_regs *);

#else

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)

#define INTERRUPT_HANDLED   IRQ_HANDLED
#define INTERRUPT_HANDLER_TYPE  static irqreturn_t
#define INTERRUPT_NOT_HANDLED   IRQ_NONE
typedef irqreturn_t(*dm7820_handler_t) (int, void *, struct pt_regs *);

#else

#define INTERRUPT_HANDLER_TYPE static irqreturn_t
#define INTERRUPT_HANDLED   IRQ_HANDLED
#define INTERRUPT_NOT_HANDLED IRQ_NONE
typedef irqreturn_t(*dm7820_handler_t) (int, void *);

#endif
#endif

/**
 * @} DM7820_Kernel_Interrupt_Handler_Macros
 */

/**
 * @defgroup DM7820_Kernel_File_Ops_Struct_Macros File Ops Kernel compatibility
 * @{
 */

 /**
 * @def READV_AIO_READ
 * @brief
 *      In Kernel 2.6.19 readv in the file operations structure
 *      was replaced with aio_read.
 */

/**
 * @def WRITEV_AIO_WRITE
 * @brief
 *      In Kernel 2.6.19 writev in the file operations structure
 *      was replaced with aio_write.
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)

#define     READV_AIO_READ  .readv
#define     WRITEV_AIO_WRITE    .writev

#else

#define     READV_AIO_READ  .aio_read
#define     WRITEV_AIO_WRITE    .aio_write

#endif

/**
 * @} DM7820_Kernel_File_Ops_Struct_Macros
 */

/*=============================================================================
Kernel PCI device list
 =============================================================================*/

/**
 * @defgroup DM7820_Kernel_PCI_Device_List_Macros DM7820 kernel compatibility PCI device list macros
 * @{
 */

/**
 * @def PCI_NEXT_DEVICE
 * @brief
 *      Entity which returns pointer to next device in kernel's PCI device list.
 */

/**
 * @def PCI_RELEASE_DEVICE
 * @brief
 *      Entity which decrements usage counter incremented by PCI_NEXT_DEVICE.
 *
 * @param
 *      device
 *
 *      Address of PCI device structure to decrement usage count of.
 *
 * @note
 *      This applies only to the 2.6 kernel.
 */

/*
 * 2.4 and 2.6 kernels need different functions to walk the kernel's list of PCI
 * devices.  pci_find_device() exists in the 2.6 kernel but it's use is
 * discouraged because list entries can disappear while walking the list.
 * pci_get_device() adjusts a list entry's reference count to prevent it from
 * going away while accessed.
 */

#if defined(RTD_2_4_KERNEL)

#define PCI_NEXT_DEVICE         pci_find_device
#define PCI_RELEASE_DEVICE(device)

#else

#define PCI_NEXT_DEVICE         pci_get_device
#define PCI_RELEASE_DEVICE(device)  pci_dev_put((device))

#endif /* RTD_2_4_KERNEL */

/**
 * @} DM7820_Kernel_PCI_Device_List_Macros
 */

/*=============================================================================
Access to device I/O memory
 =============================================================================*/

/**
 * @defgroup DM7820_Kernel_Device_IO_Memory_Access_Macros DM7820 kernel compatibility device I/O memory access macros
 * @{
 */

/**
 * @def IO_MEMORY_READ8
 * @brief
 *      Entity which reads an 8-bit value from device I/O memory
 */

/**
 * @def IO_MEMORY_READ16
 * @brief
 *      Entity which reads a 16-bit value from device I/O memory
 */

/**
 * @def IO_MEMORY_READ32
 * @brief
 *      Entity which reads a 32-bit value from device I/O memory
 */

/**
 * @def IO_MEMORY_WRITE8
 * @brief
 *      Entity which writes an 8-bit value to device I/O memory
 */

/**
 * @def IO_MEMORY_WRITE16
 * @brief
 *      Entity which writes a 16-bit value to device I/O memory
 */

/**
 * @def IO_MEMORY_WRITE32
 * @brief
 *      Entity which writes a 32-bit value to device I/O memory
 */

/*
 * Theoretically, the address returned from ioremap() or ioremap_nocache()
 * should not be used to directly access memory.  This may work on some
 * architectures but may fail on others.  Therefore, special techniques must be
 * used to access device I/O memory to make code portable.  2.4 and some 2.6
 * kernels (2.6.8 and older) share the same access methods whereas other 2.6
 * kernels (2.6.9 and newer) use a different scheme.
 */

#if defined (RTD_2_4_KERNEL)

#define IO_MEMORY_READ8     readb
#define IO_MEMORY_READ16    readw
#define IO_MEMORY_READ32    readl
#define IO_MEMORY_WRITE8    writeb
#define IO_MEMORY_WRITE16   writew
#define IO_MEMORY_WRITE32   writel

#else

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 9)

#define IO_MEMORY_READ8     readb
#define IO_MEMORY_READ16    readw
#define IO_MEMORY_READ32    readl
#define IO_MEMORY_WRITE8    writeb
#define IO_MEMORY_WRITE16   writew
#define IO_MEMORY_WRITE32   writel

#else

#define IO_MEMORY_READ8     ioread8
#define IO_MEMORY_READ16    ioread16
#define IO_MEMORY_READ32    ioread32
#define IO_MEMORY_WRITE8    iowrite8
#define IO_MEMORY_WRITE16   iowrite16
#define IO_MEMORY_WRITE32   iowrite32

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 9) */

#endif /* RTD_2_4_KERNEL */

/**
 * @} DM7820_Kernel_Device_IO_Memory_Access_Macros
 */

/*=============================================================================
DMA mappings
 =============================================================================*/

/**
 * @defgroup DM7820_Kernel_DMA_Mapping_Macros DM7820 kernel compatibility DMA mapping macros
 * @{
 */

/**
 * @def CREATE_PERMANENT_DMA_MAPPING
 *
 * @brief
 *      Set up a coherent/consistent DMA mapping.  This entails allocating a
 *      buffer suitable for DMA and creating a device-accessible address for the
 *      buffer.
 *
 * @param
 *      device
 *
 *      Address of PCI device structure (2.4 kernel) or device structure (2.6
 *      kernel).
 *
 * @param
 *      size
 *
 *      Size of DMA buffer in bytes.
 *
 * @param
 *      bus_addr
 *
 *      Address where buffer's bus address should be stored.
 *
 * @param
 *      alloc_flags
 *
 *      Memory allocation flags to be used when allocating DMA buffer.  This
 *      applies only to the 2.6 kernel.
 *
 * @retval
 *      Buffer's kernel virtual address.
 *
 * @note
 *      For the 2.4 kernel, memory for the DMA buffer is always allocated using
 *      the GFP_ATOMIC flag.  This flag ensures that the allocation will never
 *      sleep waiting for memory to become available.  Therefore, this macro is
 *      safe to use even at interrupt level on the 2.4 kernel.
 */

/**
 * @def FREE_PERMANENT_DMA_MAPPING
 *
 * @brief
 *      Destroy a coherent/consistent DMA mapping.
 *
 * @param
 *      device
 *
 *      Address of PCI device structure (2.4 kernel) or device structure (2.6
 *      kernel).
 *
 * @param
 *      size
 *
 *      Size of DMA buffer in bytes.
 *
 * @param
 *      virt_addr
 *
 *      DMA buffer's kernel virtual address.
 *
 * @param
 *      bus_addr
 *
 *      DMA buffer's bus address.
 */

/**
 * @def DM7820_GFP_NOWARN
 *
 * @brief
 *      Suppress page allocation failure warnings.
 *
 * @note
 *      This provides a wrapper for the __GFP_NOWARN get free page allocation
 *      flag, which does not exist in the 2.4 kernel.
 */

/*
 * 2.4 and 2.6 kernels differ in the way consistent/coherent DMA mappings are
 * handled
 */

#if defined(RTD_2_4_KERNEL)

#define CREATE_PERMANENT_DMA_MAPPING(device, size, bus_addr, alloc_flags) \
    pci_alloc_consistent((device), (size), (bus_addr))

#define FREE_PERMANENT_DMA_MAPPING(device, size, virt_addr, bus_addr) \
    pci_free_consistent((device), (size), (virt_addr), (bus_addr))

#define DM7820_GFP_NOWARN

#else

#define CREATE_PERMANENT_DMA_MAPPING(device, size, bus_addr, alloc_flags) \
    dma_alloc_coherent((device), (size), (bus_addr), (alloc_flags))

#define FREE_PERMANENT_DMA_MAPPING(device, size, virt_addr, bus_addr) \
    dma_free_coherent((device), (size), (virt_addr), (bus_addr))

#define DM7820_GFP_NOWARN   __GFP_NOWARN

#endif /* RTD_2_4_KERNEL */

/**
 * @} DM7820_Kernel_DMA_Mapping_Macros
 */

/**
 * @} DM7820_Kernel_Header
 */

#endif /* __dm7820_kernel_h__ */
