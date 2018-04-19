/*=============================================================================
Title        :
Description  : QuickUSB Linux Driver
Notes        : None
History      :

Copyright  2012 Bitwise Systems.  All rights reserved.
This software contains confidential information and trade secrets of
Bitwise Systems and is protected by United States and international
copyright laws.  Use, disclosure, or reproduction is prohibited without
the prior express written permission of Bitwise Systems, except as agreed
in the QuickUSB Plug-In Module license agreement.

Use, duplication or disclosure by the U.S. Government is subject to
restrictions as provided in DFARS 227.7202-1(a) and 227.7202-3(a)
(1998), and FAR 12.212, as applicable.  Bitwise Systems, 6489 Calle Real,
Suite E, Goleta, CA  93117.

Bitwise Systems
6489 Calle Real, Suite E
Goleta, CA  93117
Voice: (805) 683-6469
Fax  : (805) 683-4833
Web  : www.bitwisesys.com
email: support@bitwisesys.com

=============================================================================*/
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/timer.h>

typedef int             BOOL;
typedef int             BOOLEAN;
typedef unsigned char   BYTE, CHAR, UCHAR;
typedef unsigned char * PUCHAR;
typedef signed long     LONG;
typedef unsigned long   ULONG, DWORD;
typedef unsigned short  USHORT, WCHAR, WORD;
typedef void *          PVOID;


#include "Version.h"
#include "ioctl.h"

//#define DEBUG
#define VENDOR_ID       0x0fbb
#define PRODUCT_ID      0x0001
#define DRIVER_NAME     "qusb"
#define MINOR_BASE      0xAB
#define IMPLEMENT_ASYNC (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25))
#define QUSB_DEFAULT_TIMEOUT 1000

#define QUSB_EP_BULK_IN  0x86
#define QUSB_EP_BULK_OUT 0x02

//#define MAX_STRING_LENGTH 128
#define MAX_DATA_LENGTH 64

//#define MAX_ASYNC_RW 253

#define FALSE                 0
#define TRUE                  1

#define QUSB_INTERNAL_BUFFER_SIZE ((int)(2 * PAGE_SIZE))

#define QUSB_THREAD_CMD_RUN          0x0000E001
#define QUSB_THREAD_CMD_SHUTDOWN     0x0000E002

// Print macros
#ifdef DEBUG
#define QUSB_PRINTK(x) printk(("QuickUsb: ")); printk x
#else
#define QUSB_PRINTK(x)
#endif

//#if !defined(compat_ptr)
//#ifndef compat_ptr
//asdf
//#define compat_ptr(x) ((void __user *)(x))
//#endif
extern inline void __user *compat_ptr(u32 uptr);



// QuickUSB Device struct.  Each connected module gets an instance
// of this struct whose pointer is stored in the private data of
// the file descriptor (struct file *filp)
struct qusb_dev {
    // The USB device
    struct usb_device *udev;

    // The USB interface
    struct usb_interface *interface;

    // The bulk IN and bulk OUT endpoint addresses and maximum packet sizes
    __u8 bulk_in_address;
    __u8 bulk_out_address;
    size_t bulk_in_max_packet;
    size_t bulk_out_max_packet;

    // The internal kernel data buffers used for non-DMA bulk reads and bulk writes
    unsigned char *buffer_ptr;
    unsigned char *buffer_area;

    // To prevent lockups, all URBs must timeout
    int timeout;

    // Reference counting for the number of times a device has been opened
    struct kref ref;
    int open_count;

#if IMPLEMENT_ASYNC
    // We must keep track of asynchronously issued URBs that have not completed
    // so that they may be cancelled if the QuickUSB device is removed
    struct usb_anchor pendingAsyncReadUrbs;
    struct usb_anchor pendingAsyncWriteUrbs;

    // We must synchronize URB submission to ensure they are both sequential
    // and are not sent on removed devices
    struct mutex io_mutex;

    unsigned int pendingWrites;
    unsigned int pendingReads;
#endif
};

struct async_request_context {
    // The QuickUSB device
    struct qusb_dev* dev;
    
    // The IOCB for the data request
    struct kiocb *iocb;

    // The scatter-gatter IO request
    struct usb_sg_request io;

    // The scatter-gather list of memory to use DMA with
    struct scatterlist *sgl;

    // A list of the user-space pages to be used with DMA
    struct page **pages;

    // The number of user-space pages (and locked pages) for the requests
    int numPages, lockedPages;

    // Request type
    BOOL read;

    // Read requests first require a control packet to be sent
    struct urb *ctrlRequest;
    struct usb_ctrlrequest *setup_packet;
    struct completion ctrlDone;

    // Each request gets a timeout timer
    struct timer_list timer;
};



// Internal driver file operations
extern int qusb_probe(struct usb_interface *interface, const struct usb_device_id *device_id);
extern int qusb_open(struct inode *inode, struct file *filp);
extern int qusb_release(struct inode *inode, struct file *filp);
extern void qusb_disconnect(struct usb_interface *interface);
extern void qusb_delete(struct kref *kref);

// Read/Write file operations
extern ssize_t qusb_sync(struct file *filp, char __user *buf, size_t count, BOOL read, BOOL sendReadLength);

#if IMPLEMENT_ASYNC
extern ssize_t qusb_aio_read(struct kiocb *iocb, const struct iovec *vec, unsigned long count, loff_t pos);
extern ssize_t qusb_aio_write(struct kiocb *iocb, const struct iovec *vec, unsigned long count, loff_t pos);
extern int qusb_aio_cancel(struct kiocb *iocb, struct io_event *event);
extern ssize_t qusb_async(struct kiocb *iocb, char __user *buf, size_t count, BOOL read);
#else
extern ssize_t qusb_read(struct file *filp, char __user *buf, size_t count, loff_t *pos);
extern ssize_t qusb_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos);
#endif

// IOCTL operations (See http://lwn.net/Articles/119652/)
extern int qusb_ioctl(struct file *filp, unsigned int cmd, void __user *arg, BYTE compat);
#if defined(HAVE_UNLOCKED_IOCTL)
    // Starting with the 2.6.11 kernels, we may specify unlocked_ioctl which will be used in place of ioctl and not use the BKL
    extern long qusb_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
    // For 2.6.10 kernels and earlier, we must specify ioctl, which will use the BKL
    extern int qusb_locked_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif

#if defined(HAVE_COMPAT_IOCTL)
    // For kernels 2.6.11 and later we may specify a compat_ioctl which is called when a 32-bit process makes an ioctl call in a 64-bit kernel
    extern long qusb_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif

// Helper functions
extern long qusb_GetStringDescriptor(struct file *filp, UCHAR index, PVOID buffer, ULONG length);
extern long qusb_GetDeviceDescriptor(struct file *filp, struct usb_device_descriptor *device_descriptor);
extern long qusb_VendorRequest(struct file *filp, QUSB_VENDOR_OR_CLASS_REQUEST_CONTROL *vcrc, BYTE compat);

#if IMPLEMENT_ASYNC
// Scatter/gather DMA functions
extern void qusb_sg_clean(struct usb_sg_request *io);

extern void qusb_sg_complete(struct urb *urb);
extern int qusb_sg_init(struct usb_sg_request *io, struct usb_device *dev,
                unsigned pipe, unsigned period, struct scatterlist *sg,
                int nents, size_t length, gfp_t mem_flags, void *context);
extern void qusb_sg_submit_async(struct usb_sg_request *io, struct usb_anchor *anchor);
extern void qusb_sg_async_wait(struct usb_sg_request *io);
extern void qusb_sg_cancel(struct usb_sg_request *io);
extern void qusb_sg_timedout(struct usb_sg_request *io);
#endif
