/****************************************************************************
 *
 * ACTIVE SILICON LIMITED
 *
 * File name   : cdadrv.c
 * Function    : Linux driver for CDA library.
 * Project     : AP04
 * Authors     : Lawrence Rust
 * Systems     : ANSI C.
 * Version     : See cdadrv.h, "#define CDA_DRV_VERSION".
 * Release Date: See cdadrv.h, "#define CDA_DRV_VERSION".
 *
 * Copyright (c) 1997-2007 Active Silicon Ltd.
 ****************************************************************************
 *
 * Comments:
 * --------
 * This file contains the CDA Linux driver functions.
 * 
 ****************************************************************************
 */

/* Imports */

/* Standard in kernel modules */

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
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0) )
#include <linux/iobuf.h>
#endif
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0) )
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/pagemap.h>
#include <linux/vmalloc.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#endif

/* Exports */
#define CDA_INIT
#include "debug.h"
#include "linuxdrv.h"
#include "cdadrv.h"
#include "cdapci.h"
#include <cda_buf.h>
#include <cda_lib.h>
#include "picc_dio.h"

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0) )
MODULE_LICENSE("GPL");
#endif

static CDA_FProbe* const CDA_pfnProbeArray[] =
{
#if defined _LFG_LINUX
   &LFG_Probe,
/* &LFGAUDIO_Probe,*/
#endif
#if defined _PHX_LINUX
   &PHD_Probe_AsPhxGeneric,
   &PHD_Probe_AsPhxD10PE1,
   &PHD_Probe_AsPhxD10PE4,
   &PHD_Probe_AsPhxD20PE1,
   &PHD_Probe_AsPhxD20PE4,
   &PHD_Probe_AsPhxD24P32,
   &PHD_Probe_AsPhxD24PE1,
   &PHD_Probe_AsPhxD32P32,
   &PHD_Probe_AsPhxD36P32,
   &PHD_Probe_AsPhxD36P64,
   &PHD_Probe_AsPhxD36P64U,
   &PHD_Probe_AsPhxD36PE1,
   &PHD_Probe_AsPhxD48P32,
   &PHD_Probe_AsPhxD48P64,
   &PHD_Probe_AsPhxD48P64U,
   &PHD_Probe_AsPhxD48PE1,
   &PHD_Probe_AsPhxD48PE4,
   &PHD_Probe_AsPhxD64PE4
#endif  
  /* Add more card types here */
};

/*
 * Macros & constants
 */
#ifndef CDA_MAX_UNITS
#define CDA_MAX_UNITS 16        /* Max no. devices */
#endif

#ifndef CDA_EVENTQ
#define CDA_EVENTQ 8            /* Size of event queue */
#endif

#define copy_to_user_ret(to, from, size, errcode) \
   if (copy_to_user((to), (from), (size))) return (errcode)
#define copy_from_user_ret(to, from, size, errcode) \
   if (copy_from_user((to), (from), (size))) return (errcode)
#define put_user_ret( l, p, e) \
   if (put_user((l), (p))) return(e)
#define get_user_ret( l, p, e) \
   if (get_user((l), (p))) return(e)

/*
 * Local data types
 */
#ifndef NDEBUG
enum EMagic { kMagic = 0xDEADC0DEL };
short (*g_pfnMessageBox)(const char *) = NULL;
int g_iTraceLevel = 2;
#endif

typedef struct SLock SLock;     /* Locked buffer info */

/* Device state */
typedef enum EState
  {
  kStateRemoved,
  kStateStopped,
  kStateStarting,
  kStateStartFailed,
  kStateStarted,
  kStateRemoving,
  kStateStopping
  } EState;

/* Device instance */
struct CDA_SInstance
  {
#ifndef NDEBUG
  enum EMagic eMagic;           /* For assert checks */
#endif
  char szDeviceName[ 32];
  int devicenum;                /* Device instance number */
  EState eState;                /* Device operational state */
  atomic_t openCount;
  struct file * pFile;          /* Device owner, NULL if unclaimed */

  /* Registration data */
  const CDA_SVtable* pVtable;   /* -> device function table */
  CDA_SDevice* pDevice;         /* Device context */

  /* Event queue */
  spinlock_t lockEventQ;        /* IRQ lock to protect the eventArray, head & tail */
  struct
    {
    ui32 event;                 /* Device specific event ID */
    ui32 data;                  /* Event specific data */
    } eventArray[ CDA_EVENTQ];
  volatile unsigned uEventHead; /* Maybe written at IRQ time */
  unsigned uEventTail;          /* Written only at task time */
  wait_queue_head_t WaitQ;     /* Processes/threads waiting for an event */
#if LINUX_VERSION_CODE >= 0x20600
  struct workqueue_struct * task; 
  struct work_struct work;        
#else
  struct tq_struct task;        /* Deferred IRQ task that wakes processes on WaitQ */
#endif
#ifdef DEBUG
  volatile unsigned uIrqOverflows; /* No. device events lost */
#endif

  SLock* pLockHead;             /*  Linked list of locked buffers */
  };


/* Locked buffer info */
struct SLock
  {
  SLock *pPrev, *pNext;         /* Doubly linked list */
  void* pvBuffer;               /* -> user space buffer */
  size_t len;
#if LINUX_VERSION_CODE >= 0x20600
  struct scatterlist *sglist;
  struct page **pages;
  unsigned int offset;
  unsigned int nr_pages;
  unsigned int direction;
  unsigned int sglen;
  struct CDA_SInstance *pInst;
#else
  struct kiobuf *pkiobuf;
#endif
  };

  
/*
 * Prototypes
 */
#ifdef MODULE
extern int init_module( void);
extern void cleanup_module( void);
static int CDA_Initialise( void);
static int CDA_Terminate( void);
#else
extern int CDA_Initialise( void);
extern int CDA_Terminate( void);
#endif
static loff_t CDA_seek( struct file *, loff_t, int);
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0) )
static ssize_t CDA_read( struct file *, char __user *, size_t, loff_t *);
static ssize_t CDA_write( struct file *, const char __user *, size_t, loff_t *);
#else
static ssize_t CDA_read( struct file *, char *, size_t, loff_t *);
static ssize_t CDA_write( struct file *, const char *, size_t, loff_t *);
#endif
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36) )
static long CDA_ioctl( struct file *, unsigned int, unsigned long);
#else
static int CDA_ioctl( struct inode *, struct file *, unsigned int, unsigned long);
#endif
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 11) )
#if defined _LP64
static long CDA_compat( struct file *, unsigned int, unsigned long);
#endif
#endif
static int CDA_open( struct inode *, struct file *);
static int CDA_release( struct inode *, struct file *);

int CDA_DeviceIdle( CDA_SInstance*);
static int CDA_RemoveDevice( CDA_SInstance*);
static int CDA_IoctlRegList( CDA_SInstance*, struct file *, CDA_SIoctlRegList*);
#if LINUX_VERSION_CODE >= 0x20614   /* 2.6.20 */
static void CDA_IrqTask( struct work_struct *ws);
#else
static void CDA_IrqTask( void * pv);
#endif
static int CDA_WaitEvent( CDA_SInstance*, CDA_SIoctlEvent*);
static int CDA_LockBuffer( CDA_SInstance*, void*, uiSG*, int);
static int CDA_FlushBuffer(SLock *pLock);
static int CDA_UnlockBuffer(SLock *pLock);
static int CDA_IoctlConfigRestore( CDA_SInstance*);
static int CDA_IoctlPutEvent     ( CDA_SInstance*, CDA_SIoctlEvent*);


/*
 * Module data
 */
static unsigned s_uDevices = 0;                 /* No. registered devices */
static CDA_SInstance s_instArray[ CDA_MAX_UNITS];   /* Device instance data */

static int major = CDA_MAJOR_NUM;
static const char * name = CDA_BASE_NAME;

#if defined MODULE && LINUX_VERSION_CODE > 0x20115
 MODULE_AUTHOR( "Active Silicon Limited <cda.support@activesilicon.co.uk>");
 MODULE_DESCRIPTION( "CDA driver");

 /* Module parameters */
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0) )
 MODULE_PARM( major, "i");
 MODULE_PARM( name,  "s");
#else
 module_param( major, int,  0444 );
 module_param( name, charp, 0444 );
#endif
#ifdef DEBUG_TRACE_LEVEL
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0) )
 MODULE_PARM( DEBUG_TRACE_LEVEL, "0-10i"); /* The makefile sets this name to 'debug' */
#else
 module_param( DEBUG_TRACE_LEVEL, int, 0444 ); 
#endif
#endif
#endif


#ifdef MODULE
/*
 * Module Initialisation - called when the module is loaded into memory
 * Single threaded
 */ 
int init_module( void)
  {
#ifdef DEBUG
  #define BUILD " (DEBUG)"
#else
  #define BUILD ""
#endif
  const char* tempName = "phddrv";
  printk( "%s: LOADED Active Silicon %s driver version %lu.%02lu" BUILD " built at " __TIME__ " on " __DATE__ " --PICC irq hack--\n", tempName, name, CDA_DRV_VERSION / 0x10000, CDA_DRV_VERSION % 0x10000);

  TRACE( 2, ( "CDA: trace level %d\n", DEBUG_TRACE_LEVEL));
  
  //PICC_DIO Code
  #if PICC_DIO_ENABLE
        //get the DIO region of memeory 
	if(request_region(PICC_DIO_BASE, PICC_DIO_LENGTH, tempName) == NULL) {
		printk("%s: request_region failed, base: 0x%03X\n", tempName, PICC_DIO_BASE);
		return 1;
	}
	printk("%s: PICC_DIO ports allocated, base: 0x%03X\n", tempName, PICC_DIO_BASE);

        //Set page to 1
        outb_p(0x01,PICC_DIO_BASE+PICC_DIO_PAGE);
        //Configure ports A,B,C as output
        outb_p(0x80,PICC_DIO_BASE+PICC_DIO_CTRL);
        
  #endif

  /* Probe & register all card types */
  return CDA_Initialise();
  }


/*
 * Module finalisation - called when the module is unloaded
 * Single threaded
 */
void cleanup_module( void)
  {
  const char* tempName = "phddrv";
  TRACE( 3, ("CDA: %s()\n", __FUNCTION__));

  //PICC_DIO Code
  #if PICC_DIO_ENABLE
    //release the DIO region of memeory 
    release_region(PICC_DIO_BASE, PICC_DIO_LENGTH);
    printk("%s: PICC_DIO ports released, base: 0x%03X\n", tempName, PICC_DIO_BASE);
  #endif
    printk( "%s: UNLOADED Active Silicon %s driver version %lu.%02lu" BUILD " built at " __TIME__ " on " __DATE__ " --PICC irq hack--\n", tempName, name, CDA_DRV_VERSION / 0x10000, CDA_DRV_VERSION % 0x10000);
    
  CDA_Terminate();
  }
#endif

#if 0
        struct module *owner;
        loff_t (*llseek) (struct file *, loff_t, int);
        ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
        ssize_t (*aio_read) (struct kiocb *, char __user *, size_t, loff_t);
        ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
        ssize_t (*aio_write) (struct kiocb *, const char __user *, size_t, loff_t);
        int (*readdir) (struct file *, void *, filldir_t);
        unsigned int (*poll) (struct file *, struct poll_table_struct *);
        int (*ioctl) (struct inode *, struct file *, unsigned int, unsigned long);
        int (*unlocked_ioctl) (struct file *, unsigned int, unsigned long);
        int (*compat_ioctl) (struct file *, unsigned int, unsigned long);
        int (*mmap) (struct file *, struct vm_area_struct *);
        int (*open) (struct inode *, struct file *);
        int (*flush) (struct file *);
        int (*release) (struct inode *, struct file *);
        int (*fsync) (struct file *, struct dentry *, int datasync);
        int (*aio_fsync) (struct kiocb *, int datasync);
        int (*fasync) (int, struct file *, int);
        int (*lock) (struct file *, int, struct file_lock *);
        ssize_t (*readv) (struct file *, const struct iovec *, unsigned long, loff_t *);
        ssize_t (*writev) (struct file *, const struct iovec *, unsigned long, loff_t *);
        ssize_t (*sendfile) (struct file *, loff_t *, size_t, read_actor_t, void __user *);
        ssize_t (*sendpage) (struct file *, struct page *, int, size_t, loff_t *, int);
        unsigned long (*get_unmapped_area)(struct file *, unsigned long, unsigned long, unsigned long, unsigned long);
#endif

/*
 * Probe for & register all devices
 * Single threaded
 */
int CDA_Initialise( void)
  {
  /* File operations */
  static struct file_operations s_fileOps =
    {
#if LINUX_VERSION_CODE >= 0x20300
    THIS_MODULE,  /* struct module *owner */
#endif /* LINUX_VERSION_CODE >= 0x20300 */

    CDA_seek,     /* loff_t (*llseek) (struct file *, loff_t, int) */
    CDA_read,     /* ssize_t (*read) (struct file *, char *, size_t, loff_t *) */

#if LINUX_VERSION_CODE >= 0x20614   /* 2.6.20 */
/* *write comes before *aio_read in 2.6.20 */
    CDA_write,    /* ssize_t (*write) (struct file *, const char *, size_t, loff_t *) */
#if LINUX_VERSION_CODE >= 0x20600
    0,            /* ssize_t (*aio_read) (struct kiocb *, char __user *, size_t, loff_t); */
#endif /* LINUX_VERSION_CODE >= 0x20600 */
#else
#if LINUX_VERSION_CODE >= 0x20600
    0,            /* ssize_t (*aio_read) (struct kiocb *, char __user *, size_t, loff_t); */
#endif /* LINUX_VERSION_CODE >= 0x20600 */

    CDA_write,    /* ssize_t (*write) (struct file *, const char *, size_t, loff_t *) */
#endif /* LINUX_VERSION_CODE >= 0x20614 */

#if LINUX_VERSION_CODE >= 0x20600
    0,            /* ssize_t (*aio_write) (struct kiocb *, const char __user *, size_t, loff_t); */
#endif /* LINUX_VERSION_CODE >= 0x20600 */

    0,            /* int (*readdir) (struct file *, void *, filldir_t) */
    0,            /* unsigned int (*poll) (struct file *, struct poll_table_struct *) */

#if LINUX_VERSION_CODE >= 0x20624   /* 2.6.36 */
/* *ioctl was removed in 2.6.36 */
#else
    CDA_ioctl,    /* int (*ioctl) (struct inode *, struct file *, unsigned int, unsigned long) */
#endif /* LINUX_VERSION_CODE >= 0x20624 */

#if LINUX_VERSION_CODE >= 0x2060B
#if LINUX_VERSION_CODE >= 0x20624   /* 2.6.36 */
/* Moved CDA_ioctl here from v2.6.36 */
    CDA_ioctl,    /* long (*unlocked_ioctl) (struct file *, unsigned int, unsigned long) */
#else
    0,            /* long (*unlocked_ioctl) (struct file *, unsigned int, unsigned long) */
#endif
#if defined _LP64
    CDA_compat,   /* long (*compat_ioctl) (struct file *, unsigned int, unsigned long) */
#else
    0,            /* int (*compat_ioctl) (struct file *, unsigned int, unsigned long) */
#endif /* defined _LP64 */
#endif /* LINUX_VERSION_CODE >= 0x2060B */

    0,            /* int (*mmap) (struct file *, struct vm_area_struct *) */
    CDA_open,     /* int (*open) (struct inode *, struct file *) */
#if LINUX_VERSION_CODE >= 0x20200
    0,            /* int (*flush) (struct file *) */
#endif /* LINUX_VERSION_CODE >= 0x20200 */

    CDA_release,  /* int (*release) (struct inode *, struct file *) */
    0,            /* int (*fsync) (struct file *, struct dentry *) */
#if LINUX_VERSION_CODE >= 0x20600
    0,            /* int (*aio_fsync) (struct kiocb *, int datasync); */
#endif /* LINUX_VERSION_CODE >= 0x20600 */

    0,            /* int (*fasync) (int, struct file *, int) */
#if LINUX_VERSION_CODE < 0x20300
    0,            /* int (*check_media_change) (kdev_t dev) */
    0,            /* int (*revalidate) (kdev_t dev) */
#endif /* LINUX_VERSION_CODE < 0x20300 */

    0             /* int (*lock) (struct file *, int, struct file_lock *) */
#if LINUX_VERSION_CODE >= 0x20300
    ,
#if LINUX_VERSION_CODE >= 0x20613   /* 2.6.19 */
/* *readv and *writev were removed in 2.6.19 */
#else
    0,            /* ssize_t (*readv) (struct file *, const struct iovec *, unsigned long, loff_t *); */
    0,            /* ssize_t (*writev) (struct file *, const struct iovec *, unsigned long, loff_t *); */
#endif /* LINUX_VERSION_CODE >= 0x20613 */

#if LINUX_VERSION_CODE >= 0x20600
    0,            /* ssize_t (*sendfile) (struct file *, loff_t *, size_t, read_actor_t, void __user *); */
#endif /* LINUX_VERSION_CODE >= 0x20600 */

    0,            /* ssize_t (*sendpage) (struct file *, struct page *, int, size_t, loff_t *, int); */
    0             /* unsigned long (*get_unmapped_area)(struct file *, unsigned long, unsigned long, unsigned long, unsigned long); */
#if LINUX_VERSION_CODE >= 0x20608
    , 0           /* int (*check_flags)(int); */
    , 0           /* int (*dir_notify)(struct file *, unsigned long); */
    , 0           /* int (*flock)(struct file *, int, struct file_lock *); */
#endif /* LINUX_VERSION_CODE >= 0x20608 */
#if LINUX_VERSION_CODE >= 0x20611 /* 2.6.17 */
    , 0           /* ssize_t (*splice_write)(); */
    , 0           /* ssize_t (*splice_read)(); */
#endif /* LINUX_VERSION_CODE >= 0x20611 */
#if LINUX_VERSION_CODE >= 0x20619 /* 2.6.25 - may have been earlier */
    , 0           /* int (*setlease)(); */
#endif /* LINUX_VERSION_CODE >= 0x20619 */
#endif /* LINUX_VERSION_CODE >= 0x20300 */
    
    };
  int i;

  TRACE( 9, ("%s()\n", __FUNCTION__));

  /* Register as a character device */
  i = register_chrdev( major, name, &s_fileOps);
  if ( 0 > i)
    {
    TRACE( 0, ( "!!! %s: register_chrdrv failed with %d\n", __FUNCTION__, i));
    return i;
    }

  if ( 0 == major)
    major = i;

  /* Call all declared init functions */
  for ( i = 0; i < lengthof( CDA_pfnProbeArray); ++i)
    {
    /* The init function should probe for and then call
     * CDA_RegisterDevice for each detected device
     */
    (*CDA_pfnProbeArray[ i])(); 
    }

  if ( 0 == s_uDevices)
    {
    TRACE( 1, ("*** %s: No devices found\n", __FUNCTION__));
    CDA_Terminate();
    return -ENODEV;
    }

  return 0;
  }


/*
 * Deregister this module
 */
int CDA_Terminate( void)
  {
  int iRet = 0;
  
  TRACE( 9, ("%s()\n", __FUNCTION__));
  
  /* De-register all cards */
  while ( s_uDevices)
    {
    CDA_SInstance* pInst = &s_instArray[ --s_uDevices];
#ifndef NDEBUG
    ASSERT( kMagic == pInst->eMagic);
#endif

    /* Tell the card it's being removed */
    CDA_RemoveDevice( pInst);
    }

  /* De-register the device */
#if LINUX_VERSION_CODE < 0x20617 /* < 2.6.23 */
  iRet = unregister_chrdev( major, name);
  if ( iRet < 0)
    {
    TRACE( 0, ("!!! %s: unregister_chrdev(%d,'%s') failed %d\n",
       __FUNCTION__, major, name, iRet));
    }
#else /* now returns void */
  unregister_chrdev( major, name);
#endif

  return iRet;
  }


/*
 * Single threaded
 */
int CDA_RegisterDevice(
  const CDA_SVtable* pVtable,
  CDA_SDevice* pDevice
) {
  CDA_SInstance* pInst;
  int iRet;

  TRACE( 9, ( "%s(%p,%p)\n", __FUNCTION__, pVtable, pDevice));
  ASSERT( NULL != pVtable);

  /* The following methods are requied */
  ASSERT( pVtable->pfnStart);
  ASSERT( pVtable->pfnGetInfo);
  ASSERT( pVtable->pfnRead);
  ASSERT( pVtable->pfnWrite);

  /* Allocate a device instance */
  if ( s_uDevices >= lengthof( s_instArray))
    {
    TRACE( 1, ("*** %s: Too many devices\n", __FUNCTION__));
    return -ENOMEM;
    }
  pInst = &s_instArray[ s_uDevices++];
#ifndef NDEBUG
  ASSERT( kMagic != pInst->eMagic);

  /* Init instance data */
  pInst->eMagic = kMagic;
#endif
  sprintf( pInst->szDeviceName, "%s%u", name, s_uDevices - 1);

  //PICTURE-C save device number
  pInst->devicenum = s_uDevices-1;
  
  atomic_set( &pInst->openCount, 0);
  pInst->pFile = NULL;
  pInst->pVtable = pVtable;
  pInst->pDevice = pDevice;

  /* Empty the eventQ */
  spin_lock_init( &pInst->lockEventQ);
  pInst->uEventHead = pInst->uEventTail = 0;
  init_waitqueue_head(&pInst->WaitQ);
#if LINUX_VERSION_CODE >= 0x20600
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
  INIT_WORK(&pInst->work, CDA_IrqTask);
#else
  INIT_WORK(&pInst->work, CDA_IrqTask, pInst);
#endif
#else
  pInst->task.routine = CDA_IrqTask;
  pInst->task.data = pInst;
#endif
#ifdef DEBUG
  pInst->uIrqOverflows = 0;
#endif

  TRACE( 2, ( "%s: ensure you've created the device file\n  e.g. mknod /dev/%s c %d %u\n",
    __PRETTY_FUNCTION__, pInst->szDeviceName, major, s_uDevices - 1));

  /* Start the device */
  pInst->eState = kStateStarting;
  iRet = (*pVtable->pfnStart)( pDevice, pInst, pInst->szDeviceName);
  if ( 0 == iRet)
    {
    pInst->eState = kStateStarted;
    }
  else
    {
    /* De-register this device */
    CDA_RemoveDevice( pInst);
    --s_uDevices;
    }

  return iRet;
  }


/*
 * Remove a device
 */
static int CDA_RemoveDevice(
  CDA_SInstance* pInst
) {
  TRACE( 9, ("%s(%p)\n", __FUNCTION__,pInst));

  ASSERT( NULL != pInst);
#ifndef NDEBUG
  ASSERT( kMagic == pInst->eMagic);
#endif

  TRACE( 2, ("%s() removing '%s' (%p)\n", __FUNCTION__, pInst->szDeviceName, pInst));

  /* De-allocate hardware resources */
  if ( kStateStarted == pInst->eState
    || kStateRemoving == pInst->eState
  ) {
    TRACE( 3, ("%s() stopping '%s'\n", __FUNCTION__, pInst->szDeviceName));
    if ( pInst->pVtable->pfnStop)
      (*pInst->pVtable->pfnStop)( pInst->pDevice);
    }
  pInst->eState = kStateStopped;

  /* Tell driver to release resources */
  if ( pInst->pVtable->pfnFinal)
    {
    TRACE( 3, ("%s() finalising '%s'\n", __FUNCTION__, pInst->szDeviceName));
    (*pInst->pVtable->pfnFinal)( pInst->pDevice);
    }
  pInst->eState = kStateRemoved;

  /* Ensure all buffers unlocked */
  CDA_DeviceIdle( pInst);

#ifndef NDEBUG
  pInst->eMagic = ~kMagic;
#endif
  
  return 0;
  }


/*
 * Open the device
 */
static int CDA_open(
  struct inode *pnode,
  struct file * pFile
) {
  unsigned uMinor;
  int iCount;
  CDA_SInstance* pInst = NULL;
  int iRet = 0;

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0) )
  MOD_INC_USE_COUNT;
#endif

  TRACE( 9, ( "%s(%p,%p)\n",  __FUNCTION__, pnode, pFile));
  ASSERT( NULL != pnode);
  ASSERT( NULL != pFile);

  /* Get device index */
  uMinor = MINOR( pnode->i_rdev);
  if ( uMinor < s_uDevices)
    {
    pInst = &s_instArray[ uMinor];
#ifndef NDEBUG
    ASSERT( kMagic == pInst->eMagic);
#endif

    /* Bump open count */
    atomic_inc( &pInst->openCount);
    iCount = atomic_read( &pInst->openCount);

    /* Link file to this device */
    pFile->private_data = pInst;

    if ( kStateStarted != pInst->eState)
      {
      TRACE( 0, ("!!! %s '%s' not started\n", __FUNCTION__, pInst->szDeviceName));
      iRet = -EBUSY;
      }
    else if ( pInst->pVtable->pfnOpen)
      {
      /* Call device specific open */
      iRet = (*pInst->pVtable->pfnOpen)( pInst->pDevice, pnode, pFile, iCount);
      }

    /* Check for errors */
    if ( 0 > iRet)
      {
      pFile->private_data = NULL;
      atomic_dec( &pInst->openCount);
      }
    }
  else
    {
    TRACE( 5, ( "*** %s: device %u not registered\n",  __FUNCTION__, uMinor));
    iRet = -ENXIO;
    }

  if ( iRet >= 0)
    {
    TRACE( 2, ( "%s: '%s' opened sucessfully\n",  __FUNCTION__, pInst->szDeviceName));
    }
  else
    {
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0) )
    MOD_DEC_USE_COUNT;
#endif
    }

  return iRet;
  }


/*
 * Close the device.
 */
static int CDA_release(
  struct inode * pnode,
  struct file * pFile
) {
  CDA_SInstance* pInst;
  int iOpen;
  int iRet = 0;

  TRACE( 9, ( "%s(%p,%p)\n", __FUNCTION__, pnode, pFile));
  ASSERT( NULL != pnode);
  ASSERT( NULL != pFile);

  /* Get instance */
  pInst = pFile->private_data;
  if ( NULL == pInst)
    {
    TRACE( 0, ( "!!! %s: invalid instance\n", __FUNCTION__));
    return -EINVAL;
    }
#ifndef NDEBUG
  ASSERT( kMagic == pInst->eMagic);
#endif
  ASSERT( kStateStarted == pInst->eState);
  
  iOpen = atomic_read( &pInst->openCount);
  if ( iOpen < 1)
    {
    TRACE( 1, ( "*** %s: '%s' no pending refs\n", __FUNCTION__, pInst->szDeviceName));
    return -EINVAL;
    }

  TRACE( 2, ( "%s: closing '%s'\n",  __FUNCTION__, pInst->szDeviceName));
  pFile->private_data = NULL;
  TRACE( 2, ( "%s: pFile->private_data has been set to NULL\n", __FUNCTION__));

  if ( pInst->pVtable->pfnClose)
    {
    /* Call device specific close */
    TRACE(4, ("%s: calling pfnClose\n", __FUNCTION__));
    iRet = (*pInst->pVtable->pfnClose)( pInst->pDevice, pnode, pFile, iOpen);
    }

  /* Release any claim if open count falls to 0 */
  /* Decr. open count */
  atomic_dec( &pInst->openCount);
  if ( 0 == atomic_read( &pInst->openCount) )
    {
    pInst->pFile = NULL;

    /* Wakeup all waiting processes */
    TRACE(4, ("%s Waking up any sleeping processes\n", __FUNCTION__));
    wake_up_interruptible( &pInst->WaitQ); 

    /* Release all locked buffers */
    TRACE(4, ("%s Setting device to Idle\n", __FUNCTION__));
    CDA_DeviceIdle( pInst);
    }
  else
    {
    TRACE(4, ("%s Not waking up any sleeping processes\n", __FUNCTION__));
    }
    
  TRACE( 2, ( "%s: '%s' closed\n",  __FUNCTION__, pInst->szDeviceName));

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0) )
  MOD_DEC_USE_COUNT;
#endif
  return iRet;
  }


/*
 * Release all resources acquired by an open instance
 */
int CDA_DeviceIdle( CDA_SInstance* pInst)
  {
  SLock* pLock;
  
  TRACE( 9, ( "%s(%p)\n", __FUNCTION__, pInst));

  ASSERT( NULL != pInst);
#ifndef NDEBUG
  ASSERT( kMagic == pInst->eMagic);
#endif

  /* Unlock all locked buffers */
  TRACE(4, ("%s: Checking for unreleased application buffers\n", __FUNCTION__));
  while ( NULL != (pLock = pInst->pLockHead) )
    {
    TRACE( 1, ("*** %s() Unlocking buffer %p len %Zd\n",
      __FUNCTION__, pLock->pvBuffer, pLock->len ));
    CDA_UnlockBuffer( pLock );
    pInst->pLockHead = pLock->pNext;
    kfree( pLock);
    }

#ifdef DEBUG
  if ( pInst->uIrqOverflows)
    {
    TRACE( 2, ("%s() %u missed events\n", __FUNCTION__, pInst->uIrqOverflows));
    pInst->uIrqOverflows = 0;
    }
#endif

  return 0;
  }


/*
 * Seek, Read & Write are not implemented
 */
static loff_t CDA_seek(
  struct file * pFile,
  loff_t offset,
  int mode
) {
  TRACE( 9, ( "%s(%p,%ld,%d)\n", __FUNCTION__, pFile, (long)offset, mode));
  (void)pFile;
  (void)offset;
  (void)mode;
  return -EINVAL;
  }


static ssize_t CDA_read(
  struct file * pFile,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0) )
  char * pBuf,
#else
  char __user * pBuf,
#endif
  size_t len,
  loff_t * pOff
) {
  TRACE( 9, ( "%s(%p,%p,%Zd,%p)\n", __FUNCTION__, pFile, pBuf, len, pOff));
  (void)pFile;
  (void)pBuf;
  (void)len;
  (void)pOff;
  return -EINVAL;
  } 


static ssize_t CDA_write(
  struct file * pFile,
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0) )
  const char * pBuf,
#else
  const char __user * pBuf,
#endif
  size_t len,
  loff_t * pOff
) {
  TRACE( 9, ( "%s(%p,%p,%Zd,%p)\n", __FUNCTION__, pFile, pBuf, len, pOff));
  (void)pFile;
  (void)pBuf;
  (void)len;
  (void)pOff;
  return -EINVAL;
  } 


#if defined _LP64
/*
 * Handle user32 ioctl calls in 64 bit driver
 */
static long CDA_compat(
  struct file * pFile,
  unsigned int iFunc,
  unsigned long ulParam
) {
  char header[ CDA_SGTABLE_HEADER_SIZE];
  CDA_SInstance* pInst;
  int iRet = 0;

  TRACE( 4, ( "%s(%p,%08x,%08lx)\n", __FUNCTION__, pFile, iFunc, ulParam));
  ASSERT( NULL != pFile);

  switch(iFunc) /* Convert 32 bit ioctl codes to 64 bit ioctl codes */
  {
    case 0x4004E000:
    case 0x8004E000: iFunc = CDA_IOCTL_DEVICEINFO;        break;
    case 0xC004E001: iFunc = CDA_IOCTL_REGLIST;           break;
    case 0xC004E002: iFunc = CDA_IOCTL_MEMORY_LOCK;       break;
    case 0x8004E003:
    case 0x4004E003: iFunc = CDA_IOCTL_MEMORY_UNLOCK;     break;
    case 0x4004E004:
    case 0x8004E004: iFunc = CDA_IOCTL_EVENT;             break;
    case 0x8004E005:
    case 0x4004E005: iFunc = CDA_IOCTL_CONFIG_RESTORE;    break;
    case 0x8004E006:
    case 0x4004E006: iFunc = CDA_IOCTL_CLAIM;             break;
    case 0x8004E007:
    case 0x4004E007: iFunc = CDA_IOCTL_EVENT_PUT;         break;
    case 0xC004E00A: iFunc = CDA_IOCTL_MEMORY_LOCK_RISC;  break;
    case 0xC004E00C: iFunc = CDA_IOCTL_MEMORY_FLUSH_RISC; break;
    case 0xC004E00D: iFunc = CDA_IOCTL_MEMORY_FLUSH_DMA;  break;
    default:
       TRACE(1, ("%s: !!!WARNING!!! Unrecognised 32 bit Ioctl code 0x%08X\n", __FUNCTION__, iFunc));
       break;
  }

  /* Get instance data */
  pInst = pFile->private_data;
  if ( NULL == pInst)
    {
    TRACE( 0, ( "!!! %s: invalid instance\n", __FUNCTION__));
    return -EINVAL;
    }
#ifndef NDEBUG
  ASSERT( kMagic == pInst->eMagic);
#endif
  ASSERT( kStateStarted == pInst->eState);

  /* Ensure device open */
  if ( atomic_read( &pInst->openCount) < 1)
    {
    TRACE( 0, ( "!!! %s: device '%s' not open\n", __FUNCTION__, pInst->szDeviceName));
    return -EINVAL;
    }


  switch(iFunc)
    {
      case CDA_IOCTL_MEMORY_LOCK:
      case CDA_IOCTL_MEMORY_LOCK_RISC:
        if ( NULL != pInst->pFile)
          {
          unsigned long ulVirtAddress; /* 64 bit */
          /* Lock & create a DMA scatter list for a user buffer */
          copy_from_user_ret( header, (void*)ulParam, sizeof( header), -EFAULT );
          ulVirtAddress = (unsigned long)_CDA_SGTable_Get_VirtualAddr(header);
          ulVirtAddress &= 0x00000000FFFFFFFF;
          _CDA_SGTable_Set_VirtualAddr(header, (void*)ulVirtAddress);
          if( CDA_IOCTL_MEMORY_LOCK == iFunc )
            iRet = CDA_LockBuffer( pInst, header, (uiSG*)(((char*)ulParam) + CDA_SGTABLE_HEADER_SIZE), 1);
          else
            iRet = CDA_LockBuffer( pInst, header, (uiSG*)(((char*)ulParam) + CDA_SGTABLE_HEADER_SIZE), 0);

          /* Return the no. entries to the caller */
          copy_to_user_ret( (void*)ulParam, header, sizeof( header), -EFAULT );
          }
        else
          {
          TRACE( 1, ( "*** %s: not device owner\n", __FUNCTION__));
          iRet = -EINVAL;
          }
        break;

  case CDA_IOCTL_MEMORY_FLUSH_RISC:
  case CDA_IOCTL_MEMORY_FLUSH_DMA:
    /* Flush a user buffer */
    if ( NULL != pInst->pFile)
      {
      void* pvBuffer;
      ui32 len;
      SLock* pLock;
      unsigned long ulVirtAddress;
      /* Get the buffer info from user space */
      copy_from_user_ret( header, (void*)ulParam, sizeof( header), -EFAULT );
      ulVirtAddress = _CDA_SGTable_Get_VirtualAddr( header);
      ulVirtAddress &= 0x00000000FFFFFFFF;
      pvBuffer = (void*) ulVirtAddress;
      len = _CDA_SGTable_Get_MemorySize( header);

      /* Find the matching lock */
      iRet = -EINVAL;
      for ( pLock = pInst->pLockHead; ; pLock = pLock->pNext )
        {
        if ( NULL == pLock)
          {
          TRACE( 1, ( "*** %s: scatter list not locked\n", __FUNCTION__));
          return -EINVAL;
          }

        if ( pLock->pvBuffer == pvBuffer )
          {
          /* Flush the buffer */
          iRet = CDA_FlushBuffer(pLock);
          break;
          }
        }
      }
    else
      {
      TRACE( 1, ( "*** %s: not device owner\n", __FUNCTION__));
      iRet = -EINVAL;
      }
    break;


  case CDA_IOCTL_MEMORY_UNLOCK:
    /* Unlock a user buffer */
    if ( NULL != pInst->pFile)
      {
      void* pvBuffer;
      ui32 len;
      SLock* pLock;
      unsigned long ulVirtAddress;
      /* Get the buffer info from user space */
      copy_from_user_ret( header, (void*)ulParam, sizeof( header), -EFAULT );
      ulVirtAddress = _CDA_SGTable_Get_VirtualAddr( header);
      ulVirtAddress &= 0x00000000FFFFFFFF;
      pvBuffer = (void*) ulVirtAddress;
      len = _CDA_SGTable_Get_MemorySize( header);

      /* Find the matching lock */
      iRet = -EINVAL;
      for ( pLock = pInst->pLockHead; ; pLock = pLock->pNext )
        {
        if ( NULL == pLock)
          {
          TRACE( 1, ( "*** %s: scatter list not locked\n", __FUNCTION__));
          return -EINVAL;
          }

        if ( pLock->pvBuffer == pvBuffer )
          {
          /* Unlock the buffer */
          iRet = CDA_UnlockBuffer(pLock);

          /* Remove from list */
          if ( NULL != pLock->pPrev)
            {
            ASSERT( pLock != pInst->pLockHead);
            pLock->pPrev->pNext = pLock->pNext;
            }
          else
            {
            ASSERT( pLock == pInst->pLockHead);
            pInst->pLockHead = pLock->pNext;
            }

          if ( NULL != pLock->pNext)
            {
            pLock->pNext->pPrev = pLock->pPrev;
            }

          kfree( pLock);
          break;
          }
        }
      }
    else
      {
      TRACE( 1, ( "*** %s: not device owner\n", __FUNCTION__));
      iRet = -EINVAL;
      }
    break;

    case CDA_IOCTL_REGLIST:
      if ( NULL != pInst->pFile)
        {
        CDA_SIoctlRegList regs;
        ui32 UserParams[2];
        copy_from_user_ret( &UserParams, (void*)ulParam, sizeof( UserParams), -EFAULT );
        regs.size = UserParams[0];
        regs.pList = (void*)( ((unsigned long)UserParams[1]) & 0x00000000FFFFFFFF);
        iRet = CDA_IoctlRegList( pInst, pFile, &regs);
        }
      else
        {
        TRACE( 1, ( "*** %s: not device owner\n", __FUNCTION__));
        iRet = -EINVAL;
        }
      if (iRet){ TRACE(1,("%s !!!WARNING!!! CDA_IOCTL_REGLIST returning 0x%08X\n", __FUNCTION__, iRet)); }
      break;
 
    default:
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36) )
      return CDA_ioctl(NULL, pFile, iFunc, ulParam);
#else
      return CDA_ioctl(pFile, iFunc, ulParam);
#endif
	}
  return iRet;
  }
#endif


/*
 * Handle user ioctl calls
 */
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36) )
static int CDA_ioctl(
  struct inode * pnode,
#else
static long CDA_ioctl(
#endif
  struct file * pFile,
  unsigned int iFunc,
  unsigned long ulParam
) {
  CDA_SInstance* pInst;
  char header[ CDA_SGTABLE_HEADER_SIZE];
  int iRet = 0;

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36) )
  TRACE( 4, ( "%s(%p,%p,%08x,%08lx)\n", __FUNCTION__, pnode, pFile, iFunc, ulParam));
#else
  TRACE( 4, ( "%s(%p,%08x,%08lx)\n", __FUNCTION__, pFile, iFunc, ulParam));
#endif
  ASSERT( NULL != pFile);

  /* Get instance data */
  pInst = pFile->private_data;
  if ( NULL == pInst)
    {
    TRACE( 0, ( "!!! %s: invalid instance\n", __FUNCTION__));
    return -EINVAL;
    }
#ifndef NDEBUG
  ASSERT( kMagic == pInst->eMagic);
#endif
  ASSERT( kStateStarted == pInst->eState);

  /* Ensure device open */
  if ( atomic_read( &pInst->openCount) < 1)
    {
    TRACE( 0, ( "!!! %s: device '%s' not open\n", __FUNCTION__, pInst->szDeviceName));
    return -EINVAL;
    }

  switch ( iFunc)
    {
  case CDA_IOCTL_DEVICEINFO:
      {
      CDA_SIoctlDeviceInfo info;
      info.dwDriverRev = CDA_DRV_VERSION;

      ASSERT( pInst->pVtable->pfnGetInfo);
      iRet = (*pInst->pVtable->pfnGetInfo)( pInst->pDevice, &info);
      if ( 0 == iRet)
        {
        TRACE( 9, ( "copy_to_user_ret(%p, %p, %Zx)\n", (void*)ulParam, &info, sizeof(info) ));
        copy_to_user_ret( (void*)ulParam, &info, sizeof( info), -EFAULT );
        TRACE( 9, ( "copy_to_user_ret() completed sucessfully\n" ));
        }
      else
        {
        TRACE( 9, ( "PCI_GetInfo() returned %08x\n", iRet));
        }
      }
    break;

  case CDA_IOCTL_CLAIM:
    if ( NULL == pInst->pFile)
      {
      pInst->pFile = pFile;
      }
    else
      {
      TRACE( 2, ( "%s: device in use\n", __FUNCTION__));
      iRet = -EINVAL;
      }
    break;

  case CDA_IOCTL_REGLIST:
    if ( NULL != pInst->pFile)
      {
      CDA_SIoctlRegList regs;
      copy_from_user_ret( &regs, (void*)ulParam, sizeof( regs), -EFAULT );
      iRet = CDA_IoctlRegList( pInst, pFile, &regs);
      }
    else
      {
      TRACE( 1, ( "*** %s: not device owner\n", __FUNCTION__));
      iRet = -EINVAL;
      }
    if (iRet){ TRACE(1,("%s !!!WARNING!!! CDA_IOCTL_REGLIST returning 0x%08X\n", __FUNCTION__, iRet)); }
    break;

  case CDA_IOCTL_MEMORY_LOCK:
  case CDA_IOCTL_MEMORY_LOCK_RISC:
    if ( NULL != pInst->pFile)
      {
      /* Lock & create a DMA scatter list for a user buffer */
      copy_from_user_ret( header, (void*)ulParam, sizeof( header), -EFAULT );
#if defined DEBUG
      {
        int iCount;
        ui32* pTemp = (ui32*)header;
        for(iCount = 0; iCount < sizeof(header)/sizeof(ui32); iCount++)
        {
           TRACE(4, ("CDA_IOCTL_MEMORY_LOCK header[%d] = %08X\n", iCount, pTemp[iCount]));
        }
      }
      TRACE(2, ("CDA_IOCTL_MEMORY_LOCK ulParam = %ld\n", ulParam));
#endif
      if( CDA_IOCTL_MEMORY_LOCK == iFunc )
        iRet = CDA_LockBuffer( pInst, header, (uiSG*)(((char*)ulParam) + CDA_SGTABLE_HEADER_SIZE), 1);
      else
        iRet = CDA_LockBuffer( pInst, header, (uiSG*)(((char*)ulParam) + CDA_SGTABLE_HEADER_SIZE), 0);

      /* Return the no. entries to the caller */
      copy_to_user_ret( (void*)ulParam, header, sizeof( header), -EFAULT );
      }
    else
      {
      TRACE( 1, ( "*** %s: not device owner\n", __FUNCTION__));
      iRet = -EINVAL;
      }
    break;

  case CDA_IOCTL_MEMORY_FLUSH_RISC:
  case CDA_IOCTL_MEMORY_FLUSH_DMA:
    /* Flush a user buffer */
    if ( NULL != pInst->pFile)
      {
      void* pvBuffer;
      ui32 len;
      SLock* pLock;

      /* Get the buffer info from user space */
      copy_from_user_ret( header, (void*)ulParam, sizeof( header), -EFAULT );
      pvBuffer = (void*) _CDA_SGTable_Get_VirtualAddr( header);
      len = _CDA_SGTable_Get_MemorySize( header);

      /* Find the matching lock */
      iRet = -EINVAL;
      for ( pLock = pInst->pLockHead; ; pLock = pLock->pNext )
        {
        if ( NULL == pLock)
          {
          TRACE( 1, ( "*** %s: scatter list not locked\n", __FUNCTION__));
          return -EINVAL;
          }

        if ( pLock->pvBuffer == pvBuffer )
          {
          /* Flush the buffer */
          iRet = CDA_FlushBuffer(pLock);
          break;
          }
        }
      }
    else
      {
      TRACE( 1, ( "*** %s: not device owner\n", __FUNCTION__));
      iRet = -EINVAL;
      }
    break;

  case CDA_IOCTL_MEMORY_UNLOCK:
    /* Unlock a user buffer */
    if ( NULL != pInst->pFile)
      {
      void* pvBuffer;
      ui32 len;
      SLock* pLock;

      /* Get the buffer info from user space */
      copy_from_user_ret( header, (void*)ulParam, sizeof( header), -EFAULT );
      pvBuffer = (void*) _CDA_SGTable_Get_VirtualAddr( header);
      len = _CDA_SGTable_Get_MemorySize( header);

      /* Find the matching lock */
      iRet = -EINVAL;
      for ( pLock = pInst->pLockHead; ; pLock = pLock->pNext )
        {
        if ( NULL == pLock)
          {
          TRACE( 1, ( "*** %s: scatter list not locked\n", __FUNCTION__));
          return -EINVAL;
          }

        if ( pLock->pvBuffer == pvBuffer )
          {
          /* Unlock the buffer */
          iRet = CDA_UnlockBuffer(pLock);

          /* Remove from list */
          if ( NULL != pLock->pPrev)
            {
            ASSERT( pLock != pInst->pLockHead);
            pLock->pPrev->pNext = pLock->pNext;
            }
          else
            {
            ASSERT( pLock == pInst->pLockHead);
            pInst->pLockHead = pLock->pNext;
            }

          if ( NULL != pLock->pNext)
            {
            pLock->pNext->pPrev = pLock->pPrev;
            }

          kfree( pLock);
          break;
          }
        }
      }
    else
      {
      TRACE( 1, ( "*** %s: not device owner\n", __FUNCTION__));
      iRet = -EINVAL;
      }
    break;

  case CDA_IOCTL_EVENT:
    /* Wait for an event */
    if ( NULL != pInst->pFile)
      {
      CDA_SIoctlEvent ev;
      iRet = CDA_WaitEvent( pInst, &ev);
      if ( 0 <= iRet)
        {
         //Unset the bit that was set in the IRQ top half (cdapci.c)
         #if PICC_DIO_ENABLE
         if(pInst->devicenum == PICC_SHK_DEVNUM) outb_p(0x00,PICC_DIO_BASE+PICC_DIO_PORTC); //UNSET DIO board PORTC bit C0
         if(pInst->devicenum == PICC_LYT_DEVNUM) outb_p(0x00,PICC_DIO_BASE+PICC_DIO_PORTB); //UNSET DIO board PORTB bit B0 
         #endif
         copy_to_user_ret( (void*)ulParam, &ev, sizeof( ev), -EFAULT );
        }
      }
    else
      {
      TRACE( 1, ( "*** %s: not device owner\n", __FUNCTION__));
      iRet = -EINVAL;
      }
    break;

  case CDA_IOCTL_EVENT_PUT:
    if ( NULL != pInst->pFile)
      {
      CDA_SIoctlEvent event;
      copy_from_user_ret( &event, (void*)ulParam, sizeof( event), -EFAULT );
      iRet = CDA_IoctlPutEvent( pInst, &event );
      }
    else
      {
      TRACE( 1, ( "*** %s: not device owner\n", __FUNCTION__));
      iRet = -EINVAL;
      }
    break;

  case CDA_IOCTL_CONFIG_RESTORE:
    if ( NULL != pInst->pFile)
      {
      iRet = CDA_IoctlConfigRestore( pInst);
      }
    else
      {
      TRACE( 1, ( "*** %s: not device owner\n", __FUNCTION__));
      iRet = -EINVAL;
      }
    break;

  default:
    TRACE( 1, ( "*** %s: invalid ioctl %x\n", __FUNCTION__, iFunc));
    iRet = -ENOTTY; /* POSIX requirement */
    break;
    }

  TRACE( 9, ( "%s() completed sucessfully\n", __FUNCTION__));
  return iRet;
  }


/*
 * Read/Write register list to device
 */
static int CDA_IoctlRegList(
  CDA_SInstance* pInst,
  struct file * pFile,
  CDA_SIoctlRegList* pInfo
) {
  ui32 ulBufferLength;
  int iRet = 0;

  TRACE( 3, ( "%s(%p,%p,%p)\n", __FUNCTION__, pInst, pFile, pInfo));

  ulBufferLength = pInfo->size;
  ulBufferLength /= CDA_REG_RECORD_SIZE;

  TRACE( 3, ( "%s - size = %d, CDA_REG_RECORD_SIZE = %d, ulBufferLength = %d\n", __FUNCTION__, pInfo->size, CDA_REG_RECORD_SIZE, ulBufferLength));

  if ( ulBufferLength < 1) 
    {
    TRACE( 1, ("*** %s() buffer too small\n", __FUNCTION__));
    return -EINVAL;
    }

  /* For all entries in the register list */
  while ( ulBufferLength--)
    {
    ui32 uType, uAddr, uData;
    int iBits;
    char reg[ CDA_REG_RECORD_SIZE];

    copy_from_user_ret( reg, (void*)pInfo->pList, sizeof( reg), -EFAULT );

    uAddr = _CDA_RegList_GetRegAddr( reg);

    /* Determine operand size */
    uType = _CDA_RegList_GetRegType( reg);
    if ( CDA_8BIT == (CDA_8BIT & uType))
      iBits = 8;
    else if ( CDA_16BIT == (CDA_16BIT & uType))
      iBits = 16;
    else if ( CDA_32BIT == (CDA_32BIT & uType))
      iBits = 32;
    else
      {
      TRACE( 1, ("*** %s() invalid operand size 0x%08x\n", __FUNCTION__, uType));
      iRet = -EINVAL;
      break;
      }

    /* Determine operation type */
    if ( CDA_REG_WRITE & uType)
      {
      uData = _CDA_RegList_GetRegData32( reg);
#if 0
      TRACE( 3, ("%s() write address 0x%03x, %d bits, data 0x%02x\n",
        __FUNCTION__, uAddr, iBits, uData));
#endif

      ASSERT( pInst->pVtable->pfnWrite);
      iRet = (*pInst->pVtable->pfnWrite)( pInst->pDevice, pFile, uData, uAddr, iBits);
      }
    else if ( CDA_REG_READ & uType)
      {
      ASSERT( pInst->pVtable->pfnRead);
      iRet = (*pInst->pVtable->pfnRead)( pInst->pDevice, pFile, &uData, uAddr, iBits);
      if ( 0 == iRet)
        {
#if 0
        TRACE( 3, ("%s() read address 0x%03x, %d bits, data 0x%02x\n",
          __FUNCTION__, uAddr, iBits, uData));
#endif

        _CDA_RegList_SetRegData32( reg, uData);
        copy_to_user_ret( (void*)pInfo->pList, reg, sizeof( reg), -EFAULT );
        }
      }
    else
      {
      TRACE( 1, ("*** %s() invalid operation 0x%08x\n", __FUNCTION__, uType));
      iRet = -EINVAL;
      }

    if ( iRet)
      break;

    pInfo->pList = (char*)pInfo->pList + CDA_REG_RECORD_SIZE;
    }

  return iRet;
  }


/*
 * Wait for a device event (IRQ)
 */
static int CDA_WaitEvent(
  CDA_SInstance* pInst,
  CDA_SIoctlEvent* pEvent
) {
  sigset_t pending, shpending;
  unsigned long p0,p1,shp0,shp1;
  unsigned uIndex;

  TRACE( 9, ( "%s(%p,%p)\n", __FUNCTION__, pInst, pEvent));
  ASSERT( NULL != pInst);
  ASSERT( NULL != pEvent);

  if (1 > atomic_read(&pInst->openCount))
  {
  TRACE( 2, ( "%s: Device not open\n", __FUNCTION__));
  return -ENOENT;
  }

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0) )
  /* interruptible_sleep_on is deprecated, there is a race condition */

  /* Lock the eventQ */
  spin_lock_irq( &pInst->lockEventQ);

  /* Check for an event */
  while ( pInst->uEventHead == (uIndex = pInst->uEventTail))
    {

    /* No events so release spinlock */
    spin_unlock_irq( &pInst->lockEventQ);
    
    /* Put the current process to sleep.
     * Execution will be resumed after a call to module_wake_up
     * or when a signal, such as Ctrl-C, is sent to the process
     */
    TRACE( 3, ( "%s: sleeping\n", __FUNCTION__));
    interruptible_sleep_on( &pInst->WaitQ);
    TRACE( 3, ( "%s: awake\n", __FUNCTION__));

    /* Test if woken by a signal */
    if (signal_pending(current))
      {
        {
        TRACE( 2, ( "%s: woken by signal\n",__FUNCTION__));
        return -EINTR;
        }
      }

    if ( NULL == pInst->pFile)
      {
      TRACE( 2, ( "%s: woken by device close\n", __FUNCTION__));
      return -ENOENT;    /* Device closed */
      }

    /* re-acquire lock before testing for an event */
    spin_lock_irq( &pInst->lockEventQ);
    }

#else

  wait_event_interruptible( pInst->WaitQ, (pInst->uEventHead != (uIndex = pInst->uEventTail)) );
  if ( NULL == pInst->pFile)
    {
    TRACE( 2, ( "%s: woken by device close (pFile is NULL)\n", __FUNCTION__));
    return -ENOENT;    /* Device closed */
    }
  else if ( 0 == atomic_read(&pInst->openCount))
    {
    TRACE( 2, ( "%s: woken by device close (openCount is NULL)\n", __FUNCTION__));
    return -ENOENT;    /* Device closed */
    }
  /* Test if woken by a signal */
  else if (signal_pending(current))
    {
    pending = current->pending.signal;
    p0 = pending.sig[0];
    p1 = pending.sig[1];
    shpending = current->signal->shared_pending.signal;
    shp0 = shpending.sig[0];
    shp1 = shpending.sig[1];
    TRACE( 2, ( "%s: woken by signal p0=%x,p1=%x,shp0=%x,shp1=%x, oc=%d\n",__FUNCTION__,p0,p1,shp0,shp1, atomic_read(&pInst->openCount)));
    return -EINTR;
   }

#endif

  ASSERT( uIndex < lengthof( pInst->eventArray) );

  /* Get oldest event */
  pEvent->event = pInst->eventArray[ uIndex].event;
  pEvent->data = pInst->eventArray[ uIndex].data;

  /* Bump event Q tail */
  if ( ++uIndex >= lengthof( pInst->eventArray))
    uIndex = 0;
  pInst->uEventTail = uIndex;

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0) )
  /* Unlock the eventQ */
  spin_unlock_irq( &pInst->lockEventQ);
#else
#endif

  TRACE( 3, ( "%s: event %u, data %u\n", __FUNCTION__, pEvent->event, pEvent->data));
  return 0;
  }


/*
 * Handle device IRQ's
 * IRQ time
 */
void CDA_DeviceEvent(
  CDA_SInstance* pInst,
  ui32 ev,
  ui32 data
) {
  /*Top half of IRQ Handler */
  
  size_t uIndex;

  TRACE( 9, ("%s(%p,%u,%u)\n",__FUNCTION__,pInst,ev,data));
  ASSERT( NULL != pInst);
#ifndef NDEBUG
  ASSERT( kMagic == pInst->eMagic);
#endif

  /* Push event info onto event Q */
  spin_lock( &pInst->lockEventQ);

  uIndex = pInst->uEventHead;
  pInst->eventArray[ uIndex].event = ev;
  pInst->eventArray[ uIndex].data = data;

  if ( ++uIndex >= lengthof( pInst->eventArray))
    uIndex = 0;
  if ( uIndex != pInst->uEventTail)
    pInst->uEventHead = uIndex;
  else
    {
    TRACE( 2, ("%s: event 0x%08x lost\n", __FUNCTION__, ev));
#ifdef DEBUG
    ++(pInst->uIrqOverflows);
#endif
    }

  spin_unlock( &pInst->lockEventQ);

  /* Schedule bottom half task to run */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
  wake_up_interruptible( &pInst->WaitQ); 
  //schedule_work(&pInst->work);
#elif LINUX_VERSION_CODE > 0x20200
  queue_task( &pInst->task, &tq_immediate);
  mark_bh( IMMEDIATE_BH);
#else
  queue_task_irq( &pInst->task, &tq_immediate);
  mark_bh( IMMEDIATE_BH);
#endif
  }


/*
 * Deferred IRQ handler
 * This routine is called by the kernel as soon as it's safe to resume
 * normal kernel operations
 */
#if LINUX_VERSION_CODE >= 0x20614   /* 2.6.20 */
static void CDA_IrqTask( struct work_struct *ws)
#else
static void CDA_IrqTask( void * pv)
#endif
  {
#if LINUX_VERSION_CODE >= 0x20614   /* 2.6.20 */
  CDA_SInstance* pInst = container_of( ws, CDA_SInstance, work );
#else
  CDA_SInstance* pInst = (CDA_SInstance*)pv;
#endif
  TRACE( 9, ( "%s(%p)\n", __FUNCTION__, pInst));
  ASSERT( NULL != pInst);
#ifndef NDEBUG
  ASSERT( kMagic == pInst->eMagic);
#endif
  /* Wakeup all processes waiting for an event */
  wake_up_interruptible( &pInst->WaitQ);
  }


/*
 * Reconfigure device
 */
static int CDA_IoctlConfigRestore(
  CDA_SInstance* pInst
) {
  int status;

  TRACE( 9, ( "%s(%p)\n", __FUNCTION__, pInst));
  ASSERT( NULL != pInst);

  /* Call the reconfiguration handler */
  if ( pInst->pVtable->pfnReconfig)
    status = (*pInst->pVtable->pfnReconfig)( pInst->pDevice);
  else
    status = 0;

  return status;
  }


/*
 * Insert an event into the queue
 */
static int CDA_IoctlPutEvent(
  CDA_SInstance*   pInst,
  CDA_SIoctlEvent* pEvent
) {
  int status;

  TRACE( 9, ( "%s(%p)\n", __FUNCTION__, pInst));
  ASSERT( NULL != pInst);

  /* Call the put event handler */
  if ( pInst->pVtable->pfnPutEvent)
    status = (*pInst->pVtable->pfnPutEvent)( pInst->pDevice, pEvent);
  else
    status = 0;

  return status;
  }


#define PUT_ADDR_LEN_RET( _p, _a, _l, _e) do { \
  put_user_ret( (_a), (_p), (_e) ); \
  put_user_ret( (_l), (_p)+1, (_e) ); \
  TRACE( 4, ("%s putting addr/len pair to %p 0x%08lx 0x%08lx\n", __FUNCTION__, (_p), (_a), (_l))); \
} while( 0)


/*
 * Lock & create a DMA scatter list for a user space buffer
 */
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0) )
/* kernel 2.6.n */
static int CDA_pages_to_sg(struct SLock *pLock)
{
   int i = 0;

   if (NULL == pLock->pages[0])
   {
      TRACE( 1, ( "*** %s: NULL == pLock->pages[0]\n", __FUNCTION__));
      return -EINVAL;
   }
   pLock->sglist = vmalloc(sizeof(*pLock->sglist) * pLock->nr_pages);
   if (NULL == pLock->sglist)
   {
      TRACE( 1, ( "*** %s: vmalloc failed %Zd x %d\n", __FUNCTION__, sizeof(*pLock->sglist), pLock->nr_pages ));
      return -ENOMEM;
   }
   memset(pLock->sglist, 0, sizeof(*pLock->sglist) * pLock->nr_pages);

   if (NULL == pLock->pages[0])
           goto nopage;
   if (PageHighMem(pLock->pages[0]))
           /* DMA to highmem pages might not work */
           goto highmem;
#if LINUX_VERSION_CODE < 0x20618 /* < 2.6.24 */
   pLock->sglist[0].page   = pLock->pages[0];
   pLock->sglist[0].offset = pLock->offset;
   pLock->sglist[0].length = PAGE_SIZE - pLock->offset;
#else
   sg_set_page( &pLock->sglist[0], pLock->pages[0], PAGE_SIZE - pLock->offset, pLock->offset );
#endif
   for (i = 1; i < pLock->nr_pages; i++) {
      if (NULL == pLock->pages[i])
         goto nopage;
       if (PageHighMem(pLock->pages[i]))
          goto highmem;
#if LINUX_VERSION_CODE < 0x20618 /* < 2.6.24 */
      pLock->sglist[i].page   = pLock->pages[i];
      pLock->sglist[i].length = PAGE_SIZE;
#else
   sg_set_page( &pLock->sglist[i], pLock->pages[i], PAGE_SIZE, 0 );
#endif
   }
   pLock->sglist[pLock->nr_pages - 1].length = (pLock->len - (PAGE_SIZE - pLock->offset)) % PAGE_SIZE;
   if( !pLock->sglist[pLock->nr_pages - 1].length )
      pLock->sglist[pLock->nr_pages - 1].length = PAGE_SIZE;
   for (i = 0; i < pLock->nr_pages; i++) {
      TRACE( 9, ( "%s: pagelen[%d] = 0x%08X\n", __FUNCTION__, i, pLock->sglist[i].length));
   }
   return 0;

nopage:
   TRACE( 1, ( "*** %s: no pages\n", __FUNCTION__));
   vfree(pLock->sglist);
   pLock->sglist = NULL;
   return -EINVAL;

highmem:
   TRACE( 1, ( "*** %s: highmem pages\n", __FUNCTION__));
   vfree(pLock->sglist);
   pLock->sglist = NULL;
   return -EINVAL;
}

/* kernel 2.6.n */
static int CDA_dma_lock_user(struct SLock *pLock)

{
   unsigned long first,last, data;
   int err, rw = 0;

   if (pLock == NULL) return -EFAULT;

   TRACE(4, ("%s: %p/%Zd\n", __FUNCTION__, pLock->pvBuffer, pLock->len));

   switch (pLock->direction) {
   case PCI_DMA_FROMDEVICE: rw = READ;  break;
   case PCI_DMA_TODEVICE:   rw = WRITE; break;
   default:                 BUG();
   }
   data = (unsigned long)pLock->pvBuffer;
   first = (data          & PAGE_MASK) >> PAGE_SHIFT;
   last  = ((data + pLock->len - 1) & PAGE_MASK) >> PAGE_SHIFT;
   pLock->offset   = data & ~PAGE_MASK;
   pLock->nr_pages = last-first+1;
   pLock->pages = vmalloc(pLock->nr_pages * sizeof(struct page*));
   if (NULL == pLock->pages)
   {
      TRACE(1, ("%s: vmalloc failed %d * %Zd\n", __FUNCTION__, pLock->nr_pages, sizeof(struct page*)));
      return -ENOMEM;
   }

   down_read(&current->mm->mmap_sem);
   err = get_user_pages(current,
                        current->mm,
                        data & PAGE_MASK,
                        pLock->nr_pages,
                        rw == READ, 1, /* force */
                        pLock->pages, NULL);
   up_read(&current->mm->mmap_sem);
   TRACE(1, ("%s - nr_pages=%d, err=%d, data=%p/0x%ZX\n", __FUNCTION__, pLock->nr_pages, err, pLock->pvBuffer, pLock->len));
   if (err != pLock->nr_pages) {
           pLock->nr_pages = (err >= 0) ? err : 0;
           return err < 0 ? err : -EINVAL;
   }
   return 0;
}

/* kernel 2.6.n */
static int CDA_LockBuffer(CDA_SInstance* pInst, void* pTable, uiSG* pSgl, int iDma)
{
  unsigned long buffer, size, offset;
  ui32 maxentries;
  SLock* pLock;
  int iError;
  ui32 dwPage;
  struct scatterlist *sgEntry;
  uiSG dwBytesRemaining;

  TRACE( 4, ( "%s(%p,%p,%p)\n", __FUNCTION__, pInst,pTable,pSgl));
  ASSERT( NULL != pTable);

  maxentries = _CDA_SGTable_Get_AllocatedSize( pTable);
  if ( maxentries < CDA_SGTABLE_HEADER_SIZE + CDA_SGTABLE_BYTES_PER_ENTRY)
    {
    TRACE( 1, ( "*** %s: table too small\n", __FUNCTION__));
    return -EINVAL;
    }
  maxentries = (maxentries - CDA_SGTABLE_HEADER_SIZE) / CDA_SGTABLE_BYTES_PER_ENTRY;
    
  buffer = (unsigned long)_CDA_SGTable_Get_VirtualAddr( pTable);
  TRACE( 2, ( "%s: buffer = %ld\n", __FUNCTION__, buffer));
  offset = buffer % PAGE_SIZE;
  if ( 0 != (offset % 4) )
    {
    TRACE( 1, ( "*** %s: user buffer must be 4 byte aligned\n", __FUNCTION__));
    return -EINVAL;
    }

  /* Calculate address/length for first (possibly fragmented) page */
  size = _CDA_SGTable_Get_MemorySize( pTable);
  /* Allocate a lock */
  pLock = (SLock*) kmalloc( sizeof(*pLock), GFP_KERNEL);
  if ( NULL == pLock)
    {
    TRACE( 1, ( "*** %s: Out of memory for lock info\n", __FUNCTION__));
    return -ENOMEM;
    }
  memset(pLock, 0, sizeof(*pLock));
  pLock->pvBuffer = (void*)buffer;
  TRACE( 2, ("%s: pLock->pvBuffer = %p\n", __FUNCTION__, pLock->pvBuffer));
  pLock->len = size;
  if(iDma)  /* DMA buffer */
    pLock->direction = PCI_DMA_FROMDEVICE;
  else      /* RISC buffer */
    pLock->direction = PCI_DMA_TODEVICE;
  pLock->pInst = pInst;
  iError = CDA_dma_lock_user(pLock);
  if (iError)
  {
    TRACE( 1, ( "*** %s: CDA_dma_lock_user failed\n", __FUNCTION__));
    CDA_UnlockBuffer(pLock);
    kfree(pLock);
    return iError;
  }
  iError = CDA_pages_to_sg(pLock);
  if (iError)
  {
    TRACE( 1, ( "*** %s: CDA_pages_to_sg failed\n", __FUNCTION__));
    CDA_UnlockBuffer(pLock);
    kfree(pLock);
    return iError;
  }
  pLock->sglen = pci_map_sg(((SPciInstance*)(pInst->pDevice))->pDev, pLock->sglist, pLock->nr_pages, pLock->direction);
  if (0 == pLock->sglen)
  {
    TRACE( 1, ( "*** %s: pci_map_sg failed\n", __FUNCTION__));
    CDA_UnlockBuffer(pLock);
    kfree(pLock);
    return -EFAULT;
  }
#if defined DEBUG
  TRACE( 2, ("%s: pci_map_sg returned %d\n",__FUNCTION__, pLock->sglen));
  {
     int i;
     sgEntry = pLock->sglist;
     for (i = 0; i < pLock->sglen; i++) {
       TRACE( 4, ( "%s: PCI Bus Mapped address = 0x%0X/%08X\n", __FUNCTION__, cpu_to_le32(sg_dma_address(sgEntry)), sg_dma_len(sgEntry))); 
       sgEntry++;
     }
  }
#endif

  sgEntry = pLock->sglist;
  dwBytesRemaining = pLock->len;
  for (dwPage = 0 ; dwPage < (ui32)pLock->sglen; dwPage++)
  {
     uiSG dwBytesThisPage = sg_dma_len(sgEntry);
     uiSG dwAddress;

     if (dwBytesThisPage > dwBytesRemaining) dwBytesThisPage = dwBytesRemaining;
     dwAddress = sg_dma_address(sgEntry);
     copy_to_user_ret(pSgl, &dwAddress, sizeof(dwAddress), -EFAULT );
     pSgl++;
     copy_to_user_ret(pSgl, &dwBytesThisPage, sizeof(dwBytesThisPage), -EFAULT );
     pSgl++;
     dwBytesRemaining -= dwBytesThisPage;
     sgEntry++;
  }
  _CDA_SGTable_Set_NumEntries( pTable, pLock->sglen );

  /* Add to list of pending locks */
  pLock->pPrev = NULL;
  pLock->pNext = pInst->pLockHead;
  if ( NULL != pLock->pNext)
  {
     ASSERT( NULL == pLock->pNext->pPrev);
     pLock->pNext->pPrev = pLock;
  }
  pInst->pLockHead = pLock;
  return 0;
}  

/* kernel 2.6.n */
static int CDA_FlushBuffer(SLock *pLock)
{
   if (pLock == NULL) return -EFAULT;
   TRACE( 4, ( "%s(%p,%Zd,%p)\n", __FUNCTION__, pLock->pInst,pLock->len,pLock->sglist));
   if (pLock->sglist)
   {
     if( PCI_DMA_FROMDEVICE == pLock->direction)  /* DMA buffer */
       pci_dma_sync_sg_for_cpu(   ((SPciInstance*)(pLock->pInst->pDevice))->pDev, pLock->sglist, pLock->nr_pages, pLock->direction);
     else      /* PCI_DMA_TODEVICE : RISC buffer */
       pci_dma_sync_sg_for_device(((SPciInstance*)(pLock->pInst->pDevice))->pDev, pLock->sglist, pLock->nr_pages, pLock->direction);
   }
   return 0;
}  

/* kernel 2.6.n */
static int CDA_UnlockBuffer(SLock *pLock)
{
   if (pLock == NULL) return -EFAULT;
   TRACE( 4, ( "%s(%p,%Zd,%p)\n", __FUNCTION__, pLock->pInst,pLock->len,pLock->sglist));
   if (pLock->sglist)
   {
     if (pLock->sglen)
     {
        pci_unmap_sg(((SPciInstance*)(pLock->pInst->pDevice))->pDev, pLock->sglist, pLock->nr_pages, pLock->direction);
        pLock->sglen = 0;
     }

      vfree(pLock->sglist);
      pLock->sglist = NULL;
   }
   if (pLock->pages)
   {
      int iPage;
      for (iPage=0; iPage < pLock->nr_pages; iPage++)
      {
         page_cache_release(pLock->pages[iPage]);
      }
      vfree(pLock->pages);
      pLock->pages = NULL;
      pLock->nr_pages = 0;
   }
   return 0;
}  

#else
/* kernel 2.4.n */
static int CDA_LockBuffer(
  CDA_SInstance* pInst,
  void* pTable,
  uiSG* pSgl, /* !!! -> USER SPACE */
  int   iDma
) {
  unsigned long buffer, size, offset, addr, len;
  ui32 maxentries;
  SLock* pLock;
  int iError;
  int iIndex = 1;

  TRACE( 9, ( "%s(%p,%p,%p)\n", __FUNCTION__, pInst,pTable,pSgl));
  ASSERT( NULL != pTable);

  (void)iDma;
  
  maxentries = _CDA_SGTable_Get_AllocatedSize( pTable);
  if ( maxentries < CDA_SGTABLE_HEADER_SIZE + CDA_SGTABLE_BYTES_PER_ENTRY)
    {
    TRACE( 1, ( "*** %s: table too small\n", __FUNCTION__));
    return -EINVAL;
    }
  maxentries = (maxentries - CDA_SGTABLE_HEADER_SIZE) / CDA_SGTABLE_BYTES_PER_ENTRY;
    
  buffer = (unsigned long)_CDA_SGTable_Get_VirtualAddr( pTable);
  TRACE( 2, ( "%s: buffer = %ld\n", __FUNCTION__, buffer));
  offset = buffer % PAGE_SIZE;
  if ( 0 != (offset % 4) )
    {
    TRACE( 1, ( "*** %s: user buffer must be 4 byte aligned\n", __FUNCTION__));
    return -EINVAL;
    }

  /* Calculate address/length for first (possibly fragmented) page */
  size = _CDA_SGTable_Get_MemorySize( pTable);
  /* Allocate a lock */
  pLock = (SLock*) kmalloc( sizeof(*pLock), GFP_KERNEL);
  if ( NULL == pLock)
    {
    TRACE( 1, ( "*** %s: Out of memory for lock info\n", __FUNCTION__));
    return -ENOMEM;
    }
  memset(pLock, 0, sizeof(*pLock));
  pLock->pvBuffer = (void*)buffer;
  TRACE( 2, ("%s: pLock->pvBuffer = %p\n", __FUNCTION__, pLock->pvBuffer));
  pLock->len = size;
  TRACE(2, ("%s Locking 0x%08lx bytes at %p\n", __FUNCTION__, size, pLock->pvBuffer));
   /* Allocate a temporary struct kiobuf */
   iError = alloc_kiovec(1, &pLock->pkiobuf);
   if (iError)
   {
      TRACE(1, ("%s: alloc_kiovec failed with error %d\n", __FUNCTION__, iError));
      kfree(pLock);
      return iError;
   }
   TRACE( 2, ("%s allocated kiovec OK\n", __FUNCTION__));
   /* Map and lock the buffer */
   iError = map_user_kiobuf(READ, pLock->pkiobuf, (unsigned long)pLock->pvBuffer, pLock->len);
   if (iError)
   {
      TRACE(1, ("%s: map_user_kiobuf failed with error %d\n", __FUNCTION__, iError));
      kfree(pLock);
      return iError;
   }
   TRACE( 2, ("%s map_user_kiobuf mapped %d pages\n", __FUNCTION__, pLock->pkiobuf->nr_pages));
   /* Build the scatter gather list */
   /* First page may be incomplete ! */
   addr = page_to_bus(pLock->pkiobuf->maplist[0]);
   addr += pLock->pkiobuf->offset;
   len = PAGE_SIZE - pLock->pkiobuf->offset;
   if (len > size) len = size;
   PUT_ADDR_LEN_RET( pSgl, addr, len, -EFAULT);
   pSgl += 2;
   size -= len;
   /* Now the whole pages */
   iIndex = 1;
   while(size >= PAGE_SIZE)
   {
      addr = page_to_bus(pLock->pkiobuf->maplist[iIndex]);
      len = PAGE_SIZE;
      PUT_ADDR_LEN_RET( pSgl, addr, len, -EFAULT);
      pSgl += 2;
      size -= len;
      iIndex++;
   }
   if (size > 0) /* There is a fragment of a page at the end */
   {
      addr = page_to_bus(pLock->pkiobuf->maplist[iIndex]);
      len = size;
      PUT_ADDR_LEN_RET( pSgl, addr, len, -EFAULT);
      iIndex++;
   }
  /* Add to list of pending locks */
  pLock->pPrev = NULL;
  pLock->pNext = pInst->pLockHead;
  if ( NULL != pLock->pNext)
    {
    ASSERT( NULL == pLock->pNext->pPrev);
    pLock->pNext->pPrev = pLock;
    }

  pInst->pLockHead = pLock;

  _CDA_SGTable_Set_NumEntries( pTable, iIndex );

  return 0;
  }

/*
 * Translate a user space virtual address to a page
 */
/* kernel 2.4.n */
static inline struct page * uvirt_to_page(
  unsigned long adr     /* User virtual address */
) {
  struct page *pPage = NULL;
  pgd_t * pgd;

  pgd = pgd_offset( current->mm, adr);
  if ( !pgd_none( *pgd))
    {
    /* Get the PageMidlevelDirectory */
    pmd_t* pmd = pmd_offset( pgd, adr);
    if ( !pmd_none( *pmd))
      {
      /* Get the PageTableEntry */
      pte_t* ptep = pte_offset( pmd, adr);
      pte_t pte = *ptep;
      pPage = pte_page(pte);
      }
    }
  if (pPage == NULL)
    {
    TRACE(0, ("uvirt_to_page(0x%08lx) returning NULL!\n", adr));
    }
  return pPage;
  }

/* kernel2.4.n */
static int CDA_UnlockBuffer(SLock *pLock)
{
   if (pLock == NULL) return -EFAULT;
   if (pLock->pkiobuf == NULL) return -EFAULT;
   unmap_kiobuf(pLock->pkiobuf);
   free_kiovec(1, &pLock->pkiobuf);
   pLock->pkiobuf = NULL;
   return 0;
}
#endif

/*
 * Get Device Number
 */
  int CDA_GetDeviceNum(CDA_SInstance* pInst){
    return pInst->devicenum;
  }

/*
 * Overrides for Emacs so that we follow the author's tabbing style.
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
