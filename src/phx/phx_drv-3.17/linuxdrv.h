/****************************************************************************
 *
 * ACTIVE SILICON LIMITED
 *
 * File name   : linuxdrv.h
 * Function    : Linux driver external interface for CDA library.
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
 * This file contains the CDA Linux driver external interface.
 *
 ****************************************************************************
 */

#ifndef LINUXDRV_H
#define LINUXDRV_H

/* Dependencies */
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __KERNEL__
#else
#include <sys/types.h>
#include <sys/ioctl.h>
#endif

/* The base name of the device for open */
#if defined _PHX_LINUX
  #define CDA_BASE_NAME "phx" /* + 0..n */
  #define CDA_MAJOR_NUM 243

#elif defined _LFG_LINUX
  #define CDA_BASE_NAME "lfg" /* + 0..n */
  #define CDA_MAJOR_NUM 242

#else
  #error You have a missing target environment directive ( e.g. _PHX_LINUX )

#endif

#if defined _LP64
#if !defined _CDA_LP64
#define _CDA_LP64
#endif
#endif

/* IOCTL major code.  This should be unique for error checking but isn't critical */ 
#define CDA_IOCTL_CODE 0xE0  /* See kernel/Documentation/ioctl-number.txt */


/* Get device info */
enum CDA_EDeviceType { CDA_kDevicePci = 1, CDA_kDeviceIsa };
typedef struct CDA_SIoctlDeviceInfo
  {
  u_int32_t dwDriverRev;        /* b0..15 minor, b16..31 major */
  int16_t type;                 /* Bus type: CDA_kDevice... */
  u_int16_t wDeviceID;
  u_int16_t wVendorID;
  u_int16_t wPciSubVendorID;
  u_int16_t wPciSubSystemID;
  u_int16_t wFunctionNumber;     /* PCI functional unit */
  u_int32_t dwBase;             /* Bus logical base address */
  u_int32_t dwInterrupt;
  } CDA_SIoctlDeviceInfo;
#define CDA_IOCTL_DEVICEINFO _IOR( CDA_IOCTL_CODE, 0, CDA_SIoctlDeviceInfo *)


/* Register list */
typedef struct CDA_SIoctlRegList
  {
  u_int32_t size;               /* IN: size in bytes */
  void* pList;                  /* IN -> register list (read/write), see cda_lib.h for accessor macros */
  } CDA_SIoctlRegList;
#define CDA_IOCTL_REGLIST _IOWR( CDA_IOCTL_CODE, 1, CDA_SIoctlRegList *)


/* DMA memory locking */
/* Paramater -> SGTable, see cda_buf.h for accessor macros */
#define CDA_IOCTL_MEMORY_LOCK      _IOWR( CDA_IOCTL_CODE,   2, void *)
#define CDA_IOCTL_MEMORY_LOCK_RISC _IOWR( CDA_IOCTL_CODE, 0xA, void *)
#define CDA_IOCTL_MEMORY_LOCK_DMA  _IOWR( CDA_IOCTL_CODE, 0xB, void *)
#define CDA_IOCTL_MEMORY_UNLOCK    _IOW(  CDA_IOCTL_CODE,   3, void *)

/* Events */
typedef struct CDA_SIoctlEvent
  {
  u_int32_t event;              /* OUT: Device specific event ID */
  u_int32_t data;               /* OUT: Event specific data */
  } CDA_SIoctlEvent;
#define CDA_IOCTL_EVENT _IOR( CDA_IOCTL_CODE, 4, CDA_SIoctlEvent *)

/* Restore PCI config registers to state when driver loaded */
#define CDA_IOCTL_CONFIG_RESTORE _IOW( CDA_IOCTL_CODE, 5, void *)

/* Claim a device exclusively */
#define CDA_IOCTL_CLAIM _IOW( CDA_IOCTL_CODE, 6, void *)

/* Insert an event into the queue */
#define CDA_IOCTL_EVENT_PUT _IOW( CDA_IOCTL_CODE, 7, void *)

/* Memory flush/sync */
#define CDA_IOCTL_MEMORY_FLUSH_RISC _IOWR( CDA_IOCTL_CODE, 0xC, void *)
#define CDA_IOCTL_MEMORY_FLUSH_DMA  _IOWR( CDA_IOCTL_CODE, 0xD, void *)

#ifdef __cplusplus
}
#endif

#endif /* ndef LINUXDRV_H */
/* End of file */
