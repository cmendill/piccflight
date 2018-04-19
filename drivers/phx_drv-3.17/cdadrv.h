/****************************************************************************
 *
 * ACTIVE SILICON LIMITED
 *
 * File name   : cdadrv.h
 * Function    : Linux CDA driver kernel mode interface
 * Project     : AP04
 * Authors     : Lawrence Rust
 * Systems     : ANSI C.
 * Version     : See cdadrv.h, "#define CDA_DRV_VERSION".
 * Release Date: See cdadrv.h, "#define CDA_DRV_VERSION".
 *
 * Copyright (c) 1997-2006 Active Silicon Ltd.
 ****************************************************************************
 *
 * Comments:
 * --------
 * This file contains the CDA Linux driver kernel interface.
 *
 ****************************************************************************
 */

#ifndef CDADRV_H
#define CDADRV_H

/* Dependencies */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef __KERNEL__
#error kernel mode only
#endif


/*
 * Constants
 */

/* CDA Driver Version
 * ------------------
 * This is the version number of the CDA driver.
 * Note this driver is installed as device lfg0,1,2 for LFG cards,
 * phx0,1,2 for Phoenix cards etc. Note version is NOT BCD! - see 
 * example below.
#define CDA_DRV_VERSION ((unsigned long)(0x0002001E)) = v2.30
 */
#if defined _LFG_LINUX
#define CDA_DRV_VERSION ((unsigned long)(0x0002001E)) 
#elif defined _PHX_LINUX
#define CDA_DRV_VERSION ((unsigned long)(0x00030011)) 
#else
#error Must define for LFG or Phoenix
#endif

/*
 * Useful macros
 */
#ifndef CONTAINING_RECORD
#define CONTAINING_RECORD( _a, _t, _f) ((_t*)( (char*)(_a) - (char*)(&((_t*)0)->_f) ))
#endif

#ifndef lengthof
#define lengthof( _a) (sizeof( _a) / sizeof( (_a)[0]) )
#endif

/* CDA lib compatible types */
typedef u_int64_t ui64;
typedef u_int32_t ui32;
typedef u_int16_t ui16;
typedef u_int8_t ui8;

/* Forward ref'd types */
typedef struct CDA_SDevice CDA_SDevice; /* Opaque device instance data */
typedef struct CDA_SInstance CDA_SInstance; /* Opaque device instance data */

struct CDA_SIoctlDeviceInfo; /* in linuxdrv.h */
struct file;  /* in <linux/fs.h> */
struct inode; /* in <linux/fs.h> */


/* Device specific methods */
typedef void CDA_FProbe( void);
typedef int CDA_FFinal( CDA_SDevice*);
typedef int CDA_FGetInfo( CDA_SDevice*, struct CDA_SIoctlDeviceInfo*);
typedef int CDA_FStart( CDA_SDevice*, CDA_SInstance*, const char*);
typedef int CDA_FStop( CDA_SDevice*);
typedef int CDA_FOpen( CDA_SDevice*, struct inode *, struct file *, int);
typedef int CDA_FClose( CDA_SDevice*, struct inode *, struct file *, int);
typedef int CDA_FRead( CDA_SDevice*, struct file*, ui32*, uint32_t offset, int bits);
typedef int CDA_FWrite( CDA_SDevice*, struct file*, ui32 data, ui32 offset, int bits);
typedef int CDA_FReconfig( CDA_SDevice*);
typedef int CDA_FPutEvent( CDA_SDevice*, CDA_SIoctlEvent*);

/* Device function table */
typedef struct CDA_SVtable
  {
  CDA_FStart*   pfnStart;               /* Required - allocate hardware */
  CDA_FStop*    pfnStop;                /* Required - release hardware */
  CDA_FGetInfo* pfnGetInfo;             /* Required - get device info */
  CDA_FOpen*    pfnOpen;                /* Maybe 0 */
  CDA_FClose*   pfnClose;               /* Maybe 0 */
  CDA_FRead*    pfnRead;                /* Required */
  CDA_FWrite*   pfnWrite;               /* Required */
  CDA_FFinal*   pfnFinal;               /* Maybe 0 - device being destoyed */
  CDA_FReconfig* pfnReconfig;           /* Maybe 0 - restore device config */
  CDA_FPutEvent* pfnPutEvent;           /* Maybe 0 - put an event          */
  } CDA_SVtable;



/* Device event */
extern void CDA_DeviceEvent(
  CDA_SInstance*,                       /* As passed to CDA_FStart */
  ui32 ev,
  ui32 data
);

/* Device registration */
extern int CDA_RegisterDevice(
  const CDA_SVtable*,                   /* -> function table */
  CDA_SDevice*                          /* Device private context */
);


/* Add new card probes here & to CDA_pfnProbeArray below */
extern CDA_FProbe LFG_Probe;                 /* LFG Bt848 func 0       */
extern CDA_FProbe LFGAUDIO_Probe;            /* LFG Bt848 func 1       */
extern CDA_FProbe PHD_Probe_AsPhxGeneric;    /* "Generic"      Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD10PE1;     /* D10HDSDI PE1   Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD10PE4;     /* D10HDSDI PE4   Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD20PE1;     /* D20HDSDI PE1   Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD20PE4;     /* D20HDSDI PE4   Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD24P32;     /* D24CL   PCI32  Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD24PE1;     /* D24CL   PE1    Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD32P32;     /* D32     PCI32  Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD36P32;     /* D36     PCI32  Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD36P64;     /* D36     PCI64  Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD36P64U;    /* D36     PCI64U Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD36PE1;     /* D36     PE1    Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD48P32;     /* D48CL   PCI32  Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD48P64;     /* D48CL   PCI64  Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD48P64U;    /* D48CL   PCI64U Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD48PE1;     /* D48CL   PE1    Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD48PE4;     /* D48CL   PE4    Phoenix */
extern CDA_FProbe PHD_Probe_AsPhxD64PE4;     /* D64CL   PE4    Phoenix */

#ifdef __cplusplus
}
#endif

#endif /* ndef CDADRV_H */
/* End of file */
