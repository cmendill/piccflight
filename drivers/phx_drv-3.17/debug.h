/****************************************************************************
 *                                                                          *
 * debug.h                                                                  *
 *                                                                          *
 * Generic debug output                                                     *
 *                                                                          *
 * Copyright (c)1996-2004 Software Systems Consultants.                     *
 * www.softsystem.co.uk.
 *__________________________________________________________________________*
 *                                                                          *
 * Revision History:                                                        *
 *                                                                          *
 * No.   Date     By   Reason                                               *
 *--------------------------------------------------------------------------*
 * 100 10 Oct 97  lvr  Created                                              *
 * 101 24 Nov 97  lvr  CW Pro2 compatible                                   *
 * 102 25 Apr 98  lvr  CWPro3 __declspec fix                                *
 * 103 04 Aug 98  lvr  Add ATMOS & WIN16 options                            *
 * 104 11 Sep 98  lvr  Made _TRACE independent of _DEBUG                    *
 * 105 25 Sep 98  lvr  Mac version always defines _DebuggerPresent          *
 * 106 22 Oct 98  lvr  Add IsDebuggerPresent for WIN32                      *
 * 107 16 Nov 98  lvr  Mac version only defines ALERT_DEFINE_MESSAGEBOX iff *
 *                     ALERT_RESOURCE is defined.                           *
 *                     Only define ALERT_DEFINE_HANDLER if was undefined.   *
 * 108 05 Mar 99  lvr  Added WINAPI to IsDebuggerPresent prototype          *
 * 109 02 Apr 99  lvr  Added #pragma warning( push/pop ) around windows.h   *
 * 110 20 Apr 99  lvr  NT4 DDK compatibility                                *
 * 111 04 Nov 99  lvr  Added Added ANSI_strict pragma for MW windows.h      *
 * 112 17 Apr 00  lvr  Added GCC 2.2.95 & Linux                             *
 * 113 18 Apr 00  lvr  Rework COMPILATION_REQUIRES for GCC                  *
 * 114 01 May 00  lvr  Add Linux kernel mode                                *
 * 115 05 May 00  lvr  Win2K DDK compatibility                              *
 * 116 15 Sep 00  lvr  Make Unicode compatible                              *
 * 117 17 Oct 00  lvr  Fix DEBUG_TRACE_FN in NDEBUG builds to prevent linkage*
 * 118 17 Jan 00  lvr  Mod COMPILATION_REQUIRES to use typedef. Win31 mods  *
 *__________________________________________________________________________*/
#ifndef _debug_h
#define _debug_h 0x118                  /* Version no. BCD */

#if defined DEBUG && defined NDEBUG
#error !!!!Cannot define DEBUG and NDEBUG in the same build!!!!!
#endif

/* Dependencies */
#if defined macintosh
  #include <Types.h>                    /* SysDebugStr */
  #include <Errors.h>
  #if OLDROUTINELOCATIONS
    #include <Strings.h>
  #else
    #include <TextUtils.h>
  #endif

  #ifdef ALERT_RESOURCE
    #include <Dialogs.h>                /* StopAlert & ParamText */
  #endif

  #include <stdlib.h>
  #include <stdio.h>                    /* vsprintf */
  #include <string.h>                   /* strcat */

  #ifndef DSDCL_NONUB
    #include "MetroNubUtils.h"
  #endif
#elif defined _NTDDK_
  /* NT DDK */
  #include <stdio.h>                    /* vsprintf */
#elif defined _WIN32 || defined WIN32 || defined _WINDOWS
  /* Turn off warnings in windows.h */
  #if defined __MWERKS__
    #pragma ANSI_strict off
  #elif defined _MSC_VER
   /* Disable VC level 4 warnings in windows.h */
   #if _MSC_VER >= 1200 /* VC++ 6.0 & later */
    #pragma warning( push )
   #endif
   #pragma warning( disable : 4201)    /* nonstandard extension used : nameless struct/union */
   #pragma warning( disable : 4214)    /* nonstandard extension used : bit field types other than int */
   #pragma warning( disable : 4514)    /* unreferenced inline function has been removed */
  #endif
  #ifndef WIN32_LEAN_AND_MEAN
   #define WIN32_LEAN_AND_MEAN
  #endif
  
  #include <windows.h>
  
  #if defined __MWERKS__
    #pragma ANSI_strict reset
  #elif defined _MSC_VER
   #if _MSC_VER >= 1200 /* VC++ 6.0 & later */
    #pragma warning( pop )
    #pragma warning( disable : 4514)   /* unreferenced inline function has been removed */
   #else
    #pragma warning ( default : 4201)
    #pragma warning ( default : 4214)
   #endif
  #endif
  #include <stdlib.h>
  #include <stdio.h>                    /* vsprintf for floating point %f formats */
  #if defined _MSC_VER
   #pragma warning ( disable : 4127)    /* conditional expression is constant */
  #endif
#elif defined _DOS16 || defined _DOS
  #include <dos.h>                      /* geninterrupt */
  #include <stdlib.h>
  #include <stdio.h>                    /* vsprintf for floating point %f formats */
  #if defined _MSC_VER
   #pragma warning ( disable : 4127)    /* conditional expression is constant */
  #endif
#elif defined __atmos__
  #include "config.h"
  #include "atypes.h"
  #ifdef QUICKTURN
   #include "hesalibc.h"                /* vsprintf */
   #define kprintf printf
  #elif !defined TEST_MONITOR
   #include <stdlib.h>
   #include <stdio.h>                   /* vsprintf */
   #include <kernel.h>                  /* kprintf */
  #endif
#elif defined LINUX && defined __KERNEL__
  #include <linux/kernel.h>
#elif defined LINUX_VERSION_CODE && defined KERNEL_VERSION && (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
  #include <linux/kernel.h>
#else
  /* Assume unix */
  #include <stdlib.h>
  #include <stdio.h>                   /* vsprintf */
#endif

#include <stdarg.h>


#ifdef __cplusplus
extern "C" {
#endif

/* Library export attributes
 * DEBUG_EXPORT(x) definition of exported data & functions
 * DEBUG_EXTERN(x) declaration of extern data & functions
 */
#ifndef DEBUG_EXPORT
  #if defined _WIN32 || defined WIN32
    #ifdef __MWERKS__
      #define DEBUG_EXPORT(_f) __declspec( dllexport) _f
      #define DEBUG_EXTERN(_f) extern _f
    #elif defined __GNUC__
      #define DEBUG_EXPORT(_f) _f __attribute__((__dllexport__))
      #define DEBUG_EXTERN(_f) extern _f
    #else
      /* MSVC style */
      #define DEBUG_EXPORT(_f) __declspec( dllexport) _f
      #define DEBUG_EXTERN(_f) extern __declspec( dllexport) _f
    #endif
  #elif defined _WINDOWS
    #ifdef _DLL
      #define DEBUG_EXPORT(_f) __export _f
    #else
      #define DEBUG_EXPORT(_f) _f
    #endif
    #define DEBUG_EXTERN(_f) extern _f
  #elif defined macintosh
    #if __MWERKS__ >= 0x2100
      #define DEBUG_EXPORT(_f) __declspec( dllexport) _f
    #else
      #define DEBUG_EXPORT(_f) __declspec( export) _f
    #endif
    #define DEBUG_EXTERN(_f) extern _f
  #elif defined __linux__
    #define DEBUG_EXPORT(_f) _f
    #define DEBUG_EXTERN(_f) extern _f
  #elif defined __GNUC__
    #define DEBUG_EXPORT(_f) _f __attribute__((__dllexport__))
    #define DEBUG_EXTERN(_f) extern _f
  #else
    #define DEBUG_EXPORT(_f) _f
    #define DEBUG_EXTERN(_f) extern _f
  #endif
#endif


/* Expand and stringize a macro name */
#undef _STRING
#undef _STR
#define _STR(s) #s
#define _STRING(s) _STR(s)

/* pragma warnings */
/*#define WARN(s) message( "*** WARNING " _STRING( __FILE__) " line " _STRING(__LINE__) ": " s)*/

/* Compile time assertion
 * Generates 'invalid array dimension' compiler error if condition false
 */
#undef COMPILATION_REQUIRES
#ifdef __GNUC__
 #define COMPILATION_REQUIRES(e) extern char _dummy[(e) ? 1 : -1] __attribute__((unused))
#elif 0 /* Mutiple use of this form can cause problems for some compilers */
 #define COMPILATION_REQUIRES(e) typedef char _dummy[(e) ? 1 : -1]
#else
 #define COMPILATION_REQUIRES(e) extern char _dummy[(e) ? 1 : -1]
#endif


/* Debugger break */
#undef BREAKPOINT
#if !defined NDEBUG || defined DBG
  #if defined macintosh
    #define BREAKPOINT( _sz) \
      (_DebuggerPresent() ? DebugStr( "\p" _sz) : SysError( dsChkErr))
  #elif defined _NTDDK_
    #pragma warning( disable : 4127)    /* conditional expression is constant */
    #define BREAKPOINT( _sz) \
      do { DbgPrint( _sz); DbgBreakPoint(); } while( 0)
  #elif defined _WIN32 || defined WIN32
    #define BREAKPOINT( _sz) \
      do { OutputDebugStringA( _sz "\n"); if ( IsDebuggerPresent()) DebugBreak(); } while( 0)
  #elif defined _WINDOWS
    #define BREAKPOINT( _sz) \
      do { OutputDebugString( _sz "\n"); if ( IsDebuggerPresent()) DebugBreak(); } while( 0)
  #elif defined _DOS
    #define BREAKPOINT( _sz) \
      do { _TRACE( _sz "\n"); _asm int 3 } while( 0)
  #elif defined _DOS16
    #define BREAKPOINT( _sz) \
      do { _TRACE( _sz "\n"); geninterrupt( 3); } while( 0)
  #elif defined __atmos__
    #pragma WARN( "BREAKPOINT not implemented")
    #define BREAKPOINT( _sz) ((void)0)
  #elif defined __linux__
#if defined _CDA_LITTLE_ENDIAN
    #define BREAKPOINT( _sz) \
      do { _TRACE( _sz "\n"); __asm__ ("int %0" : : "i" (3)); } while( 0)
#else
    #define BREAKPOINT( _sz) ((void)0)
#endif
  #else
    #define BREAKPOINT( _sz) ((void)0)
  #endif
#else
  #define BREAKPOINT( _sz) ((void)0)
#endif


/* Trace output */
typedef void DEBUG_FTrace( const char*);
#ifndef DEBUG_TRACE_FN
 #if defined DEBUG || defined _DEBUG || defined DBG || defined DEBUG_TRACE_REQD
  #define DEBUG_TRACE_FN g_pfnTrace
  DEBUG_EXTERN( DEBUG_FTrace* DEBUG_TRACE_FN);
 #else
  #define DEBUG_TRACE_FN ((DEBUG_FTrace*)0)
 #endif
#endif

#ifdef macintosh
  static inline int _DebuggerPresent( void)
    { return 0 != (0x20 & (*(unsigned char*)0xbff))
  #ifndef DSDCL_NONUB
      | AmIBeingMWDebugged()
  #endif
      ; }
  #define _TRACE _trace
  static inline void _trace( const char* psz, ...)
  { char buf[256]; va_list va; va_start( va, psz);
    vsprintf( buf, psz, va);
    if ( DEBUG_TRACE_FN)
      DEBUG_TRACE_FN( buf);
    else if ( _DebuggerPresent())
      DebugStr( c2pstr( strcat( buf, ";g")));
    va_end( va); }
#elif defined _NTDDK_
  #ifndef _TRACE
    #define _TRACE DbgPrint
  #else
  static __inline void _trace( const char* psz, ...)
  { char buf[256]; va_list va; va_start( va, psz);
     vsprintf( buf, psz, va);
    if ( DEBUG_TRACE_FN)
      DEBUG_TRACE_FN( buf);
    else
      KdPrint(( buf));
    va_end( va); }
  #endif
  extern int GetDebugLevel(
    IN PUNICODE_STRING pRegistryPath /* -> Unicode registry path */
  );
#elif defined _WIN32 || defined WIN32
  #if (_WIN32_WINNT >= 0x0400) || (_WIN32_WINDOWS > 0x0400)
   extern WINBASEAPI BOOL WINAPI IsDebuggerPresent(VOID); /* winbase.h, kernel32.lib */
  #else
   #define IsDebuggerPresent() 1
  #endif
  #if defined _MSC_VER
   #pragma warning ( disable : 4710)    /* function not expanded */
  #endif
  #define _TRACE _trace
  static __inline void _trace( const char* psz, ...)
  { char buf[256]; va_list va; va_start( va, psz);
     vsprintf( buf, psz, va); /* NB don't use wsprintf cos that doesn't support FP */
    if ( DEBUG_TRACE_FN)
      DEBUG_TRACE_FN( buf);
    else
      OutputDebugStringA( buf);
    va_end( va); }
#elif defined _WINDOWS
  #define IsDebuggerPresent() 1
  #define _TRACE _trace
  static __inline void _trace( const char* psz, ...)
  { char buf[256]; va_list va; va_start( va, psz);
     vsprintf( buf, psz, va); /* NB don't use wsprintf cos that doesn't support FP */
    if ( DEBUG_TRACE_FN)
      DEBUG_TRACE_FN( buf);
    else
      OutputDebugString( buf);
    va_end( va); }
#elif defined _DOS16 || defined _DOS
  #define _TRACE _trace
  static void _trace( const char* psz, ...)
  { char buf[256]; va_list va; va_start( va, psz);
     vsprintf( buf, psz, va);
    if ( DEBUG_TRACE_FN)
      DEBUG_TRACE_FN( buf);
    else
      fputs( buf, stderr);
    va_end( va); }
#elif defined __atmos__
  #define _TRACE kprintf
#elif defined LINUX && defined __KERNEL__ 
  #define _TRACE printk
#elif defined LINUX_VERSION_CODE && defined KERNEL_VERSION && (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
  #define _TRACE printk
#elif defined __linux__
  #define _TRACE _trace
  static __inline__ void _trace( const char* psz, ...)
  { char buf[256]; va_list va; va_start( va, psz);
     vsprintf( buf, psz, va);
    if ( DEBUG_TRACE_FN)
      DEBUG_TRACE_FN( buf);
    else
      fputs( buf, stderr);
    va_end( va); }
#else
  /* Assume unix */
  #define _TRACE printf
#endif

#if defined DEBUG || defined _DEBUG || defined DBG
  /* Debugging enabled */
  #ifndef DEBUG_TRACE_LEVEL
    #define DEBUG_TRACE_LEVEL g_iTraceLevel
  #endif
  DEBUG_EXTERN( int DEBUG_TRACE_LEVEL);

  #define DEBUG_TRACE( _l, _f) \
    DEBUG_EXPORT( int DEBUG_TRACE_LEVEL) = _l; \
    DEBUG_EXPORT( DEBUG_FTrace* DEBUG_TRACE_FN) = _f;

  #define DEBUG_SET_TRACE_LEVEL( _l) \
    DEBUG_TRACE_LEVEL = _l

  #define DEBUG_SET_TRACE_FN( _f) \
    DEBUG_TRACE_FN = _f

  #undef TRACE
  #define TRACE( _l, _s) do { if ( DEBUG_TRACE_LEVEL >= (_l)) _TRACE _s; } while(0)
#else
  /* Debugging disabled */
  #undef TRACE
  #define TRACE( _l, _s) ((void)0)
  #if defined _MSC_VER && _MSC_VER <= 800 /* VC++ 1.5 & earlier */
   #pragma warning( disable : 4705)   /* statement has no effect */
  #endif
  #if defined DEBUG_TRACE_REQD
    #define DEBUG_TRACE( _l, _f) \
      DEBUG_EXPORT DEBUG_FTrace* DEBUG_TRACE_FN = _f;
  #else
    #define DEBUG_TRACE( _l, _f)
  #endif
  #define DEBUG_SET_TRACE_LEVEL( _l) ((void)0)
  #define DEBUG_SET_TRACE_FN( _f) ((void)0)
#endif


/* Assertions */
#if !defined _NTDDK_
 #undef ASSERT
 #if !defined NDEBUG || defined DBG
  #define ASSERT( _e) do { if ( !(_e) ) \
    ALERT( "Assertion failed: " #_e \
      " in " __FILE__ " line " _STRING( __LINE__)); } while(0)
 #else
  #define ASSERT( _e) ((void)0)
  #if defined _MSC_VER && _MSC_VER <= 800 /* VC++ 1.5 & earlier */
   #pragma warning( disable : 4705)   /* statement has no effect */
  #endif
 #endif
#endif

#undef VERIFY
#if !defined NDEBUG || defined DBG
  #define VERIFY( _e) ASSERT( _e)
#else
  #define VERIFY( _e) ((void)(_e))
#endif


/* Alerts */
#undef ALERT
#if !defined NDEBUG || defined DBG
  #define ALERT_ABORT 3  /* IDABORT */
  #define ALERT_RETRY 4  /* IDRETRY */
  #define ALERT_IGNORE 5 /* IDIGNORE */

  #ifndef ALERT_FN
   #define ALERT_FN g_pfnMessageBox
  #endif

  #ifndef ALERT_DEFINE_HANDLER
   #define ALERT_DEFINE_HANDLER( _f) DEBUG_EXPORT( ALERT_FMessageBox* ALERT_FN) = _f;
  #endif
  #ifndef ALERT_SET_HANDLER
   #define ALERT_SET_HANDLER( _f) ALERT_FN = _f
  #endif

  #ifdef macintosh
    /* Simple function to display a message box */
    #ifdef ALERT_RESOURCE
      #define ALERT_DEFINE_MESSAGEBOX( _f) static short _f( ConstStr255Param ps) { \
        ParamText( ps, "\p", "\p", "\p" ); \
        switch ( StopAlert( ALERT_RESOURCE, NULL)) \
          { case 1: return ALERT_ABORT; case 3: return ALERT_IGNORE; } \
        return ALERT_RETRY; \
      }
    #endif
    typedef short ALERT_FMessageBox( ConstStr255Param);
    #ifndef ALERT_MESSAGE
      #define ALERT_MESSAGE( _sz) (ALERT_FN ? ALERT_FN( "\p" _sz) : ALERT_RETRY)
    #endif
  #elif defined _NTDDK_
    typedef short ALERT_FMessageBox( const char*);
    #ifndef ALERT_MESSAGE
      #define ALERT_MESSAGE( _sz) (ALERT_FN ? ALERT_FN( _sz) : ALERT_RETRY)
    #endif
    #define ALERT_TERMINATE() DbgBreakPoint()
  #elif defined _WIN32 || defined WIN32
    typedef short ALERT_FMessageBox( const char*);
    #if defined _MSC_VER
     #pragma warning ( disable : 4702)    /* unreachable code */
    #endif
    #ifndef ALERT_MESSAGE
      #define ALERT_MESSAGE( _sz) (ALERT_FN ? ALERT_FN( _sz) : MessageBoxA( 0, _sz, "Alert", MB_ABORTRETRYIGNORE))
    #endif
  #elif defined _WINDOWS
    typedef short ALERT_FMessageBox( const char*);
    #if defined _MSC_VER
     #pragma warning ( disable : 4702)    /* unreachable code */
    #endif
    #ifndef ALERT_MESSAGE
      #define ALERT_MESSAGE( _sz) (ALERT_FN ? ALERT_FN( _sz) : MessageBox( 0, _sz, "Alert", MB_ABORTRETRYIGNORE))
    #endif
  #elif defined LINUX && defined __KERNEL__
    typedef short ALERT_FMessageBox( const char*);
    #ifndef ALERT_MESSAGE
      #define ALERT_MESSAGE( _sz) (ALERT_FN ? ALERT_FN( _sz) : (panic( _sz), ALERT_IGNORE))
    #endif
    #ifndef ALERT_TERMINATE
     #define ALERT_TERMINATE() ((void)0)
    #endif
  #else
    typedef short ALERT_FMessageBox( const char*);
    #ifndef ALERT_MESSAGE
      #define ALERT_MESSAGE( _sz) (ALERT_FN ? ALERT_FN( _sz) : ALERT_RETRY)
    #endif
  #endif
  DEBUG_EXTERN( ALERT_FMessageBox* ALERT_FN);

  #ifndef ALERT_TERMINATE
#if LINUX_VERSION_CODE >= 0x20600
   #define ALERT_TERMINATE() BUG()
#else
   #define ALERT_TERMINATE() abort()
#endif
  #endif

  #define ALERT( _sz) do { \
    int _i = ALERT_MESSAGE( _sz); \
    if ( ALERT_RETRY == _i) { BREAKPOINT( ">>> " _sz); break; } \
    else if ( ALERT_IGNORE == _i) break; \
    else ALERT_TERMINATE(); \
  } while(1)

#else
  #define ALERT( _sz) ((void)0)

  #define ALERT_DEFINE_MESSAGEBOX( _f)
  #define ALERT_DEFINE_HANDLER( _f)
  #define ALERT_SET_HANDLER( _f) ((void)0)
#endif


#ifdef __cplusplus
}
#endif
#endif  /* ndef _debug_h */
/* End of file */
