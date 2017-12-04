/*************************************************************************
 * ALPAO - SDK - Driver
 * @brief Common types used in Alpao SDK
 ************************************************************************/

#ifndef ASDK_TYPE_H_
#define ASDK_TYPE_H_

/* Compiler informations */
#if defined(WIN32) || defined(_WIN32)
# define IS_WIN32 1
#else
# define IS_WIN32 0
#endif

#if __WORDSIZE == 64 || defined(_WIN64) || defined(__x86_64__)
# define IS_64B 1
#else
# define IS_64B 0
#endif

/* System Headers */
#ifdef __cplusplus

#ifdef _MSC_VER // Some MSVC are not compliant with C99 stdint.h

typedef unsigned __int8  uint8_t;
typedef __int8            int8_t;
typedef unsigned __int16 uint16_t;
typedef __int16           int16_t;
typedef unsigned __int32 uint32_t;
typedef __int32           int32_t;
typedef unsigned __int64 uint64_t;
typedef __int64           int64_t;

#elif IS_WIN32
# include <cstdint>
#else
# include <stdint.h>
#endif

#include <string>

/**
 * Namespace: Alpao Common Software
 */
namespace acs
{

#else
# include <stdint.h>
#endif


#ifndef DOXYGEN /* Code fragment ignored by Doxygen */
/*
 * Define Import / Export depending on platform
 */
# if SWIG
#  define ACS_API_EXPORTS
#  define ACS_API_IMPORTS
# elif IS_WIN32
#  define ACS_API_EXPORTS __declspec(dllexport)
#  define ACS_API_IMPORTS __declspec(dllimport)
# elif __GNUC__ >= 4 
#  define ACS_API_EXPORTS __attribute__((__visibility__("default")))
#  define ACS_API_IMPORTS 
# endif
#endif

/**
 * @addtogroup asdk
 * @{
 */

typedef char        Char;       /**< signed char (8b)*/
typedef uint8_t     UChar;      /**< unsigned char (8b)*/

typedef int16_t     Short;      /**< signed short integer (16b)*/
typedef uint16_t    UShort;     /**< unsigned short integer (16b)*/

typedef int32_t     Int;        /**< signed integer (32b)*/
typedef uint32_t    UInt;       /**< unsigned integer (32b)*/

typedef int64_t     Long;       /**< signed long integer (64b)*/
typedef uint64_t    ULong;      /**< unsigned long integer (64b)*/

typedef size_t      Size_T;     /**< size type (target dependent)*/
typedef double      Scalar;     /**< Only one floating point type: standard scalar value*/

#ifdef __cplusplus
 typedef std::string  String;     /**< Standard string type*/
 typedef bool         Bool;       /**< boolean*/
#else
 typedef enum {FALSE, TRUE}  Bool; /**< boolean*/
#endif

typedef char*        CString;     /**< C style string type*/
typedef char const * CStrConst;   /**< C style const string type*/

/**
 * @brief Completion status for any function
 */
typedef enum
{
    SUCCESS = 0,         /**< Function returns successfully*/
    FAILURE = -1         /**< Function returns with error*/
} COMPL_STAT;

/** @} */


#ifdef __cplusplus
}    /* End namespace*/
#endif

#endif /* ASDK_TYPE_H_ */
