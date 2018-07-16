#ifndef _rtdalpao_library_h
#define _rtdalpao_library_h

#include <stdint.h>









/* %%%%%%%%%%%%%%%%%%%%%%% ALPAO Section begin %%%%%%%%%%%%%%%%%%%%%%% */
// #include "alpao_device_def128.h" // default device (for debuggin)
// #include "alpao_device_led64.h" // LED panel
 #include "alpao_device_dm97.h" // The DM97

#include "alpao_device.h"
#include "macros.h"

#pragma message "Target ALPAO device type [" ALPAO_DEV_TYPE "] serial [" ALPAO_DEV_SERIAL "] with [" STR(ALPAO_DEV_N_CHANNEL) "] actuators"

#define ALPAO_PRINT_DEBUG              0

#define ALPAO_N_CHANNEL                128 // data size for the ALPAO DM controller
#define ALPAO_HEADER_LENGTH            2 // leading uint16_ts for the header
#define ALPAO_CHECKSUM_LENGTH          1 // trailing uint16_t for the checksum
#define ALPAO_PAD_LENGTH               1 // pad end with an empty word

#define ALPAO_FRAME_LENGTH             (ALPAO_HEADER_LENGTH+ALPAO_N_CHANNEL+ALPAO_CHECKSUM_LENGTH) //[uint16_ts]
#define ALPAO_FRAME_SIZE               (2*ALPAO_FRAME_LENGTH) //[chars]

#define ALPAO_DATA_LENGTH              (ALPAO_FRAME_LENGTH+ALPAO_PAD_LENGTH) //[uint16_ts]
#define ALPAO_DATA_SIZE                (2*ALPAO_DATA_LENGTH) //[chars]

#define ALPAO_MIN_ANALOG_STEP          0.000122070312500 // hardcoded pow(2.0,-13);

const extern alpao_device_t alpao_device;
/* %%%%%%%%%%%%%%%%%%%%%%%% ALPAO Section end %%%%%%%%%%%%%%%%%%%%%%%% */









/* %%%%%%%%%%%%%%%%%%%%%%%% RTD Section begin %%%%%%%%%%%%%%%%%%%%%%%% */
#include "dm7820_library.h"

#define RTD_PRINT_DEBUG                0

#define RTD_PRGCLK_0_DIVISOR           8 // Programmable clock frequency = 25/RTD_PRGCLK_0_DIVISOR [MHz]
#define RTD_TIMER_A0_DIVISOR           2 // Output clock frequency = (25/RTD_PRGCLK_0_DIVISOR)/RTD_TIMER_A0_DIVISOR [MHz]
#define RTD_CLK_FREQUENCY              ((25000000.0/RTD_PRGCLK_0_DIVISOR)/RTD_TIMER_A0_DIVISOR) //[Hz]

#pragma message "RTD clock set to [" STR(RTD_CLK_FREQUENCY) "] Hz"

#ifndef RTD_DMA_BUFFER_LENGTH
#define RTD_DMA_BUFFER_LENGTH          0x200 // length of the dma buffer in unint16_ts
#define RTD_DMA_BUFFER_SIZE            (2*RTD_DMA_BUFFER_LENGTH) // dma buffer size in unint8_ts
#define RTD_DMA_BUFFER_COUNT           1
#endif
/* %%%%%%%%%%%%%%%%%%%%%%%%% RTD Section end %%%%%%%%%%%%%%%%%%%%%%%%% */









/* %%%%%%%%%%%%%%%%%%%%% RTD ALPAO Section begin %%%%%%%%%%%%%%%%%%%%% */
#define RTDALPAO_PRINT_DEBUG           0

#define RTDALPAO_MAX_REFRESH_RATE      (RTD_CLK_FREQUENCY/ALPAO_DATA_LENGTH) //[Hz]

#define LWFC_FREQ                      500.0 //[Hz]

#ifdef RTDALPAO_DITHER // to use dither compile with flag -DRTDALPAO_DITHER in gcc cli
// #define RTDALPAO_DITHERS_PER_FRAME     ((uint16_t)(DIV_CEILING(RTDALPAO_MAX_REFRESH_RATE,LWFC_FREQ)))
#define RTDALPAO_DITHERS_PER_FRAME     ((uint16_t)(DIV_FLOOR(RTDALPAO_MAX_REFRESH_RATE,LWFC_FREQ)))
#define RTDALPAO_DATA_LENGTH           ((uint16_t)(RTDALPAO_DITHERS_PER_FRAME*ALPAO_DATA_LENGTH)) //[uint16_ts]
#else
#define RTDALPAO_DITHERS_PER_FRAME     1
#define RTDALPAO_DATA_LENGTH           0x200 //[uint16_ts]
#endif

#define RTDALPAO_FRAME_SIZE            (2*RTDALPAO_DATA_LENGTH) //[chars]

// redefine the dma buffer length based on the dither/non-dither option
#define RTD_DMA_BUFFER_LENGTH          RTDALPAO_DATA_LENGTH // length of the dma buffer in unint16_ts
#define RTD_DMA_BUFFER_SIZE            (2*RTD_DMA_BUFFER_LENGTH) // dma buffer size in unint8_ts

#define RTDALPAO_REFRESH_RATE          (RTD_CLK_FREQUENCY/RTDALPAO_DATA_LENGTH) // [Hz]
#define RTDALPAO_DATA_TRANSFER_TIME    (1000000.0*RTDALPAO_DATA_LENGTH/RTD_CLK_FREQUENCY) // [us]

#pragma message "RTD DMA support: DMA buffer of [" STR(RTD_DMA_BUFFER_SIZE) "] bytes"

/* -------------------- function prototypes -------------------- */
void rtdalpao_init(void);
void rtdalpao_start_timer(void);
void rtdalpao_stop_timer(void);
void rtdalpao_send_analog_data(double [ALPAO_DEV_N_CHANNEL]); // use this function to send analog data. will (not dither)/dither output if librtdalpao is compiled (without)/with dither
void rtdalpao_print_data(void);
void rtdalpao_write_data_to_file(FILE* p_file, const double data[ALPAO_DEV_N_CHANNEL]);
void rtdalpao_print_info(void);
void rtdalpao_clean_close(void);
/* ------------------------------------------------------------- */
/* %%%%%%%%%%%%%%%%%%%%%% RTD ALPAO Section end %%%%%%%%%%%%%%%%%%%%%% */



#endif /* _rtdalpao_library_h */
