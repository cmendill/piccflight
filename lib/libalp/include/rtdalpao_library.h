#ifndef _rtdalpao_library_h
#define _rtdalpao_library_h

#include <stdint.h>









/* %%%%%%%%%%%%%%%%%%%%%%% ALPAO Section begin %%%%%%%%%%%%%%%%%%%%%%% */
// #include "alpao_device_def128.h" // default device (for debuggin)
// #include "alpao_device_led64.h" // LED panel
#include "alpao_device_dm97.h" // The DM97
#include "alpao_device.h"
#include "macros.h"
//#pragma message "Target ALPAO device type [" ALPAO_DEV_TYPE "] serial [" ALPAO_DEV_SERIAL "] with [" STR(ALPAO_DEV_N_CHANNEL) "] actuators"
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
/* %%%%%%%%%%%%%%%%%%%%%%%% ALPAO Section end %%%%%%%%%%%%%%%%%%%%%%%% */









/* %%%%%%%%%%%%%%%%%%%%%%%% RTD Section begin %%%%%%%%%%%%%%%%%%%%%%%% */
#include "dm7820_library.h"
#define RTD_PRINT_DEBUG                0
#define RTD_PRGCLK_0_DIVISOR           8 // Programmable clock frequency = 25/RTD_PRGCLK_0_DIVISOR [MHz]
#define RTD_TIMER_A0_DIVISOR           2 // Output clock frequency = (25/RTD_PRGCLK_0_DIVISOR)/RTD_TIMER_A0_DIVISOR [MHz]
#define RTD_CLK_FREQUENCY              ((25000000.0/RTD_PRGCLK_0_DIVISOR)/RTD_TIMER_A0_DIVISOR) //[Hz]
//#pragma message "RTD clock set to [" STR(RTD_CLK_FREQUENCY) "] Hz"
#define RTD_DMA_BUFFER_COUNT           1
/* %%%%%%%%%%%%%%%%%%%%%%%%% RTD Section end %%%%%%%%%%%%%%%%%%%%%%%%% */









/* %%%%%%%%%%%%%%%%%%%%% RTD ALPAO Section begin %%%%%%%%%%%%%%%%%%%%% */
#define RTDALPAO_PRINT_DEBUG           0
#define RTDALPAO_MAX_REFRESH_RATE      (RTD_CLK_FREQUENCY/ALPAO_DATA_LENGTH) //[Hz]
#define LWFC_FREQ                      500.0 //[Hz]
/* %%%%%%%%%%%%%%%%%%%%%% RTD ALPAO Section end %%%%%%%%%%%%%%%%%%%%%% */

/* -------------------- function prototypes -------------------- */
DM7820_Error rtdalpao_init(DM7820_Board_Descriptor*,uint16_t);
DM7820_Error rtdalpao_start_timer(DM7820_Board_Descriptor*);
DM7820_Error rtdalpao_stop_timer(DM7820_Board_Descriptor*);
DM7820_Error rtdalpao_send_analog_data(DM7820_Board_Descriptor*, double [ALPAO_DEV_N_CHANNEL]); 
DM7820_Error rtdalpao_clean(DM7820_Board_Descriptor*);
DM7820_Error rtd_open(unsigned long, DM7820_Board_Descriptor**);
DM7820_Error rtd_reset(DM7820_Board_Descriptor*);
DM7820_Error rtd_clear_all(DM7820_Board_Descriptor*);
DM7820_Error rtd_close(DM7820_Board_Descriptor*);
void rtdalpao_print_data(void);
void rtdalpao_write_data_to_file(FILE*, const double[ALPAO_DEV_N_CHANNEL]);
void rtdalpao_print_info(void);
/* ------------------------------------------------------------- */

#endif /* _rtdalpao_library_h */
