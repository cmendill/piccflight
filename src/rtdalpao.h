#ifndef _RTDALPAO
//ALPAO
#define ALP_AMIN                    -1.0
#define ALP_AMAX                     1.0
#define ALP_MAX_POWER                2.54841998
#define ALP_N_CHANNEL                128 // data size for the ALPAO DM controller
#define ALP_HEADER_LENGTH            2 // leading uint16_ts for the header
#define ALP_CHECKSUM_LENGTH          1 // trailing uint16_t for the checksum
#define ALP_PAD_LENGTH               1 // pad end with an empty word
#define ALP_FRAME_LENGTH             (ALP_HEADER_LENGTH+ALP_N_CHANNEL+ALP_CHECKSUM_LENGTH) //[uint16_ts]
#define ALP_FRAME_SIZE               (2*ALP_FRAME_LENGTH) //[chars]
#define ALP_DATA_LENGTH              (ALP_FRAME_LENGTH+ALP_PAD_LENGTH) //[uint16_ts]
#define ALP_DATA_SIZE                (2*ALP_DATA_LENGTH) //[chars]
#define ALP_MIN_ANALOG_STEP          0.000122070312500 // hardcoded pow(2.0,-13);
#define ALP_MAPPING                  {60, 36, 38, 12, 14, 63, 58, 39, 35, 13, 8, 16, 48, 54, 57, 33, 17, 19, 23, 20, 31, 53, 40, 59, 9, 34, 11, 21, 27, 25, 29, 7, 49, 51, 62, 37, 56, 10, 15, 1, 3, 28, 4, 42, 43, 55, 32, 47, 98, 111, 119, 101, 112, 110, 65, 89, 64, 88, 96, 121, 120, 126, 73, 116, 114, 66, 68, 94, 91, 97, 85, 74, 123, 115, 127, 117, 71, 90, 87, 79, 83, 81, 75, 124, 125, 95, 84, 80, 72, 77, 99, 103, 86, 82, 78, 76, 102}

#define ALP_MULTIPLIER               {1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, 1}




//RTD
#define RTD_PRGCLK_0_DIVISOR           8 // Programmable clock frequency = 25/RTD_PRGCLK_0_DIVISOR [MHz]
#define RTD_TIMER_A0_DIVISOR           2 // Output clock frequency = (25/RTD_PRGCLK_0_DIVISOR)/RTD_TIMER_A0_DIVISOR [MHz]
#define RTD_CLK_FREQUENCY              ((25000000.0/RTD_PRGCLK_0_DIVISOR)/RTD_TIMER_A0_DIVISOR) //[Hz]

#define _RTDALPAO
#endif
