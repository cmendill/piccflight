#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <error.h>

#include "rtdalpao_library.h"





























/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ALPAO Section begin %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

#define ALPAO_MAX_SAFE                 0x3FFF
#define ALPAO_MIN_SAFE                 0x0000

#define ALPAO_START_WORD               0xF800
#define ALPAO_INIT_COUNTER             0x5C00
#define ALPAO_END_WORD                 0xF100
#define ALPAO_MID_SCALE                0x2000

/* -------------------- function prototypes -------------------- */
static void alpao_limit_power(double[ALPAO_DEV_N_CHANNEL]);
static void alpao_build_frame(const double[ALPAO_DEV_N_CHANNEL], uint16_t[ALPAO_DATA_LENGTH]);
static void alpao_print_frame(const uint16_t frame[ALPAO_DATA_LENGTH]);
/* ------------------------------------------------------------- */

const alpao_device_t alpao_device = {ALPAO_DEV_TYPE, ALPAO_DEV_SERIAL, ALPAO_DEV_MAX_POWER_SAFE, ALPAO_DEV_N_CHANNEL, ALPAO_DEV_MAPPING, ALPAO_DEV_MULTIPLIER};









/**
  *  \brief   Power limiting function (The original function from ALPAO)
  *           /!\ The ALPAO_DEV_MAX_POWER_SAFE constant must be adapted to each DM, please contact ALPAO for details.
  *  @param   [in|out]    double buffer[ALPAO_DEV_N_CHANNEL] - A buffer with ALPAO_DEV_N_CHANNEL actuator values.
  *  @return  void
  */
static void alpao_limit_power(double buffer[ALPAO_DEV_N_CHANNEL]) {
  double power = 0.;
  size_t channel_index = 0;
  for (channel_index = 0; channel_index < ALPAO_DEV_N_CHANNEL; channel_index++) {
    power += buffer[channel_index] * buffer[channel_index];
  }
  if (power > ALPAO_DEV_MAX_POWER_SAFE) {
    double gain = sqrt(ALPAO_DEV_MAX_POWER_SAFE / power);
    printf("Power limitation, command value multiplied by %.2f\n", gain);
    for (channel_index = 0; channel_index < ALPAO_DEV_N_CHANNEL; channel_index++) {
      buffer[channel_index] *= gain;
    }
  }
}









// /**
//   *  \brief   Calculated frame check-sum (The original function from ALPAO)
//   *           This function is no longer used (but kept here for reference).
//   *           Frame checksums are now calculated in the same loop as the adc conversion in ALPAO_build_frame or rtdalpao_build_dither_frames.
//   *  @param   [in|out]    const uint16_t* buffer - [in]  A buffer with ALPAO_FRAME_LENGTH uint16_t DM frame (including the header (0xF600,0x5C00) in the first 4 bytes and the empty checksum (0xF100) at the last 2 bytes.)
//   *                                                [out] A buffer with the calculated checksum in the last 2 bytes
//   *  @return  uint8_t - The caluclated checksum
//   */
// static uint8_t checksum(const uint16_t* buffer) {
//   uint32_t sum = 0;
//   size_t i;
  
//   /* Point with a byte pointer on the integer */
//   uint8_t *p_sum = (uint8_t*)&sum;
//   /* Sum all value of the buffer in an integer value (4 Bytes) */
//   for (i = 1; i < ALPAO_FRAME_LENGTH; ++i) {
//     sum += buffer[i];
//   }

//   /* While 3 first bytes of the integer value are not = 0x00 */
//   while (sum > 0xFF) {
//     /* Sum each bytes and store their sum in the previous integer */
//     sum = p_sum[0] + p_sum[1] + p_sum[2] + p_sum[3];
//   }
//   /* Once 3 first bytes are zeros, checksum is in the 4th byte 0x000000XX, XX is the checkSum */
//   return ~p_sum[0];
// }









/**
  *  \brief   Build a single frame to be written to the hardware.
  *           The frame checksum is also calculated in a single pass.
  *  @param   [in]    const double in_data[ALPAO_DEV_N_CHANNEL] - DM command input buffer with ALPAO_DEV_N_CHANNEL values in range. [-0.25, +0.25]
  *  @param   [out]   uint16_t out_frame[ALPAO_DATA_LENGTH] - Preallocated output buffer of ALPAO_FRAME_LENGTH elements.
  *  @return  void
  */
static void alpao_build_frame(const double in_data[ALPAO_DEV_N_CHANNEL], uint16_t out_frame[ALPAO_DATA_LENGTH]) {
  size_t channel_index = 0;
  uint32_t sum = 0;

  uint8_t *p_sum = (uint8_t*)&sum;

  out_frame[0] = ALPAO_START_WORD; // Start of frame
  out_frame[1] = ALPAO_INIT_COUNTER; // Reset internal counter
  sum += out_frame[1];

  /* Convert double to UINT16 */
  for ( channel_index = 0; channel_index < ALPAO_DEV_N_CHANNEL; channel_index++ ) {
    out_frame[alpao_device.mapping[channel_index]+2] = (uint16_t) ( (alpao_device.multiplier[channel_index]*in_data[channel_index]+1.) * ALPAO_MID_SCALE );
    if ( out_frame[alpao_device.mapping[channel_index]+2] > ALPAO_MAX_SAFE ) out_frame[alpao_device.mapping[channel_index]+2] = ALPAO_MAX_SAFE;
    if ( out_frame[alpao_device.mapping[channel_index]+2] < ALPAO_MIN_SAFE ) out_frame[alpao_device.mapping[channel_index]+2] = ALPAO_MIN_SAFE;
    sum += out_frame[alpao_device.mapping[channel_index]+2];
  }
  out_frame[ALPAO_N_CHANNEL+2] = ALPAO_END_WORD;
  sum += out_frame[ALPAO_N_CHANNEL+2];

  while (sum > 0xFF) {
    sum = p_sum[0] + p_sum[1] + p_sum[2] + p_sum[3];
  }
  out_frame[ALPAO_N_CHANNEL+2] += (uint8_t)(~p_sum[0]);
  out_frame[ALPAO_N_CHANNEL+3] = 0xFEED;// End of frame
}









/**
  *  \brief   Print a single frame
  *  @param   [in]    const uint16_t frame - Frame to print (ALPAO_DATA_LENGTH words).
  *  @return  void
  */
static void alpao_print_frame(const uint16_t frame[ALPAO_DATA_LENGTH]) {
  size_t i;
  for (i = 0; i < ALPAO_DATA_LENGTH; i++) {
    printf("%04X ", frame[i]);
    if (i%8==1) printf("\n");
  }
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ALPAO Section end %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */





























/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RTD Section begin %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

#include <string.h> // required for memset and memcpy
#include <unistd.h> // required for usleep

/* -------------------- function prototypes -------------------- */
static void rtd_interrupt_service_routine(dm7820_interrupt_info);
static void rtd_timer_select_gate(DM7820_Board_Descriptor*, dm7820_tmrctr_timer, dm7820_tmrctr_gate);
static void rtd_timer_disable(DM7820_Board_Descriptor*, dm7820_tmrctr_timer);
// static void rtd_timer_enable(DM7820_Board_Descriptor*, dm7820_tmrctr_timer);
static void rtd_fifo_setup(DM7820_Board_Descriptor*, dm7820_fifo_queue, uint8_t);
static void rtd_fifo_disable(DM7820_Board_Descriptor*, dm7820_fifo_queue);
// static void rtd_fifo_enable(DM7820_Board_Descriptor*, dm7820_fifo_queue);
static void rtd_prgclk_set_mode(DM7820_Board_Descriptor*, dm7820_prgclk_clock, dm7820_prgclk_mode);
static void rtd_prgclk_disable(DM7820_Board_Descriptor*, dm7820_prgclk_clock);
static void rtd_prgclk_enable_continuous(DM7820_Board_Descriptor*, dm7820_prgclk_clock);
static void rtd_pwm_setup(DM7820_Board_Descriptor*, dm7820_pwm_modulator, uint8_t);
static void rtd_pwm_disable(DM7820_Board_Descriptor*, dm7820_pwm_modulator);
// static void rtd_pwm_enable(DM7820_Board_Descriptor*, dm7820_pwm_modulator);
static void rtd_interrupt_setup(DM7820_Board_Descriptor*, dm7820_interrupt_source, uint8_t);
static void rtd_interrupt_disable(DM7820_Board_Descriptor*, dm7820_interrupt_source);
// static void rtd_interrupt_enable(DM7820_Board_Descriptor*, dm7820_interrupt_source);
static void rtd_open(unsigned long minor_number, DM7820_Board_Descriptor**);
static void rtd_reset(DM7820_Board_Descriptor*);
static void rtd_clear_all(DM7820_Board_Descriptor*);

static void rtd_init(void);
static void rtd_start_timer(void);
static void rtd_stop_timer(void);
static void rtd_close(void);
static void rtd_cleanup(void);
static void rtd_write_dma_fifo(char*, uint32_t);
/* ------------------------------------------------------------- */

DM7820_Board_Descriptor rtd_board;
DM7820_Board_Descriptor* p_rtd_board = &rtd_board;









/**
  *  \brief   Interrupt service routine for the rtd interrupts.
  *           Enable the interrupts to use this function.
  *           Check if a certain condition occurred in the fifo or not.
  *           Internal function.
  *  @param   [in]    dm7820_interrupt_info interrupt_info - the structure with the interrupt information.
  *  @return  void
  */
volatile uint64_t dm7820_interrupt_fifo_0_empty_count;
volatile uint64_t dm7820_interrupt_fifo_0_full_count;
volatile uint64_t dm7820_interrupt_fifo_0_overflow_count;
volatile uint64_t dm7820_interrupt_fifo_0_read_request_count;
volatile uint64_t dm7820_interrupt_fifo_0_underflow_count;
volatile uint64_t dm7820_interrupt_fifo_0_write_request_count;
volatile uint64_t dm7820_interrupt_fifo_0_dma_done_count;

static void rtd_interrupt_service_routine(dm7820_interrupt_info interrupt_info) {
  DM7820_Return_Status(interrupt_info.error, "ISR Failed\n");
  switch (interrupt_info.source) {
    case DM7820_INTERRUPT_FIFO_0_EMPTY:
      dm7820_interrupt_fifo_0_empty_count++;
      break;
    case DM7820_INTERRUPT_FIFO_0_FULL:
      dm7820_interrupt_fifo_0_full_count++;
      break;
    case DM7820_INTERRUPT_FIFO_0_OVERFLOW:
      dm7820_interrupt_fifo_0_overflow_count++;
      break;
    case DM7820_INTERRUPT_FIFO_0_READ_REQUEST:
      dm7820_interrupt_fifo_0_read_request_count++;
      break;
    case DM7820_INTERRUPT_FIFO_0_UNDERFLOW:
      dm7820_interrupt_fifo_0_underflow_count++;
      break;
    case DM7820_INTERRUPT_FIFO_0_WRITE_REQUEST:
      dm7820_interrupt_fifo_0_write_request_count++;
      break;
    case DM7820_INTERRUPT_FIFO_0_DMA_DONE:
      dm7820_interrupt_fifo_0_dma_done_count++;
      break;
    default:
      break;
  }
}









/**
  *  \brief   Get the status of the fifo.
  *           Check if a certain condition occurred in the fifo or not.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_fifo_queue fifo - The fifo to check the status of.
  *  @param   [in]    dm7820_fifo_status_condition condition - The condition flag to check.
  *  @param   [out]   uint8_t* status - A pointer to a variable that will return the result of the check (i.e. whether the condition occurred or not).
  *  @return  void
  */
static void rtd_get_fifo_status(DM7820_Board_Descriptor* pboard, dm7820_fifo_queue fifo, dm7820_fifo_status_condition condition, uint8_t* status) {
  if (DM7820_FIFO_Get_Status(pboard, fifo, condition, status) == -1)
    error(EXIT_FAILURE, errno, "DM7820_FIFO_Get_Status() FAILED");
}









/**
  *  \brief   Check and exit on specified condition or clear that condition of the fifo.
  *           Check if a certain condition occurred in the fifo or not.
  *           This function will cause EXIT_FAILURE if a given condition occurres and exit_on is set.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_fifo_status_condition condition - The fifo condition to check.
  *  @param   [in]    uint8_t exit_on - set to 1 to exit on condition set to 0 to not exit.
  *  @return  void
  */
static const char* dm7820_fifo_status_condition_string[] = {"read request", "write request", "full", "empty", "overflow", "underflow"};
static void rtd_exit_on_fifo_status(DM7820_Board_Descriptor* pboard, dm7820_fifo_status_condition condition, uint8_t exit_on) {
  uint8_t fifo_status;
  rtd_get_fifo_status(pboard, DM7820_FIFO_QUEUE_0, condition, &fifo_status);
#if RTD_PRINT_DEBUG
  printf("rtd_get_fifo_status() : %s is %d\n", dm7820_fifo_status_condition_string[condition], fifo_status);
#endif
  if (fifo_status)
    if (exit_on)
      error(EXIT_FAILURE, 0, "rtd_exit_on_fifo_status() : fifo condition %s", dm7820_fifo_status_condition_string[condition]);
}









/**
  *  \brief   Clear that condition of the fifo.
  *           Check if a certain condition occurred in the fifo or not.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_fifo_status_condition condition - The fifo condition to clear.
  *  @return  void
  */
static void rtd_clear_fifo_status(DM7820_Board_Descriptor* pboard, dm7820_fifo_status_condition condition) {
  uint8_t fifo_status;
  rtd_get_fifo_status(pboard, DM7820_FIFO_QUEUE_0, condition, &fifo_status);
#if RTD_PRINT_DEBUG
  printf("rtd_get_fifo_status() : %s is %d\n", dm7820_fifo_status_condition_string[condition], fifo_status);
#endif
}









/**
  *  \brief   Select the gate of the specified timer.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_tmrctr_timer timer - The timer.
  *  @param   [in]    dm7820_tmrctr_gate gate - The gate to set for the timer
  *  @return  void
  */
#if RTD_PRINT_DEBUG
static const char* dm7820_tmrctr_timer_string[] = {"Timer 0 on first 8254 chip", "Timer 1 on first 8254 chip", "Timer 2 on first 8254 chip", "Timer 0 on second 8254 chip", "Timer 1 on second 8254 chip", "Timer 2 on second 8254 chip"};
static const char* dm7820_tmrctr_gate_string[] = {"Logic 0", "Logic 1", "8254 timer/counter A0", "8254 timer/counter A1", "8254 timer/counter A2", "8254 timer/counter B0", "8254 timer/counter B1", "8254 timer/counter B2", "Programmable clock 0", "Programmable clock 1", "Programmable clock 2", "Programmable clock 3", "Strobe signal 1", "Strobe signal 2", "Inverted strobe signal 1", "Inverted strobe signal 2", "Digital I/O port 2 bit 0", "Digital I/O port 2 bit 1", "Digital I/O port 2 bit 2", "Digital I/O port 2 bit 3", "Digital I/O port 2 bit 4", "Digital I/O port 2 bit 5", "Digital I/O port 2 bit 6", "Digital I/O port 2 bit 7", "Digital I/O port 2 bit 8", "Digital I/O port 2 bit 9", "Digital I/O port 2 bit 10", "Digital I/O port 2 bit 11", "Digital I/O port 2 bit 12", "Digital I/O port 2 bit 13", "Digital I/O port 2 bit 14", "Digital I/O port 2 bit 15"};
#endif
static void rtd_timer_select_gate(DM7820_Board_Descriptor* pboard, dm7820_tmrctr_timer timer, dm7820_tmrctr_gate gate) {
  DM7820_Error dm7820_status;
#if RTD_PRINT_DEBUG
  printf("rtd_timer_select_gate() : %s gate set to %s\n", dm7820_tmrctr_timer_string[timer], dm7820_tmrctr_gate_string[gate]);
#endif
  dm7820_status = DM7820_TmrCtr_Select_Gate(pboard, timer, gate);
  DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");
}
/**
  *  \brief   Disable the specified timer.
  *           Disables the specified timer by settings the gate to DM7820_TMRCTR_GATE_LOGIC_0
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_tmrctr_timer timer - The timer.
  *  @return  void
  */
static void rtd_timer_disable(DM7820_Board_Descriptor* pboard, dm7820_tmrctr_timer timer) {
  rtd_timer_select_gate(pboard, timer, DM7820_TMRCTR_GATE_LOGIC_0);
}
// /**
//   *  \brief   Enable the specified timer.
//   *           Enable the specified timer by settings the gate to DM7820_TMRCTR_GATE_LOGIC_1
//   *           Internal function.
//   *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
//   *  @param   [in]    dm7820_tmrctr_timer timer - The timer.
//   *  @return  void
//   */
// static void rtd_timer_enable(DM7820_Board_Descriptor* pboard, dm7820_tmrctr_timer timer) {
//   rtd_timer_select_gate(pboard, timer, DM7820_TMRCTR_GATE_LOGIC_1);
// }









/**
  *  \brief   Enable or disable the specified fifo.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_fifo_queue fifo1 - The fifo.
  *  @param   [in]    uint8_t enable - 0x00 to disable nonzero to enable
  *  @return  void
  */
#if RTD_PRINT_DEBUG
static const char* dm7820_fifo_queue_string[] = {"FIFO 0", "FIFO 1"};
#endif
static void rtd_fifo_setup(DM7820_Board_Descriptor* pboard, dm7820_fifo_queue fifo, uint8_t enable) {
  DM7820_Error dm7820_status;
#if RTD_PRINT_DEBUG
  printf("rtd_fifo_setup() : %s enable set to 0x%02X\n", dm7820_fifo_queue_string[fifo],enable);
#endif
  dm7820_status = DM7820_FIFO_Enable(pboard, fifo, enable);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");
}
/**
  *  \brief   Disable the specified fifo.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_fifo_queue fifo1 - The fifo.
  *  @return  void
  */
static void rtd_fifo_disable(DM7820_Board_Descriptor* pboard, dm7820_fifo_queue fifo) {
  rtd_fifo_setup(pboard, fifo, 0x00);
}
// /**
//   *  \brief   Enable the specified fifo.
//   *           Internal function.
//   *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
//   *  @param   [in]    dm7820_fifo_queue fifo1 - The fifo.
//   *  @return  void
//   */
// static void rtd_fifo_enable(DM7820_Board_Descriptor* pboard, dm7820_fifo_queue fifo) {
//   rtd_fifo_setup(pboard, fifo, 0xFF);
// }









/**
  *  \brief   Set the mode of the specified programmable clock.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_prgclk_clock prgclk - The programmable clock.
  *  @param   [in]    dm7820_prgclk_mode mode - The mode.
  *  @return  void
  */
#if RTD_PRINT_DEBUG
static const char* dm7820_prgclk_clock_string[] = {"Programmable clock 0", "Programmable clock 1", "Programmable clock 2", "Programmable clock 3"};
static const char* dm7820_prgclk_mode_string[] = {"Disabled", "Continuous mode", "Reserved (do not use)", "One shot mode"};
#endif
static void rtd_prgclk_set_mode(DM7820_Board_Descriptor* pboard, dm7820_prgclk_clock prgclk, dm7820_prgclk_mode mode) {
  DM7820_Error dm7820_status;
#if RTD_PRINT_DEBUG
  printf("rtd_prgclk_set_mode() : %s mode set to %s\n", dm7820_prgclk_clock_string[prgclk], dm7820_prgclk_mode_string[mode]);
#endif
  dm7820_status = DM7820_PrgClk_Set_Mode(pboard, prgclk, mode);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");
}
/**
  *  \brief   Disable the specified programmable clock.
  *           Disables the specified programmable clock by settings the mode to DM7820_PRGCLK_MODE_DISABLED.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_prgclk_clock prgclk - The programmable clock.
  *  @return  void
  */
static void rtd_prgclk_disable(DM7820_Board_Descriptor* pboard, dm7820_prgclk_clock prgclk) {
  rtd_prgclk_set_mode(pboard, prgclk, DM7820_PRGCLK_MODE_DISABLED);
}
/**
  *  \brief   Put the the specified programmable clock in continuos mode.
  *           Enables the specified programmable clock in continuos mode by settings the mode to DM7820_PRGCLK_MODE_CONTINUOUS.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_prgclk_clock prgclk - The programmable clock.
  *  @return  void
  */
static void rtd_prgclk_enable_continuous(DM7820_Board_Descriptor* pboard, dm7820_prgclk_clock prgclk) {
  rtd_prgclk_set_mode(pboard, prgclk, DM7820_PRGCLK_MODE_CONTINUOUS);
}









/**
  *  \brief   Enable or disable the specified pulse width modulator.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_pwm_modulator pwm1 - The pulse width modulator.
  *  @param   [in]    uint8_t enable - 0x00 to disable nonzero to enable
  *  @return  void
  */
#if RTD_PRINT_DEBUG
static const char* dm7820_pwm_string[] = {"Pulse width modulator 0", "Pulse width modulator 1"};
#endif
static void rtd_pwm_setup(DM7820_Board_Descriptor* pboard, dm7820_pwm_modulator pwm, uint8_t enable) {
  DM7820_Error dm7820_status;
#if RTD_PRINT_DEBUG
  printf("rtd_pwm_setup() : %s enable set to 0x%02X\n", dm7820_pwm_string[pwm], enable);
#endif
  dm7820_status = DM7820_PWM_Enable(pboard, pwm, enable);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");
}
/**
  *  \brief   Disable the specified pulse width modulator.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_pwm_modulator pwm1 - The pulse width modulator.
  *  @return  void
  */
static void rtd_pwm_disable(DM7820_Board_Descriptor* pboard, dm7820_pwm_modulator pwm) {
  rtd_pwm_setup(pboard, pwm, 0x00);
}
// /**
//   *  \brief   Enable the specified pulse width modulator.
//   *           Internal function.
//   *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
//   *  @param   [in]    dm7820_pwm_modulator pwm1 - The pulse width modulator.
//   *  @return  void
//   */
// static void rtd_pwm_enable(DM7820_Board_Descriptor* pboard, dm7820_pwm_modulator pwm) {
//   rtd_pwm_setup(pboard, pwm, 0xFF);
// }









/**
  *  \brief   Enable or disable the specified interrupt source.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_interrupt_source source - The interrupt source.
  *  @param   [in]    uint8_t enable - 0x00 to disable nonzero to enable
  *  @return  void
  */
#if RTD_PRINT_DEBUG
static const char* dm7820_interrupt_source_string[] = {"Advanced interrupt block 0 interrupt",
                                                       "Advanced interrupt block 1 interrupt (1)",
                                                       "FIFO block FIFO 0 empty interrupt (2)",
                                                       "FIFO block FIFO 0 full interrupt (3)",
                                                       "FIFO block FIFO 0 overflow interrupt (4)",
                                                       "FIFO block FIFO 0 read request interrupt (5)",
                                                       "FIFO block FIFO 0 underflow interrupt (6)",
                                                       "FIFO block FIFO 0 write request interrupt (7)",
                                                       "FIFO block FIFO 1 empty interrupt (8)",
                                                       "FIFO block FIFO 1 full interrupt (9)",
                                                       "FIFO block FIFO 1 overflow interrupt (10)",
                                                       "FIFO block FIFO 1 read request interrupt (11)",
                                                       "FIFO block FIFO 1 underflow interrupt (12)",
                                                       "FIFO block FIFO 1 write request interrupt (13)",
                                                       "Incremental encoder block 0 channel A negative rollover interrupt (14)",
                                                       "Incremental encoder block 0 channel A positive rollover interrupt (15)",
                                                       "Incremental encoder block 0 channel B negative rollover interrupt (16)",
                                                       "Incremental encoder block 0 channel B positive rollover interrupt (17)",
                                                       "Incremental encoder block 1 channel A negative rollover interrupt (18)",
                                                       "Incremental encoder block 1 channel A positive rollover interrupt (19)",
                                                       "Incremental encoder block 1 channel B negative rollover interrupt (20)",
                                                       "Incremental encoder block 1 channel B positive rollover interrupt (21)",
                                                       "Programmable clock block 0 interrupt (22)",
                                                       "Programmable clock block 1 interrupt (23)",
                                                       "Programmable clock block 2 interrupt (24)",
                                                       "Programmable clock block 3 interrupt (25)",
                                                       "Pulse width modulator block 0 interrupt (26)",
                                                       "Pulse width modulator block 1 interrupt (27)",
                                                       "8254 timer/counter A0 interrupt (28)",
                                                       "8254 timer/counter A1 interrupt (29)",
                                                       "8254 timer/counter A2 interrupt (30)",
                                                       "8254 timer/counter B0 interrupt (31)",
                                                       "8254 timer/counter B1 interrupt (32)",
                                                       "8254 timer/counter B2 interrupt (33)",
                                                       "FIFO block FIFO 0 DMA done interrupt (34) [Applications cannot control this interrupt but they can get its status.]",
                                                       "FIFO block FIFO 1 DMA done interrupt (35) [Applications cannot control this interrupt but they can get its status.]",
                                                       "Value which indicates no interrupt source (36) [User level ignores this. The kernel uses this in the interrupt handler. This must be the last entry.]"};
#endif
static void rtd_interrupt_setup(DM7820_Board_Descriptor* pboard, dm7820_interrupt_source source, uint8_t enable) {
  DM7820_Error dm7820_status;
#if RTD_PRINT_DEBUG
  printf("rtd_interrupt_setup() : %s enable set to 0x%02X\n", dm7820_interrupt_source_string[source], enable);
#endif
  dm7820_status = DM7820_General_Enable_Interrupt(pboard, source, enable);
  DM7820_Return_Status(dm7820_status,"DM7820_General_Enable_Interrupt()");
}
/**
  *  \brief   Enable the specified interrupt source.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_interrupt_source source - The interrupt source.
  *  @return  void
  */
static void rtd_interrupt_disable(DM7820_Board_Descriptor* pboard, dm7820_interrupt_source source) {
  rtd_interrupt_setup(pboard, source, 0x00);
}
// /**
//   *  \brief   Disable the specified interrupt source.
//   *           Internal function.
//   *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
//   *  @param   [in]    dm7820_interrupt_source source - The interrupt source.
//   *  @return  void
//   */
// static void rtd_interrupt_enable(DM7820_Board_Descriptor* pboard, dm7820_interrupt_source source) {
//   rtd_interrupt_setup(pboard, source, 0xFF);
// }









/**
  *  \brief   Open the RTD DM7820 board for configuration.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor** pboard - The pointer a pointer of the board discriptor
  *  @return  void
  */
static void rtd_open(unsigned long minor_number, DM7820_Board_Descriptor** pboard) {
  DM7820_Error dm7820_status;
  /* ---------------- Device initialization ---------------- */
#if RTD_PRINT_DEBUG 
  printf("rtd_open() : Opening device with minor number %lu\n", minor_number);
#endif
  dm7820_status = DM7820_General_Open_Board(minor_number, pboard);
  DM7820_Return_Status(dm7820_status, "DM7820_General_Open_Board()");
}









/**
  *  \brief   Reset the RTD DM7820 board.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - The pointer to the board discriptor
  *  @return  void
  */
static void rtd_reset(DM7820_Board_Descriptor* pboard) {
  DM7820_Error dm7820_status;
  /* ---------------- Device Reset ---------------- */
#if RTD_PRINT_DEBUG
  printf("rtd_reset() : Resetting device\n");
#endif
  dm7820_status = DM7820_General_Reset(pboard);
  DM7820_Return_Status(dm7820_status, "DM7820_General_Reset()");
}









/**
  *  \brief   Total cleanup of the RTD DM7820 board configuration.
  *           De-initialize the following peripherals
  *            1. all FIFOs\n
  *            2. all 8254 timers/counters\n
  *            3. all Programmable clocks\n
  *            3. all PWMs\n
  *            3. all interrupt sources\n
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - The pointer to the board discriptor
  *  @return  void
  */
static void rtd_clear_all(DM7820_Board_Descriptor* pboard) {
  rtd_fifo_disable(pboard, DM7820_FIFO_QUEUE_0);
  rtd_fifo_disable(pboard, DM7820_FIFO_QUEUE_1);

  rtd_timer_disable(pboard, DM7820_TMRCTR_TIMER_A_0);
  rtd_timer_disable(pboard, DM7820_TMRCTR_TIMER_A_1);
  rtd_timer_disable(pboard, DM7820_TMRCTR_TIMER_A_2);
  rtd_timer_disable(pboard, DM7820_TMRCTR_TIMER_B_0);
  rtd_timer_disable(pboard, DM7820_TMRCTR_TIMER_B_1);
  rtd_timer_disable(pboard, DM7820_TMRCTR_TIMER_B_2);

  rtd_prgclk_disable(pboard, DM7820_PRGCLK_CLOCK_0);
  rtd_prgclk_disable(pboard, DM7820_PRGCLK_CLOCK_1);
  rtd_prgclk_disable(pboard, DM7820_PRGCLK_CLOCK_2);
  rtd_prgclk_disable(pboard, DM7820_PRGCLK_CLOCK_3);

  rtd_pwm_disable(pboard, DM7820_PWM_MODULATOR_0);
  rtd_pwm_disable(pboard, DM7820_PWM_MODULATOR_1);
  
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_ADVINT_0);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_ADVINT_1);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_FIFO_0_EMPTY);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_FIFO_0_FULL);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_FIFO_0_OVERFLOW);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_FIFO_0_READ_REQUEST);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_FIFO_0_UNDERFLOW);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_FIFO_0_WRITE_REQUEST);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_FIFO_1_EMPTY);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_FIFO_1_FULL);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_FIFO_1_OVERFLOW);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_FIFO_1_READ_REQUEST);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_FIFO_1_UNDERFLOW);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_FIFO_1_WRITE_REQUEST);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_INCENC_0_CHANNEL_A_NEGATIVE_ROLLOVER);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_INCENC_0_CHANNEL_A_POSITIVE_ROLLOVER);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_INCENC_0_CHANNEL_B_NEGATIVE_ROLLOVER);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_INCENC_0_CHANNEL_B_POSITIVE_ROLLOVER);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_INCENC_1_CHANNEL_A_NEGATIVE_ROLLOVER);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_INCENC_1_CHANNEL_A_POSITIVE_ROLLOVER);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_INCENC_1_CHANNEL_B_NEGATIVE_ROLLOVER);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_INCENC_1_CHANNEL_B_POSITIVE_ROLLOVER);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_PRGCLK_0);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_PRGCLK_1);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_PRGCLK_2);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_PRGCLK_3);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_PWM_0);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_PWM_1);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_TMRCTR_A_0);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_TMRCTR_A_1);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_TMRCTR_A_2);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_TMRCTR_B_0);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_TMRCTR_B_1);
  rtd_interrupt_disable(pboard, DM7820_INTERRUPT_TMRCTR_B_2);
}









/**
  *  \brief   Initialize the RTD clock output and 16-bit data outputs to send data to the ALPAO system.
  *           The clock signal for the ALPAO is generated by chaining two peripherals to the 25 MHz master clock on the RTD DM7820 board.
  *            - Programmable clock 0\n
  *              The 25 MHz main clock is set as the master of the programmable clock 0.
  *              The programmable clock 0 period is set to RTD_PRGCLK_0_DIVISOR (as a clock divisor).
  *              The programmable clock generates an impulse train at the configured frequency.
  *            - 8254 timer/counter A0\n
  *              The programmable clock 0 is set as the clock of the counter.
  *              The counter is set to count to RTD_TIMER_A0_DIVISOR, and generate a square wave.
  *           The clock signal is output on the 8254 timer/counter A0 output line of the RTD board (i.e. port2[2] - pin 11 of CN10).\n
  *           The data output uses the FIFO0 peripheral of the RTD DM7820 board. (i.e. port0[0-15] - pins [17-47] (odd pins) of CN10)
  *           The FIFO0 output clock is set to the 8254 timer/counter A0.
  *           The FIFO0 input clock is set to the PCI data input.\n
  *           This function opens the board and initialized the above configuration
  *           The board handle is kept internal.
  *           Externally accesible function.
  *  @param   void
  *  @return  void
  */
uint16_t* dma_buffer; // DMA Buffer
uint16_t sleep_time_us = (uint16_t)(RTD_DMA_BUFFER_SIZE*1000000.0/RTD_CLK_FREQUENCY);
static void rtd_init(void) {
  DM7820_Error dm7820_status;

  rtd_open(0, &p_rtd_board);

  rtd_reset(p_rtd_board);

  rtd_clear_all(p_rtd_board);

  /* ================================ Standard output initialization ================================ */
#if RTD_PRINT_DEBUG
  printf("rtd_init() : Initialize standard ouptouts ...\n");
#endif

  /* Set Port 0 to peripheral output */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Configure Port 0 (all pins) to peripheral output\n");
#endif
  dm7820_status = DM7820_StdIO_Set_IO_Mode(p_rtd_board, DM7820_STDIO_PORT_0, 0xFFFF, DM7820_STDIO_MODE_PER_OUT);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

  /* Set Port 0 peripheral to the fifo 0 peripheral */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Configure Port 0 output to the fifo 0 peripheral\n");
#endif
  dm7820_status = DM7820_StdIO_Set_Periph_Mode(p_rtd_board, DM7820_STDIO_PORT_0, 0xFFFF, DM7820_STDIO_PERIPH_FIFO_0);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

  /* Set Port 2 to peripheral output */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Configure Port 2 (all pins) to peripheral output\n");
#endif
  dm7820_status = DM7820_StdIO_Set_IO_Mode(p_rtd_board, DM7820_STDIO_PORT_2, 0xFFFF, DM7820_STDIO_MODE_PER_OUT);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

  /* Set Port 2 peripheral to the clock and timer peripherals */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Configure Port 2 output to the clock and timer peripherals\n");
#endif
  dm7820_status = DM7820_StdIO_Set_Periph_Mode(p_rtd_board, DM7820_STDIO_PORT_2, 0xFFFF, DM7820_STDIO_PERIPH_CLK_OTHER);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");


  /* ================================ Programmable clock 0 initialization ================================ */
#if RTD_PRINT_DEBUG
  printf("rtd_init() : Initialize Programmable Clock 0 ...\n");
#endif

  /* Set master clock to 25 MHz clock */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Set master clock of Programmable Clock 0 to 25 MHz\n");
#endif
  dm7820_status = DM7820_PrgClk_Set_Master(p_rtd_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Master()");

  /* Set clock stop trigger so that clock is never stopped */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Set stop trigger to none\n");
#endif
  dm7820_status = DM7820_PrgClk_Set_Stop_Trigger(p_rtd_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_STOP_NONE);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Stop_Trigger()");

  /* Set clock period to obtain 25/RTD_PRGCLK_0_DIVISOR [MHz] */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Set clock frequency to %e MHz by setting the period to %d\n", (25.0/RTD_PRGCLK_0_DIVISOR), RTD_PRGCLK_0_DIVISOR);
#endif
  dm7820_status = DM7820_PrgClk_Set_Period(p_rtd_board, DM7820_PRGCLK_CLOCK_0, RTD_PRGCLK_0_DIVISOR);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");

  /* Set clock start trigger to start immediately */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Set start trigger to immediate\n");
#endif
  dm7820_status = DM7820_PrgClk_Set_Start_Trigger(p_rtd_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_START_IMMEDIATE);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Start_Trigger()");


  /* ================================ 8254 timer/counter A0 initialization ================================ */
#if RTD_PRINT_DEBUG
  printf("rtd_init() : Set Timer A0 clock source to Programmable clock 0 ...\n");
#endif
  dm7820_status = DM7820_TmrCtr_Select_Clock(p_rtd_board, DM7820_TMRCTR_TIMER_A_0, DM7820_TMRCTR_CLOCK_PROG_CLOCK_0);
  DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Clock()");

  /* Set up the timer by
   * 1) setting waveform mode to square wave generator,
   * 2) setting count mode to binary, and
   * 3) loading divisor value to obtain the frequency */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Set Timer A0 squarewave mode, count to %d in binary; setting clock frequency to %e MHz\n", RTD_TIMER_A0_DIVISOR, ((25.0/RTD_PRGCLK_0_DIVISOR)/RTD_TIMER_A0_DIVISOR));
#endif
  dm7820_status = DM7820_TmrCtr_Program(p_rtd_board, DM7820_TMRCTR_TIMER_A_0, DM7820_TMRCTR_WAVEFORM_SQUARE_WAVE, DM7820_TMRCTR_COUNT_MODE_BINARY, RTD_TIMER_A0_DIVISOR);
  DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Program()");

  /* Set timer gate to high to enable counting */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Set timer/counter A0 gate to logic high\n");
#endif
  dm7820_status = DM7820_TmrCtr_Select_Gate(p_rtd_board, DM7820_TMRCTR_TIMER_A_0, DM7820_TMRCTR_GATE_LOGIC_1);
  DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");


  /* ========================== FIFO 0 initialization ========================== */
#if RTD_PRINT_DEBUG
  printf("rtd_init() : Initialize FIFO 0 ...\n");
#endif

  /* Set input clock to PCI write to FIFO 0 Read/Write Port Register */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Set FIFO 0 input clock to PCI Write\n");
#endif
  dm7820_status = DM7820_FIFO_Set_Input_Clock(p_rtd_board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_INPUT_CLOCK_PCI_WRITE);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Input_Clock()");

  /* Set FIFO 0 output clock to timer A0 */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Set FIFO 0 output clock to timer A0\n");
#endif
  dm7820_status = DM7820_FIFO_Set_Output_Clock(p_rtd_board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_OUTPUT_CLOCK_8254_A_0);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Output_Clock()");

  /* Set data input to PCI data */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Set FIFO 0 data input PCI data\n");
#endif
  dm7820_status = DM7820_FIFO_Set_Data_Input(p_rtd_board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_0_DATA_INPUT_PCI_DATA);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Data_Input()");

  /* ========================== DMA initialization ========================== */
#if RTD_PRINT_DEBUG
  printf("rtd_init() : Initialize DMA on FIFO 0 ...\n");
#endif

  /* Set FIFO 0 DREQ to REQUEST WRITE */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Setting FIFO 0 DREQ source\n");
#endif
  dm7820_status = DM7820_FIFO_Set_DMA_Request(p_rtd_board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_DMA_REQUEST_WRITE);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_DMA_Request()");
  
  /* create the DMA buffer */
#if RTD_PRINT_DEBUG
  printf("rtd_init() : Allocating DMA buffer of %d bytes\n", RTD_DMA_BUFFER_SIZE);
#endif
  dm7820_status = DM7820_FIFO_DMA_Create_Buffer(&dma_buffer, RTD_DMA_BUFFER_SIZE);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Create_Buffer()");

  /* initialize the DMA buffer */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Initializing DMA\n");
#endif
  dm7820_status = DM7820_FIFO_DMA_Initialize(p_rtd_board, DM7820_FIFO_QUEUE_0, RTD_DMA_BUFFER_COUNT, RTD_DMA_BUFFER_SIZE);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Initialize()");

  /* configure DMA direction*/
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Configuring DMA: PCI_TO_DM7820\n");
#endif
  dm7820_status = DM7820_FIFO_DMA_Configure(p_rtd_board, DM7820_FIFO_QUEUE_0, DM7820_DMA_DEMAND_ON_PCI_TO_DM7820, RTD_DMA_BUFFER_SIZE);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Configure()");

  /* ========================== Secondary FIFO 0 configuration ========================== */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :     Enable FIFO 0\n");
#endif
  dm7820_status = DM7820_FIFO_Enable(p_rtd_board, DM7820_FIFO_QUEUE_0, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

#if RTD_PRINT_DEBUG
  printf("rtd_init() :       Clear FIFO 0 flags...\n");
#endif

#if RTD_PRINT_DEBUG
  printf("rtd_init() :     Resetting interrupt counters\n");
#endif
  dm7820_interrupt_fifo_0_empty_count=0;
  dm7820_interrupt_fifo_0_full_count=0;
  dm7820_interrupt_fifo_0_overflow_count=0;
  dm7820_interrupt_fifo_0_read_request_count=0;
  dm7820_interrupt_fifo_0_underflow_count=0;
  dm7820_interrupt_fifo_0_write_request_count=0;
  dm7820_interrupt_fifo_0_dma_done_count=0;

#if RTD_PRINT_DEBUG
  printf("rtd_init() :     register and prioritize interrupt service routine\n");
#endif
  dm7820_status = DM7820_General_InstallISR(p_rtd_board, rtd_interrupt_service_routine);
  DM7820_Return_Status(dm7820_status, "DM7820_General_InstallISR()");

  dm7820_status = DM7820_General_SetISRPriority(p_rtd_board, 99);
  DM7820_Return_Status(dm7820_status, "DM7820_General_SetISRPriority()");
  
  /* ---------------- clear all fifo status flags ---------------- */
  rtd_clear_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_EMPTY);
  rtd_clear_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_FULL);
  rtd_clear_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_OVERFLOW);
  rtd_clear_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_UNDERFLOW);

#if RTD_PRINT_DEBUG
  printf("rtd_init() :       Check initial FIFO 0 status ...\n");
#endif

  /* ---------------- check fifo status flags and exit ---------------- */
  rtd_exit_on_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_EMPTY, 0);
  rtd_exit_on_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_FULL, 1);
  rtd_exit_on_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_OVERFLOW, 1);
  rtd_exit_on_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_UNDERFLOW, 0);
}









/**
  *  \brief   Start the timer
  *           The board handle is kept internal.
  *           Externally accesible function.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - The pointer to the board discriptor
  *  @return  void
  */
static void rtd_start_timer(void) {
#if RTD_PRINT_DEBUG
  printf("rtd_start_timer() : start timer\n");
#endif
  rtd_prgclk_enable_continuous(p_rtd_board, DM7820_PRGCLK_CLOCK_0);
}









/**
  *  \brief   Stop the timer
  *           The board handle is kept internal.
  *           Externally accesible function.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - The pointer to the board discriptor
  *  @return  void
  */
static void rtd_stop_timer(void) {
#if RTD_PRINT_DEBUG
  printf("rtd_stop_timer() : stop timer\n");
#endif
  rtd_prgclk_disable(p_rtd_board, DM7820_PRGCLK_CLOCK_0);
}









/**
  *  \brief   Close the board opended by the rtd_open() function.
  *           Internal function.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - The pointer to the board discriptor
  *  @return  void
  */
static void rtd_close(void) {
  DM7820_Error dm7820_status;

  /* ---------------- Final processing before exit ---------------- */
#if RTD_PRINT_DEBUG
  printf("rtd_close() : Closing device\n");
#endif
  dm7820_status = DM7820_General_Close_Board(p_rtd_board);
  DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");
  p_rtd_board=NULL;
}









/**
  *  \brief   Clean and close the board opended and initialized by rtd_init()
  *           The board handle is kept internal.
  *           Externally accesible function.
  *  @param   void
  *  @return  void
  */
static void rtd_cleanup(void) {
  DM7820_Error dm7820_status;

#if RTD_PRINT_DEBUG
  printf("rtd_cleanup() :     remove interrupt service routine\n");
#endif
  dm7820_status = DM7820_General_RemoveISR(p_rtd_board);
  DM7820_Return_Status(dm7820_status, "DM7820_General_RemoveISR()");

#if RTD_DMA
#if RTD_PRINT_DEBUG
  printf("rtd_cleanup() : De-allocating DMA buffer of %d bytes\n", RTD_DMA_BUFFER_SIZE);
#endif
  dm7820_status = DM7820_FIFO_DMA_Free_Buffer(&dma_buffer, RTD_DMA_BUFFER_SIZE);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Create_Buffer()");

  // ************* disable DMA transfer
#if RTD_PRINT_DEBUG
  printf("rtd_cleanup() : disable DMA\n");
#endif
  dm7820_status = DM7820_FIFO_DMA_Enable(p_rtd_board, DM7820_FIFO_QUEUE_0, 0x00, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Enable()");
#endif

  rtd_timer_disable(p_rtd_board, DM7820_TMRCTR_TIMER_A_0);

  rtd_prgclk_disable(p_rtd_board, DM7820_PRGCLK_CLOCK_0);

  rtd_fifo_disable(p_rtd_board, DM7820_FIFO_QUEUE_1);

  rtd_close();
}









/**
  *  \brief   Load the data using the dma fifo write.
  *           The board handle is kept internal.
  *           Externally accesible function.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    char* buffer - The buffer with the data to be written to the fifo.
  *  @param   [in]    uint32_t size - The size of the buffer in bytes.
  *  @return  void
  */
uint64_t fifo_0_dma_done_count, fifo_0_empty_count;
static void rtd_write_dma_fifo(char* buffer, uint32_t size) {
  // size - the size of the transfer in bytes
  DM7820_Error dm7820_status;
  uint8_t fifo_status;

  //Everything written must be an integer number of 16bit words
  if(size % 2)
    printf("rtd_write_dma_fifo() : rtd_write_fifo: BAD DATA SIZE\n");

  /* ========================== write source buffer to FIFO 0 via the PCI bus ========================== */
#if RTD_PRINT_DEBUG
  printf("rtd_write_dma_fifo() : Write %d char buffer to DMA buffer\n",size);
#endif

  /* copy source buffer to DMA */
  memset(dma_buffer, 0, RTD_DMA_BUFFER_SIZE);
  memcpy(dma_buffer, buffer, size);


  //Sleep until fifo is empty (NOT SURE WHY WE NEED THIS)
  do {
    dm7820_status = DM7820_FIFO_Get_Status(p_rtd_board,DM7820_FIFO_QUEUE_0,DM7820_FIFO_STATUS_EMPTY,&fifo_status);
    usleep(10);
  } while(!fifo_status);

  /* ========================== send DMA to FIFO ========================== */
  fifo_0_dma_done_count = dm7820_interrupt_fifo_0_dma_done_count;
  
  // Copy the user space buffer to send out via fifo
#if RTD_PRINT_DEBUG
  printf("rtd_write_dma_fifo() : Copy the userspace DMA buffer to fifo\n");
#endif
  dm7820_status = DM7820_FIFO_DMA_Write(p_rtd_board, DM7820_FIFO_QUEUE_0, dma_buffer, RTD_DMA_BUFFER_COUNT);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Write()");


  /* Reconfigure DMA */
#if RTD_PRINT_DEBUG
  printf("rtd_write_dma_fifo() : Reconfigure DMA\n");
#endif
  dm7820_status = DM7820_FIFO_DMA_Configure(p_rtd_board, DM7820_FIFO_QUEUE_0, DM7820_DMA_DEMAND_ON_PCI_TO_DM7820, RTD_DMA_BUFFER_SIZE);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Configure()");

  /* Enable & Start DMA transfer */
#if RTD_PRINT_DEBUG
  printf("rtd_write_dma_fifo() : re-enable DMA\n");
#endif
  dm7820_status = DM7820_FIFO_DMA_Enable(p_rtd_board, DM7820_FIFO_QUEUE_0, 0xFF, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Enable()");

  /* Wait for DMA transfer to finish */
  while(fifo_0_dma_done_count==dm7820_interrupt_fifo_0_dma_done_count)
    usleep(10);

  /* ---------------- check fifo status flags and exit ---------------- */
  rtd_exit_on_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_EMPTY, 0);
  rtd_exit_on_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_FULL, 1);
  rtd_exit_on_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_OVERFLOW, 1);
  rtd_exit_on_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_UNDERFLOW, 0);

#if RTD_PRINT_DEBUG
  printf("rtd_write_dma_fifo() : done\n");
#endif
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RTD Section end %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */





























/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RTD ALPAO Section begin %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

#define RTDALPAO_HARDWARE 1

uint16_t rtdalpao_dma_data[RTDALPAO_DATA_LENGTH] = {ALPAO_MID_SCALE}; 

/* -------------------- function prototypes -------------------- */
static void rtdalpao_send_analog_frame(double[ALPAO_DEV_N_CHANNEL]);
static void rtdalpao_send_digital_frame(char*, uint32_t);
#ifdef RTDALPAO_DITHER
static void rtdalpao_send_analog_dither_frames(const double[ALPAO_DEV_N_CHANNEL]);
static void rtdalpao_send_digital_dither_frames(char*);
static void rtdalpao_build_dither_frames(const double[ALPAO_DEV_N_CHANNEL], uint16_t[RTDALPAO_DATA_LENGTH]);
static uint8_t is_actuator_up_down(size_t, double);
#endif
/* ------------------------------------------------------------- */









/**
  *  \brief   Function to initialize the RTD board and ALPAO device
  *  @param   [in]    void
  *  @return  void
  */
void rtdalpao_init(void) {
#if RTDALPAO_HARDWARE
  rtd_init();
#endif
#if RTDALPAO_PRINT_DEBUG
  printf("rtdalpao_init() : initializing the board\n");
#endif
}









/**
  *  \brief   Function to start the RTD board timer
  *  @param   [in]    void
  *  @return  void
  */
void rtdalpao_start_timer(void) {
#if RTDALPAO_HARDWARE
  rtd_start_timer();
#endif
#if RTDALPAO_PRINT_DEBUG
  printf("rtdalpao_start_timer() : starting timer\n");
#endif
}









/**
  *  \brief   Function to stop the RTD board timer
  *  @param   [in]    void
  *  @return  void
  */
void rtdalpao_stop_timer(void) {
#if RTDALPAO_HARDWARE
  rtd_stop_timer();
#endif
#if RTDALPAO_PRINT_DEBUG
  printf("rtdalpao_stop_timer() : stopping timer\n");
#endif
}









/**
  *  \brief   Send an analog frame out without dithering.
  *           Just outputs the frame to digital pins.
  *           The power limitation and AD conversion is kept internal.
  *  @param   [in]    char* block - digital block data to be sent out
  *  @return  void
  */
static void rtdalpao_send_analog_frame(double in_data[ALPAO_DEV_N_CHANNEL]) {
  alpao_limit_power(in_data);
  alpao_build_frame(in_data, rtdalpao_dma_data);
  rtdalpao_send_digital_frame((char*)rtdalpao_dma_data,ALPAO_FRAME_SIZE);
}









/**
  *  \brief   Send digital data frame out, the frame size is kept internally.
  *  @param   [in]    char* frame - digital frame to be sent out
  *  @return  void
  */
void rtdalpao_send_digital_frame(char* frame, uint32_t size) {
#if RTDALPAO_HARDWARE
  rtd_write_dma_fifo(frame, size);
#endif
}









#ifdef RTDALPAO_DITHER
/**
  *  \brief   Function to decide wheather to make the dither step high or not based on the frame number on the block and the fraction of the round up.
  *  @param   [in]    size_t frame_number - Frame number (in range [1,dither_frames_per_block]) in the block
  *  @param   [in]    double fraction - Fraction (in range [0.0,1.0]) the original double values remainder of ALPAO_MIN_ANALOG_STEP as a fraction of the ALPAO_MIN_ANALOG_STEP 
  *  @return  uint8_t 1 or 0 - based on dither step high or not
  */
static uint8_t is_actuator_up_down(size_t frame_number, double fraction) {
  uint8_t on, off;
  if (fraction == 1.0) {
    return 0;
  } else if ((fraction < 1.0) && (fraction >= 0.5)) {
    on = 0;
    fraction = 1.0 - fraction;
    off = 1;
  } else if ((fraction < 0.5) && (fraction > 0.0)) {
    on = 1;
    off = 0;
  } else if (fraction == 0.0) {
    return 0;
  } else {
    on = 1;
    off = 0;
  }
  return ( ((frame_number%((int16_t)(1.0/fraction))) == 0)?on:off );
}









/**
  *  \brief   Build a block of RTDALPAO_DITHERS_PER_FRAME frames with dithering based on the roundup remainder.
  *  @param   [in]    const double in_data[ALPAO_N_CHANNEL] - Deformable mirror command input buffer with ALPAO_N_CHANNEL values in range [-0.25, +0.25]
  *  @param   [out]   uint16_t out_block[RTDALPAO_DATA_LENGTH] - Preallocated output buffer of RTDALPAO_DATA_LENGTH elements
  *  @return  void
  */
void rtdalpao_build_dither_frames(const double in_data[ALPAO_DEV_N_CHANNEL], uint16_t out_block[RTDALPAO_DATA_LENGTH]) {

  size_t frame_number, channel_index, main_index, sub_index, index;
  uint32_t sum;

  uint8_t *p_sum = (uint8_t*)&sum;

  double fraction[ALPAO_DEV_N_CHANNEL];
  uint16_t frame[ALPAO_DEV_N_CHANNEL];

  for ( channel_index = 0; channel_index < ALPAO_DEV_N_CHANNEL; channel_index++ ) {
    frame[channel_index] = (uint16_t) ( (alpao_device.multiplier[channel_index]*in_data[channel_index]+1.0) * ALPAO_MID_SCALE );
    fraction[channel_index] = fmod(in_data[channel_index],ALPAO_MIN_ANALOG_STEP)/ALPAO_MIN_ANALOG_STEP + ((in_data[channel_index]<=0.0)?1.0:0.0);
  }

  for (frame_number = 1; frame_number <= RTDALPAO_DITHERS_PER_FRAME; frame_number++) {

    index = frame_number-1;
    sum = 0;

    main_index = index*ALPAO_DATA_LENGTH;

    out_block[main_index+0] = ALPAO_START_WORD;
    out_block[main_index+1] = ALPAO_INIT_COUNTER;
    sum += out_block[main_index+1];

    for ( channel_index = 0; channel_index < ALPAO_DEV_N_CHANNEL; channel_index++ ) {
      sub_index = main_index+alpao_device.mapping[channel_index]+2;
      out_block[sub_index] = frame[channel_index] + is_actuator_up_down(frame_number,fraction[channel_index]);
      if ( out_block[sub_index] > ALPAO_MAX_SAFE ) out_block[sub_index] = ALPAO_MAX_SAFE;
      if ( out_block[sub_index] < ALPAO_MIN_SAFE ) out_block[sub_index] = ALPAO_MIN_SAFE;
      sum += out_block[sub_index];
    }
    sub_index = main_index+ALPAO_N_CHANNEL+2;
    out_block[sub_index] = ALPAO_END_WORD;
    sum += out_block[sub_index];

    while (sum > 0xFF) {
      sum = p_sum[0] + p_sum[1] + p_sum[2] + p_sum[3];
    }
    out_block[sub_index] += (uint8_t)(~p_sum[0]);
    out_block[sub_index+1] = 0xFEED;// End of frame

  }
}









/**
  *  \brief   Send an analog frame out with dithering.
  *           The power limitation and AD conversion is kept internal.
  *  @param   [in]    char* block - digital block data to be sent out
  *  @return  void
  */
static void rtdalpao_send_analog_dither_frames(const double in_data[ALPAO_DEV_N_CHANNEL]) {
  alpao_limit_power(in_data);
  rtdalpao_build_dither_frames(in_data, rtdalpao_dma_data);
  rtdalpao_send_digital_dither_frames((char*)rtdalpao_dma_data);
}









/**
  *  \brief   Send digital data block out, the block size is kept internally.
  *  @param   [in]    char* block - digital block data to be sent out
  *  @return  void
  */
static void rtdalpao_send_digital_dither_frames(char* block) {
#if RTDALPAO_HARDWARE
  rtd_write_dma_fifo(block, RTD_DMA_BUFFER_SIZE);
#endif
}
#endif









/**
  *  \brief   Send an analog frame out with or without dithering depending on RTDALPAO_DITHER.
  *  @param   [in]    char* block - digital block data to be sent out
  *  @return  void
  */
void rtdalpao_send_analog_data(double in_data[ALPAO_DEV_N_CHANNEL]) {
#ifdef RTDALPAO_DITHER
  rtdalpao_send_analog_dither_frames(in_data);
#else
  rtdalpao_send_analog_frame(in_data);
#endif
}









/**
  *  \brief   Print a block of RTDALPAO_DITHERS_PER_FRAME frames
  *  @param   [in]    const uint16_t block[RTDALPAO_DATA_LENGTH] - Block to print (RTDALPAO_DITHERS_PER_FRAME frames of ALPAO_DATA_LENGTH words each)
  *  @return  void
  */
void rtdalpao_print_data(void) {
  uint16_t frame_number;
  for (frame_number = 0; frame_number < RTDALPAO_DITHERS_PER_FRAME; frame_number++) {
    printf("\n--------------- fr = %02d ---------------\n",frame_number);
    alpao_print_frame(rtdalpao_dma_data+(frame_number*ALPAO_DATA_LENGTH));
    printf("\n---------------------------------------\n");
  }
}









/**
  *  \brief   Put analog data and digital data to an open file pointer
  *  @param   [in]    FILE* p_file - Open file pointer to write data to
  *  @param   [in]    const double data[ALPAO_DEV_N_CHANNEL] - Analog data to be written to the file
  *  @return  void
  */
static uint8_t write_header = 1;
void rtdalpao_write_data_to_file(FILE* p_file, const double data[ALPAO_DEV_N_CHANNEL]) {
  uint16_t frames_per_block = RTDALPAO_DITHERS_PER_FRAME;
  uint16_t frame_length = ALPAO_DATA_LENGTH;
  if (write_header) {
    fwrite(&(alpao_device.n_act), sizeof(uint8_t), 1, p_file);
    fwrite(&frames_per_block, sizeof(uint16_t), 1, p_file);
    fwrite(&frame_length, sizeof(uint16_t), 1, p_file);
    fwrite(&(alpao_device.mapping), sizeof(uint8_t), ALPAO_DEV_N_CHANNEL, p_file);
    fwrite(&(alpao_device.multiplier), sizeof(int8_t), ALPAO_DEV_N_CHANNEL, p_file);
    write_header = 0;
  }
  fwrite(data, sizeof(double), ALPAO_DEV_N_CHANNEL, p_file);
  fwrite(rtdalpao_dma_data, sizeof(uint16_t), RTDALPAO_DATA_LENGTH, p_file);
}









/**
  *  \brief   Print information about the current configuration of the librtdalpao
  *  @param   [void]
  *  @return  void
  */
void rtdalpao_print_info(void) {
  printf("---------------------- RTD board info ----------------------\n");
  printf("RTD clock frequency                  = %f [Hz]\n", RTD_CLK_FREQUENCY);
  printf("RTD DMA length                       = %d (%dx%d) [words]\n", RTDALPAO_DATA_LENGTH, ALPAO_DATA_LENGTH, RTDALPAO_DITHERS_PER_FRAME);
  printf("Maximum possible refresh rate        = %f [Hz]\n", RTDALPAO_REFRESH_RATE);
  printf("Dithering                            = %sabled\n", (RTDALPAO_DITHERS_PER_FRAME==1)?"dis":"en");
  printf("Dithers per frame                    = %d [frames]\n", RTDALPAO_DITHERS_PER_FRAME);
  printf("Time to clock-out 1 data transfer    = %f [us]\n", RTDALPAO_DATA_TRANSFER_TIME);
  printf("------------------------ ALPAO info ------------------------\n");
  printf("ALPAO Frame length (with pad)        = %d [words]\n", ALPAO_DATA_LENGTH);
  printf("---------------------- ALPAO dev info ----------------------\n");
  printf("device type                          = %s\n", alpao_device.type); // or ALPAO_DEV_TYPE
  printf("device serial                        = %s\n", alpao_device.serial); // or ALPAO_DEV_SERIAL
  printf("max power                            = %f\n", alpao_device.max_power); // or ALPAO_DEV_MAX_POWER_SAFE
  printf("number of actuators                  = %d\n", alpao_device.n_act); // or ALPAO_DEV_N_CHANNEL for static arrays definitions
  printf("actuator mapping                     = [index,\tmap,\tX  ]\n");
  uint8_t act;
  for(act = 0; act < alpao_device.n_act; act++)
    printf("                                       [%03d,\t%03d,\t%+03d]\n", act, alpao_device.mapping[act], alpao_device.multiplier[act]);
  printf("\b\n");
  printf("------------------------ LOWFC info ------------------------\n");
  printf("LWFC frequency                       = %f [Hz]\n", LWFC_FREQ);
  printf("Main loop period                     = %f [us]\n", 1000000.0/LWFC_FREQ);
  printf("------------------------------------------------------------\n");
}









/**
  *  \brief   Function to de-initialize the RTD board and ALPAO device
  *  @param   [in]    void
  *  @return  void
  */
void rtdalpao_clean_close(void) {
#if RTDALPAO_HARDWARE
  rtd_cleanup();
#endif
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RTD ALPAO Section end %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
