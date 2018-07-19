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
static double alpao_limit_analog_value(double value);
static void alpao_limit_power(double[ALPAO_DEV_N_CHANNEL]);
static uint8_t alpao_checksum(const uint16_t[ALPAO_FRAME_LENGTH]);
static void alpao_init_buffer(uint16_t[ALPAO_DATA_LENGTH]);
static void alpao_build_frame(const double[ALPAO_DEV_N_CHANNEL], uint16_t[ALPAO_DATA_LENGTH]);
static void alpao_print_frame(const uint16_t frame[ALPAO_DATA_LENGTH]);
/* ------------------------------------------------------------- */

static const alpao_device_t alpao_device = {ALPAO_DEV_TYPE,
                                            ALPAO_DEV_SERIAL,
                                            ALPAO_DEV_MAX_POWER_SAFE,
                                            ALPAO_DEV_ANALOG_LIMIT,
                                            ALPAO_DEV_N_CHANNEL,
                                            ALPAO_DEV_MAPPING,
                                            ALPAO_DEV_MULTIPLIER,
                                            ALPAO_DEV_OFFSET};









/**
  *  \brief   Analog value limiting functions
  *           this function is used to limit the analog values in a window [-ALPAO_DEV_MAX_POWER_SAFE, ALPAO_DEV_MAX_POWER_SAFE]
  *  @param   [in]    double value - analog value to be clamped.
  *  @return  double - clamped analog value.
  */
static double alpao_limit_analog_value(double value) {
  if (value > ALPAO_DEV_ANALOG_LIMIT)
    value = ALPAO_DEV_ANALOG_LIMIT;
  if (value < -ALPAO_DEV_ANALOG_LIMIT)
    value = -ALPAO_DEV_ANALOG_LIMIT;
  return value;
}









/**
  *  \brief   Power limiting function (The original function from ALPAO)
  *           /!\ The ALPAO_DEV_MAX_POWER_SAFE constant must be adapted to each DM, please contact ALPAO for details.
  *  @param   [in|out]    double buffer[ALPAO_DEV_N_CHANNEL] - [in]  A buffer with ALPAO_DEV_N_CHANNEL actuator values.
  *                                                            [out] A buffer with ALPAO_DEV_N_CHANNEL actuator values (power limitation applied).
  *  @return  void
  */
static void alpao_limit_power(double buffer[ALPAO_DEV_N_CHANNEL]) {
  double power = 0.0;
  size_t channel_index = 0;
  for (channel_index = 0; channel_index < ALPAO_DEV_N_CHANNEL; channel_index++) {
    buffer[channel_index] = alpao_limit_analog_value(buffer[channel_index]);
    power += buffer[channel_index] * buffer[channel_index];
  }
  if (power > ALPAO_DEV_MAX_POWER_SAFE) {
    double gain = sqrt(ALPAO_DEV_MAX_POWER_SAFE / power);
    for (channel_index = 0; channel_index < ALPAO_DEV_N_CHANNEL; channel_index++)
      buffer[channel_index] *= gain;
  }
}









/**
  *  \brief   Calculated frame check-sum (The original function from ALPAO)
  *  @param   [in|out]    const uint16_t* buffer - [in]  A buffer with ALPAO_DATA_LENGTH uint16_t DM frame (including the header (0xF600,0x5C00) in the first 4 bytes and the empty checksum (0xF100) at the last 2 bytes.)
  *                                                [out] A buffer with the calculated checksum in the last 2 bytes
  *  @return  uint8_t - The caluclated checksum
  */
static uint8_t alpao_checksum(const uint16_t buffer[ALPAO_FRAME_LENGTH]) {
  uint32_t sum = 0;
  size_t i;
  uint8_t *p_sum = (uint8_t*)&sum;
  for (i = 1; i < ALPAO_FRAME_LENGTH; ++i)
    sum += buffer[i];
  while (sum > 0xFF)
    sum = p_sum[0] + p_sum[1] + p_sum[2] + p_sum[3];
  return ~p_sum[0];
}
  








/**
  *  \brief   Initialize the uint16_t buffer to ALPAO_MID_SCALE
  *  @param   [in|out]    const uint16_t* buffer - [in]  A buffer with ALPAO_DATA_LENGTH uint16_t DM frame.
  *                                                [out] A buffer filled with ALPAO_MID_SCALE.
  *  @return  uint8_t - The caluclated checksum
  */
static void alpao_init_buffer(uint16_t buffer[ALPAO_DATA_LENGTH]) {
  size_t i;
  for (i = 0; i < ALPAO_DATA_LENGTH; ++i)
    buffer[i]=ALPAO_MID_SCALE;
}









/**
  *  \brief   Build a single frame to be written to the hardware.
  *           The frame checksum is also calculated in a single pass.
  *  @param   [in]    const double in_data[ALPAO_DEV_N_CHANNEL] - DM command input buffer with ALPAO_DEV_N_CHANNEL values in range. [-0.25, +0.25]
  *  @param   [out]   uint16_t out_frame[ALPAO_DATA_LENGTH] - Preallocated output buffer of ALPAO_FRAME_LENGTH elements.
  *  @return  void
  */
static void alpao_build_frame(const double in_data[ALPAO_DEV_N_CHANNEL], uint16_t out_frame[ALPAO_DATA_LENGTH]) {
  size_t channel_index = 0, data_index = 0;

  alpao_init_buffer(out_frame);
  out_frame[0] = ALPAO_START_WORD; // Start of frame
  out_frame[1] = ALPAO_INIT_COUNTER; // Reset internal counter

  for ( channel_index = 0; channel_index < ALPAO_DEV_N_CHANNEL; channel_index++ ) {
    data_index = alpao_device.mapping[channel_index]+ALPAO_HEADER_LENGTH;
    out_frame[data_index] = (uint16_t) ( (alpao_device.multiplier[channel_index]*(in_data[channel_index]+alpao_device.offset[channel_index])+1.0) * ALPAO_MID_SCALE );
    if ( out_frame[data_index] > ALPAO_MAX_SAFE ) out_frame[data_index] = ALPAO_MAX_SAFE;
    if ( out_frame[data_index] < ALPAO_MIN_SAFE ) out_frame[data_index] = ALPAO_MIN_SAFE;
  }
  out_frame[ALPAO_N_CHANNEL+ALPAO_HEADER_LENGTH] = ALPAO_END_WORD;
  out_frame[ALPAO_N_CHANNEL+ALPAO_HEADER_LENGTH] += alpao_checksum(out_frame);
  out_frame[ALPAO_N_CHANNEL+ALPAO_HEADER_LENGTH+ALPAO_PAD_LENGTH] = 0xFEED;// End of frame
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
static DM7820_Error rtd_timer_select_gate(DM7820_Board_Descriptor*, dm7820_tmrctr_timer, dm7820_tmrctr_gate);
static DM7820_Error rtd_timer_disable(DM7820_Board_Descriptor*, dm7820_tmrctr_timer);
static DM7820_Error rtd_timer_enable(DM7820_Board_Descriptor*, dm7820_tmrctr_timer);
static DM7820_Error rtd_fifo_setup(DM7820_Board_Descriptor*, dm7820_fifo_queue, uint8_t);
static DM7820_Error rtd_fifo_disable(DM7820_Board_Descriptor*, dm7820_fifo_queue);
static DM7820_Error rtd_fifo_enable(DM7820_Board_Descriptor*, dm7820_fifo_queue);
static DM7820_Error rtd_prgclk_set_mode(DM7820_Board_Descriptor*, dm7820_prgclk_clock, dm7820_prgclk_mode);
static DM7820_Error rtd_prgclk_disable(DM7820_Board_Descriptor*, dm7820_prgclk_clock);
static DM7820_Error rtd_prgclk_enable_continuous(DM7820_Board_Descriptor*, dm7820_prgclk_clock);
static DM7820_Error rtd_pwm_setup(DM7820_Board_Descriptor*, dm7820_pwm_modulator, uint8_t);
static DM7820_Error rtd_pwm_disable(DM7820_Board_Descriptor*, dm7820_pwm_modulator);
static DM7820_Error rtd_pwm_enable(DM7820_Board_Descriptor*, dm7820_pwm_modulator);
static DM7820_Error rtd_interrupt_setup(DM7820_Board_Descriptor*, dm7820_interrupt_source, uint8_t);
static DM7820_Error rtd_interrupt_disable(DM7820_Board_Descriptor*, dm7820_interrupt_source);
static DM7820_Error rtd_interrupt_enable(DM7820_Board_Descriptor*, dm7820_interrupt_source);
DM7820_Error rtd_open(unsigned long, DM7820_Board_Descriptor**);
DM7820_Error rtd_reset(DM7820_Board_Descriptor*);
DM7820_Error rtd_clear_all(DM7820_Board_Descriptor*);
DM7820_Error rtd_close(DM7820_Board_Descriptor*);

static DM7820_Error rtd_init(DM7820_Board_Descriptor*, uint16_t);
static DM7820_Error rtd_start_timer(DM7820_Board_Descriptor*);
static DM7820_Error rtd_stop_timer(DM7820_Board_Descriptor*);
static DM7820_Error rtd_cleanup(DM7820_Board_Descriptor*);
static DM7820_Error rtd_write_dma_fifo(DM7820_Board_Descriptor*, char*);
/* ------------------------------------------------------------- */









/**
  *  \brief   Interrupt service routine for the rtd interrupts.
  *           Enable the interrupts to use this function.
  *           Check if a certain condition occurred in the fifo or not.
  *  @param   [in]    dm7820_interrupt_info interrupt_info - the structure with the interrupt information.
  *  @return  void
  */
static volatile uint64_t dm7820_interrupt_fifo_0_empty_count;
static volatile uint64_t dm7820_interrupt_fifo_0_full_count;
static volatile uint64_t dm7820_interrupt_fifo_0_overflow_count;
static volatile uint64_t dm7820_interrupt_fifo_0_read_request_count;
static volatile uint64_t dm7820_interrupt_fifo_0_underflow_count;
static volatile uint64_t dm7820_interrupt_fifo_0_write_request_count;
static volatile uint64_t dm7820_interrupt_fifo_0_dma_done_count;

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
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_fifo_queue fifo - The fifo to check the status of.
  *  @param   [in]    dm7820_fifo_status_condition condition - The condition flag to check.
  *  @param   [out]   uint8_t* status - A pointer to a variable that will return the result of the check (i.e. whether the condition occurred or not).
  *  @return  DM7820_Error
  */
static DM7820_Error rtd_get_fifo_status(DM7820_Board_Descriptor* p_rtd_board, dm7820_fifo_queue fifo, dm7820_fifo_status_condition condition, uint8_t* status) {
  return DM7820_FIFO_Get_Status(p_rtd_board, fifo, condition, status);
}









/**
  *  \brief   Check and exit on specified condition or clear that condition of the fifo.
  *           Check if a certain condition occurred in the fifo or not.
  *           This function will cause EXIT_FAILURE if a given condition occurres and exit_on is set.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_fifo_status_condition condition - The fifo condition to check.
  *  @param   [in]    uint8_t exit_on - set to 1 to exit on condition set to 0 to not exit.
  *  @return  void
  */
static const char* dm7820_fifo_status_condition_string[] = {"read request",
                                                            "write request",
                                                            "full",
                                                            "empty",
                                                            "overflow",
                                                            "underflow"};
static void rtd_exit_on_fifo_status(DM7820_Board_Descriptor* pboard, dm7820_fifo_status_condition condition, uint8_t exit_on) {
#if RTD_PRINT_DEBUG
  printf("rtd_exit_on_fifo_status() : %s is %d\n", dm7820_fifo_status_condition_string[condition], fifo_status); 
#endif
  uint8_t fifo_status;
  rtd_get_fifo_status(pboard, DM7820_FIFO_QUEUE_0, condition, &fifo_status);
  if (fifo_status)
    if (exit_on)
      error(EXIT_FAILURE, 0, "rtd_exit_on_fifo_status() : fifo condition %s", dm7820_fifo_status_condition_string[condition]);
}









/**
  *  \brief   Clear that condition of the fifo.
  *           Check if a certain condition occurred in the fifo or not.
  *  @param   [in]    DM7820_Board_Descriptor* pboard - A pointer to the board discriptor.
  *  @param   [in]    dm7820_fifo_status_condition condition - The fifo condition to clear.
  *  @return  void
  */
static DM7820_Error rtd_clear_fifo_status(DM7820_Board_Descriptor* pboard, dm7820_fifo_status_condition condition) {
  uint8_t fifo_status;
  return rtd_get_fifo_status(pboard, DM7820_FIFO_QUEUE_0, condition, &fifo_status);
}









/**
  *  \brief   Select the gate of the specified timer.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_tmrctr_timer timer - The timer.
  *  @param   [in]    dm7820_tmrctr_gate gate - The gate to set for the timer
  *  @return  void
  */
#if RTD_PRINT_DEBUG
static const char* dm7820_tmrctr_timer_string[] = {"Timer 0 on first 8254 chip",
                                                   "Timer 1 on first 8254 chip",
                                                   "Timer 2 on first 8254 chip",
                                                   "Timer 0 on second 8254 chip",
                                                   "Timer 1 on second 8254 chip",
                                                   "Timer 2 on second 8254 chip"};
static const char* dm7820_tmrctr_gate_string[] = {"Logic 0",
                                                  "Logic 1",
                                                  "8254 timer/counter A0",
                                                  "8254 timer/counter A1",
                                                  "8254 timer/counter A2",
                                                  "8254 timer/counter B0",
                                                  "8254 timer/counter B1",
                                                  "8254 timer/counter B2",
                                                  "Programmable clock 0",
                                                  "Programmable clock 1",
                                                  "Programmable clock 2",
                                                  "Programmable clock 3",
                                                  "Strobe signal 1",
                                                  "Strobe signal 2",
                                                  "Inverted strobe signal 1",
                                                  "Inverted strobe signal 2",
                                                  "Digital I/O port 2 bit 0",
                                                  "Digital I/O port 2 bit 1",
                                                  "Digital I/O port 2 bit 2",
                                                  "Digital I/O port 2 bit 3",
                                                  "Digital I/O port 2 bit 4",
                                                  "Digital I/O port 2 bit 5",
                                                  "Digital I/O port 2 bit 6",
                                                  "Digital I/O port 2 bit 7",
                                                  "Digital I/O port 2 bit 8",
                                                  "Digital I/O port 2 bit 9",
                                                  "Digital I/O port 2 bit 10",
                                                  "Digital I/O port 2 bit 11",
                                                  "Digital I/O port 2 bit 12",
                                                  "Digital I/O port 2 bit 13",
                                                  "Digital I/O port 2 bit 14",
                                                  "Digital I/O port 2 bit 15"};
#endif
static DM7820_Error rtd_timer_select_gate(DM7820_Board_Descriptor* p_rtd_board, dm7820_tmrctr_timer timer, dm7820_tmrctr_gate gate) {
#if RTD_PRINT_DEBUG
  printf("rtd_timer_select_gate() : %s gate set to %s\n", dm7820_tmrctr_timer_string[timer], dm7820_tmrctr_gate_string[gate]);
#endif
  return DM7820_TmrCtr_Select_Gate(p_rtd_board, timer, gate);
}
/**
  *  \brief   Disable the specified timer.
  *           Disables the specified timer by settings the gate to DM7820_TMRCTR_GATE_LOGIC_0
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_tmrctr_timer timer - The timer.
  *  @return  void
  */
static DM7820_Error rtd_timer_disable(DM7820_Board_Descriptor* p_rtd_board, dm7820_tmrctr_timer timer) {
  return rtd_timer_select_gate(p_rtd_board, timer, DM7820_TMRCTR_GATE_LOGIC_0);
}
/**
  *  \brief   Enable the specified timer.
  *           Enable the specified timer by settings the gate to DM7820_TMRCTR_GATE_LOGIC_1
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_tmrctr_timer timer - The timer.
  *  @return  void
  */
static DM7820_Error rtd_timer_enable(DM7820_Board_Descriptor* p_rtd_board, dm7820_tmrctr_timer timer) {
  return rtd_timer_select_gate(p_rtd_board, timer, DM7820_TMRCTR_GATE_LOGIC_1);
}









/**
  *  \brief   Enable or disable the specified fifo.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_fifo_queue fifo1 - The fifo.
  *  @param   [in]    uint8_t enable - 0x00 to disable nonzero to enable
  *  @return  void
  */
#if RTD_PRINT_DEBUG
static const char* dm7820_fifo_queue_string[] = {"FIFO 0",
                                                 "FIFO 1"};
#endif
static DM7820_Error rtd_fifo_setup(DM7820_Board_Descriptor* p_rtd_board, dm7820_fifo_queue fifo, uint8_t enable) {
#if RTD_PRINT_DEBUG
  printf("rtd_fifo_setup() : %s enable set to 0x%02X\n", dm7820_fifo_queue_string[fifo],enable);
#endif
  return DM7820_FIFO_Enable(p_rtd_board, fifo, enable);
}
/**
  *  \brief   Disable the specified fifo.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_fifo_queue fifo1 - The fifo.
  *  @return  void
  */
static DM7820_Error rtd_fifo_disable(DM7820_Board_Descriptor* p_rtd_board, dm7820_fifo_queue fifo) {
  return rtd_fifo_setup(p_rtd_board, fifo, 0x00);
}
/**
  *  \brief   Enable the specified fifo.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_fifo_queue fifo1 - The fifo.
  *  @return  void
  */
static DM7820_Error rtd_fifo_enable(DM7820_Board_Descriptor* p_rtd_board, dm7820_fifo_queue fifo) {
  return rtd_fifo_setup(p_rtd_board, fifo, 0xFF);
}









/**
  *  \brief   Set the mode of the specified programmable clock.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_prgclk_clock prgclk - The programmable clock.
  *  @param   [in]    dm7820_prgclk_mode mode - The mode.
  *  @return  DM7820_Error
  */
#if RTD_PRINT_DEBUG
static const char* dm7820_prgclk_clock_string[] = {"Programmable clock 0",
                                                   "Programmable clock 1",
                                                   "Programmable clock 2",
                                                   "Programmable clock 3"};
static const char* dm7820_prgclk_mode_string[] = {"Disabled",
                                                  "Continuous mode",
                                                  "Reserved (do not use)",
                                                  "One shot mode"};
#endif
static DM7820_Error rtd_prgclk_set_mode(DM7820_Board_Descriptor* p_rtd_board, dm7820_prgclk_clock prgclk, dm7820_prgclk_mode mode) {
#if RTD_PRINT_DEBUG
  printf("rtd_prgclk_set_mode() : %s mode set to %s\n", dm7820_prgclk_clock_string[prgclk], dm7820_prgclk_mode_string[mode]);
#endif
  return DM7820_PrgClk_Set_Mode(p_rtd_board, prgclk, mode);
}
/**
  *  \brief   Disable the specified programmable clock.
  *           Disables the specified programmable clock by settings the mode to DM7820_PRGCLK_MODE_DISABLED.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_prgclk_clock prgclk - The programmable clock.
  *  @return  DM7820_Error
  */
static DM7820_Error rtd_prgclk_disable(DM7820_Board_Descriptor* p_rtd_board, dm7820_prgclk_clock prgclk) {
  return rtd_prgclk_set_mode(p_rtd_board, prgclk, DM7820_PRGCLK_MODE_DISABLED);
}
/**
  *  \brief   Put the the specified programmable clock in continuos mode.
  *           Enables the specified programmable clock in continuos mode by settings the mode to DM7820_PRGCLK_MODE_CONTINUOUS.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_prgclk_clock prgclk - The programmable clock.
  *  @return  DM7820_Error
  */
static DM7820_Error rtd_prgclk_enable_continuous(DM7820_Board_Descriptor* p_rtd_board, dm7820_prgclk_clock prgclk) {
  return rtd_prgclk_set_mode(p_rtd_board, prgclk, DM7820_PRGCLK_MODE_CONTINUOUS);
}









/**
  *  \brief   Enable or disable the specified pulse width modulator.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_pwm_modulator pwm1 - The pulse width modulator.
  *  @param   [in]    uint8_t enable - 0x00 to disable nonzero to enable
  *  @return  DM7820_Error
  */
#if RTD_PRINT_DEBUG
static const char* dm7820_pwm_string[] = {"Pulse width modulator 0",
                                          "Pulse width modulator 1"};
#endif
static DM7820_Error rtd_pwm_setup(DM7820_Board_Descriptor* p_rtd_board, dm7820_pwm_modulator pwm, uint8_t enable) {
#if RTD_PRINT_DEBUG
  printf("rtd_pwm_setup() : %s enable set to 0x%02X\n", dm7820_pwm_string[pwm], enable);
#endif
  return DM7820_PWM_Enable(p_rtd_board, pwm, enable);
}
/**
  *  \brief   Disable the specified pulse width modulator.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_pwm_modulator pwm1 - The pulse width modulator.
  *  @return  DM7820_Error
  */
static DM7820_Error rtd_pwm_disable(DM7820_Board_Descriptor* p_rtd_board, dm7820_pwm_modulator pwm) {
  return rtd_pwm_setup(p_rtd_board, pwm, 0x00);
}
/**
  *  \brief   Enable the specified pulse width modulator.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_pwm_modulator pwm1 - The pulse width modulator.
  *  @return  DM7820_Error
  */
static DM7820_Error rtd_pwm_enable(DM7820_Board_Descriptor* p_rtd_board, dm7820_pwm_modulator pwm) {
  return rtd_pwm_setup(p_rtd_board, pwm, 0xFF);
}









/**
  *  \brief   Enable or disable the specified interrupt source.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_interrupt_source source - The interrupt source.
  *  @param   [in]    uint8_t enable - 0x00 to disable nonzero to enable
  *  @return  DM7820_Error
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
static DM7820_Error rtd_interrupt_setup(DM7820_Board_Descriptor* p_rtd_board, dm7820_interrupt_source source, uint8_t enable) {
#if RTD_PRINT_DEBUG
  printf("rtd_interrupt_setup() : %s enable set to 0x%02X\n", dm7820_interrupt_source_string[source], enable);
#endif
  return DM7820_General_Enable_Interrupt(p_rtd_board, source, enable);
}
/**
  *  \brief   Enable the specified interrupt source.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_interrupt_source source - The interrupt source.
  *  @return  DM7820_Error
  */
static DM7820_Error rtd_interrupt_disable(DM7820_Board_Descriptor* p_rtd_board, dm7820_interrupt_source source) {
  return rtd_interrupt_setup(p_rtd_board, source, 0x00);
}
/**
  *  \brief   Disable the specified interrupt source.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - A pointer to the board discriptor.
  *  @param   [in]    dm7820_interrupt_source source - The interrupt source.
  *  @return  DM7820_Error
  */
static DM7820_Error rtd_interrupt_enable(DM7820_Board_Descriptor* p_rtd_board, dm7820_interrupt_source source) {
  return rtd_interrupt_setup(p_rtd_board, source, 0xFF);
}









/**
  *  \brief   Open the RTD DM7820 board for configuration.
  *  @param   [in]    DM7820_Board_Descriptor** p_p_rtd_board - The pointer a pointer of the board discriptor
  *  @return  void
  */
DM7820_Error rtd_open(unsigned long minor_number, DM7820_Board_Descriptor** p_p_rtd_board) {
  /* ---------------- Device initialization ---------------- */
#if RTD_PRINT_DEBUG 
  printf("rtd_open() : Opening device with minor number %lu\n", minor_number);
#endif
  return DM7820_General_Open_Board(minor_number, p_p_rtd_board);
}









/**
  *  \brief   Reset the RTD DM7820 board.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - The pointer to the board discriptor
  *  @return  void
  */
DM7820_Error rtd_reset(DM7820_Board_Descriptor* p_rtd_board) {
  /* ---------------- Device Reset ---------------- */
#if RTD_PRINT_DEBUG
  printf("rtd_reset() : Resetting device\n");
#endif
  return DM7820_General_Reset(p_rtd_board);
}









/**
  *  \brief   Total cleanup of the RTD DM7820 board configuration.
  *           De-initialize the following peripherals
  *            1. all FIFOs\n
  *            2. all 8254 timers/counters\n
  *            3. all Programmable clocks\n
  *            3. all PWMs\n
  *            3. all interrupt sources\n
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - The pointer to the board discriptor
  *  @return  void
  */
DM7820_Error rtd_clear_all(DM7820_Board_Descriptor* p_rtd_board) {
  DM7820_Error dm7820_status;

  dm7820_status = rtd_fifo_disable(p_rtd_board, DM7820_FIFO_QUEUE_0);
  dm7820_status = rtd_fifo_disable(p_rtd_board, DM7820_FIFO_QUEUE_1);

  dm7820_status = rtd_timer_disable(p_rtd_board, DM7820_TMRCTR_TIMER_A_0);
  dm7820_status = rtd_timer_disable(p_rtd_board, DM7820_TMRCTR_TIMER_A_1);
  dm7820_status = rtd_timer_disable(p_rtd_board, DM7820_TMRCTR_TIMER_A_2);
  dm7820_status = rtd_timer_disable(p_rtd_board, DM7820_TMRCTR_TIMER_B_0);
  dm7820_status = rtd_timer_disable(p_rtd_board, DM7820_TMRCTR_TIMER_B_1);
  dm7820_status = rtd_timer_disable(p_rtd_board, DM7820_TMRCTR_TIMER_B_2);

  dm7820_status = rtd_prgclk_disable(p_rtd_board, DM7820_PRGCLK_CLOCK_0);
  dm7820_status = rtd_prgclk_disable(p_rtd_board, DM7820_PRGCLK_CLOCK_1);
  dm7820_status = rtd_prgclk_disable(p_rtd_board, DM7820_PRGCLK_CLOCK_2);
  dm7820_status = rtd_prgclk_disable(p_rtd_board, DM7820_PRGCLK_CLOCK_3);
  
  dm7820_status = rtd_pwm_disable(p_rtd_board, DM7820_PWM_MODULATOR_0);
  dm7820_status = rtd_pwm_disable(p_rtd_board, DM7820_PWM_MODULATOR_1);

  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_ADVINT_0);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_ADVINT_1);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_FIFO_0_EMPTY);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_FIFO_0_FULL);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_FIFO_0_OVERFLOW);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_FIFO_0_READ_REQUEST);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_FIFO_0_UNDERFLOW);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_FIFO_0_WRITE_REQUEST);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_FIFO_1_EMPTY);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_FIFO_1_FULL);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_FIFO_1_OVERFLOW);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_FIFO_1_READ_REQUEST);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_FIFO_1_UNDERFLOW);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_FIFO_1_WRITE_REQUEST);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_INCENC_0_CHANNEL_A_NEGATIVE_ROLLOVER);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_INCENC_0_CHANNEL_A_POSITIVE_ROLLOVER);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_INCENC_0_CHANNEL_B_NEGATIVE_ROLLOVER);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_INCENC_0_CHANNEL_B_POSITIVE_ROLLOVER);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_INCENC_1_CHANNEL_A_NEGATIVE_ROLLOVER);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_INCENC_1_CHANNEL_A_POSITIVE_ROLLOVER);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_INCENC_1_CHANNEL_B_NEGATIVE_ROLLOVER);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_INCENC_1_CHANNEL_B_POSITIVE_ROLLOVER);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_PRGCLK_0);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_PRGCLK_1);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_PRGCLK_2);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_PRGCLK_3);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_PWM_0);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_PWM_1);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_TMRCTR_A_0);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_TMRCTR_A_1);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_TMRCTR_A_2);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_TMRCTR_B_0);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_TMRCTR_B_1);
  dm7820_status = rtd_interrupt_disable(p_rtd_board, DM7820_INTERRUPT_TMRCTR_B_2);

  return dm7820_status;
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
  *  @param   void
  *  @return  void
  */
static uint16_t* rtd_dma_buffer; // DMA buffer : global to this unit to be used in the rtd_cleanup() and rtd_write_dma_fifo() functions
static uint16_t rtd_dma_buffer_size; // buffer size : global to this unit to be used in the rtd_cleanup() and rtd_write_dma_fifo() functions
static DM7820_Error rtd_init(DM7820_Board_Descriptor* p_rtd_board, uint16_t dma_buffer_size) {
  DM7820_Error dm7820_status;

  rtd_dma_buffer_size = dma_buffer_size;

  /* ================================ Standard output initialization ================================ */
#if RTD_PRINT_DEBUG
  printf("rtd_init() : Initialize standard outputs ...\n");
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
  printf("rtd_init() : Allocating DMA buffer of %d bytes\n", rtd_dma_buffer_size);
#endif
  dm7820_status = DM7820_FIFO_DMA_Create_Buffer(&rtd_dma_buffer, rtd_dma_buffer_size);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Create_Buffer()");

  /* initialize the DMA buffer */
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Initializing DMA\n");
#endif
  dm7820_status = DM7820_FIFO_DMA_Initialize(p_rtd_board, DM7820_FIFO_QUEUE_0, RTD_DMA_BUFFER_COUNT, rtd_dma_buffer_size);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Initialize()");

  /* configure DMA direction*/
#if RTD_PRINT_DEBUG
  printf("rtd_init() :   Configuring DMA: PCI_TO_DM7820\n");
#endif
  dm7820_status = DM7820_FIFO_DMA_Configure(p_rtd_board, DM7820_FIFO_QUEUE_0, DM7820_DMA_DEMAND_ON_PCI_TO_DM7820, rtd_dma_buffer_size);
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
  dm7820_interrupt_fifo_0_empty_count = 0;
  dm7820_interrupt_fifo_0_full_count = 0;
  dm7820_interrupt_fifo_0_overflow_count = 0;
  dm7820_interrupt_fifo_0_read_request_count = 0;
  dm7820_interrupt_fifo_0_underflow_count = 0;
  dm7820_interrupt_fifo_0_write_request_count = 0;
  dm7820_interrupt_fifo_0_dma_done_count = 0;

#if RTD_PRINT_DEBUG
  printf("rtd_init() :     register and prioritize interrupt service routine\n");
#endif
  dm7820_status = DM7820_General_InstallISR(p_rtd_board, rtd_interrupt_service_routine);
  DM7820_Return_Status(dm7820_status, "DM7820_General_InstallISR()");

  dm7820_status = DM7820_General_SetISRPriority(p_rtd_board, 99);
  DM7820_Return_Status(dm7820_status, "DM7820_General_SetISRPriority()");
  
  /* ---------------- clear all fifo status flags ---------------- */
  dm7820_status = rtd_clear_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_EMPTY);
  dm7820_status = rtd_clear_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_FULL);
  dm7820_status = rtd_clear_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_OVERFLOW);
  dm7820_status = rtd_clear_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_UNDERFLOW);

#if RTD_PRINT_DEBUG
  printf("rtd_init() :       Check initial FIFO 0 status ...\n");
#endif

  /* ---------------- check fifo status flags and exit ---------------- */
  rtd_exit_on_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_EMPTY, 0);
  rtd_exit_on_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_FULL, 1);
  rtd_exit_on_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_OVERFLOW, 1);
  rtd_exit_on_fifo_status(p_rtd_board, DM7820_FIFO_STATUS_UNDERFLOW, 0);

  return dm7820_status;
}









/**
  *  \brief   Start the timer
  *           The board handle is kept internal.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - The pointer to the board discriptor
  *  @return  void
  */
static DM7820_Error rtd_start_timer(DM7820_Board_Descriptor* p_rtd_board) {
#if RTD_PRINT_DEBUG
  printf("rtd_start_timer() : start timer\n");
#endif
  return rtd_prgclk_enable_continuous(p_rtd_board, DM7820_PRGCLK_CLOCK_0);
}









/**
  *  \brief   Stop the timer
  *           The board handle is kept internal.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - The pointer to the board discriptor
  *  @return  void
  */
static DM7820_Error rtd_stop_timer(DM7820_Board_Descriptor* p_rtd_board) {
#if RTD_PRINT_DEBUG
  printf("rtd_stop_timer() : stop timer\n");
#endif
  return rtd_prgclk_disable(p_rtd_board, DM7820_PRGCLK_CLOCK_0);
}









/**
  *  \brief   Close the board opended by the rtd_open() function.
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - The pointer to the board discriptor
  *  @return  void
  */
DM7820_Error rtd_close(DM7820_Board_Descriptor* p_rtd_board) {
  /* ---------------- Final processing before exit ---------------- */
#if RTD_PRINT_DEBUG
  printf("rtd_close() : Closing device\n");
#endif
  return DM7820_General_Close_Board(p_rtd_board);
  // p_rtd_board=NULL;
}









/**
  *  \brief   Clean and close the board opended and initialized by rtd_init()
  *  @param   [in]    DM7820_Board_Descriptor* p_rtd_board - The pointer to the board discriptor
  *  @return  void
  */
static DM7820_Error rtd_cleanup(DM7820_Board_Descriptor* p_rtd_board) {
#if RTD_PRINT_DEBUG
  printf("rtd_cleanup():\n");
#endif
  DM7820_Error dm7820_status;
  dm7820_status = DM7820_General_RemoveISR(p_rtd_board);
  dm7820_status = DM7820_FIFO_DMA_Free_Buffer(&rtd_dma_buffer, rtd_dma_buffer_size);
  dm7820_status = DM7820_FIFO_DMA_Enable(p_rtd_board, DM7820_FIFO_QUEUE_0, 0x00, 0x00); // disable DMA transfer
  dm7820_status = rtd_timer_disable(p_rtd_board, DM7820_TMRCTR_TIMER_A_0);
  dm7820_status = rtd_prgclk_disable(p_rtd_board, DM7820_PRGCLK_CLOCK_0);
  dm7820_status = rtd_fifo_disable(p_rtd_board, DM7820_FIFO_QUEUE_1);
  return dm7820_status;
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
static uint64_t fifo_0_dma_done_count, fifo_0_empty_count;
static DM7820_Error rtd_write_dma_fifo(DM7820_Board_Descriptor* p_rtd_board, char* buffer) {
  // size - the size of the transfer in bytes
  DM7820_Error dm7820_status;
  uint8_t fifo_status;

  //Everything written must be an integer number of 16bit words
  if(rtd_dma_buffer_size % 2)
    printf("rtd_write_dma_fifo() : rtd_write_fifo: BAD DATA SIZE\n");

  /* ========================== write source buffer to FIFO 0 via the PCI bus ========================== */
#if RTD_PRINT_DEBUG
  printf("rtd_write_dma_fifo() : Write %d char buffer to DMA buffer\n",size);
#endif

  /* copy source buffer to DMA */
  memset(rtd_dma_buffer, 0, rtd_dma_buffer_size);
  memcpy(rtd_dma_buffer, buffer, rtd_dma_buffer_size);


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
  dm7820_status = DM7820_FIFO_DMA_Write(p_rtd_board, DM7820_FIFO_QUEUE_0, rtd_dma_buffer, RTD_DMA_BUFFER_COUNT);
  if(dm7820_status != 0)
    return dm7820_status;

  /* Reconfigure DMA */
#if RTD_PRINT_DEBUG
  printf("rtd_write_dma_fifo() : Reconfigure DMA\n");
#endif
  dm7820_status = DM7820_FIFO_DMA_Configure(p_rtd_board, DM7820_FIFO_QUEUE_0, DM7820_DMA_DEMAND_ON_PCI_TO_DM7820, rtd_dma_buffer_size);
  if(dm7820_status != 0)
    return dm7820_status;

  /* Enable & Start DMA transfer */
#if RTD_PRINT_DEBUG
  printf("rtd_write_dma_fifo() : re-enable DMA\n");
#endif
  dm7820_status = DM7820_FIFO_DMA_Enable(p_rtd_board, DM7820_FIFO_QUEUE_0, 0xFF, 0xFF);
  if(dm7820_status != 0)
    return dm7820_status;

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
static uint16_t* rtdalpao_dma_data_buffer;
/* -------------------- function prototypes -------------------- */
static DM7820_Error rtdalpao_send_analog_dither_frames(DM7820_Board_Descriptor*, double[ALPAO_DEV_N_CHANNEL]);
static DM7820_Error rtdalpao_send_digital_dither_frames(DM7820_Board_Descriptor*, char*);
static void rtdalpao_build_dither_frames(const double[ALPAO_DEV_N_CHANNEL], uint16_t*);
static uint8_t is_actuator_up_down(size_t, double);
/* ------------------------------------------------------------- */









/**
  *  \brief   Function to initialize the RTD board and ALPAO device
  *  @param   [in]    void
  *  @return  void
  */
static uint16_t rtdalpao_dithers_per_frame, rtdalpao_data_length, rtdalpao_data_size; // needs to be global because used in rtdalpao_print_data()
DM7820_Error rtdalpao_init(DM7820_Board_Descriptor* p_rtd_board, uint16_t dithers_per_frame) {
  rtdalpao_dithers_per_frame = dithers_per_frame;
  rtdalpao_data_length = ((rtdalpao_dithers_per_frame<3)?0x200:(rtdalpao_dithers_per_frame*ALPAO_DATA_LENGTH)); // in uint16_t
  rtdalpao_data_size = rtdalpao_data_length*2; // in bytes
  rtdalpao_dma_data_buffer = (uint16_t*)malloc(rtdalpao_data_size);
#if RTDALPAO_PRINT_DEBUG
  printf("rtdalpao_init() : initializing the board\n");
#endif
#if RTDALPAO_HARDWARE
  return rtd_init(p_rtd_board, rtdalpao_data_size);
#else
  return 0;
#endif
}









/**
  *  \brief   Function to start the RTD board timer
  *  @param   [in]    void
  *  @return  void
  */
DM7820_Error rtdalpao_start_timer(DM7820_Board_Descriptor* p_rtd_board) {
#if RTDALPAO_PRINT_DEBUG
  printf("rtdalpao_start_timer() : starting timer\n");
#endif
#if RTDALPAO_HARDWARE
  return rtd_start_timer(p_rtd_board);
#else
  return 0;
#endif
}









/**
  *  \brief   Function to stop the RTD board timer
  *  @param   [in]    void
  *  @return  void
  */
DM7820_Error rtdalpao_stop_timer(DM7820_Board_Descriptor* p_rtd_board) {
#if RTDALPAO_PRINT_DEBUG
  printf("rtdalpao_stop_timer() : stopping timer\n");
#endif
#if RTDALPAO_HARDWARE
  return rtd_stop_timer(p_rtd_board);
#else
  return 0;
#endif
}









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
  *  \brief   Build a block of rtdalpao_dithers_per_frame frames with dithering based on the roundup remainder.
  *  @param   [in]    const double in_data[ALPAO_N_CHANNEL] - Deformable mirror command input buffer with ALPAO_N_CHANNEL values in range [-0.25, +0.25]
  *  @param   [out]   uint16_t out_block - Preallocated output buffer of rtdalpao_data_length elements
  *  @return  void
  */
void rtdalpao_build_dither_frames(const double in_data[ALPAO_DEV_N_CHANNEL], uint16_t* out_block) {

  size_t frame_number, channel_index, main_index, sub_index, frame_index;
  double fraction[ALPAO_DEV_N_CHANNEL];
  uint16_t frame[ALPAO_DEV_N_CHANNEL];

  for ( channel_index = 0; channel_index < ALPAO_DEV_N_CHANNEL; channel_index++ ) {
    frame[channel_index] = (uint16_t) ( (alpao_device.multiplier[channel_index]*(in_data[channel_index]+alpao_device.offset[channel_index])+1.0) * ALPAO_MID_SCALE );
    fraction[channel_index] = fmod(in_data[channel_index],ALPAO_MIN_ANALOG_STEP)/ALPAO_MIN_ANALOG_STEP + ((in_data[channel_index]<=0.0)?1.0:0.0);
  }

  for (frame_number = 1; frame_number <= rtdalpao_dithers_per_frame; frame_number++) {
    frame_index = frame_number-1;
    main_index = frame_index*ALPAO_DATA_LENGTH;

    alpao_init_buffer(&out_block[main_index]);
    out_block[main_index+0] = ALPAO_START_WORD; // Start of frame
    out_block[main_index+1] = ALPAO_INIT_COUNTER; // Reset internal counter
    /* Convert double to UINT16 */
    for ( channel_index = 0; channel_index < ALPAO_DEV_N_CHANNEL; channel_index++ ) {
      sub_index = main_index+alpao_device.mapping[channel_index]+ALPAO_HEADER_LENGTH;
      out_block[sub_index] = frame[channel_index] + is_actuator_up_down(frame_number,fraction[channel_index]);
      if ( out_block[sub_index] > ALPAO_MAX_SAFE ) out_block[sub_index] = ALPAO_MAX_SAFE;
      if ( out_block[sub_index] < ALPAO_MIN_SAFE ) out_block[sub_index] = ALPAO_MIN_SAFE;
    }
    sub_index = main_index+ALPAO_N_CHANNEL+ALPAO_HEADER_LENGTH;
    out_block[sub_index] = ALPAO_END_WORD;
    out_block[sub_index] += alpao_checksum(&out_block[main_index]);
    out_block[sub_index+ALPAO_PAD_LENGTH] = 0xFEED;// End of frame
  }
}









/**
  *  \brief   Send an analog frame out with dithering.
  *           The power limitation and AD conversion is kept internal.
  *  @param   [in]    char* block - digital block data to be sent out
  *  @return  void
  */
static DM7820_Error rtdalpao_send_analog_dither_frames(DM7820_Board_Descriptor* p_rtd_board, double in_data[ALPAO_DEV_N_CHANNEL]) {
  alpao_limit_power(in_data);
  rtdalpao_build_dither_frames(in_data, rtdalpao_dma_data_buffer);
  return rtdalpao_send_digital_dither_frames(p_rtd_board, (char*)rtdalpao_dma_data_buffer);
}









/**
  *  \brief   Send digital data block out, the block size is kept internally.
  *  @param   [in]    char* block - digital block data to be sent out
  *  @return  void
  */
static DM7820_Error rtdalpao_send_digital_dither_frames(DM7820_Board_Descriptor* p_rtd_board, char* block) {
#if RTDALPAO_HARDWARE
  return rtd_write_dma_fifo(p_rtd_board, block);
#else
  return 0;
#endif
}









/**
  *  \brief   Send an analog frame out with or without dithering depending on RTDALPAO_DITHER.
  *  @param   [in]    char* block - digital block data to be sent out
  *  @return  void
  */
DM7820_Error rtdalpao_send_analog_data(DM7820_Board_Descriptor* p_rtd_board, double in_data[ALPAO_DEV_N_CHANNEL]) {
  return rtdalpao_send_analog_dither_frames(p_rtd_board, in_data);
}









/**
  *  \brief   Print the block of rtdalpao_dithers_per_frame frames
  *           The data is rtdalpao_dma_data_buffer internal to this unit
  *  @param   [in]    void
  *  @return  void
  */
void rtdalpao_print_data(void) {
  uint16_t frame_number;
  for (frame_number = 0; frame_number < rtdalpao_dithers_per_frame; frame_number++) {
    printf("\n--------------- fr = %02d ---------------\n",frame_number);
    alpao_print_frame(rtdalpao_dma_data_buffer+(frame_number*ALPAO_DATA_LENGTH));
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
  uint16_t frame_length = ALPAO_DATA_LENGTH;
  if (write_header) {
    fwrite(&(alpao_device.n_act), sizeof(uint8_t), 1, p_file);
    fwrite(&rtdalpao_dithers_per_frame, sizeof(uint16_t), 1, p_file);
    fwrite(&frame_length, sizeof(uint16_t), 1, p_file);
    fwrite(&(alpao_device.mapping), sizeof(uint8_t), ALPAO_DEV_N_CHANNEL, p_file);
    fwrite(&(alpao_device.multiplier), sizeof(int8_t), ALPAO_DEV_N_CHANNEL, p_file);
    fwrite(&(alpao_device.offset), sizeof(double), ALPAO_DEV_N_CHANNEL, p_file);
    write_header = 0;
  }
  fwrite(data, sizeof(double), ALPAO_DEV_N_CHANNEL, p_file);
  fwrite(rtdalpao_dma_data_buffer, sizeof(uint16_t), rtdalpao_data_length, p_file);
}









/**
  *  \brief   Print information about the current configuration of the librtdalpao
  *  @param   [void]
  *  @return  void
  */
void rtdalpao_print_info(void) {
  printf("---------------------- RTD board info ----------------------\n");
  printf("RTD clock frequency                  = %f [Hz]\n", RTD_CLK_FREQUENCY);
  printf("RTD DMA length                       = %d (%dx%d) [words]\n", rtdalpao_data_length, ALPAO_DATA_LENGTH, rtdalpao_dithers_per_frame);
  printf("Maximum possible refresh rate        = %f [Hz]\n",  (RTD_CLK_FREQUENCY/rtdalpao_data_length));
  printf("Dithering                            = %sabled\n", (rtdalpao_dithers_per_frame>1)?"en":"dis");
  printf("Dithers per frame                    = %d [frames]\n", rtdalpao_dithers_per_frame);
  printf("Time to clock-out 1 data transfer    = %f [us]\n", (1000000.0*rtdalpao_data_length/RTD_CLK_FREQUENCY));
  printf("------------------------ ALPAO info ------------------------\n");
  printf("ALPAO Frame length (with pad)        = %d [words]\n", ALPAO_DATA_LENGTH);
  printf("---------------------- ALPAO dev info ----------------------\n");
  printf("device type                          = %s\n", alpao_device.type); // or ALPAO_DEV_TYPE
  printf("device serial                        = %s\n", alpao_device.serial); // or ALPAO_DEV_SERIAL
  printf("max power                            = %f\n", alpao_device.max_power); // or ALPAO_DEV_MAX_POWER_SAFE
  printf("analog limit                         = %f\n", alpao_device.analog_limit); // or ALPAO_DEV_ANALOG_LIMIT
  printf("number of actuators                  = %d\n", alpao_device.n_act); // or ALPAO_DEV_N_CHANNEL for static arrays definitions
  printf("actuator mapping                     = [ind,  \tmap,\tmul,\toffset    ]\n");
  uint8_t act;
  for(act = 0; act < alpao_device.n_act; act++)
    printf("                                       [%03d,\t%03d,\t%+03d,\t%+1.7f]\n", act, alpao_device.mapping[act], alpao_device.multiplier[act], alpao_device.offset[act]);
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
DM7820_Error rtdalpao_clean(DM7820_Board_Descriptor* p_rtd_board) {
#if RTDALPAO_PRINT_DEBUG
  printf("rtdalpao_clean()\n");
#endif
#if RTDALPAO_HARDWARE
  return rtd_cleanup(p_rtd_board);
#else
  return 0;
#endif
}
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RTD ALPAO Section end %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */



