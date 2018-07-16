#include <stdio.h>
#include <unistd.h>

#include "rtdalpao_library.h"

int main(void) {

  size_t index;
  double data[ALPAO_DEV_N_CHANNEL];

  uint32_t period_us;
  period_us = (uint16_t)(1000000.0/LWFC_FREQ);

  /* Fill channel data array with 0 */
  for (index = 0; index < ALPAO_DEV_N_CHANNEL; index++) {
    data[index] = 0.0;
  }

  /* Init RTD board */
  rtdalpao_init();

  rtdalpao_print_info();

  /* Start timer */
  rtdalpao_start_timer();

  index = ALPAO_DEV_N_CHANNEL;
  while (index--) {

    data[index] = -0.1;

    /* ------ build and send the frame ------ */
    rtdalpao_send_analog_data(data);
    /* -------------------------------------- */

    data[index] = 0.0;

    /* Sleep to simulate camera */
    usleep(period_us);
  }

  /* ------ build and send the frame ------ */
  rtdalpao_send_analog_data(data);
  /* -------------------------------------- */

  /* Sleep before stop timer */
  usleep(RTDALPAO_DATA_TRANSFER_TIME);

  /* Stop timer */
  rtdalpao_stop_timer();

  /* Sleep before cleanup */
  usleep(1500);

  /* Cleanup RTD board */
  rtdalpao_clean_close();

  return 0;

}