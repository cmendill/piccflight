#include <stdio.h>
#include <unistd.h>
#include <math.h>

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
  
  unsigned short t = 0, x = 0, y = 0, width=8, height=16, act=0;
  double amplitude=0.1, k=1.5, omega=0.002, x0=3.5, y0=3.5;

  for (t = 0; t < 5000; t++) {

    for (y = 0; y < height; y++) {
      for (x = 0; x < width; x++) {
        act = x+width*y;
        data[act] = amplitude*sin( k*sqrt(pow((x-x0),2)+pow((y-y0),2)) - omega*t );
      }
    }

    /* ------ build and send the frame ------ */
    rtdalpao_send_analog_data(data);
    /* -------------------------------------- */

    /* Sleep to simulate camera */
    usleep(period_us);

  }

  /* Fill channel data array with 0 */
  for (index = 0; index < ALPAO_DEV_N_CHANNEL; index++) {
    data[index] = 0.0;
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