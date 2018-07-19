#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "dm7820_library.h"
#include "rtdalpao_library.h"

double limit_analog_value(double);
uint8_t limit_actuator_address(int);

double limit_analog_value(double value) {
  if (value > 0.25) {
    printf("Actuator analog value %f is greater than %f ", value, 0.25);
    value = 0.25;
     printf("clamping to %f\n", value);
  } else if (value < -0.25) {
    printf("Actuator analog value %f is less than %f ", value, -0.25);
    value = -0.25;
     printf("clamping to %f\n", value);
  }
  return value;
}

uint8_t limit_actuator_address(int address) {
  if (address > ALPAO_DEV_N_CHANNEL) {
    printf("Actuator %d is greater than %d ", address, ALPAO_DEV_N_CHANNEL);
    address = ALPAO_DEV_N_CHANNEL;
    printf("setting it to %d\n", address);
  } else if (address < 0) {
    printf("Actuator %d is less than %d ", address, 0);
    address = 0;
    printf("setting it to %d\n", address);
  }
  return (uint8_t)address;
}

int main( int argc, char *argv[] ) {

  DM7820_Board_Descriptor* p_rtd_board;
  DM7820_Error dm7820_status;

  double analog_value = ALPAO_MIN_ANALOG_STEP;
  uint8_t actuator = ALPAO_DEV_N_CHANNEL;
  uint8_t a_actuator = 0;
  uint8_t b_actuator = ALPAO_DEV_N_CHANNEL;
  uint8_t a_flag = 0, b_flag = 0, sweep_flag = 0, print_flag = 0, file_flag = 0;
  FILE* fp;
  uint32_t period_us = 5000;
  uint16_t n_dither = 1;

  int opt;
  while ((opt = getopt (argc, argv, "a:b:t:d:v:f:p")) != -1) {
    switch (opt) {
      case 'v': // set analog value of actuator(s)
        analog_value = limit_analog_value(atof(optarg));
        break;
      case 't': // set period for the sweep
        period_us = atol(optarg);
        break;
      case 'd': // set number of dither steps
        n_dither = atol(optarg);
        break;
      case 'a': // set actuator to start sweep
        a_actuator = limit_actuator_address(atoi(optarg));
        a_flag = 1;
        break;
      case 'b': // set to actuator end sweep
        b_actuator = limit_actuator_address(atoi(optarg));
        b_flag = 1;
        break;
      case 'f': // file to write to
        file_flag = 1;
        fp = fopen(optarg, "wb");
        break;
      case 'p': // print
        print_flag = 1;
        break;
      case 'h': // help
      default:
        printf("Usage:\n");
        printf("\tTo set an actuator to an analog value. (Use actuator value greater than %d to set all actuators)\n", ALPAO_DEV_N_CHANNEL);
        printf("\t\t%s -a or -b actuator_number -v analog_value (analog_value is clamped [-0.25,+0.25])\n", argv[0]);
        printf("\tTo do a sweep from one actuator to another with analog values\n");
        printf("\t\t%s -a sweep_from_actuator_number -b sweep_to_actuator_number -t sweep_period_us -v analog_value\n", argv[0]);
        return 0;
        break;
    }
  }

  if (a_flag && b_flag) {
    sweep_flag = 1;
  } else if (a_flag && !b_flag) {
    sweep_flag = 0;
    actuator = a_actuator;
  } else if (!a_flag && b_flag) {
    sweep_flag = 0;
    actuator = b_actuator;
  } else {
    return 0;
  }
  
  size_t index;
  int8_t pm;
  double data[ALPAO_DEV_N_CHANNEL]={0};

  //Open driver
  printf("rtd_open\n");
  if((dm7820_status = rtd_open(0, &p_rtd_board)))
    perror("rtd_open");

  //Reset board
  printf("rtd_reset\n");
  if((dm7820_status = rtd_reset(p_rtd_board)))
    perror("rtd_reset");

  //Clear all
  printf("rtd_clear_all\n");
  if((dm7820_status = rtd_clear_all(p_rtd_board)))
    perror("rtd_clear_all");

  //Init ALPAO interface
  printf("rtdalpao_init\n");
  if((dm7820_status = rtdalpao_init(p_rtd_board,n_dither)))
    perror("rtdalpao_init");

  //Print ALPAO info
  rtdalpao_print_info();
  
  /* Start timer */
  dm7820_status = rtdalpao_start_timer(p_rtd_board);

  if (sweep_flag) {

    printf("Sweep actuators from %d to %d with %f\n", a_actuator, b_actuator, analog_value);

    pm = ((a_actuator>b_actuator)?-1:+1);
    index = a_actuator;
    do {

      data[index] = analog_value;

      /* ------ build and send the frame ------ */
      dm7820_status = rtdalpao_send_analog_data(p_rtd_board,data);
      /* -------------------------------------- */
      if (print_flag) rtdalpao_print_data();
      if (file_flag) rtdalpao_write_data_to_file(fp, data);

      data[index] = 0.0;

      /* Sleep to simulate camera */
      usleep(period_us);

      index = index+pm;

    } while (index!=(b_actuator+pm));

  } else {

    /* build the analog data frame */
    if ( (actuator>=0) && (actuator<ALPAO_DEV_N_CHANNEL) ) {
      printf("Setting actuator %d to %f\n", actuator, analog_value);
      data[actuator] = analog_value;
    } else if (actuator==ALPAO_DEV_N_CHANNEL) {
      printf("Setting all actuators to %f\n", analog_value);
      for (index = 0; index < ALPAO_DEV_N_CHANNEL; index++) {
        data[index] = analog_value;
      }
    } else {
      printf("Setting all actuators to %f\n", 0.0);
      for (index = 0; index < ALPAO_DEV_N_CHANNEL; index++) {
        data[index] = 0.0;
      }
    }

    /* ------ build and send the frame ------ */
    dm7820_status = rtdalpao_send_analog_data(p_rtd_board, data);
    /* -------------------------------------- */
    if (print_flag) rtdalpao_print_data();
    if (file_flag) rtdalpao_write_data_to_file(fp, data);
  }

  /* Sleep before stop timer */
  usleep(1500);

  /* Stop timer */
  dm7820_status = rtdalpao_stop_timer(p_rtd_board);

  /* Sleep before cleanup */
  usleep(1500);

  
  //Cleanup ALPAO interface
  printf("Open\n");
  if((dm7820_status = rtdalpao_clean(p_rtd_board)))
    perror("rtdalpao_clean");

  //Close driver
  printf("Open\n");
  if((dm7820_status = rtd_close(p_rtd_board)))
    perror("rtd_close");

  //Close file
  if (file_flag) fclose(fp);

  return 0;

}