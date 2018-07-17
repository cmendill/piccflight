#ifndef _alpao_device_h
#define _alpao_device_h

#include <stdint.h>

#include "macros.h"

struct alpao_device_struct {
  char type[N_ELEMENTS(ALPAO_DEV_TYPE)];
  char serial[N_ELEMENTS(ALPAO_DEV_SERIAL)];
  double max_power;
  double analog_limit;
  uint8_t n_act;
  uint8_t mapping[ALPAO_DEV_N_CHANNEL];
  int8_t multiplier[ALPAO_DEV_N_CHANNEL];
  double offset[ALPAO_DEV_N_CHANNEL];
};
typedef struct alpao_device_struct alpao_device_t;

#endif /* _alpao_device_h */