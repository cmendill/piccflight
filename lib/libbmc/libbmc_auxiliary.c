#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include <fitsio.h>

#include "libbmc_auxiliary.h"
#include "libbmc.h"


/**************************************************************/
/* LIBBMC_AUX_LIMIT_ACTUATOR_ADDRESS                          */
/**************************************************************/
signed char libbmc_aux_limit_actuator_address (int* p_address, int min_address, int max_address) {
  if (*p_address > max_address) {
    *p_address = max_address;
    return -1; // p_address modified
  } else if (*p_address < min_address) {
    *p_address = min_address;
    return -1; // p_address modified
  } else {
    return 0; // p_address unmodified
  }
}






/**************************************************************/
/* LIBBMC_AUX_LIMIT_ACTUATOR_VALUE                            */
/**************************************************************/
signed char libbmc_aux_limit_actuator_value (float* p_value, float min_value, float max_value) {
  if (*p_value > max_value) {
    *p_value = max_value;
    return -1; // p_value modified
  } else if (*p_value < min_value) {
    *p_value = min_value;
    return -1; // p_value modified
  } else {
    return 0; // p_value unmodified
  }
}






#define MAXCHAR 25
/**************************************************************/
/* LIBBMC_AUX_LOAD_CMD_TXT_FILE                               */
/**************************************************************/
signed char libbmc_aux_load_cmd_vact_file (const char* p_filename, float max_voltage, float act[LIBBMC_NACT]) {

  FILE *fp;
  char str[MAXCHAR];
  size_t iact = 0;

  float temp_act[LIBBMC_NACT] = {0};

  fp = fopen(p_filename, "r");
  if (fp == NULL){
    printf("Could not open file %s\n",p_filename);
    return -1;
  }

  while (fgets(str, MAXCHAR, fp) != NULL) {
    temp_act[iact] = strtof(str, NULL);
    if (temp_act[iact] >= max_voltage) {
      printf("Voltage incorrect : act[%ld] = %f\n; actuator values not loaded.\n", iact, temp_act[iact]);
      return -1;
    }
    // printf("act[%ld] = %f\n", iact, act[iact]);
    iact++;
  }
  fclose(fp);

  memcpy( act, temp_act, sizeof(float)*LIBBMC_NACT );

  return 0;
}

#ifdef _LIBBMC_ROUND_MAP
#define BMC_NLOC 34
#endif

#ifdef _LIBBMC_SQUARE_MAP
#define BMC_NLOC 32
#endif

signed char libbmc_aux_print_acts (float act_values[LIBBMC_NACT]) {
  size_t i;
  size_t mask[LIBBMC_NACT] = LIBBMC_ACTUATOR_MASK;
  size_t* act_mask = mask;
  for (i=0; i<(BMC_NLOC*BMC_NLOC); i++) {
    if (i == *act_mask) {
      printf ("%05.1f, ", *act_values);
      act_mask++;
      act_values++;
    } else {
      printf ("       ");
    }
    if ((i%BMC_NLOC) == (BMC_NLOC-1))
      printf ("\b \n");
  }
  return 0;
}

signed char libbmc_aux_print_tstpnts (float tstpnt_values[LIBBMC_NTSTPNT]) {
  size_t i;
  printf ("[");
  for(i = 0; i<LIBBMC_NTSTPNT; i++) {
    printf ("%05.1f, ", tstpnt_values[i]);
  }
  printf ("\b\b]\n");
  return 0;
}
