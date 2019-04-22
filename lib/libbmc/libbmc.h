#ifndef _LIBBMC_H_
#define _LIBBMC_H_

#include <libusb.h>
#include <math.h>

// #include "libbmc_round_map.h"
#include "libbmc_square_map.h"

#define LIBBMC_DEBUG 0

#define LIBBMC_VENDOR_ID 0x04d8
#define LIBBMC_PRODUCT_ID 0x0052

#define LIBBMC_SHORT_USLEEP 50000 // 0.05 s



// ---- TX: PC to USB device -------------------------------------------------------------------------------------------

#define LIBBMC_CMD_BYTE_0 0x0a // byte 0 of all commands, both control and data


// ---- send control command: PC to USB device -------------------------------------------------------------------------
// control command specification
#define LIBBMC_CTRL_SIZE 512 // size of control commands
#define LIBBMC_CTRL_LENGTH (LIBBMC_CTRL_SIZE/2)

// control packet byte 1 values
#define LIBBMC_CTRL_CMD_TOGGLE_5VA 0xfc // toggle 5 V analog on/off
#define LIBBMC_CTRL_CMD_TOGGLE_5VD 0xfd // toggle 5 V digital on/off
#define LIBBMC_CTRL_CMD_GET_STATUS 0xfa // request status
#define LIBBMC_CTRL_CMD_SET_FAN {0xe5,0xe4,0xe3} // fan states
#define LIBBMC_CTRL_CMD_SET_TEC {0xe1,0xe2,0xec} // tec states
#define LIBBMC_CTRL_CMD_TOGGLE_LEDS 0xbb // toggle leds on off


#define LIBBMC_CTRL_CMD_TOGGLE_HV 0xfe // toggle high voltage on/off
#define LIBBMC_CTRL_CMD_TOGGLE_CTRLR 0xe0 // toggle controller on/off
#define LIBBMC_CTRL_CMD_SET_POT 0xeb // set pot to range 100/150/200/225 V
#define LIBBMC_CTRL_CMD_SET_VOLT_RANGE {0x01,0x02,0x03,0x04} // byte 2 and 3 of toggle high voltage on/off and toggle controller on/off commands (each 2 elements corresponds to 100,150,200,225 voltage range)


// ---- send actuator data: PC to USB device ---------------------------------------------------------------------------
// actuator data specification
#define LIBBMC_DATA_SIZE 2120 // size of actuator data
#define LIBBMC_DATA_LENGTH (LIBBMC_DATA_SIZE/2)

#define LIBBMC_DATA_BYTE_1 0xfb // actuator data packet byte 1 value

#define LIBBMC_NTSTPNT 11 // number of testpoints
#define LIBBMC_TESTPOINT_MAP {83,179,275,755,851,947,1043,371,467,563,659} // map of test points











// ---- RX: USB device to PC -------------------------------------------------------------------------------------------

// ---- recieve status data: USB device to PC --------------------------------------------------------------------------
// status packet specification
#define LIBBMC_STAT_SIZE LIBBMC_CTRL_SIZE 

#define LIBBMC_STAT_BYTE_0 0x03 // status packet byte 0 value
#define LIBBMC_STAT_BYTE_85 0x3e // status packet byte 85 value
#define LIBBMC_STAT_BYTE_123 0xfb // status packet byte 123 value








// ---- enums, structs and data types ----------------------------------------------------------------------------------

// Voltage range
// libbmc_status_t.range
#define LIBBMC_VOLT_RANGE_CTRL_VALUE {100.0,150.0,200.0,230.0}
enum libbmc_volt_range_enum {
  LIBBMC_VOLT_RANGE_100V, // voltage range 100 V
  LIBBMC_VOLT_RANGE_150V, // voltage range 150 V
  LIBBMC_VOLT_RANGE_200V, // voltage range 200 V
  LIBBMC_VOLT_RANGE_225V // voltage range 225 V
};

// 3.3 V and 5 V status
// libbmc_status_t.volt_3v3, libbmc_status_t.status.volt_5
#define LIBBMC_VOLT_STAT_LABEL {"LIBBMC_VOLT_ERROR",\
                                "LIBBMC_VOLT_GOOD"}
enum libbmc_volt_state_enum {
  LIBBMC_VOLT_ERROR, 
  LIBBMC_VOLT_GOOD
};

// temperature status
// libbmc_status_t.over_temp
#define LIBBMC_TEMP_STAT_LABEL {"LIBBMC_TEMP_NORM", \
                                "LIBBMC_TEMP_OVER"}
enum libbmc_temp_state_enum {
  LIBBMC_TEMP_NORM, // normal temperature
  LIBBMC_TEMP_OVER // over temperature
};

// binary status
// libbmc_status_t.supply_5va, libbmc_status_t.supply_5vd, libbmc_status_t.supply_hv, libbmc_status_t.leds
#define LIBBMC_BI_STAT_LABEL {"LIBBMC_OFF", \
                              "LIBBMC_ON"}
enum libbmc_bi_state_enum {
  LIBBMC_OFF,
  LIBBMC_ON
};

// power status
// libbmc_status_t.power
#define LIBBMC_PWR_STAT_LABEL {"LIBBMC_PWR_OFF", \
                               "LIBBMC_PWR_RAMP_UP_01", \
                               "LIBBMC_PWR_RAMP_UP_02", \
                               "LIBBMC_PWR_RAMP_UP_03", \
                               "LIBBMC_PWR_RAMP_UP_04", \
                               "LIBBMC_PWR_RAMP_UP_05", \
                               "LIBBMC_PWR_ON", \
                               "LIBBMC_PWR_UNKNOWN_07", \
                               "LIBBMC_PWR_UNKNOWN_08", \
                               "LIBBMC_PWR_RAMP_DOWN_09"}
enum libbmc_pwr_state_enum {
  LIBBMC_PWR_OFF, // power off
  LIBBMC_PWR_RAMP_UP_01, // voltage ramping up to on state 1
  LIBBMC_PWR_RAMP_UP_02, // voltage ramping up to on state 2
  LIBBMC_PWR_RAMP_UP_03, // voltage ramping up to on state 3
  LIBBMC_PWR_RAMP_UP_04, // voltage ramping up to on state 4
  LIBBMC_PWR_RAMP_UP_05, // voltage ramping up to on state 5
  LIBBMC_PWR_ON, // power on
  LIBBMC_PWR_UNKNOWN_07, // unknown
  LIBBMC_PWR_UNKNOWN_08, // unknown
  LIBBMC_PWR_RAMP_DOWN_09 // power ramping down to off state 9
};


// TEC status
// libbmc_status_t.tec
#define LIBBMC_TEC_CTRL_LABEL {"LIBBMC_TEC_LOW", \
                               "LIBBMC_TEC_MID", \
                               "LIBBMC_TEC_HIGH"}
enum libbmc_tec_state_enum {
  LIBBMC_TEC_LOW,
  LIBBMC_TEC_MID,
  LIBBMC_TEC_HIGH
};

// Fan status
// libbmc_status_t.fan
#define LIBBMC_FAN_CTRL_LABEL {"LIBBMC_FAN_OFF",\
                               "LIBBMC_FAN_LOW",\
                               "LIBBMC_FAN_HIGH"}
enum libbmc_fan_state_enum {
  LIBBMC_FAN_OFF,
  LIBBMC_FAN_LOW,
  LIBBMC_FAN_HIGH
};




#define LIBBMC_ERR_NAME {"LIBBMC_SUCCESS", \
                         "LIBBMC_ERROR_IO", \
                         "LIBBMC_ERROR_INVALID_PARAM", \
                         "LIBBMC_ERROR_ACCESS", \
                         "LIBBMC_ERROR_NO_DEVICE", \
                         "LIBBMC_ERROR_NOT_FOUND", \
                         "LIBBMC_ERROR_BUSY", \
                         "LIBBMC_ERROR_TIMEOUT", \
                         "LIBBMC_ERROR_OVERFLOW", \
                         "LIBBMC_ERROR_PIPE", \
                         "LIBBMC_ERROR_INTERRUPTED", \
                         "LIBBMC_ERROR_NO_MEM", \
                         "LIBBMC_ERROR_NOT_SUPPORTED", \
                         "LIBBMC_ERROR_CONTROLLER_DISABLED", \
                         "LIBBMC_ERROR_CONTROLLER_ENABLED", \
                         "LIBBMC_ERROR_TESTPOINTS_UNACCEPTABLE", \
                         "LIBBMC_ERROR_INVALID_STATUS", \
                         "LIBBMC_ERROR_OTHER"}
enum libbmc_error_enum {
  // error codes from libusb_error
  LIBBMC_SUCCESS = LIBUSB_SUCCESS, // 0
  LIBBMC_ERROR_IO = LIBUSB_ERROR_IO, // -1
  LIBBMC_ERROR_INVALID_PARAM = LIBUSB_ERROR_INVALID_PARAM, // -2
  LIBBMC_ERROR_ACCESS = LIBUSB_ERROR_ACCESS, // -3
  LIBBMC_ERROR_NO_DEVICE = LIBUSB_ERROR_NO_DEVICE, // -4
  LIBBMC_ERROR_NOT_FOUND = LIBUSB_ERROR_NOT_FOUND, // -5
  LIBBMC_ERROR_BUSY = LIBUSB_ERROR_BUSY, // -6
  LIBBMC_ERROR_TIMEOUT = LIBUSB_ERROR_TIMEOUT, // -7
  LIBBMC_ERROR_OVERFLOW = LIBUSB_ERROR_OVERFLOW, // -8
  LIBBMC_ERROR_PIPE = LIBUSB_ERROR_PIPE, // -9
  LIBBMC_ERROR_INTERRUPTED = LIBUSB_ERROR_INTERRUPTED, // -10
  LIBBMC_ERROR_NO_MEM = LIBUSB_ERROR_NO_MEM, // -11
  LIBBMC_ERROR_NOT_SUPPORTED = LIBUSB_ERROR_NOT_SUPPORTED, // -12
  // BMC specific error codes
  LIBBMC_ERROR_CONTROLLER_DISABLED = -13, // for operations that require the controller to be enabled
  LIBBMC_ERROR_CONTROLLER_ENABLED = -14, // for operations that require the controller to be disabled
  LIBBMC_ERROR_TESTPOINTS_UNACCEPTABLE = -15, // for operations that require the controller to be disabled
  LIBBMC_ERROR_INVALID_STATUS = -16, // status check bytes do not match
  LIBBMC_ERROR_OTHER = LIBUSB_ERROR_OTHER // -99
};
typedef enum libbmc_error_enum libbmc_error_t;
#define LIBBMC_ERR_DESC {"Success", \
                         "Input/Output Error", \
                         "Invalid parameter", \
                         "Access denied (insufficient permissions)", \
                         "No such device (it may have been disconnected)", \
                         "Entity not found", \
                         "Resource busy", \
                         "Operation timed out", \
                         "Overflow", \
                         "Pipe error", \
                         "System call interrupted (perhaps due to signal)", \
                         "Insufficient memory", \
                         "Operation not supported or unimplemented on this platform", \
                         "Controller is disabled", /* BMC specific : for operations that require the controller to be enabled */ \
                         "Controller is enabled", /* BMC specific : for operations that require the controller to be disabled */ \
                         "One or more test points not within range", /* BMC specific : one or more test points not within range */ \
                         "Invalid status", /* BMC specific : status check bytes not matched */ \
                         "Other error"} /* any oter error */
#define LIBBMC_ERROR_COUNT 18



// ---- status data structure ------------------------------------------------------------------------------------------
#define LIBBMC_STAT_HLBYTE(HBYTE,LBYTE) (HBYTE*256+LBYTE)
#define LIBBMC_VOLT_REF 2.048
#define LIBBMC_STAT_3RAIL1_V(HBYTE,LBYTE) ((LIBBMC_VOLT_REF*LIBBMC_STAT_HLBYTE(HBYTE,LBYTE)/4095.0)*3)
#define LIBBMC_STAT_3RAIL2_V(HBYTE,LBYTE) ((LIBBMC_VOLT_REF*LIBBMC_STAT_HLBYTE(HBYTE,LBYTE)/4095.0)*2)
#define LIBBMC_STAT_5RAIL1_V(HBYTE,LBYTE) ((LIBBMC_VOLT_REF*LIBBMC_STAT_HLBYTE(HBYTE,LBYTE)/4095.0)*3)
#define LIBBMC_STAT_5RAIL2_V(HBYTE,LBYTE) ((LIBBMC_VOLT_REF*LIBBMC_STAT_HLBYTE(HBYTE,LBYTE)/4095.0)*3)
#define LIBBMC_STAT_CURRENT_MA(HBYTE,LBYTE) ((LIBBMC_VOLT_REF*LIBBMC_STAT_HLBYTE(HBYTE,LBYTE)/4095.0)*2000)
#define LIBBMC_STAT_LN_FN(VAL) (log(VAL/(4095.0-VAL)))
#define LIBBMC_STAT_TEMP_C(VAL) ((1/(0.003354016+(0.000248656*VAL) + (0.00000209*VAL*VAL))) - 273.15)
#define LIBBMC_STAT_HV_REF_V(HBYTE,LBYTE) ((LIBBMC_VOLT_REF*LIBBMC_STAT_HLBYTE(HBYTE,LBYTE)/4095.0)*2)
#define LIBBMC_STAT_VOLT_IN_V(HBYTE,LBYTE) ((LIBBMC_VOLT_REF*LIBBMC_STAT_HLBYTE(HBYTE,LBYTE)/4095.0)*11)
#define LIBBMC_STAT_TEMP_IC_C(HBYTE,LBYTE) (6.81+(0.6318-(LIBBMC_VOLT_REF*LIBBMC_STAT_HLBYTE(HBYTE,LBYTE)/4095.0))/0.00215)
#define LIBBMC_STAT_TESTPOINT_V(HBYTE,LBYTE) ((LIBBMC_VOLT_REF*LIBBMC_STAT_HLBYTE(HBYTE,LBYTE)/4095.0)*119.34)
#define LIBBMC_STAT_SERIAL(UHBYTE,LHBYTE,ULBYTE,LLBYTE) ((UHBYTE<<24)+(LHBYTE<<16)+(ULBYTE<<8)+LLBYTE)

//Status struct --> Align on 64bit boundry
struct libbmc_status_struct {
  uint8_t power;
  uint8_t supply_5va;
  uint8_t supply_5vd;
  uint8_t supply_hv; 
  uint8_t leds;
  uint8_t tec;
  uint8_t fan;
  uint8_t over_temp;
  //----
  uint8_t fw_major;
  uint8_t fw_minor;
  uint8_t range;
  uint8_t volt_3v3;
  uint8_t volt_5;
  uint8_t pad1;
  uint8_t pad2;
  uint8_t pad3;
  //----
  uint32_t serial;
  float voltage_input_v;
  //----
  float rail_3v1_v;
  float rail_3v2_v;
  //----
  float rail_5v1_v;
  float rail_5v2_v;
  //----
  float current_ma;
  float main_brd_temp_c;
  //----
  float top_brd_temp_c;
  float mid_brd_temp_c;
  //----
  float bot_brd_temp_c;
  float heatsink_temp_c;
  //----
  float ambient_temp_c;
  float sock1_temp_c;
  //----
  float sock2_temp_c;
  float hv_ref_v;
  //----
  float testpoint_v[11];
  float ic_temp_c[11];
  //----
  float hv_supp_v[2];
};

typedef struct libbmc_status_struct libbmc_status_t;


// ---- device data structure ------------------------------------------------------------------------------------------

struct libbmc_device_struct {
  libusb_device_handle* handle;
  uint8_t out_endpoint;
  uint8_t in_endpoint;
  libbmc_status_t status;
};
typedef struct libbmc_device_struct libbmc_device_t;

libbmc_error_t libbmc_open_device (libbmc_device_t*); // opens the BMC device
libbmc_error_t libbmc_close_device (libbmc_device_t*); // closes the BMC device 

libbmc_error_t libbmc_toggle_leds_on (libbmc_device_t*); // turns the leds on
libbmc_error_t libbmc_toggle_leds_off (libbmc_device_t*); // turns the leds off

libbmc_error_t libbmc_toggle_5va_on (libbmc_device_t*); // turns the 5V analog on
libbmc_error_t libbmc_toggle_5va_off (libbmc_device_t*); // turns the 5V analog off

libbmc_error_t libbmc_toggle_5vd_on (libbmc_device_t*); // turns the 5V digital on
libbmc_error_t libbmc_toggle_5vd_off (libbmc_device_t*); // turns the 5V digital off

libbmc_error_t libbmc_toggle_hv_on (libbmc_device_t*); // turns the high voltage on
libbmc_error_t libbmc_toggle_hv_off (libbmc_device_t*); // turns the high voltage off

libbmc_error_t libbmc_toggle_controller_start (libbmc_device_t*); // turns the controller on
libbmc_error_t libbmc_toggle_controller_stop (libbmc_device_t*); // turns the controller off

libbmc_error_t libbmc_set_fan (libbmc_device_t*, enum libbmc_fan_state_enum); // set the fan to a given state

libbmc_error_t libbmc_set_tec (libbmc_device_t*, enum libbmc_tec_state_enum); // set the tec to a given state

libbmc_error_t libbmc_set_range (libbmc_device_t*, enum libbmc_volt_range_enum); // set the range pot without turning hv on

libbmc_error_t libbmc_set_tstpnts (libbmc_device_t*, float[LIBBMC_NTSTPNT]); // set the test points

libbmc_error_t libbmc_set_acts_tstpnts (libbmc_device_t*, float[LIBBMC_NACT], float[LIBBMC_NTSTPNT]); // set the actuators along with test points

libbmc_error_t libbmc_set_acts (libbmc_device_t*, float[LIBBMC_NACT]); // send an actuator command

libbmc_error_t libbmc_set_acts_verify (libbmc_device_t*, float[LIBBMC_NACT]); // send an actuator command with test point verification

libbmc_error_t libbmc_get_status (libbmc_device_t*); // refresh DM state
const char* libbmc_error_name (libbmc_error_t); // return the error name
const char* libbmc_strerror (libbmc_error_t); // return readable error string

void libbmc_print_device (libbmc_device_t*); // print the device including status
void libbmc_print_status (libbmc_status_t*); // print only the device status
void libbmc_get_print_status (libbmc_device_t*); // refresh DM state and print status

#endif
