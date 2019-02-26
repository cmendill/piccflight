#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include "libbmc.h"

#if LIBBMC_DEBUG
static void _libbmc_print_transfer (uint8_t*, int);
#endif
static libbmc_error_t _libbmc_parse_status (uint8_t[LIBBMC_STAT_SIZE], libbmc_status_t*);
static libbmc_error_t _libbmc_write (libbmc_device_t*, uint8_t*, uint16_t); // send data : PC to USB device
static libbmc_error_t _libbmc_read (libbmc_device_t*, uint8_t[LIBBMC_STAT_SIZE]); // recieve data : USB device to PC
static libbmc_error_t _libbmc_transaction (libbmc_device_t*, uint8_t*, uint16_t); // send data and receive status
static libbmc_error_t _libbmc_init_status (libbmc_device_t*);

// all toggles are private, instead use *_on and *_off
static libbmc_error_t _libbmc_toggle_5va (libbmc_device_t*);
static libbmc_error_t _libbmc_toggle_5vd (libbmc_device_t*);
static libbmc_error_t _libbmc_toggle_hv (libbmc_device_t*);
static libbmc_error_t _libbmc_toggle_controller (libbmc_device_t*);
// also private
static libbmc_error_t _libbmc_set_acts (libbmc_device_t*, float[LIBBMC_NACT]);
static libbmc_error_t _libbmc_set_tstpnts (libbmc_device_t*, float[LIBBMC_NTSTPNT]);
static libbmc_error_t _libbmc_set_acts_tstpnts (libbmc_device_t*, float[LIBBMC_NACT], float[LIBBMC_NTSTPNT]);










libusb_context* p_ctx = NULL; // a libusb session global










/**********************************************************************************************************************/
/* LIBBMC_OPEN_DEVICE                                                                                                 */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - iterate through all usb devices and find the BMC dm using the vendor_id and product_id                          */
/*  - the device is open when this function returns                                                                   */
/**********************************************************************************************************************/
libbmc_error_t libbmc_open_device (libbmc_device_t* p_libbmc_device) {
  libusb_device **pp_devs; // array of all libusb_device pointers
  ssize_t dev_count; // device count
  int ret0; // for return values of libusb_init ()

  ret0 = libusb_init (&p_ctx); // initialize libusb library
  if (ret0 < 0) {
#if LIBBMC_DEBUG
    printf ("BMC: libusb_init failed : %s\n", libbmc_error_name (ret0));
#endif
    return LIBUSB_ERROR_OTHER;
  }
#if LIBBMC_DEBUG
  printf ("BMC: libusb_init successful\n");
#endif

  libusb_set_debug (p_ctx, 3); //set verbosity level to 3, as suggested in the libusb documentation

  dev_count = libusb_get_device_list (p_ctx, &pp_devs); // get the list of devices
  if (dev_count < 0) // libusb_get_device_list () failed
    return dev_count; // return error

  // libusb_get_device_list () successful
#if LIBBMC_DEBUG
  printf ("BMC: libusb_get_device_list successful\n");
#endif

  struct libusb_device_descriptor dev_desc; // device descriptor
  ssize_t i; // for iterating through the list of devices
  int ret1; // for return values of libusb_get_device_descriptor ()

  for (i = 0; i < dev_count; i++) { // iterate through devices
    ret1 = libusb_get_device_descriptor (pp_devs[i], &dev_desc); // get device descriptor
    if (ret1 < 0) { // libusb_get_device_descriptor () failed
#if LIBBMC_DEBUG
      printf ("BMC: libusb_get_device_descriptor failed : %s\n", libbmc_error_name (ret1));
#endif
      libusb_free_device_list (pp_devs, 1); // free the list of devices
      return ret1; // return error
    }

    // libusb_get_device_descriptor () successful
#if LIBBMC_DEBUG
    printf ("BMC: libusb_get_device_descriptor successful\n");
#endif

    if ((dev_desc.idVendor == LIBBMC_VENDOR_ID) && (dev_desc.idProduct == LIBBMC_PRODUCT_ID)) { // dm found
      int ret2; // for return values of libusb_open ()
      ret2 = libusb_open (pp_devs[i], &p_libbmc_device->handle); // open device
      if (ret2 < 0) { // libusb_open () failed
#if LIBBMC_DEBUG
        printf ("BMC: libusb_open failed : %s\n", libbmc_error_name (ret2));
#endif
        libusb_close (p_libbmc_device->handle); // close opened device
        libusb_free_device_list (pp_devs, 1); // free the list of devices
        return ret2; // return error
      }

      // libusb_open successful
#if LIBBMC_DEBUG
      printf ("BMC: libusb_open successful\n");
#endif

      int ret3; // for return values of libusb_get_config_descriptor ()
      struct libusb_config_descriptor* p_config_desc; // config descriptor
      ret3 = libusb_get_config_descriptor (pp_devs[i], 0, &p_config_desc); // get config descriptor
      if (ret3 < 0) { // libusb_get_config_descriptor () failed
#if LIBBMC_DEBUG
        printf ("BMC: libusb_get_config_descriptor failed : %s\n", libbmc_error_name (ret3));
#endif
        libusb_close (p_libbmc_device->handle); // close opened device
        libusb_free_device_list (pp_devs, 1); // free the list of devices
        libusb_free_config_descriptor (p_config_desc); // free the config descriptor
        return ret3; // return error
      }

      // libusb_get_config_descriptor () successful
#if LIBBMC_DEBUG
      printf ("BMC: libusb_get_config_descriptor successful\n");
#endif

      // The DM (libusb_device_descriptor)dev_desc has only one (libusb_config_descriptor*)p_config_desc (i.e. dev_desc.bNumConfigurations==1) so no need to enumerate configuration descriptors
      // That (libusb_config_descriptor*)p_config_desc has only one (libusb_interface)interface (i.e. p_config_desc->bNumInterfaces==1) so no need to enumerate interfaces
      // That (libusb_interface)p_config_desc->interface[0] has only one (libusb_interface_descriptor*)altsetting (i.e. p_config_desc->interface[0].num_altsetting==1) so no need to enumerate altsettings
      // That (libusb_interface_descriptor*)altsettings has only two (libusb_endpoint_descriptor*)endpoints (i.e. dev_desc.bNumEndpoints==2) so no need to enumerate endpoints
      // Those two (libusb_endpoint_descriptor*)endpoints each have an (uint8_t)bEndpointAddress

      uint8_t endpoint0, endpoint1;
      endpoint0 = p_config_desc->interface[0].altsetting[0].endpoint[0].bEndpointAddress;
      endpoint1 = p_config_desc->interface[0].altsetting[0].endpoint[1].bEndpointAddress;

      if (endpoint0 & 0x80) { // endpoint0 = 0x8x
        p_libbmc_device->in_endpoint = endpoint0; // 0x81
        p_libbmc_device->out_endpoint = endpoint1; // 0x01
      } else { // endpoint0 = 0x0x
        p_libbmc_device->in_endpoint = endpoint1; // 0x81
        p_libbmc_device->out_endpoint = endpoint0; // 0x01
      }

      libusb_free_device_list (pp_devs, 1); // free the list of devices
      libusb_free_config_descriptor (p_config_desc); // free the config descriptor

      // configure the device

      int ret4; // for return values of libusb_set_auto_detach_kernel_driver ()
      ret4 = libusb_set_auto_detach_kernel_driver (p_libbmc_device->handle, 1); // auto detach the kernel driver when claiming
      if (ret4 < 0) { // libusb_set_auto_detach_kernel_driver () failed
#if LIBBMC_DEBUG
        printf ("BMC: libusb_set_auto_detach_kernel_driver failed : %s\n", libbmc_error_name (ret4));
#endif
        libusb_close (p_libbmc_device->handle); // close opened device
        return ret4; // return error
      }

      // libusb_set_auto_detach_kernel_driver () successful
#if LIBBMC_DEBUG
      printf ("BMC: libusb_set_auto_detach_kernel_driver successful\n");
#endif

      int ret5; // for return values of libusb_claim_interface ()
      ret5 = libusb_claim_interface (p_libbmc_device->handle, 0); // claim interface
      if (ret5 < 0) { // libusb_claim_interface () failed
#if LIBBMC_DEBUG
        printf ("BMC: libusb_claim_interface failed : %s\n", libbmc_error_name (ret5));
#endif
        libusb_release_interface (p_libbmc_device->handle, 0); // release claimed interface
        libusb_close (p_libbmc_device->handle); // close opened device
        return ret5; // return error
      }

      // libusb_claim_interface () successful
#if LIBBMC_DEBUG
      printf ("BMC: libusb_claim_interface successful\n");
#endif

      int ret6; // for return values of libbmc_get_status ()
      ret6 = _libbmc_init_status (p_libbmc_device); // initialize the device and status fields
      if (ret6 < 0) { // libbmc_get_status () failed
#if LIBBMC_DEBUG
        printf ("BMC: libbmc_get_status failed : %s\n", libbmc_error_name (ret6));
#endif
        libusb_release_interface (p_libbmc_device->handle, 0); // release claimed interface
        libusb_close (p_libbmc_device->handle); // close opened device
        return ret6; // return error
      }

      // libbmc_get_status () successful
#if LIBBMC_DEBUG
      printf ("BMC: libbmc_get_status successful\n");
#endif

      return ret6; // libbmc_open_device () successful
    }
  }
  return LIBBMC_ERROR_NO_DEVICE;
}










/**********************************************************************************************************************/
/* LIBBMC_CLOSE_DEVICE                                                                                                */
/* p_libbmc_device : device handle                                                                                    */
/* libbmc_error_t return : error                                                                                      */
/*  - close a device opened and claimed with libbmc_open_device ()                                                    */
/**********************************************************************************************************************/
libbmc_error_t libbmc_close_device (libbmc_device_t* p_libbmc_device) {
  int ret1; // for return values of libusb_release_interface ()

  ret1 = libusb_release_interface (p_libbmc_device->handle, 0); // release claimed interface

  if (ret1 != 0) { // libusb_release_interface () failed
#if LIBBMC_DEBUG
    printf ("BMC: libusb_release_interface failed : %s\n", libbmc_error_name (ret1));
#endif
    return ret1; // return error
  }

  // libusb_release_interface () successfull
#if LIBBMC_DEBUG
  printf ("BMC: libusb_release_interface successful\n");
#endif

  libusb_close (p_libbmc_device->handle); // close device

  libusb_exit (p_ctx); //close the session
  return ret1; // libbmc_close_device () successful
}










/**********************************************************************************************************************/
/* _LIBBMC_WRITE                                                                                                      */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* uint8_t* tx_data : data to be written                                                                              */
/* uint16_t size : size of data to be written                                                                         */
/* libbmc_error_t return : error                                                                                      */
/*  - write to the DM (from PC to USB device)                                                                         */
/*  - write size is variable                                                                                          */
/**********************************************************************************************************************/
static libbmc_error_t _libbmc_write (libbmc_device_t* p_libbmc_device, uint8_t* tx_data, uint16_t size) {
  int ret1; // for return values of libusb_bulk_transfer ()
  int n_tx=0;

#if LIBBMC_DEBUG
  _libbmc_print_transfer (tx_data, size); // print the transferred data
#endif

  ret1 = libusb_bulk_transfer (p_libbmc_device->handle, p_libbmc_device->out_endpoint, (unsigned char*)tx_data, size, &n_tx, 0); // transfer data on the endpoint

#if LIBBMC_DEBUG
  if ((ret1 == 0) && (n_tx == size)) // libusb_bulk_transfer () successfull and transferred all data
    printf ("tx success\ntx %d on out ep 0x%02x\n", n_tx, p_libbmc_device->out_endpoint);
  else // libusb_bulk_transfer () failed
    printf ("tx failed\n%s\ntx %d on out ep 0x%02x\n", libbmc_error_name (ret1), n_tx, p_libbmc_device->out_endpoint);
#endif

  return ret1; // may or may not be succesfull 
}










/**********************************************************************************************************************/
/* _LIBBMC_READ                                                                                                       */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* uint8_t rx_data[LIBBMC_STAT_SIZE] : pointer to hold read data                                                      */
/* libbmc_error_t return : error                                                                                      */
/*  - read from the DM (from USB device to PC)                                                                        */
/*  - always reads LIBBMC_STAT_SIZE size data                                                                         */
/**********************************************************************************************************************/
static libbmc_error_t _libbmc_read (libbmc_device_t* p_libbmc_device, uint8_t rx_data[LIBBMC_STAT_SIZE]) {
  int ret1; // for return values of libusb_bulk_transfer ()
  int n_rx=0;

  ret1 = libusb_bulk_transfer (p_libbmc_device->handle, p_libbmc_device->in_endpoint, (unsigned char*)rx_data, LIBBMC_STAT_SIZE, &n_rx, 0); // transfer data on the endpoint

#if LIBBMC_DEBUG
  if ((ret1 == 0) && (n_rx == LIBBMC_STAT_SIZE)) // libusb_bulk_transfer () successfull and transferred all data
    printf ("rx success\nrx %d on in ep 0x%02x\n", n_rx, p_libbmc_device->in_endpoint);
  else // libusb_bulk_transfer () failed
    printf ("rx failed\n%s\nrx %d on in ep 0x%02x\n", libbmc_error_name (ret1), n_rx, p_libbmc_device->in_endpoint);
#endif

#if LIBBMC_DEBUG
  _libbmc_print_transfer (rx_data, LIBBMC_STAT_SIZE); // print the transferred data
#endif

  return ret1; // may or may not be succesfull 
}










/**********************************************************************************************************************/
/* _LIBBMC_TRANSACTION                                                                                                */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* uint8_t* tx_data : data to write                                                                                   */
/* uint16_t size : number of bytes to write                                                                           */
/* libbmc_error_t return : error                                                                                      */
/*  - write to the DM (from PC to USB device)                                                                         */
/*  - followed by a get status to update the device status stucture                                                   */
/**********************************************************************************************************************/
static libbmc_error_t _libbmc_transaction (libbmc_device_t* p_libbmc_device, uint8_t* tx_data, uint16_t size) {
  int ret1; // for return values of _libbmc_write ()
  ret1 = _libbmc_write (p_libbmc_device, tx_data, size);
  usleep (LIBBMC_SHORT_USLEEP); // wait to settle
  if (ret1 != 0) // _libbmc_write () failed
    return ret1; // return error
  else // _libbmc_write () successful
    return libbmc_get_status (p_libbmc_device); // return libbmc_get_status ()
}










/**********************************************************************************************************************/
/* _LIBBMC_PARSE_STATUS                                                                                               */
/* uint8_t rx_data[LIBBMC_STAT_SIZE] : status data packet                                                             */
/* libbmc_status_t* p_libbmc_status : pointer to status struct                                                        */
/* libbmc_error_t return : error                                                                                      */
/*  - parse the received data packet in to a status struct                                                            */
/**********************************************************************************************************************/
static libbmc_error_t _libbmc_parse_status (uint8_t rx_data[LIBBMC_STAT_SIZE], libbmc_status_t* p_libbmc_status) {
  if ((rx_data[0] == LIBBMC_STAT_BYTE_0) && (rx_data[85] == LIBBMC_STAT_BYTE_85) && (rx_data[123] == LIBBMC_STAT_BYTE_123)) { // verify check bytes 0, 85, 123
    p_libbmc_status->power = rx_data[1];
    p_libbmc_status->supply_5va = rx_data[2];
    p_libbmc_status->supply_5vd = rx_data[3];
    p_libbmc_status->supply_hv = rx_data[4];

    p_libbmc_status->rail_3v1_v = LIBBMC_STAT_3RAIL1_V(rx_data[5],rx_data[6]);
    p_libbmc_status->rail_3v2_v = LIBBMC_STAT_3RAIL2_V(rx_data[7],rx_data[8]);
    p_libbmc_status->rail_5v1_v = LIBBMC_STAT_5RAIL1_V(rx_data[9],rx_data[10]);
    p_libbmc_status->rail_5v2_v = LIBBMC_STAT_5RAIL2_V(rx_data[21],rx_data[22]);

    p_libbmc_status->hv_supp_v[0] = LIBBMC_STAT_TESTPOINT_V(rx_data[11],rx_data[12]);
    p_libbmc_status->hv_supp_v[1] = LIBBMC_STAT_TESTPOINT_V(rx_data[38],rx_data[39]);

    p_libbmc_status->current_ma = LIBBMC_STAT_CURRENT_MA(rx_data[13],rx_data[14]);

    p_libbmc_status->main_brd_temp_c = LIBBMC_STAT_TEMP_C(LIBBMC_STAT_LN_FN(LIBBMC_STAT_HLBYTE(rx_data[15],rx_data[16])));
    p_libbmc_status->heatsink_temp_c = LIBBMC_STAT_TEMP_C(LIBBMC_STAT_LN_FN(LIBBMC_STAT_HLBYTE(rx_data[17],rx_data[18])));
    p_libbmc_status->ambient_temp_c = LIBBMC_STAT_TEMP_C(LIBBMC_STAT_LN_FN(LIBBMC_STAT_HLBYTE(rx_data[19],rx_data[20])));
    p_libbmc_status->top_brd_temp_c = LIBBMC_STAT_TEMP_C(LIBBMC_STAT_LN_FN(LIBBMC_STAT_HLBYTE(rx_data[40],rx_data[41])));
    p_libbmc_status->mid_brd_temp_c = LIBBMC_STAT_TEMP_C(LIBBMC_STAT_LN_FN(LIBBMC_STAT_HLBYTE(rx_data[100],rx_data[101])));
    p_libbmc_status->bot_brd_temp_c = LIBBMC_STAT_TEMP_C(LIBBMC_STAT_LN_FN(LIBBMC_STAT_HLBYTE(rx_data[70],rx_data[71])));
    p_libbmc_status->sock1_temp_c = LIBBMC_STAT_TEMP_C(LIBBMC_STAT_LN_FN(LIBBMC_STAT_HLBYTE(rx_data[42],rx_data[43])));
    p_libbmc_status->sock2_temp_c = LIBBMC_STAT_TEMP_C(LIBBMC_STAT_LN_FN(LIBBMC_STAT_HLBYTE(rx_data[44],rx_data[45])));

    p_libbmc_status->hv_ref_v = LIBBMC_STAT_HV_REF_V(rx_data[23],rx_data[24]);
    p_libbmc_status->voltage_input_v = LIBBMC_STAT_VOLT_IN_V(rx_data[25],rx_data[26]);
    p_libbmc_status->volt_3v3 = rx_data[27];
    p_libbmc_status->volt_5 = rx_data[28];
    p_libbmc_status->ic_temp_c[ 0] = LIBBMC_STAT_TEMP_IC_C(rx_data[30],rx_data[31]);
    p_libbmc_status->ic_temp_c[ 1] = LIBBMC_STAT_TEMP_IC_C(rx_data[32],rx_data[33]);
    p_libbmc_status->ic_temp_c[ 2] = LIBBMC_STAT_TEMP_IC_C(rx_data[34],rx_data[35]);
    p_libbmc_status->ic_temp_c[ 3] = LIBBMC_STAT_TEMP_IC_C(rx_data[60],rx_data[61]);
    p_libbmc_status->ic_temp_c[ 4] = LIBBMC_STAT_TEMP_IC_C(rx_data[62],rx_data[63]);
    p_libbmc_status->ic_temp_c[ 5] = LIBBMC_STAT_TEMP_IC_C(rx_data[64],rx_data[65]);
    p_libbmc_status->ic_temp_c[ 6] = LIBBMC_STAT_TEMP_IC_C(rx_data[76],rx_data[77]);
    p_libbmc_status->ic_temp_c[ 7] = LIBBMC_STAT_TEMP_IC_C(rx_data[90],rx_data[91]);
    p_libbmc_status->ic_temp_c[ 8] = LIBBMC_STAT_TEMP_IC_C(rx_data[92],rx_data[93]);
    p_libbmc_status->ic_temp_c[ 9] = LIBBMC_STAT_TEMP_IC_C(rx_data[94],rx_data[95]);
    p_libbmc_status->ic_temp_c[10] = LIBBMC_STAT_TEMP_IC_C(rx_data[106],rx_data[107]);
    p_libbmc_status->testpoint_v[ 0] = LIBBMC_STAT_TESTPOINT_V(rx_data[48],rx_data[49]);
    p_libbmc_status->testpoint_v[ 1] = LIBBMC_STAT_TESTPOINT_V(rx_data[50],rx_data[51]);
    p_libbmc_status->testpoint_v[ 2] = LIBBMC_STAT_TESTPOINT_V(rx_data[36],rx_data[37]);
    p_libbmc_status->testpoint_v[ 3] = LIBBMC_STAT_TESTPOINT_V(rx_data[78],rx_data[79]);
    p_libbmc_status->testpoint_v[ 4] = LIBBMC_STAT_TESTPOINT_V(rx_data[80],rx_data[81]);
    p_libbmc_status->testpoint_v[ 5] = LIBBMC_STAT_TESTPOINT_V(rx_data[66],rx_data[67]);
    p_libbmc_status->testpoint_v[ 6] = LIBBMC_STAT_TESTPOINT_V(rx_data[68],rx_data[69]);
    p_libbmc_status->testpoint_v[ 7] = LIBBMC_STAT_TESTPOINT_V(rx_data[108],rx_data[109]);
    p_libbmc_status->testpoint_v[ 8] = LIBBMC_STAT_TESTPOINT_V(rx_data[110],rx_data[111]);
    p_libbmc_status->testpoint_v[ 9] = LIBBMC_STAT_TESTPOINT_V(rx_data[96],rx_data[97]);
    p_libbmc_status->testpoint_v[10] = LIBBMC_STAT_TESTPOINT_V(rx_data[98],rx_data[99]);
    p_libbmc_status->tec = rx_data[124];
    p_libbmc_status->fan = rx_data[125];
    p_libbmc_status->over_temp = rx_data[132];
    p_libbmc_status->serial = LIBBMC_STAT_SERIAL(rx_data[126],rx_data[127],rx_data[128],rx_data[129]);
    p_libbmc_status->fw_major = rx_data[130];
    p_libbmc_status->fw_minor = rx_data[131];
    return LIBBMC_SUCCESS; // return succsess
  } else // verify check bytes failed
    return LIBBMC_ERROR_OTHER; // return error
}










/**********************************************************************************************************************/
/* _LIBBMC_TOGGLE_5VA                                                                                                 */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - toggle 5v analog                                                                                                */
/**********************************************************************************************************************/
static libbmc_error_t _libbmc_toggle_5va (libbmc_device_t* p_libbmc_device) {
  uint8_t toggle_5va_req[LIBBMC_CTRL_SIZE]={0};
  toggle_5va_req[0] = LIBBMC_CMD_BYTE_0;
  toggle_5va_req[1] = LIBBMC_CTRL_CMD_TOGGLE_5VA;
  return _libbmc_transaction (p_libbmc_device, toggle_5va_req, LIBBMC_CTRL_SIZE);
}
/**********************************************************************************************************************/
/* LIBBMC_TOGGLE_5VA_ON                                                                                               */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - turn on 5v analog                                                                                               */
/*  - checks if 5v analog already on                                                                                  */
/**********************************************************************************************************************/
libbmc_error_t libbmc_toggle_5va_on (libbmc_device_t* p_libbmc_device) {
  int ret1, ret2; // for return values of libbmc_get_status (), _libbmc_toggle_5va ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.supply_5va == LIBBMC_ON) // 5va already on
      return LIBBMC_SUCCESS; // return success
    else { // 5va is off
      ret2 = _libbmc_toggle_5va (p_libbmc_device); // toggle 5va to on
      if (ret2 != 0) // _libbmc_toggle_5va () failed
        return ret2; // return error
      else { // _libbmc_toggle_5va () successful
        if (p_libbmc_device->status.supply_5va == LIBBMC_ON) // 5va on verified
          return LIBBMC_SUCCESS; // return success
        else // 5va on verification failed
          return LIBBMC_ERROR_OTHER; // return error
      }
    }
  }
}
/**********************************************************************************************************************/
/* LIBBMC_TOGGLE_5VA_OFF                                                                                              */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - turn off 5v analog                                                                                              */
/*  - checks if 5v analog already off                                                                                 */
/**********************************************************************************************************************/
libbmc_error_t libbmc_toggle_5va_off (libbmc_device_t* p_libbmc_device) {
  int ret1, ret2; // for return values of libbmc_get_status(), _libbmc_toggle_5va ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.supply_5va == LIBBMC_OFF) // 5va already off
      return LIBBMC_SUCCESS; // return success
    else { // 5va is on 
      ret2 = _libbmc_toggle_5va (p_libbmc_device); // toggle 5va to off
      if (ret2 != 0) // _libbmc_toggle_5va () failed
        return ret2; // return error
      else { // _libbmc_toggle_5va () successful
        if (p_libbmc_device->status.supply_5va == LIBBMC_OFF) // 5va off verified
          return LIBBMC_SUCCESS; // return success
        else // 5va off verification failed
          return LIBBMC_ERROR_OTHER; // return error
      }
    }
  }
}










/**********************************************************************************************************************/
/* _LIBBMC_TOGGLE_5VD                                                                                                 */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - toggle 5v digital                                                                                               */
/**********************************************************************************************************************/
static libbmc_error_t _libbmc_toggle_5vd (libbmc_device_t* p_libbmc_device) {
  uint8_t toggle_5vd_req[LIBBMC_CTRL_SIZE]={0};
  toggle_5vd_req[0] = LIBBMC_CMD_BYTE_0;
  toggle_5vd_req[1] = LIBBMC_CTRL_CMD_TOGGLE_5VD;
  return _libbmc_transaction (p_libbmc_device, toggle_5vd_req, LIBBMC_CTRL_SIZE);
}
/**********************************************************************************************************************/
/* LIBBMC_TOGGLE_5VD_ON                                                                                               */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - turn on 5v digital                                                                                              */
/*  - checks if 5v digital already on                                                                                 */
/**********************************************************************************************************************/
libbmc_error_t libbmc_toggle_5vd_on (libbmc_device_t* p_libbmc_device) {
  int ret1, ret2; // for return values of libbmc_get_status (), _libbmc_toggle_5vd ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.supply_5vd == LIBBMC_ON) // 5vd already on
      return LIBBMC_SUCCESS; // return success
    else { // 5vd is off
      ret2 = _libbmc_toggle_5vd (p_libbmc_device); // toggle 5vd to on
      if (ret2 != 0) // _libbmc_toggle_5vd () failed
        return ret2; // return error
      else { // _libbmc_toggle_5vd () successful
        if (p_libbmc_device->status.supply_5vd == LIBBMC_ON) // 5vd on verified
          return LIBBMC_SUCCESS; // return success
        else // 5vd on verification failed
          return LIBBMC_ERROR_OTHER; // return error
      }
    }
  }
}
/**********************************************************************************************************************/
/* LIBBMC_TOGGLE_5VD_OFF                                                                                              */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - turn off 5v digital                                                                                             */
/*  - checks if 5v digital already off                                                                                */
/**********************************************************************************************************************/
libbmc_error_t libbmc_toggle_5vd_off (libbmc_device_t* p_libbmc_device) {
  int ret1, ret2; // for return values of libbmc_get_status (), _libbmc_toggle_5vd ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.supply_5vd == LIBBMC_OFF) // 5vd already off
      return LIBBMC_SUCCESS; // return success
    else { // 5vd is on 
      ret2 = _libbmc_toggle_5vd (p_libbmc_device); // toggle 5vd to off
      if (ret2 != 0) // _libbmc_toggle_5vd () failed
        return ret2; // return error
      else { // _libbmc_toggle_5vd () successful
        if (p_libbmc_device->status.supply_5vd == LIBBMC_OFF) // 5vd off verified
          return LIBBMC_SUCCESS; // return success
        else // 5vd off verification failed
          return LIBBMC_ERROR_OTHER; // return error
      }
    }
  }
}










/**********************************************************************************************************************/
/* _LIBBMC_TOGGLE_HV                                                                                                  */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - toggle high voltage                                                                                             */
/**********************************************************************************************************************/
static libbmc_error_t _libbmc_toggle_hv (libbmc_device_t* p_libbmc_device) {
  uint8_t toggle_hv_req[LIBBMC_CTRL_SIZE]={0};
  toggle_hv_req[0] = LIBBMC_CMD_BYTE_0;
  toggle_hv_req[1] = LIBBMC_CTRL_CMD_TOGGLE_HV;
  return _libbmc_transaction (p_libbmc_device, toggle_hv_req, LIBBMC_CTRL_SIZE);
}
/**********************************************************************************************************************/
/* LIBBMC_TOGGLE_HV_ON                                                                                                */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - turn high voltage on                                                                                            */
/*  - checks if high voltage is already on                                                                            */
/**********************************************************************************************************************/
libbmc_error_t libbmc_toggle_hv_on (libbmc_device_t* p_libbmc_device) {
  int ret1, ret2; // for return values of libbmc_get_status (), _libbmc_toggle_hv ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.supply_hv == LIBBMC_ON) // hv already on
      return LIBBMC_SUCCESS; // return success
    else { // hv is off
      ret2 = _libbmc_toggle_hv (p_libbmc_device); // toggle hv to on
      if (ret2 != 0) // _libbmc_toggle_hv () failed
        return ret2; // return error
      else { // _libbmc_toggle_hv () successful
        if (p_libbmc_device->status.supply_hv == LIBBMC_ON) // hv on verified
          return LIBBMC_SUCCESS; // return success
        else // hv on verification failed
          return LIBBMC_ERROR_OTHER; // return error
      }
    }
  }
}
/**********************************************************************************************************************/
/* LIBBMC_TOGGLE_HV_OFF                                                                                               */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - turn high voltage off                                                                                           */
/*  - checks if high voltage is already off                                                                           */
/**********************************************************************************************************************/
libbmc_error_t libbmc_toggle_hv_off (libbmc_device_t* p_libbmc_device) {
  int ret1, ret2; // for return values of libbmc_get_status (), _libbmc_toggle_hv ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.supply_hv == LIBBMC_OFF) // hv already off
      return LIBBMC_SUCCESS; // return success
    else { // hv is on 
      ret2 = _libbmc_toggle_hv (p_libbmc_device); // toggle hv to off
      if (ret2 != 0) // _libbmc_toggle_hv () failed
        return ret2; // return error
      else { // _libbmc_toggle_hv () successful
        if (p_libbmc_device->status.supply_hv == LIBBMC_OFF) // hv off verified
          return LIBBMC_SUCCESS; // return success
        else // hv off verification failed
          return LIBBMC_ERROR_OTHER; // return error
      }
    }
  }
}









/**********************************************************************************************************************/
/* _LIBBMC_TOGGLE_CTRL                                                                                                */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - toggle start/stop of controller                                                                                 */
/**********************************************************************************************************************/
static libbmc_error_t _libbmc_toggle_controller (libbmc_device_t* p_libbmc_device) {
  uint8_t libbmc_range_volt_cmd[] = LIBBMC_CTRL_CMD_SET_VOLT_RANGE;
  uint8_t toggle_ctrl_req[LIBBMC_CTRL_SIZE]={0};
  toggle_ctrl_req[0] = LIBBMC_CMD_BYTE_0;
  toggle_ctrl_req[1] = LIBBMC_CTRL_CMD_TOGGLE_CTRLR;
  toggle_ctrl_req[2] = libbmc_range_volt_cmd[p_libbmc_device->status.range];
  return _libbmc_transaction (p_libbmc_device, toggle_ctrl_req, LIBBMC_CTRL_SIZE);
}
/**********************************************************************************************************************/
/* LIBBMC_TOGGLE_CONTROLLER_START                                                                                     */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - start controller at a given voltage range                                                                       */
/*  - checks if controller is already on                                                                              */
/**********************************************************************************************************************/
libbmc_error_t libbmc_toggle_controller_start (libbmc_device_t* p_libbmc_device) {
  int ret1, ret2; // for return values of libbmc_get_status (), _libbmc_toggle_controller ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.power == LIBBMC_PWR_ON) // controller already on
      return LIBBMC_SUCCESS; // return success
    else { // controller is off 
      ret2 = _libbmc_toggle_controller (p_libbmc_device); // toggle controller to on
      if (ret2 != 0) // _libbmc_toggle_controller () failed
        return ret2; // return error
      else { // _libbmc_toggle_controller () successful
        if (p_libbmc_device->status.power == LIBBMC_PWR_ON) // controller on verified
          return LIBBMC_SUCCESS; // return success
        else // controller on verification failed
          return LIBBMC_ERROR_OTHER; // return error
      }
    }
  }
}
/**********************************************************************************************************************/
/* LIBBMC_TOGGLE_CONTROLLER_STOP                                                                                      */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - stop controller                                                                                                 */
/*  - checks if controller is already off                                                                             */
/**********************************************************************************************************************/
libbmc_error_t libbmc_toggle_controller_stop (libbmc_device_t* p_libbmc_device) {
  int ret1, ret2; // for return values of libbmc_get_status (), _libbmc_toggle_controller ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.power == LIBBMC_PWR_OFF) // controller already on
      return LIBBMC_SUCCESS; // return success
    else { // controller is off 
      ret2 = _libbmc_toggle_controller (p_libbmc_device); // toggle controller to on
      if (ret2 != 0) // _libbmc_toggle_controller () failed
        return ret2; // return error
      else { // _libbmc_toggle_controller () successful
        if (p_libbmc_device->status.power == LIBBMC_PWR_OFF) // controller on verified
          return LIBBMC_SUCCESS; // return success
        else // controller on verification failed
          return LIBBMC_ERROR_OTHER; // return error
      }
    }
  }
}











/**********************************************************************************************************************/
/* _LIBBMC_INIT_STATUS                                                                                                */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - get dm status and initialize values not in the status packet                                                    */
/**********************************************************************************************************************/
static libbmc_error_t _libbmc_init_status (libbmc_device_t* p_libbmc_device) {
  int ret1; // for return values of libbmc_get_status ()
  ret1 = libbmc_get_status (p_libbmc_device);
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    // initialize values not in the status packet with default values
    p_libbmc_device->status.range = LIBBMC_VOLT_RANGE_100V;
    p_libbmc_device->status.leds = LIBBMC_ON;
  }
  return ret1;
}










/**********************************************************************************************************************/
/* LIBBMC_GET_STATUS                                                                                                  */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - get dm status                                                                                                   */
/*  - update the status structure in the device                                                                       */
/**********************************************************************************************************************/
libbmc_error_t libbmc_get_status (libbmc_device_t* p_libbmc_device) {
  int ret1, ret2; // for return values of _libbmc_write (), _libbmc_read ()
  uint8_t status_req[LIBBMC_CTRL_SIZE]={0};
  status_req[0] = LIBBMC_CMD_BYTE_0;
  status_req[1] = LIBBMC_CTRL_CMD_GET_STATUS;
  ret1 = _libbmc_write (p_libbmc_device, status_req, LIBBMC_CTRL_SIZE);
  if (ret1 != 0) // _libbmc_write () failed
    return ret1; // return error
  else { // _libbmc_write () successful
    uint8_t status[LIBBMC_STAT_SIZE]={0};
    ret2 = _libbmc_read (p_libbmc_device, status);
    if (ret2 != 0) // _libbmc_read () failed
      return ret2; // return error
    else // _libbmc_read () successful
      return _libbmc_parse_status (status, &(p_libbmc_device->status));
  }
  return ret2;
}










/**********************************************************************************************************************/
/* LIBBMC_SET_FAN                                                                                                     */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* enum libbmc_fan_state_enum state : fan state                                                                       */
/* libbmc_error_t return : error                                                                                      */
/*  - set fan modes off|low|high                                                                                      */
/*  - checks if fan is already in state                                                                               */
/**********************************************************************************************************************/
libbmc_error_t libbmc_set_fan (libbmc_device_t* p_libbmc_device, enum libbmc_fan_state_enum state) {
  int ret1, ret2; // for return values of libbmc_get_status (), _libbmc_transaction ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.fan == state) // fan is already in state
      return LIBBMC_SUCCESS; // return success
    else { // fan is in a different state
      uint8_t libbmc_fan_state_cmd[] = LIBBMC_CTRL_CMD_SET_FAN;
      uint8_t set_fan_req[LIBBMC_CTRL_SIZE]={0};
      set_fan_req[0] = LIBBMC_CMD_BYTE_0;
      set_fan_req[1] = libbmc_fan_state_cmd[state];
      ret2 = _libbmc_transaction (p_libbmc_device, set_fan_req, LIBBMC_CTRL_SIZE); // set fan to state
      if (ret2 != 0) // _libbmc_transaction () failed
        return ret2; // return error
      else { // _libbmc_transaction () successful
        if (p_libbmc_device->status.fan == state) // fan state verified
          return LIBBMC_SUCCESS; // return success
        else // fan state verification failed
          return LIBBMC_ERROR_OTHER; // return error
      }
    }
  }
}









/**********************************************************************************************************************/
/* LIBBMC_SET_TEC                                                                                                     */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* enum libbmc_fan_state_enum state : tec state                                                                       */
/* libbmc_error_t return : error                                                                                      */
/*  - toggle thermoelectric cooler mode low|med|high                                                                  */
/*  - checks if tec is already in state                                                                               */
/**********************************************************************************************************************/
libbmc_error_t libbmc_set_tec (libbmc_device_t* p_libbmc_device, enum libbmc_tec_state_enum state) {
  int ret1, ret2; // for return values of libbmc_get_status (), _libbmc_transaction ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.tec == state) // tec is already in state
      return LIBBMC_SUCCESS; // return success
    else { // tec is in a different state
      uint8_t libbmc_tec_state_cmd[] = LIBBMC_CTRL_CMD_SET_TEC;
      uint8_t set_tec_req[LIBBMC_CTRL_SIZE]={0};
      set_tec_req[0] = LIBBMC_CMD_BYTE_0;
      set_tec_req[1] = libbmc_tec_state_cmd[state];
      ret2 = _libbmc_transaction (p_libbmc_device, set_tec_req, LIBBMC_CTRL_SIZE); // set tec to state
      if (ret2 != 0) // _libbmc_transaction () failed
        return ret2; // return error
      else { // _libbmc_set_tec () successful
        if (p_libbmc_device->status.tec == state) // tec state verified
          return LIBBMC_SUCCESS; // return success
        else // tec state verification failed
          return LIBBMC_ERROR_OTHER; // return error
      }
    }
  }
}









/**********************************************************************************************************************/
/* LIBBMC_SET_POT                                                                                                     */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* enum libbmc_volt_range_enum range : voltage range                                                                  */
/* libbmc_error_t return : error                                                                                      */
/*  - set voltage range of the controller                                                                             */
/*  - does not turn the controller on, does not even send any signals to the controller                               */
/*  NOTE                                                                                                              */
/*  - there is no corresponding register with the range value.                                                        */
/*    set range to LIBBMC_VOLT_RANGE_100V on init                                                                     */
/*  - checks if voltage is already set to range                                                                       */
/**********************************************************************************************************************/
libbmc_error_t libbmc_set_range (libbmc_device_t* p_libbmc_device, enum libbmc_volt_range_enum range) {
  int ret1; // for return values of libbmc_get_status ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.range == range) // voltage range is already set to range
      return LIBBMC_SUCCESS; // return success
    else { // voltage range is a different range
      if (p_libbmc_device->status.power == LIBBMC_PWR_ON) // controller is on
        return LIBBMC_ERROR_CONTROLLER_ENABLED; // return controller enabled error
      else { // controller is off
        p_libbmc_device->status.range = range;
        return LIBBMC_SUCCESS; // return success
      }
    }
  }
}









/**********************************************************************************************************************/
/* LIBBMC_TOGGLE_LEDS                                                                                                 */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - toggle leds                                                                                                     */
/*  NOTE                                                                                                              */
/*  - there is no corresponding register with the led setting.                                                        */
/*    set led settings to LIBBMC_ON on init                                                                           */
/**********************************************************************************************************************/
static libbmc_error_t _libbmc_toggle_leds (libbmc_device_t* p_libbmc_device) {
  uint8_t toggle_leds_req[LIBBMC_CTRL_SIZE]={0};
  toggle_leds_req[0] = LIBBMC_CMD_BYTE_0;
  toggle_leds_req[1] = LIBBMC_CTRL_CMD_TOGGLE_LEDS;
  return _libbmc_transaction (p_libbmc_device, toggle_leds_req, LIBBMC_CTRL_SIZE);
}
/**********************************************************************************************************************/
/* LIBBMC_TOGGLE_LEDS_ON                                                                                              */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - turn on leds                                                                                                    */
/*  - checks if leds are already on                                                                                   */
/**********************************************************************************************************************/
libbmc_error_t libbmc_toggle_leds_on (libbmc_device_t* p_libbmc_device) {
  int ret1; // for return values of _libbmc_toggle_leds ()
  if (p_libbmc_device->status.leds == LIBBMC_ON) // leds already on
    return LIBBMC_SUCCESS; // return success
  else { // leds is off
    ret1 = _libbmc_toggle_leds (p_libbmc_device); // toggle leds to on
    if (ret1 != 0) // _libbmc_toggle_leds () failed
      return ret1; // return error
    else { // _libbmc_toggle_leds () successful
      p_libbmc_device->status.leds = LIBBMC_ON; // theres no led status in the status packet
      return LIBBMC_SUCCESS; // return success
    }
  }
}
/**********************************************************************************************************************/
/* LIBBMC_TOGGLE_LEDS_OFF                                                                                             */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - turn off leds                                                                                                   */
/*  - checks if leds are already off                                                                                  */
/**********************************************************************************************************************/
libbmc_error_t libbmc_toggle_leds_off (libbmc_device_t* p_libbmc_device) {
  int ret1; // for return values of _libbmc_toggle_leds ()
  if (p_libbmc_device->status.leds == LIBBMC_OFF) // leds already off
    return LIBBMC_SUCCESS; // return success
  else { // leds is on 
    ret1 = _libbmc_toggle_leds (p_libbmc_device); // toggle leds to off
    if (ret1 != 0) // _libbmc_toggle_leds () failed
      return ret1; // return error
    else { // _libbmc_toggle_leds () successful
      p_libbmc_device->status.leds = LIBBMC_OFF; // theres no led status in the status packet
      return LIBBMC_SUCCESS; // return success
    }
  }
}










/**********************************************************************************************************************/
/* _LIBBMC_SET_TSTPNTS                                                                                                */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* float tstpnt_values[LIBBMC_NTSTPNT] : voltages to set the test points to                                           */
/* libbmc_error_t return : error                                                                                      */
/*  - set all test points                                                                                             */
/*  ** CAUTION **                                                                                                     */
/*  - does not check controller status before sending                                                                 */
/**********************************************************************************************************************/
static libbmc_error_t _libbmc_set_tstpnts (libbmc_device_t* p_libbmc_device, float tstpnt_values[LIBBMC_NTSTPNT]) {

  size_t itestpoint;
  size_t mapping[LIBBMC_NTSTPNT] = LIBBMC_TESTPOINT_MAP;
  float libbmc_hv_range_value[4] = LIBBMC_VOLT_RANGE_CTRL_VALUE;
  const float multiplier = 0xffff/libbmc_hv_range_value[p_libbmc_device->status.range];

  uint8_t send_data_req[LIBBMC_DATA_SIZE]={0};
  send_data_req[0] = LIBBMC_CMD_BYTE_0;
  send_data_req[1] = LIBBMC_DATA_BYTE_1;

  uint16_t* arr_uint16;
  arr_uint16 = (uint16_t*) send_data_req;

  uint32_t tempval;

  for (itestpoint = 0; itestpoint<LIBBMC_NTSTPNT; itestpoint++) { // analog to digital conversion
    tempval = (uint32_t)(round(tstpnt_values[itestpoint]*multiplier));
    arr_uint16[mapping[itestpoint]+1] = (uint16_t)((tempval>0xffff)?0xffff:tempval);
  }

  return _libbmc_transaction (p_libbmc_device, send_data_req, LIBBMC_DATA_SIZE);
}
/**********************************************************************************************************************/
/* LIBBMC_SET_TSTPNTS                                                                                                 */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* float tstpnt_values[LIBBMC_NTSTPNT] : voltages to set the test points to                                           */
/* libbmc_error_t return : error                                                                                      */
/*  - set all test points                                                                                             */
/*  - checks if controller is on before setting testpoints                                                            */
/**********************************************************************************************************************/
libbmc_error_t libbmc_set_tstpnts (libbmc_device_t* p_libbmc_device, float tstpnt_values[LIBBMC_NTSTPNT]) {
  int ret1; // for return tstpnt_values of libbmc_get_status ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.power == LIBBMC_PWR_ON) // controller is on
      return _libbmc_set_tstpnts (p_libbmc_device, tstpnt_values);
    else // controller is off
      return LIBBMC_ERROR_CONTROLLER_DISABLED;
  }
}










/**********************************************************************************************************************/
/* _LIBBMC_SET_ACTS                                                                                                   */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* float act_values[LIBBMC_NACT] : voltages to set the actuators to                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - set all actuators                                                                                               */
/*  ** CAUTION **                                                                                                     */
/*  - does not check controller status before sending                                                                 */
/**********************************************************************************************************************/
static libbmc_error_t _libbmc_set_acts (libbmc_device_t* p_libbmc_device, float act_values[LIBBMC_NACT]) {
  size_t iactuator;
  size_t actuator_mapping[LIBBMC_DATA_LENGTH] = LIBBMC_ACTUATOR_MAP;
  float libbmc_hv_range_value[4] = LIBBMC_VOLT_RANGE_CTRL_VALUE;
  const float multiplier = 0xffff/libbmc_hv_range_value[p_libbmc_device->status.range];

  uint8_t send_data_req[LIBBMC_DATA_SIZE]={0};
  send_data_req[0] = LIBBMC_CMD_BYTE_0;
  send_data_req[1] = LIBBMC_DATA_BYTE_1;

  uint16_t* arr_uint16;
  arr_uint16 = (uint16_t*) send_data_req;

  uint32_t tempval;

  for (iactuator = 0; iactuator<LIBBMC_NACT; iactuator++) { // analog to digital conversion
    tempval = (uint32_t)(round(act_values[iactuator]*multiplier));
    arr_uint16[actuator_mapping[iactuator]+1] = (uint16_t)((tempval>0xffff)?0xffff:tempval);
  }

  return _libbmc_transaction (p_libbmc_device, send_data_req, LIBBMC_DATA_SIZE);
}
/**********************************************************************************************************************/
/* LIBBMC_SEND_DATA                                                                                                   */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* float act_values[LIBBMC_NACT] : voltages to set the actuators to                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - set all actuators                                                                                               */
/*  - checks if controller is on before sending command                                                               */
/**********************************************************************************************************************/
libbmc_error_t libbmc_set_acts (libbmc_device_t* p_libbmc_device, float act_values[LIBBMC_NACT]) {
  int ret1; // for return values of libbmc_get_status ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.power == LIBBMC_PWR_ON) // controller is on
      return _libbmc_set_acts (p_libbmc_device, act_values);
    else // controller is off
      return LIBBMC_ERROR_CONTROLLER_DISABLED;
  }
}










/**********************************************************************************************************************/
/* _LIBBMC_SET_ACTS_TSTPNTS                                                                                           */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* float act_values[LIBBMC_NACT] : voltages to set the actuators to                                                   */
/* float tstpnt_values[LIBBMC_NTSTPNT] : voltages to set the test points to                                           */
/* libbmc_error_t return : error                                                                                      */
/*  - set all actuators                                                                                               */
/*  - set all test points                                                                                             */
/*  ** CAUTION **                                                                                                     */
/*  - does not check controller status before sending                                                                 */
/**********************************************************************************************************************/
static libbmc_error_t _libbmc_set_acts_tstpnts (libbmc_device_t* p_libbmc_device, float act_values[LIBBMC_NACT], float tstpnt_values[LIBBMC_NTSTPNT]) {
  size_t iactuator, itestpoint;
  size_t actuator_mapping[LIBBMC_DATA_LENGTH] = LIBBMC_ACTUATOR_MAP;
  size_t testpoint_mapping[LIBBMC_NTSTPNT] = LIBBMC_TESTPOINT_MAP;

  float libbmc_hv_range_value[4] = LIBBMC_VOLT_RANGE_CTRL_VALUE;
  const float multiplier = 0xffff/libbmc_hv_range_value[p_libbmc_device->status.range];

  uint8_t send_data_req[LIBBMC_DATA_SIZE]={0};
  send_data_req[0] = LIBBMC_CMD_BYTE_0;
  send_data_req[1] = LIBBMC_DATA_BYTE_1;

  uint16_t* arr_uint16;
  arr_uint16 = (uint16_t*) send_data_req;

  uint32_t tempval;

  for (iactuator = 0; iactuator<LIBBMC_NACT; iactuator++) { // analog to digital conversion
    tempval = (uint32_t)(round(act_values[iactuator]*multiplier));
    arr_uint16[actuator_mapping[iactuator]+1] = (uint16_t)((tempval>0xffff)?0xffff:tempval);
  }

  for (itestpoint = 0; itestpoint<LIBBMC_NTSTPNT; itestpoint++) { // analog to digital conversion
    tempval = (uint32_t)(round(tstpnt_values[itestpoint]*multiplier));
    arr_uint16[testpoint_mapping[itestpoint]+1] = (uint16_t)((tempval>0xffff)?0xffff:tempval);
  }

  return _libbmc_transaction (p_libbmc_device, send_data_req, LIBBMC_DATA_SIZE);
}
/**********************************************************************************************************************/
/* LIBBMC_SET_ACTS_TSTPNTS                                                                                            */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* float act_values[LIBBMC_NACT] : voltages to set the actuators to                                                   */
/* float tstpnt_values[LIBBMC_NTSTPNT] : voltages to set the test points to                                           */
/* libbmc_error_t return : error                                                                                      */
/*  - set all actuators                                                                                               */
/*  - set all test points                                                                                             */
/*  - checks if controller is on before sending command                                                               */
/**********************************************************************************************************************/
libbmc_error_t libbmc_set_acts_tstpnts (libbmc_device_t* p_libbmc_device, float act_values[LIBBMC_NACT], float tstpnt_values[LIBBMC_NTSTPNT]) {
  int ret1; // for return values of libbmc_get_status ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.power == LIBBMC_PWR_ON) // controller is on
      return _libbmc_set_acts_tstpnts (p_libbmc_device, act_values, tstpnt_values);
    else // controller is off
      return LIBBMC_ERROR_CONTROLLER_DISABLED;
  }
}










/**********************************************************************************************************************/
/* _IS_VALUE_WITHIN                                                                                                   */
/* float* p_input : pointer to value to test                                                                          */
/* float target : target value                                                                                        */
/* float window_pct : percentage of the target value to test if input is within                                       */
/* char return : 1 if value is within range, 0 if not                                                                 */
/*  - checks if a value in range of target percentage window                                                          */
/**********************************************************************************************************************/
static char _is_value_within (float input, float target, float window_pct) {
  float upper_bound, lower_bound;
  upper_bound = target*(1.0 + window_pct);
  lower_bound = target*(1.0 - window_pct);
  return ((lower_bound<input) && (input<upper_bound)); /*is in range?*/
}
/**********************************************************************************************************************/
/* LIBBMC_SEND_DATA_VERIFY                                                                                            */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* float act_values[LIBBMC_NACT] : voltages to set the actuators to                                                   */
/* libbmc_error_t return : error                                                                                      */
/*  - set all actuators                                                                                               */
/*  - checks if controller is on before sending command                                                               */
/*  - the test points are validated to be within 10% of a set of evenly spaced values in the full range               */
/**********************************************************************************************************************/
libbmc_error_t libbmc_set_acts_verify (libbmc_device_t* p_libbmc_device, float act_values[LIBBMC_NACT]) {
  int ret1, ret2; // for return values of libbmc_get_status ()
  ret1 = libbmc_get_status (p_libbmc_device); // first libbmc_get_status ()
  if (ret1 != 0) // libbmc_get_status () failed
    return ret1; // return error
  else { // libbmc_get_status () successful
    if (p_libbmc_device->status.power == LIBBMC_PWR_ON) { // controller is on

      size_t itestpoint;
      float tstpnt_vals[LIBBMC_NTSTPNT] = {0};

      float test_point_max, test_point_increment;
      // test_point_max = libbmc_hv_range_value[p_libbmc_device->status.range];
      test_point_max = 50.0;
      test_point_increment = test_point_max/(LIBBMC_NTSTPNT-1);

      for (itestpoint = 0; itestpoint<LIBBMC_NTSTPNT; itestpoint++) // analog to digital conversion
        tstpnt_vals[itestpoint] = itestpoint*test_point_increment;

      ret2 = libbmc_set_acts_tstpnts (p_libbmc_device, act_values, tstpnt_vals);

      if (ret2 != 0) // libbmc_get_status () failed
        return ret1; // return error
      else {
        char testpoint_acceptable = 1;
        for (itestpoint = 0; itestpoint<LIBBMC_NTSTPNT; itestpoint++) // analog to digital conversion
          testpoint_acceptable &= _is_value_within(p_libbmc_device->status.testpoint_v[itestpoint], tstpnt_vals[itestpoint], 0.05);

        if (testpoint_acceptable)
          return LIBBMC_SUCCESS;
        else
          return LIBBMC_ERROR_TESTPOINTS_UNACCEPTABLE;
      }

    } else // controller is off
      return LIBBMC_ERROR_CONTROLLER_DISABLED;
  }
}










/**********************************************************************************************************************/
/* LIBBMC_PRINT_DEVICE                                                                                                */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* void return                                                                                                        */
/*  - Print the device endpoints                                                                                      */
/**********************************************************************************************************************/
void libbmc_print_device (libbmc_device_t* p_libbmc_device) {
  printf ("----------------------------------------------------\n");
  printf ("out_endpoint  : 0x%02x\n", p_libbmc_device->out_endpoint);
  printf ("in_endpoint   : 0x%02x\n", p_libbmc_device->in_endpoint);
  libbmc_print_status (&(p_libbmc_device->status));
  printf ("----------------------------------------------------\n");
}










/**********************************************************************************************************************/
/* LIBBMC_PRINT_DEVICE                                                                                                */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* void return                                                                                                        */
/*  - Print the device endpoints                                                                                      */
/**********************************************************************************************************************/
void libbmc_get_print_status (libbmc_device_t* p_libbmc_device) {
  int ret1;
  ret1 = libbmc_get_status (p_libbmc_device);
  if (ret1 != 0) // libbmc_get_status () failed
    printf ("BMC: libbmc_get_status failed : %s\n", libbmc_error_name (ret1));
  else // libbmc_get_status () successful
    libbmc_print_status (&(p_libbmc_device->status));
}










/**********************************************************************************************************************/
/* LIBBMC_PRINT_STATUS                                                                                                */
/* libbmc_device_t* p_libbmc_device : device handle                                                                   */
/* void return                                                                                                        */
/*  - Print the device endpoints                                                                                      */
/**********************************************************************************************************************/
void libbmc_print_status (libbmc_status_t* p_libbmc_status) {
  int i;
  char* libbmc_pwr_state_label[10] = LIBBMC_PWR_STAT_LABEL;
  char* libbmc_fan_state_label[3] = LIBBMC_FAN_CTRL_LABEL;
  char* libbmc_volt_state_label[2] = LIBBMC_VOLT_STAT_LABEL;
  char* libbmc_temp_stat_label[2] = LIBBMC_TEMP_STAT_LABEL;
  char* libbmc_bi_state_label[2] = LIBBMC_BI_STAT_LABEL;
  char* libbmc_tec_state_label[3] = LIBBMC_TEC_CTRL_LABEL;
  float libbmc_hv_range_value[4] = LIBBMC_VOLT_RANGE_CTRL_VALUE;
  printf ("----------------------------------------------------\n");
  printf ("power               : %s\n", libbmc_pwr_state_label[p_libbmc_status->power]);
  printf ("leds                : %s\n", libbmc_bi_state_label[p_libbmc_status->leds]);
  printf ("supply 5v analog    : %s\n", libbmc_bi_state_label[p_libbmc_status->supply_5va]);
  printf ("supply 5v digital   : %s\n", libbmc_bi_state_label[p_libbmc_status->supply_5vd]);
  printf ("supply HV           : %s\n", libbmc_bi_state_label[p_libbmc_status->supply_hv]);
  printf ("HV range            : %f\n", libbmc_hv_range_value[p_libbmc_status->range]);

  printf ("3V 1                : %f V\n", p_libbmc_status->rail_3v1_v);
  printf ("3V 2                : %f V\n", p_libbmc_status->rail_3v2_v);
  printf ("5V 1                : %f V\n", p_libbmc_status->rail_5v1_v);
  printf ("5V 2                : %f V\n", p_libbmc_status->rail_5v2_v);

  printf ("HV Supply 1         : %f V\n", p_libbmc_status->hv_supp_v[0]);
  printf ("HV Supply 2         : %f V\n", p_libbmc_status->hv_supp_v[1]);

  printf ("current             : %f ma\n", p_libbmc_status->current_ma);

  printf ("Main board temp     : %f C\n", p_libbmc_status->main_brd_temp_c);
  printf ("Heat sink temp      : %f C\n", p_libbmc_status->heatsink_temp_c);
  printf ("Ambient temp        : %f C\n", p_libbmc_status->ambient_temp_c);
  printf ("Top board temp      : %f C\n", p_libbmc_status->top_brd_temp_c);
  printf ("Mid board temp      : %f C\n", p_libbmc_status->mid_brd_temp_c);
  printf ("Bot board temp      : %f C\n", p_libbmc_status->bot_brd_temp_c);
  printf ("Socket 1 temp       : %f C\n", p_libbmc_status->sock1_temp_c);
  printf ("Socket 2 temp       : %f C\n", p_libbmc_status->sock2_temp_c);

  printf ("HV Reference        : %f V\n", p_libbmc_status->hv_ref_v);
  printf ("Input Voltage       : %f V\n", p_libbmc_status->voltage_input_v);
  printf ("3.3 V status        : %s\n", libbmc_volt_state_label[p_libbmc_status->volt_3v3]);
  printf ("5V status           : %s\n", libbmc_volt_state_label[p_libbmc_status->volt_5]);

  printf ("IC temperatures (C) : ");
  printf ("[");
  for(i = 0; i < 11; i++)
    printf ("%07.4f, ", p_libbmc_status->ic_temp_c[i]);
  printf ("\b\b]\n");

  printf ("Test point (V)      : ");
  printf ("[");
  for(i = 0; i < LIBBMC_NTSTPNT; i++)
    printf ("%07.4f, ", p_libbmc_status->testpoint_v[i]);
  printf ("\b\b]\n");

  printf ("Fan                 : %s\n", libbmc_fan_state_label[p_libbmc_status->fan]);

  printf ("TEC                 : %s\n", libbmc_tec_state_label[p_libbmc_status->tec]);

  printf ("Over Temperature    : %s\n", libbmc_temp_stat_label[p_libbmc_status->over_temp]);
  printf ("Serial number       : %d\n", p_libbmc_status->serial);
  printf ("Firmware version    : %d.%d\n", p_libbmc_status->fw_major, p_libbmc_status->fw_minor);
  printf ("----------------------------------------------------\n");
}









#if LIBBMC_DEBUG
/**********************************************************************************************************************/
/* _LIBBMC_PRINT_TRANSFER                                                                                             */
/* uint8_t* data : data to print                                                                                      */
/* int n : number of values in data                                                                                   */
/* void return                                                                                                        */
/*  - print usb transfers                                                                                             */
/**********************************************************************************************************************/
static void _libbmc_print_transfer (uint8_t* data, int n) {
  int i;
  printf ("----------------------------------------------------\n");
  printf ("     00 01 02 03 04 05 06 07  08 09 0A 0B 0C 0D 0E 0F\n");
  for (i = 0; i < n; i++) {
    if (i%0x10==0)
      printf ("%04x ", i);
    printf ("%02x ", data[i]);
    if (i%0x10==0xf)
      printf ("\n");
    if (i%0x10==0x7)
      printf (" ");
  }
  printf ("----------------------------------------------------\n");
}
#endif









/**********************************************************************************************************************/
/* LIBBMC_ERROR_NAME                                                                                                  */
/* libbmc_error_t error : error to get the name of                                                                    */
/* char* return : error name                                                                                          */
/*  - get the name of the error                                                                                       */
/**********************************************************************************************************************/
const char* libbmc_error_name (libbmc_error_t error) {
  char* error_str[LIBBMC_ERROR_COUNT] = LIBBMC_ERR_DESC;
  if ((-error)>=(LIBBMC_ERROR_COUNT-1))
    return error_str[LIBBMC_ERROR_COUNT-1];
  else
    return error_str[-error];
}










/**********************************************************************************************************************/
/* LIBBMC_STRERROR                                                                                                    */
/* libbmc_error_t error : error to get the name of                                                                    */
/* char* return : error in readable form                                                                              */
/*  - get the error in readable form                                                                                  */
/**********************************************************************************************************************/
const char* libbmc_strerror (libbmc_error_t error) {
  char* descriptions[LIBBMC_ERROR_COUNT] = LIBBMC_ERR_DESC;
  if ((-error)>=(LIBBMC_ERROR_COUNT-1))
    return descriptions[LIBBMC_ERROR_COUNT-1];
  else
    return descriptions[-error];
}