// https://github.com/closedcube/ClosedCube_HDC1010_Arduino
// http://www.ti.com/lit/ds/symlink/hdc1010.pdf

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <inttypes.h>
#include "libhdc.h"


int hdc_open(char *dev) {
  int fd;
  if ((fd = open(dev, O_RDWR)) < 0) { 
    return -1;
  }
  return fd;
}


int hdc_init(int fd, hdc_device_t* device) {
  if (ioctl(fd, I2C_SLAVE, *device) < 0) { // target device address
    return -1;
  }
  char reg = HDC_CONF_REG;
  if (write(fd, &reg, 1) != 1) { // set address to config register
    return -1;
  }
  usleep(100000); // settle time

  // soft reset | normal 0b 0--- ---- ---- ----
  // heater on | off     0b --0- ---- ---- ----
  // mode t|rh | t&rh    0b ---0 ---- ---- ----
  // tres 14|11          0b ---- -0-- ---- ----
  // rhres 14|11|8       0b ---- --00 ---- ----
  uint8_t value[2] = {0x00,0x00};
  if (write(fd, value, 2) != 2) { // write configuration
    return -1;
  }
  usleep(100000); // settle time

  return 0;
}



int hdc_read_register(int fd, hdc_device_t* device, hdc_register reg, char size, unsigned int* value) {
  int i;
  char *bytes;
  
  if (ioctl(fd,I2C_SLAVE,*device) < 0) { // target device address
    return -1;
  }
  if (write(fd, &reg, 1) != 1) { // set address to address register
    return -1;
  }
  usleep(100000); // settle time
  
  bytes = (char*)malloc(size);
  if (read(fd, bytes, size) != size) { // read register value
    return -1;
  }
  usleep(100000); // settle time
  
  *value = 0;
  for(i = 0; i < size; i++) { // flip endianness
    *value += bytes[i]<<(8*(size-1-i));
  }
  free(bytes);
  return 0;
}



int hdc_read_config(int fd, hdc_device_t* device, hdc_config* config) {
  unsigned int raw_value;
  hdc_read_register(fd, device, HDC_CONF_REG, 2, &raw_value);
  config->raw_data = (uint8_t)(raw_value >> 8);
  return 0;
}



int hdc_write_config(int fd, hdc_device_t* device, hdc_config* config) {
  char reg = HDC_CONF_REG;
  if (ioctl(fd,I2C_SLAVE,*device) < 0) { // target device address
    return -1;
  }
  if (write(fd, &reg, 1) != 1) { // set address to config register
    return -1;
  }
  usleep(100000); // settle time

  uint8_t value[2] = {((config->raw_data>>0)&0xFF), ((config->raw_data>>8)&0xFF)};
  if (write(fd, value, 2) != 2) { // write configuration
    return -1;
  }
  usleep(100000); // settle time

  return 0;
}


int hdc_get_t(int fd, hdc_device_t* device, float* t) {
  unsigned int raw_t;
  hdc_read_register(fd, device, HDC_TEMP_REG, 2, &raw_t);
  *t = 0;
  *t = ((float)raw_t/HDC_POW16)*165.0 - 40.0;
  return 0;
}


int hdc_get_rh(int fd, hdc_device_t* device, float* rh) {
  unsigned int int_rh;
  hdc_read_register(fd, device, HDC_HUMI_REG, 2, &int_rh);
  *rh = 0;
  *rh = ((float)int_rh/HDC_POW16)*100.0;
  return 0;
}


int hdc_cleanup(int fd) {
  return close(fd);
}


int hdc_get_info(int fd, hdc_device_t* device) {
  unsigned int man_id, ser_id[3], dev_id;
  hdc_config config;
  int temp[3] = {14,11,8};

  hdc_read_register(fd, device, HDC_MAN_ID_REG, 2, &man_id);
  printf("HDC: manufacture id    : 0x%04x\n", man_id);

  hdc_read_register(fd, device, HDC_SER_ID1_REG, 2, &ser_id[0]);
  hdc_read_register(fd, device, HDC_SER_ID2_REG, 2, &ser_id[1]);
  hdc_read_register(fd, device, HDC_SER_ID3_REG, 2, &ser_id[2]);
  printf("HDC: serial id         : 0x%04x%04x%04x\n", ser_id[0]&0xffff, ser_id[1]&0xffff, ser_id[2]&0xffc0);

  hdc_read_register(fd, device, HDC_DEV_ID_REG, 2, &dev_id);
  printf("HDC: device id         : 0x%04x\n", dev_id);

  hdc_read_config(fd, device, &config);
  printf("HDC: -- device configuration --\n");
  printf("HDC: Humi. Resol       : %d\n",temp[config.HDC_CONF_HRES]);
  printf("HDC: Temp. Resol       : %d\n",temp[config.HDC_CONF_HRES]);
  printf("HDC: Batt. Stat.       : %s2.8V\n", config.HDC_CONF_BTST?"<":">");
  printf("HDC: Mode of acq.      : temperature %s humidity\n", config.HDC_CONF_MODE?"and":"or");
  printf("HDC: Heater            : heater %sabled\n", config.HDC_CONF_HEAT?"en":"dis");
  printf("HDC: Software reset    : %s\n", config.HDC_CONF_RST ?"reset":"normal");

  return 0;
}
