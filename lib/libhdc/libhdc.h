#ifndef _LIBHDC
#define _LIBHDC

#define HDC_POW16 65536.0f

typedef unsigned char hdc_device_t;

typedef enum { // register map
  HDC_TEMP_REG     = 0x00, // Temperature   - read only
  HDC_HUMI_REG     = 0x01, // Humidity      - read only
  HDC_CONF_REG     = 0x02, // Configuration - read write
  HDC_SER_ID1_REG  = 0xFB, // Serial ID     - read only
  HDC_SER_ID2_REG  = 0xFC, // Serial ID     - read only
  HDC_SER_ID3_REG  = 0xFD, // Serial ID     - read only
  HDC_MAN_ID_REG   = 0xFE, // Manuf. ID     - read only
  HDC_DEV_ID_REG   = 0xFF  // Device ID     - read only
} hdc_register;

typedef union {
  uint8_t raw_data;
  struct {
    uint8_t HDC_CONF_HRES : 2; // Humi. Resol    - bit 8:9 - read write
    uint8_t HDC_CONF_TRES : 1; // Temp. Resol    - bit 10  - read write
    uint8_t HDC_CONF_BTST : 1; // Batt. Stat.    - bit 11  - read only
    uint8_t HDC_CONF_MODE : 1; // Mode of acq.   - bit 12  - read write
    uint8_t HDC_CONF_HEAT : 1; // Heater         - bit 13  - read write
    uint8_t HDC_CONF_RSVD : 1; // reserved       - bit 14
    uint8_t HDC_CONF_RST  : 1; // Software reset - bit 15  - read write
  };
} hdc_config;


//Function Prototypes
int hdc_open(char *dev);
int hdc_init(int fd, hdc_device_t* device);
int hdc_read_register(int fd, hdc_device_t* device, hdc_register reg, char size, unsigned int* value);
int hdc_read_config(int fd, hdc_device_t* device, hdc_config* config);
int hdc_write_config(int fd, hdc_device_t* device, hdc_config* config);
int hdc_get_t(int fd, hdc_device_t* device, float* t);
int hdc_get_rh(int fd, hdc_device_t* device, float* rh);
int hdc_cleanup(int fd);
int hdc_get_info(int fd, hdc_device_t* device);

#endif
