#ifndef __POZYX_H__
#define __POZYX_H__

#include "geometry_msgs/msg/point.h"
#include "geometry_msgs/msg/quaternion.h"
#include "mobi_interfaces/srv/get_calib_status.h"
#include "pozyx_definitions.h"
#include "stm32l4xx.h"

typedef struct pozyx_s {
  /* I2C */
  I2C_HandleTypeDef *hi2c;
  uint16_t device_address;

  /* DMA */
  uint8_t tx_buf[1];
  // Note: 22 bytes for calib data. Biggest data for sensors is 8 bytes for quaternion.
  volatile uint8_t rx_buf[4 * sizeof(int16_t)];

  int8_t reading_device; // The device which is currently read! -1 for no device

  // Data
  geometry_msgs__msg__Quaternion *orientation;
  geometry_msgs__msg__Point *position;

  uint8_t who_am_i;
  uint16_t network_id;
  uint8_t firmware_version;
  uint8_t hardware_version;

  mobi_interfaces__srv__GetCalibStatus_Response *calib_status;

} pozyx_t;

void pozyx_init(pozyx_t *pozyx, I2C_HandleTypeDef *hi2c_device, uint16_t device_address);

void pozyx_write_DMA(pozyx_t *pozyx, uint8_t reg_addr, uint8_t data);
void pozyx_mem_read_DMA(pozyx_t *pozyx, uint8_t reg_addr, uint8_t len);

void pozyx_read_DMA_complete(pozyx_t *pozyx);

void pozyx_read_who_am_i(pozyx_t *pozyx);
void pozyx_read_firmware_version(pozyx_t *pozyx);
void pozyx_read_harware_version(pozyx_t *pozyx);
void pozyx_read_network_id(pozyx_t *pozyx);

void pozyx_read_calibration_state(pozyx_t *pozyx);

#endif /* __POZYX_H__ */
