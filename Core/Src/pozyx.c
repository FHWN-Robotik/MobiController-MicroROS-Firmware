#include "pozyx.h"

#include "geometry_msgs/msg/point.h"
#include "geometry_msgs/msg/quaternion.h"
#include "i2c.h"
#include "mobi_interfaces/srv/get_calib_status.h"
#include "rcutils/logging_macros.h"

void pozyx_init(pozyx_t *pozyx, I2C_HandleTypeDef *hi2c_device, uint16_t device_address) {
  pozyx->hi2c = hi2c_device;
  pozyx->device_address = device_address;

  HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(pozyx->hi2c, pozyx->device_address << 1, 5, 1000);
  if (status != HAL_OK) {
    RCUTILS_LOG_ERROR_NAMED(LOGGER_NAME, "Error setting up Pozyx!");
  }

  pozyx->calib_status = mobi_interfaces__srv__GetCalibStatus_Response__create();
  pozyx->orientation = geometry_msgs__msg__Quaternion__create();
  pozyx->position = geometry_msgs__msg__Point__create();
}

void pozyx_write_DMA(pozyx_t *pozyx, uint8_t reg_addr, uint8_t data) {
  pozyx->tx_buf[0] = reg_addr;
  // pozyx->tx_buf[1] = data;

  while (pozyx->hi2c->State != HAL_I2C_STATE_READY) {
  }

  HAL_StatusTypeDef status =
    HAL_I2C_Master_Transmit_DMA(pozyx->hi2c, pozyx->device_address << 1, pozyx->tx_buf, sizeof(pozyx->tx_buf));
  i2c_check_status(pozyx->hi2c, status);
}

void pozyx_mem_read_DMA(pozyx_t *pozyx, uint8_t reg_addr, uint8_t len) {
  pozyx_write_DMA(pozyx, reg_addr, 0);

  while (pozyx->hi2c->State != HAL_I2C_STATE_READY) {
  }

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(pozyx->hi2c, pozyx->device_address << 1, reg_addr, sizeof(uint8_t),
                                                  (uint8_t *)pozyx->rx_buf, len);
  i2c_check_status(pozyx->hi2c, status);
}

void pozyx_read_DMA_complete(pozyx_t *pozyx) {
  switch (pozyx->reading_device) {
  case POZYX_WHO_AM_I: {
    pozyx->who_am_i = pozyx->rx_buf[0];
    break;
  }

  case POZYX_NETWORK_ID: {
    pozyx->network_id = (int16_t)((pozyx->rx_buf[1] << 8) | pozyx->rx_buf[0]);
    break;
  }

  case POZYX_HARDWARE_VER: {
    pozyx->hardware_version = pozyx->rx_buf[0];
    break;
  }

  case POZYX_FIRMWARE_VER: {
    pozyx->firmware_version = pozyx->rx_buf[0];
    break;
  }

  case POZYX_CALIB_STATUS: {
    pozyx->calib_status->system = (pozyx->rx_buf[0] >> 6) & 0x03;
    pozyx->calib_status->gyro = (pozyx->rx_buf[0] >> 4) & 0x03;
    pozyx->calib_status->accel = (pozyx->rx_buf[0] >> 2) & 0x03;
    pozyx->calib_status->mag = pozyx->rx_buf[0] & 0x03;
    break;
  }

  default:
    break;
  }

  pozyx->reading_device = -1;
}

void pozyx_read_who_am_i(pozyx_t *pozyx) {
  pozyx->reading_device = POZYX_WHO_AM_I;
  pozyx_mem_read_DMA(pozyx, POZYX_WHO_AM_I, 1);
}

void pozyx_read_firmware_version(pozyx_t *pozyx) {
  pozyx->reading_device = POZYX_FIRMWARE_VER;
  pozyx_mem_read_DMA(pozyx, POZYX_FIRMWARE_VER, 1);
}

void pozyx_read_harware_version(pozyx_t *pozyx) {
  pozyx->reading_device = POZYX_HARDWARE_VER;
  pozyx_mem_read_DMA(pozyx, POZYX_HARDWARE_VER, 1);
}

void pozyx_read_network_id(pozyx_t *pozyx) {
  pozyx->reading_device = POZYX_NETWORK_ID;
  pozyx_mem_read_DMA(pozyx, POZYX_NETWORK_ID, 2);
}

void pozyx_read_calibration_state(pozyx_t *pozyx) {
  pozyx->reading_device = POZYX_CALIB_STATUS;
  pozyx_mem_read_DMA(pozyx, POZYX_CALIB_STATUS, 1);
}