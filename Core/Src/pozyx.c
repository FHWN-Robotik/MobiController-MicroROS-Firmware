/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: pozyx.c
 * Created Date: Thursday, April 11th 2024, 1:58:20 pm
 * Author: Florian
 * Description: This file implements the pozyx related functions.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

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

  pozyx->orientation = *geometry_msgs__msg__Quaternion__create();
  pozyx->position = *geometry_msgs__msg__Point__create();

  pozyx->who_am_i = 0;
  pozyx->network_id = 0;
  pozyx->firmware_version = 0;
  pozyx->hardware_version = 0;
  pozyx->calib_status = 0;
  pozyx->new_pos_available = false;

  // Setup interrupt
  while (pozyx->hi2c->State != HAL_I2C_STATE_READY) {
  }
  pozyx_set_interrupt_mask(pozyx, POZYX_INT_MASK_POS);
  while (pozyx->hi2c->State != HAL_I2C_STATE_READY) {
  }
  pozyx_config_interrupt_pin(pozyx, 5, PIN_MODE_PUSHPULL, PIN_ACTIVE_HIGH, 0);
}

void pozyx_write_DMA(pozyx_t *pozyx, uint8_t reg_addr, uint8_t data) {
  pozyx->tx_buf[0] = reg_addr;
  pozyx->tx_buf[1] = data;

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
    pozyx->calib_status = pozyx->rx_buf[0];
    break;
  }

  case POZYX_POS_X: {
    // Get the position from the buffer. Bytes are in Little-Endian format
    pozyx->position.x =
      (double)((pozyx->rx_buf[3] << 24) | (pozyx->rx_buf[2] << 16) | (pozyx->rx_buf[1] << 8) | pozyx->rx_buf[0]);
    pozyx->position.y =
      (double)((pozyx->rx_buf[7] << 24) | (pozyx->rx_buf[6] << 16) | (pozyx->rx_buf[5] << 8) | pozyx->rx_buf[4]);
    pozyx->position.z =
      (double)((pozyx->rx_buf[11] << 24) | (pozyx->rx_buf[10] << 16) | (pozyx->rx_buf[9] << 8) | pozyx->rx_buf[8]);
    break;
  }

  case POZYX_QUAT_W: {
    pozyx->orientation.w = (double)((pozyx->rx_buf[1] << 8) | pozyx->rx_buf[0]) / POZYX_QUAT_DIV;
    pozyx->orientation.x = (double)((pozyx->rx_buf[3] << 8) | pozyx->rx_buf[2]) / POZYX_QUAT_DIV;
    pozyx->orientation.y = (double)((pozyx->rx_buf[5] << 8) | pozyx->rx_buf[4]) / POZYX_QUAT_DIV;
    pozyx->orientation.z = (double)((pozyx->rx_buf[7] << 8) | pozyx->rx_buf[6]) / POZYX_QUAT_DIV;

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

void pozyx_read_position(pozyx_t *pozyx) {
  pozyx->reading_device = POZYX_POS_X;
  pozyx_mem_read_DMA(pozyx, POZYX_POS_X, 3 * sizeof(int32_t));
}

void pozyx_read_quaternions(pozyx_t *pozyx) {
  pozyx->reading_device = POZYX_QUAT_W;
  pozyx_mem_read_DMA(pozyx, POZYX_QUAT_W, 4 * sizeof(int16_t));
}

bool pozyx_check_for_position_and_update(pozyx_t *pozyx) {
  if (!pwr_manager_get_power_pozyx() || !pozyx->new_pos_available) {
    return false;
  }

  pozyx_read_position(pozyx);

  while (pozyx->reading_device != -1) {
  }

  pozyx_read_quaternions(pozyx);

  pozyx->new_pos_available = false;
  return true;
}

void pozyx_config_interrupt_pin(pozyx_t *pozyx, uint8_t pin, uint8_t mode, uint8_t bActiveHigh, uint8_t bLatch) {
  uint8_t int_config = pin | (mode << 3) | (bActiveHigh << 4) | (bLatch << 5);
  pozyx_write_DMA(pozyx, POZYX_INT_CONFIG, int_config);
}

void pozyx_set_interrupt_mask(pozyx_t *pozyx, uint8_t mask) { pozyx_write_DMA(pozyx, POZYX_INT_MASK, mask); }