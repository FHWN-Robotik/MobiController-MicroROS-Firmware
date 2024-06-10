/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: canlib.c
 * Created Date: Monday, February 26th 2024, 2:53:10 pm
 * Author: Florian Hye
 * Description: This file implements the canlib functions.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#include "canlib.h"
#include "can.h"
#include "rmw_microros/time_sync.h"
#include "utils.h"

void canlib_init(canlib_t *can, CAN_HandleTypeDef *can_handle) {
  can->can_handle = can_handle;
  can->is_driving = false;
  can->last_update = rmw_uros_epoch_millis();
  HAL_CAN_Start(can_handle);
}

HAL_StatusTypeDef canlib_send_data(canlib_t *can, uint32_t ide, uint32_t ext_id, uint32_t rtr, uint32_t dlc,
                                   const uint8_t data[]) {
  CAN_TxHeaderTypeDef tx_header;
  tx_header.IDE = ide;
  tx_header.ExtId = ext_id;
  tx_header.RTR = rtr;
  tx_header.DLC = dlc;
  tx_header.TransmitGlobalTime = DISABLE;

  uint32_t tx_mailbox;
  return HAL_CAN_AddTxMessage(can->can_handle, &tx_header, data, &tx_mailbox);
}

HAL_StatusTypeDef canlib_send_extended(canlib_t *can, uint32_t ext_id, const uint8_t data[]) {
  return canlib_send_data(can, CAN_ID_EXT, ext_id, CAN_RTR_DATA, 8, data);
}

HAL_StatusTypeDef canlib_send_stop(canlib_t *can) {
  uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // Set is driving
  can->is_driving = false;
  can->last_update = rmw_uros_epoch_millis();

  return canlib_send_extended(can, CANLIB_MOVE_LATERALLY, data);
}

/**
 * @brief Drive the robot
 * @param vx Velocity in x direction in mm/s
 * @param vy Velocity in x direction in mm/s
 * @param vphi Velocity in x direction in mrad/s
 * @retval HAL status
 */
HAL_StatusTypeDef canlib_drive(canlib_t *can, int16_t vx, int16_t vy, int16_t vphi) {
  // NOTE: The motorcontoller does wierd stuff above a value of 2077 mm/s
  vx = clamp(vx, -2000, 2000);
  vy = clamp(vy, -2000, 2000);
  vphi = clamp(vphi, -2000, 2000);

  int16_t val_x = vx * 10;     // m/s * 10000 -> mm/s * 10
  int16_t val_y = vy * 10;     // m/s * 10000 -> mm/s * 10
  int16_t val_phi = vphi * 10; // rad/s * 10000 -> mrad/s * 10

  uint8_t x_low = val_x & 0xFF;         // Extract the low byte of val_x
  uint8_t x_high = (val_x >> 8) & 0xFF; // Extract the high byte of val_x

  uint8_t y_low = val_y & 0xFF;         // Extract the low byte of val_y
  uint8_t y_high = (val_y >> 8) & 0xFF; // Extract the high byte of val_y

  uint8_t phi_low = val_phi & 0xFF;         // Extract the low byte of val_phi
  uint8_t phi_high = (val_phi >> 8) & 0xFF; // Extract the high byte of val_phi

  uint8_t data[8] = {
    x_low, x_high, y_low, y_high, phi_low, phi_high, 0x00, 0x00,
  };

  // Set is driving
  if (x_low == 0 && x_high == 0 && y_low == 0 && y_high == 0 && phi_low == 0 && phi_high == 0) {
    can->is_driving = false;
  } else {
    can->is_driving = true;
  }
  can->last_update = rmw_uros_epoch_millis();

  // Send CAN frame
  return canlib_send_extended(can, CANLIB_MOVE_LATERALLY, data);
}