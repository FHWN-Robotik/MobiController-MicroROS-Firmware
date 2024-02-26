#include "canlib.h"
#include "can.h"
#include "utils.h"

void canlib_init(canlib_t *can, CAN_HandleTypeDef *can_handle) {
  can->can_handle = can_handle;
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
  uint8_t data[8] = {};
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
  // vx = etl::clamp<int16_t>(vx, -3276, 3276);
  // vy = etl::clamp<int16_t>(vy, -3276, 3276);
  // vphi = etl::clamp<int16_t>(vphi, -3276, 3276);

  // FIXME: the motorcontoller does wierd stuff above a value of 2077 mm/s
  // vx = etl::clamp<int16_t>(vx, -2000, 2000);
  // vy = etl::clamp<int16_t>(vy, -2000, 2000);
  // vphi = etl::clamp<int16_t>(vphi, -2000, 2000);

  vx = clamp(vx, -2000, 2000);
  vy = clamp(vy, -2000, 2000);
  vphi = clamp(vphi, -2000, 2000);

  int16_t val_x = vx * 10; // m/s * 10000 -> mm/s * 10
  uint16_t x_twos = twos_complement(val_x);
  // uint8_t x_high = x_twos >> 8;
  // uint8_t x_low = (uint8_t)x_twos;

  int16_t val_y = vy * 10; // m/s * 10000 -> mm/s * 10
  uint16_t y_twos = twos_complement(val_y);
  // uint8_t y_high = y_twos >> 8;
  // uint8_t y_low = (uint8_t)y_twos;

  int16_t val_phi = vphi * 10; // rad/s * 10000 -> mrad/s * 10
  uint16_t phi_twos = twos_complement(val_phi);
  // uint8_t phi_high = phi_twos >> 8;
  // uint8_t phi_low = (uint8_t)phi_twos;

  uint8_t x_low = val_x & 0xFF;         // Extract the low byte of val_x
  uint8_t x_high = (val_x >> 8) & 0xFF; // Extract the high byte of val_x

  uint8_t y_low = val_y & 0xFF;         // Extract the low byte of val_y
  uint8_t y_high = (val_y >> 8) & 0xFF; // Extract the high byte of val_y

  uint8_t phi_low = val_phi & 0xFF;         // Extract the low byte of val_phi
  uint8_t phi_high = (val_phi >> 8) & 0xFF; // Extract the high byte of val_phi

  uint8_t data[8] = {
      x_low, x_high, y_low, y_high, phi_low, phi_high, 0x00, 0x00,
  };
  return canlib_send_extended(can, CANLIB_MOVE_LATERALLY, data);
}