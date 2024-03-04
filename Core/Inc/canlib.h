/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: canlib.h
 * Created Date: Monday, February 26th 2024, 2:51:45 pm
 * Author: Florian Hye
 * Description: This file defines the canlib functions.
 * These are used to more easily controll the motor controler via CAN.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#ifndef __CANLIB_H_
#define __CANLIB_H_

#include "can.h"
#include "stm32l4xx.h"

#define CANLIB_MOVE_LATERALLY 0x00002A01

typedef struct canlib_s {
  CAN_HandleTypeDef *can_handle;
} canlib_t;

void canlib_init(canlib_t *can, CAN_HandleTypeDef *can_handle);

HAL_StatusTypeDef canlib_send_data(canlib_t *can, uint32_t ide, uint32_t ext_id, uint32_t rtr, uint32_t dlc,
                                   const uint8_t data[]);
HAL_StatusTypeDef canlib_send_extended(canlib_t *can, uint32_t ext_id, const uint8_t data[]);
HAL_StatusTypeDef canlib_send_stop(canlib_t *can);
HAL_StatusTypeDef canlib_drive(canlib_t *can, int16_t vx, int16_t vy, int16_t vphi);

#endif /* __CANLIB_H_ */
