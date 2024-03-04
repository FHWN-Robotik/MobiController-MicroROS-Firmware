/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: interrupts.c
 * Created Date: Monday, March 4th 2024, 12:35:40 pm
 * Author: Florian Hye
 * Description: This file has all interrupt hanlders for this firmware.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#include "i2c.h"
#include "main.h"
#include "stm32l4xx.h"

/*
 * I2C interrupts
 */

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  // TX Done .. Do Something!
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance != I2C1) {
    return;
  }

  bno055_read_DMA_complete(&imu);
}
