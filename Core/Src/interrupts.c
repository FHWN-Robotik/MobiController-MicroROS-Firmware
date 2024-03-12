/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: interrupts.c
 * Created Date: Monday, March 4th 2024, 12:35:40 pm
 * Author: Florian Hye
 * Description: This file has all interrupt hanlders for this firmware.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#include "encoder.h"
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
  if (hi2c != imu.i2c_handle) {
    return;
  }

  bno055_read_DMA_complete(&imu);
}

/*
 * GPIO external interrupt
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

  // Handle encoder ext interrupts
  if (GPIO_Pin == encoder_1.gpio_a_pin) {
    encoder_handle_interrupt(&encoder_1);
  } else if (GPIO_Pin == encoder_2.gpio_a_pin) {
    encoder_handle_interrupt(&encoder_2);
  } else if (GPIO_Pin == encoder_3.gpio_a_pin) {
    encoder_handle_interrupt(&encoder_3);
  } else if (GPIO_Pin == encoder_4.gpio_a_pin) {
    encoder_handle_interrupt(&encoder_4);
  }
}

/*
 * ADC
 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  // Conversion Complete & DMA Transfer Complete As Well
  if (hadc != pwr_manager.adc)
    return;

  pwr_manager.battery_voltage =
    __LL_ADC_CALC_DATA_TO_VOLTAGE(3300UL, pwr_manager.adc_res, LL_ADC_RESOLUTION_12B) * (14 / 3.3) * 0.001;

  (void)pwr_manager_check_for_battery_warning(&pwr_manager);
}