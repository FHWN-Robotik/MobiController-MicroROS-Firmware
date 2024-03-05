/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: power_manager.c
 * Created Date: Monday, March 4th 2024, 5:10:28 pm
 * Author: Florian Hye
 * Description: This file implements the power manager functions.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#include "power_manager.h"
#include "adc.h"
#include "gpio.h"

void pwr_manager_init(pwr_manager_t *pwr_manager, ADC_HandleTypeDef *hadc) {
  pwr_manager->adc = hadc;
  pwr_manager->adc_res = 0;
  pwr_manager->battery_voltage = 0;
  pwr_manager->battery_warning_triggerd = false;

  HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
}

void pwr_manager_read_battery_voltage(pwr_manager_t *pwr_manager) {
  HAL_ADC_Start_DMA(pwr_manager->adc, &pwr_manager->adc_res, 1);
}
