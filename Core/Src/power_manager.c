/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: power_manager.c
 * Created Date: Monday, March 4th 2024, 5:10:28 pm
 * Author: Florian Hye
 * Description: This file implements the power manager functions.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#include <rcutils/logging_macros.h>

#include "adc.h"
#include "gpio.h"
#include "power_manager.h"

void pwr_manager_init(pwr_manager_t *pwr_manager, ADC_HandleTypeDef *hadc) {
  pwr_manager->adc = hadc;
  pwr_manager->adc_res = 0;
  pwr_manager->battery_voltage = 0;
  pwr_manager->battery_warning_triggerd = false;
  pwr_manager->is_battery_connected = true; // Assume a battery is connected.
  pwr_manager->charge_percentage = 1;

  HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);

  // Init with pozyx and led turned off.
  pwr_manager_set_power_pozyx(false);
  pwr_manager_set_power_led(false);
}

void pwr_manager_read_battery_voltage(pwr_manager_t *pwr_manager) {
  HAL_ADC_Start_DMA(pwr_manager->adc, &pwr_manager->adc_res, 1);
}

void pwr_manager_set_power_pozyx(bool pwr) { HAL_GPIO_WritePin(ONOFF_POZYX_GPIO_Port, ONOFF_POZYX_Pin, !pwr); }

void pwr_manager_set_power_led(bool pwr) { HAL_GPIO_WritePin(ONOFF_LED_STRIP_GPIO_Port, ONOFF_LED_STRIP_Pin, !pwr); }

bool pwr_manager_get_power_pozyx() {
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(ONOFF_POZYX_GPIO_Port, ONOFF_POZYX_Pin);
  return (bool)!pin_state;
}

bool pwr_manager_get_power_led() {
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(ONOFF_LED_STRIP_GPIO_Port, ONOFF_LED_STRIP_Pin);
  return (bool)!pin_state;
}

void pwr_manager_toggle_power_pozyx() { HAL_GPIO_TogglePin(ONOFF_POZYX_GPIO_Port, ONOFF_POZYX_Pin); }

void pwr_manager_toggle_power_led() { HAL_GPIO_TogglePin(ONOFF_LED_STRIP_GPIO_Port, ONOFF_LED_STRIP_Pin); }

void pwr_manager_check_for_battery_warning(pwr_manager_t *pwr_manager) {
  // Check if a battery is connected
  if (pwr_manager->battery_voltage == 0) {
    if (pwr_manager->is_battery_connected) {
      RCUTILS_LOG_WARN_NAMED(LOGGER_NAME, "There is no battery connected, ignoring low battery voltage.");
      pwr_manager->is_battery_connected = false;
      pwr_manager->battery_warning_triggerd = false;
    }
    return;
  }

  // If we get here, assume a bettery to be connected.
  if (!pwr_manager->is_battery_connected) {
    RCUTILS_LOG_WARN_NAMED(LOGGER_NAME, "There is a battery connected.");
    pwr_manager->is_battery_connected = true;
  }

  // Check if the battery voltage is bellow 11 V
  if (pwr_manager->battery_voltage <= BAT_MIN_VOLTAGE) {
    if (!pwr_manager->battery_warning_triggerd) {
      RCUTILS_LOG_WARN_NAMED(LOGGER_NAME, "Battery is empty!");
      pwr_manager->battery_warning_triggerd = true;
    }
    return;
  }
}
