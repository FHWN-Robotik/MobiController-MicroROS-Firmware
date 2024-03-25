/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: power_manager.h
 * Created Date: Monday, March 4th 2024, 5:06:14 pm
 * Author: Florian Hye
 * Description: This file defines the function of the power manager.
 *              It is used to manage the power of the LED strip and the Pozyx tag.
 *              Furthermore, it handles the ADC for reading the battery voltage.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#ifndef __POWER_MANAGER_H_
#define __POWER_MANAGER_H_

#include "adc.h"
#include "stdbool.h"
#include "stm32l4xx.h"

typedef struct pwr_manager_s {
  ADC_HandleTypeDef *adc;

  bool battery_warning_triggerd;

  uint32_t adc_res;
  float battery_voltage;
  bool is_battery_connected;
} pwr_manager_t;

void pwr_manager_init(pwr_manager_t *pwr_manager, ADC_HandleTypeDef *hadc);

void pwr_manager_read_battery_voltage(pwr_manager_t *pwr_manager);

void pwr_manager_set_power_pozyx(bool pwr);
void pwr_manager_set_power_led(bool pwr);

bool pwr_manager_get_power_pozyx(void);
bool pwr_manager_get_power_led(void);

void pwr_manager_toggle_power_pozyx(void);
void pwr_manager_toggle_power_led(void);

void pwr_manager_check_for_battery_warning(pwr_manager_t *pwr_manager);

#endif /* __POWER_MANAGER_H_ */