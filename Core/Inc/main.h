/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055_dma.h"
#include "encoder.h"
#include "hcsr04.h"
#include "led_strip.h"
#include "power_manager.h"
#include "pozyx.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern struct BNO055_s imu;
extern struct canlib_s can;

extern struct encoder_s encoder_1;
extern struct encoder_s encoder_2;
extern struct encoder_s encoder_3;
extern struct encoder_s encoder_4;

extern struct hcsr04_s ultra_1;
extern struct hcsr04_s ultra_2;
extern struct hcsr04_s ultra_3;
extern struct hcsr04_s ultra_4;
extern struct hcsr04_s ultra_5;
extern struct hcsr04_s ultra_6;

extern struct pwr_manager_s pwr_manager;

extern struct led_strip_s led_strip;

extern struct pozyx_s pozyx;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define POZYX_INT1_Pin GPIO_PIN_0
#define POZYX_INT1_GPIO_Port GPIOC
#define POZYX_INT1_EXTI_IRQn EXTI0_IRQn
#define ONOFF_LED_STRIP_Pin GPIO_PIN_1
#define ONOFF_LED_STRIP_GPIO_Port GPIOC
#define US_TRIG_6_Pin GPIO_PIN_2
#define US_TRIG_6_GPIO_Port GPIOC
#define ONOFF_POZYX_Pin GPIO_PIN_3
#define ONOFF_POZYX_GPIO_Port GPIOC
#define US_ECHO_4_Pin GPIO_PIN_0
#define US_ECHO_4_GPIO_Port GPIOA
#define US_ECHO_6_Pin GPIO_PIN_1
#define US_ECHO_6_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define SMPS_EN_Pin GPIO_PIN_4
#define SMPS_EN_GPIO_Port GPIOA
#define SMPS_V1_Pin GPIO_PIN_5
#define SMPS_V1_GPIO_Port GPIOA
#define SMPS_PG_Pin GPIO_PIN_6
#define SMPS_PG_GPIO_Port GPIOA
#define SMPS_SW_Pin GPIO_PIN_7
#define SMPS_SW_GPIO_Port GPIOA
#define US_TRIG_1_Pin GPIO_PIN_4
#define US_TRIG_1_GPIO_Port GPIOC
#define ADC_BAT_VOLT_Pin GPIO_PIN_0
#define ADC_BAT_VOLT_GPIO_Port GPIOB
#define ENCODER_2_A_Pin GPIO_PIN_1
#define ENCODER_2_A_GPIO_Port GPIOB
#define ENCODER_2_A_EXTI_IRQn EXTI1_IRQn
#define US_TRIG_3_Pin GPIO_PIN_2
#define US_TRIG_3_GPIO_Port GPIOB
#define US_ECHO_2_Pin GPIO_PIN_10
#define US_ECHO_2_GPIO_Port GPIOB
#define US_ECHO_5_Pin GPIO_PIN_11
#define US_ECHO_5_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_13
#define LD4_GPIO_Port GPIOB
#define POZYX_INT2_Pin GPIO_PIN_14
#define POZYX_INT2_GPIO_Port GPIOB
#define POZYX_INT2_EXTI_IRQn EXTI15_10_IRQn
#define ENCODER_3_A_Pin GPIO_PIN_15
#define ENCODER_3_A_GPIO_Port GPIOB
#define ENCODER_3_A_EXTI_IRQn EXTI15_10_IRQn
#define ENCODER_3_B_Pin GPIO_PIN_6
#define ENCODER_3_B_GPIO_Port GPIOC
#define ENCODER_1_A_Pin GPIO_PIN_7
#define ENCODER_1_A_GPIO_Port GPIOC
#define ENCODER_1_A_EXTI_IRQn EXTI9_5_IRQn
#define ENCODER_4_B_Pin GPIO_PIN_8
#define ENCODER_4_B_GPIO_Port GPIOC
#define US_ECHO_1_Pin GPIO_PIN_8
#define US_ECHO_1_GPIO_Port GPIOA
#define US_ECHO_3_Pin GPIO_PIN_9
#define US_ECHO_3_GPIO_Port GPIOA
#define ENCODER_4_A_Pin GPIO_PIN_10
#define ENCODER_4_A_GPIO_Port GPIOA
#define ENCODER_4_A_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define US_TRIG_2_Pin GPIO_PIN_15
#define US_TRIG_2_GPIO_Port GPIOA
#define ENCODER_2_B_Pin GPIO_PIN_10
#define ENCODER_2_B_GPIO_Port GPIOC
#define ENCODER_1_B_Pin GPIO_PIN_11
#define ENCODER_1_B_GPIO_Port GPIOC
#define US_TRIG_4_Pin GPIO_PIN_12
#define US_TRIG_4_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LED_OUT_Pin GPIO_PIN_4
#define LED_OUT_GPIO_Port GPIOB
#define US_TRIG_5_Pin GPIO_PIN_5
#define US_TRIG_5_GPIO_Port GPIOB
#define USER_BTN_Pin GPIO_PIN_8
#define USER_BTN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LOGGER_NAME "MOBI"
#define FIRMWARE_VERSION "v2.0.0"
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
