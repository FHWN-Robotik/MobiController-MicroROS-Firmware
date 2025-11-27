/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055_dma.h"
#include "bootloader.h"
#include "canlib.h"
#include "encoder.h"
#include "hcsr04.h"
#include "led_strip.h"
#include "pozyx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
BNO055_t imu;
canlib_t can;

encoder_t encoder_1;
encoder_t encoder_2;
encoder_t encoder_3;
encoder_t encoder_4;

hcsr04_t ultra_1;
hcsr04_t ultra_2;
hcsr04_t ultra_3;
hcsr04_t ultra_4;
hcsr04_t ultra_5;
hcsr04_t ultra_6;

pwr_manager_t pwr_manager = {
  .battery_warning_triggerd = false,
};

led_strip_t led_strip;

pozyx_t pozyx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  printf("\n---------------------------------------------\n");
  printf("DEBUG INTERFACE!\n");
  printf("MobiController Firmware\n");
  printf("version %s\n", FIRMWARE_VERSION);
  printf("Time of compilation: %s %s\n", __DATE__, __TIME__);
  printf("Compiler version: %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
  printf("Copyright FHWN Florian Hye\n");
  printf("---------------------------------------------\n");

  printf("Init power manager...\n");
  pwr_manager_init(&pwr_manager, &hadc1);

  printf("Init led stip...\n");
  led_strip_init(&led_strip);
  pwr_manager_set_power_led(true);
  led_strip_power_on_animation(&led_strip, &(mobi_interfaces__msg__ColorRGBW){.r = 0, .g = 220, .b = 255, .w = 0});

  // Should boot to bootloader?
  bool btn_state = !(bool)HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin);
  if (btn_state) {
    pwr_manager_set_power_led(true);
    led_strip_power_on_animation(&led_strip, &(mobi_interfaces__msg__ColorRGBW){.r = 255, .g = 0, .b = 100, .w = 0});
    printf("Interrupting boot! Rebooting into DFU-Mode\n");
    HAL_Delay(1800);
    jump_to_bootloader();
    return 0;
  }

  printf("Initializing i2c devices...\n");
  bno055_init(&imu, &hi2c1, BNO055_I2C_ADDR_LO);

  printf("Starting CAN...\n");
  canlib_init(&can, &hcan1);
  canlib_send_stop(&can); // Stop motors!

  printf("Init encoders...\n");
  encoder_init(&encoder_1, ENCODER_1_A_GPIO_Port, ENCODER_1_A_Pin, ENCODER_1_B_GPIO_Port, ENCODER_1_B_Pin);
  encoder_init(&encoder_2, ENCODER_2_A_GPIO_Port, ENCODER_2_A_Pin, ENCODER_2_B_GPIO_Port, ENCODER_2_B_Pin);
  encoder_init(&encoder_3, ENCODER_3_A_GPIO_Port, ENCODER_3_A_Pin, ENCODER_3_B_GPIO_Port, ENCODER_3_B_Pin);
  encoder_init(&encoder_4, ENCODER_4_A_GPIO_Port, ENCODER_4_A_Pin, ENCODER_4_B_GPIO_Port, ENCODER_4_B_Pin);

  printf("Init Ultrasonic sensors...\n");
  hcsr04_init(&ultra_1, &htim1, TIM_CHANNEL_1, HAL_TIM_ACTIVE_CHANNEL_1, TIM_IT_CC1, US_TRIG_1_GPIO_Port,
              US_TRIG_1_Pin);
  hcsr04_init(&ultra_2, &htim2, TIM_CHANNEL_3, HAL_TIM_ACTIVE_CHANNEL_3, TIM_IT_CC3, US_TRIG_2_GPIO_Port,
              US_TRIG_2_Pin);
  hcsr04_init(&ultra_3, &htim1, TIM_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_2, TIM_IT_CC2, US_TRIG_3_GPIO_Port,
              US_TRIG_3_Pin);
  hcsr04_init(&ultra_4, &htim2, TIM_CHANNEL_1, HAL_TIM_ACTIVE_CHANNEL_1, TIM_IT_CC1, US_TRIG_4_GPIO_Port,
              US_TRIG_4_Pin);
  hcsr04_init(&ultra_5, &htim2, TIM_CHANNEL_4, HAL_TIM_ACTIVE_CHANNEL_4, TIM_IT_CC4, US_TRIG_5_GPIO_Port,
              US_TRIG_5_Pin);
  hcsr04_init(&ultra_6, &htim2, TIM_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_2, TIM_IT_CC2, US_TRIG_6_GPIO_Port,
              US_TRIG_6_Pin);

  printf("Starting FreeRTOS...\n");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Channel == ultra_1.active_channel && htim == ultra_1.htim) { // US 1
    hcsr04_handle_period_elapsed_interrupt(&ultra_1);
  } else if (htim->Channel == ultra_2.active_channel && htim == ultra_2.htim) { // US 2
    hcsr04_handle_period_elapsed_interrupt(&ultra_2);
  } else if (htim->Channel == ultra_3.active_channel && htim == ultra_3.htim) { // US 3
    hcsr04_handle_period_elapsed_interrupt(&ultra_3);
  } else if (htim->Channel == ultra_4.active_channel && htim == ultra_4.htim) { // US 4
    hcsr04_handle_period_elapsed_interrupt(&ultra_4);
  } else if (htim->Channel == ultra_5.active_channel && htim == ultra_5.htim) { // US 5
    hcsr04_handle_period_elapsed_interrupt(&ultra_5);
  } else if (htim->Channel == ultra_6.active_channel && htim == ultra_6.htim) { // US 6
    hcsr04_handle_period_elapsed_interrupt(&ultra_6);
  }

  // LED Strip loop
  if (htim->Instance == TIM6) {
    led_strip_handle_timer_interrupt(&led_strip);
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
    printf("ERROR: Error_Handler\n");
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  printf("Wrong parameters value: file %s on line %ld\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
