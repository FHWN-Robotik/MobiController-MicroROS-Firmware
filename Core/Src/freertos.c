/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"

#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/temperature.h>
#include <std_msgs/msg/bool.h>
#include <stdbool.h>
#include <uxr/client/transport.h>

#include "bno055_dma.h"
#include "utils.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCCHECK(fn)                                                                \
  {                                                                                \
    rcl_ret_t temp_rc = fn;                                                        \
    if ((temp_rc != RCL_RET_OK)) {                                                 \
      printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
      return;                                                                      \
    }                                                                              \
  }

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
rcl_publisher_t temp_pup;
rcl_publisher_t imu_pup;

/* USER CODE END Variables */
/* Definitions for ros_task */
osThreadId_t ros_taskHandle;
uint32_t ros_task_buffer[3000];
osStaticThreadDef_t ros_task_control_block;
const osThreadAttr_t ros_task_attributes = {
    .name = "ros_task",
    .cb_mem = &ros_task_control_block,
    .cb_size = sizeof(ros_task_control_block),
    .stack_mem = &ros_task_buffer[0],
    .stack_size = sizeof(ros_task_buffer),
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport* transport);
bool cubemx_transport_close(struct uxrCustomTransport* transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void* microros_allocate(size_t size, void* state);
void microros_deallocate(void* pointer, void* state);
void* microros_reallocate(void* pointer, size_t size, void* state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void* state);

void timer_1s_callback(rcl_timer_t* timer, int64_t last_call_time);
void timer_100ms_callback(rcl_timer_t* timer, int64_t last_call_time);
/* USER CODE END FunctionPrototypes */

void start_ros_task(void* argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ros_task */
  ros_taskHandle = osThreadNew(start_ros_task, NULL, &ros_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_start_ros_task */
/**
 * @brief  Function implementing the ros_task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_start_ros_task */
void start_ros_task(void* argument) {
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN start_ros_task */
  /* Infinite loop */

  // micro-ROS configuration

  rmw_uros_set_custom_transport(
      true,
      NULL,
      cubemx_transport_open,
      cubemx_transport_close,
      cubemx_transport_write,
      cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // micro-ROS app

  rcl_publisher_t publisher_button;
  std_msgs__msg__Bool msg_button;

  rclc_support_t support;
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_allocator_t allocator;

  rcl_node_t node;

  allocator = rcl_get_default_allocator();

  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 255));  // Domain ID 255 is set so the Domain ID of the agent is used https://github.com/micro-ROS/micro-ROS-Agent/issues/182

  // create init_options
  // rclc_support_init(&support, 0, NULL, &allocator);
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "stm32_node", "", &support));  // Node Name: stm32_node, Namespace: ""

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher_button,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/button"));

  RCCHECK(rclc_publisher_init_default(&temp_pup, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "/temperature"));

  RCCHECK(rclc_publisher_init_default(&imu_pup, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu"));

  // Timers
  rcl_timer_t timer_1s;
  RCCHECK(rclc_timer_init_default(&timer_1s, &support, RCL_MS_TO_NS(1000), timer_1s_callback));

  rcl_timer_t timer_100ms;
  RCCHECK(rclc_timer_init_default(&timer_100ms, &support, RCL_MS_TO_NS(100), timer_100ms_callback));

  // Init executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_1s));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_100ms));

  bool last_button_state = false;

  for (;;) {
    // NOTE: The Button is inverted!
    bool btn_state = !(bool)HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin);

    if (btn_state != last_button_state) {
      last_button_state = btn_state;
      msg_button.data = btn_state;

      RCCHECK(rcl_publish(&publisher_button, &msg_button, NULL));
    }

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    osDelay(10);
  }
  /* USER CODE END start_ros_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void timer_1s_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;

  if (timer != NULL) {
    bno055_read_temp(&imu);
    stamp_header(&imu.temperature.header.stamp);
    RCCHECK(rcl_publish(&temp_pup, &imu.temperature, NULL));
  }
}

void timer_100ms_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;

  if (timer != NULL) {
    bno055_read_quaternion(&imu);
    bno055_read_angular_velocity(&imu);
    bno055_read_linear_acceleration(&imu);

    sensor_msgs__msg__Imu imu_msg = {
        .header = {
            .frame_id = "imu",
        },
        .orientation_covariance = {0.0159, 0, 0, 0, 0.0159, 0, 0, 0, 0.0159},
        .angular_velocity_covariance = {0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04},
        .linear_acceleration_covariance = {0.017, 0, 0, 0, 0.017, 0, 0, 0, 0.017},
        .orientation = imu.orientation,
        .angular_velocity = imu.angular_velocity,
        .linear_acceleration = imu.linear_acceleration,
    };

    stamp_header(&imu_msg.header.stamp);

    RCCHECK(rcl_publish(&imu_pup, &imu_msg, NULL));
  }
}

/* USER CODE END Application */
