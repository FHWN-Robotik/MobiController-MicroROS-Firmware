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
#include <geometry_msgs/msg/twist.h>
#include <math.h>
#include <micro_ros_utilities/string_utilities.h>
#include <mobi_interfaces/msg/encoders_stamped.h>
#include <mobi_interfaces/msg/ultra_ranges.h>
#include <mobi_interfaces/srv/get_imu_calib_data.h>
#include <mobi_interfaces/srv/get_imu_calib_status.h>
#include <mobi_interfaces/srv/set_imu_calib_data.h>
#include <mobi_interfaces/srv/set_power.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/logging.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <rosidl_runtime_c/service_type_support_struct.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/temperature.h>
#include <std_msgs/msg/bool.h>
#include <std_srvs/srv/trigger.h>
#include <stdbool.h>
#include <uxr/client/transport.h>

#include "bno055_dma.h"
#include "bootloader.h"
#include "canlib.h"
#include "hcsr04.h"
#include "utils.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK)) {                                                                                     \
      RCUTILS_LOG_DEBUG_NAMED(LOGGER_NAME, "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc);         \
      return;                                                                                                          \
    }                                                                                                                  \
  }

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// Publishers
rcl_publisher_t publisher_button;
rcl_publisher_t temp_pup;
rcl_publisher_t imu_pup;
rcl_publisher_t encoders_pup;
rcl_publisher_t battery_state_pub;
rcl_publisher_t ultra_ranges_pup;

// Publisher msgs
std_msgs__msg__Bool msg_button;
mobi_interfaces__msg__EncodersStamped encoders_msg;
sensor_msgs__msg__Imu imu_msg = {
  .orientation_covariance = {0.0159, 0, 0, 0, 0.0159, 0, 0, 0, 0.0159},
  .angular_velocity_covariance = {0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04},
  .linear_acceleration_covariance = {0.017, 0, 0, 0, 0.017, 0, 0, 0, 0.017},
};
sensor_msgs__msg__BatteryState battery_state_msg = {
  .header.frame_id = "battery",
  .voltage = 0,

  .power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN,
  .power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION,
  .power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN,
  .design_capacity = 10.4,
  .present = true,

  .temperature = NAN,
  .current = NAN,
  .charge = NAN,
  .capacity = NAN,
  .percentage = NAN,
};
mobi_interfaces__msg__UltraRanges ultra_ranges_msg; // This will be initialized in "start_ros_task"

// Subscribers
rcl_subscription_t cmd_vel_sub;

// Subscriber msgs
geometry_msgs__msg__Twist cmd_vel_msg;

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
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);

void timer_1s_callback(rcl_timer_t *timer, int64_t last_call_time);
void timer_100ms_callback(rcl_timer_t *timer, int64_t last_call_time);
void timer_250ms_callback(rcl_timer_t *timer, int64_t last_call_time);
void imu_get_calib_status_callback(const void *imu_get_calib_status_req, void *imu_get_calib_status_res);
void imu_get_calib_data_callback(const void *imu_get_calib_data_req, void *imu_get_calib_data_res);
void imu_set_calib_data_callback(const void *imu_set_calib_data_req, void *imu_set_calib_data_res);
void boot_bootlaoder_callback(const void *boot_bootlaoder_req, void *boot_bootlaoder_res);
void pozyx_set_pwr_callback(const void *pozyx_set_pwr_req, void *pozyx_set_pwr_res);
void cmd_vel_callback(const void *msgin);
/* USER CODE END FunctionPrototypes */

void start_ros_task(void *argument);

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
void start_ros_task(void *argument) {
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN start_ros_task */
  /* Infinite loop */

  // micro-ROS configuration

  rmw_uros_set_custom_transport(true, NULL, cubemx_transport_open, cubemx_transport_close, cubemx_transport_write,
                                cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    RCUTILS_LOG_ERROR_NAMED(LOGGER_NAME, "Error on default allocators (line %d)", __LINE__);
  }

  // micro-ROS app

  // Publishers
  hcsr04_init_range_msg(&ultra_ranges_msg.front_left, micro_ros_string_utilities_init("us_front_left"));
  hcsr04_init_range_msg(&ultra_ranges_msg.front_right, micro_ros_string_utilities_init("us_front_right"));
  hcsr04_init_range_msg(&ultra_ranges_msg.center_left, micro_ros_string_utilities_init("us_center_left"));
  hcsr04_init_range_msg(&ultra_ranges_msg.center_right, micro_ros_string_utilities_init("us_center_right"));
  hcsr04_init_range_msg(&ultra_ranges_msg.rear_left, micro_ros_string_utilities_init("us_rear_left"));
  hcsr04_init_range_msg(&ultra_ranges_msg.rear_right, micro_ros_string_utilities_init("us_rear_right"));

  // Services
  rcl_service_t imu_get_calib_status_srv = rcl_get_zero_initialized_service();
  mobi_interfaces__srv__GetImuCalibStatus_Request imu_get_calib_status_req;
  mobi_interfaces__srv__GetImuCalibStatus_Response imu_get_calib_status_res;

  rcl_service_t imu_get_calib_data_srv = rcl_get_zero_initialized_service();
  mobi_interfaces__srv__GetImuCalibData_Request imu_get_calib_data_req;
  mobi_interfaces__srv__GetImuCalibData_Response imu_get_calib_data_res;

  rcl_service_t imu_set_calib_data_srv = rcl_get_zero_initialized_service();
  mobi_interfaces__srv__SetImuCalibData_Request imu_set_calib_data_req;
  mobi_interfaces__srv__SetImuCalibData_Response imu_set_calib_data_res;

  rcl_service_t boot_bootloader_srv = rcl_get_zero_initialized_service();
  std_srvs__srv__Trigger_Request boot_bootloader_req;
  std_srvs__srv__Trigger_Response boot_bootloader_res;

  rcl_service_t pozyx_set_pwr_srv = rcl_get_zero_initialized_service();
  mobi_interfaces__srv__SetPower_Request pozyx_set_pwr_req;
  mobi_interfaces__srv__SetPower_Response pozyx_set_pwr_res;

  // RCLC Support
  rclc_support_t support;
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_allocator_t allocator;

  rcl_node_t node;

  allocator = rcl_get_default_allocator();

  // Setup logging
  RCCHECK(rcutils_logging_initialize_with_allocator(allocator));
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);

  RCCHECK(rcl_init_options_init(&init_options, allocator));

  // Domain ID 255 is set so the Domain ID of the agent is used
  // https://github.com/micro-ROS/micro-ROS-Agent/issues/182
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 255));

  // create init_options
  // rclc_support_init(&support, 0, NULL, &allocator);
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node -- Node Name: stm32_node, Namespace: ""
  RCCHECK(rclc_node_init_default(&node, "stm32_node", "", &support));

  // Sync time
  RCCHECK(rmw_uros_sync_session(1000));

  // create publisher
  RCCHECK(
    rclc_publisher_init_default(&publisher_button, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/button"));
  RCCHECK(rclc_publisher_init_default(&temp_pup, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
                                      "/temperature"));
  RCCHECK(rclc_publisher_init_default(&imu_pup, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu"));
  RCCHECK(rclc_publisher_init_default(&encoders_pup, &node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(mobi_interfaces, msg, EncodersStamped), "/encoders"));
  RCCHECK(rclc_publisher_init_default(&battery_state_pub, &node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "/battery_state"));
  RCCHECK(rclc_publisher_init_default(&ultra_ranges_pup, &node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(mobi_interfaces, msg, UltraRanges), "/ultra_ranges"));

  // create subsribers
  RCCHECK(rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                         "/cmd_vel"));

  // Initialize Services
  RCCHECK(rclc_service_init_default(&imu_get_calib_status_srv, &node,
                                    ROSIDL_GET_SRV_TYPE_SUPPORT(mobi_interfaces, srv, GetImuCalibStatus),
                                    "/imu_get_calib_status"));
  RCCHECK(rclc_service_init_default(&imu_get_calib_data_srv, &node,
                                    ROSIDL_GET_SRV_TYPE_SUPPORT(mobi_interfaces, srv, GetImuCalibData),
                                    "/imu_get_calib_data"));
  RCCHECK(rclc_service_init_default(&imu_set_calib_data_srv, &node,
                                    ROSIDL_GET_SRV_TYPE_SUPPORT(mobi_interfaces, srv, SetImuCalibData),
                                    "/imu_set_calib_data"));
  RCCHECK(rclc_service_init_default(&boot_bootloader_srv, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                    "/boot_bootloader"));
  RCCHECK(rclc_service_init_default(&pozyx_set_pwr_srv, &node,
                                    ROSIDL_GET_SRV_TYPE_SUPPORT(mobi_interfaces, srv, SetPower), "/pozyx_set_power"));

  // Timers
  rcl_timer_t timer_1s;
  RCCHECK(rclc_timer_init_default(&timer_1s, &support, RCL_MS_TO_NS(1000), timer_1s_callback));

  rcl_timer_t timer_100ms;
  RCCHECK(rclc_timer_init_default(&timer_100ms, &support, RCL_MS_TO_NS(100), timer_100ms_callback));

  rcl_timer_t timer_250ms;
  RCCHECK(rclc_timer_init_default(&timer_250ms, &support, RCL_MS_TO_NS(250), timer_250ms_callback))

  // Init executor
  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 9, &allocator));

  // Add Timers to executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer_1s));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_100ms));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_250ms));

  // Add Subscriptions to executor
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

  // Add Services to executor
  RCCHECK(rclc_executor_add_service(&executor, &imu_get_calib_status_srv, &imu_get_calib_status_req,
                                    &imu_get_calib_status_res, imu_get_calib_status_callback));
  RCCHECK(rclc_executor_add_service(&executor, &imu_get_calib_data_srv, &imu_get_calib_data_req,
                                    &imu_get_calib_data_res, imu_get_calib_data_callback));
  RCCHECK(rclc_executor_add_service(&executor, &imu_set_calib_data_srv, &imu_set_calib_data_req,
                                    &imu_set_calib_data_res, imu_set_calib_data_callback));
  RCCHECK(rclc_executor_add_service(&executor, &boot_bootloader_srv, &boot_bootloader_req, &boot_bootloader_res,
                                    boot_bootlaoder_callback));
  RCCHECK(rclc_executor_add_service(&executor, &pozyx_set_pwr_srv, &pozyx_set_pwr_req, &pozyx_set_pwr_res,
                                    pozyx_set_pwr_callback));

  // Optional prepare for avoiding allocations during spin
  RCCHECK(rclc_executor_prepare(&executor));

  bool last_button_state = false;

  for (;;) {
    // NOTE: The Button is inverted!
    bool btn_state = !(bool)HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin);

    if (btn_state != last_button_state) {
      last_button_state = btn_state;
      msg_button.data = btn_state;

      RCCHECK(rcl_publish(&publisher_button, &msg_button, NULL));
    }

    // Boot into bootloader if flag is set.
    // Note: The flag gets set by the service /boot_bootloader
    if (should_jump_to_bootloader) {
      pwr_manager_set_power_led(true);
      led_strip_power_on_animation(&led_strip, &(mobi_interfaces__msg__ColorRGBW){.r = 255, .g = 0, .b = 100, .w = 0});
      osDelay(1800);
      jump_to_bootloader();
    }

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    osDelay(10);
  }

  /* USER CODE END start_ros_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void timer_1s_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;

  if (timer == NULL)
    return;

  // Temperature
  bno055_read_temp(&imu);
  stamp_header(&imu.temperature->header.stamp);
  RCCHECK(rcl_publish(&temp_pup, &imu.temperature, NULL));

  // Battery voltage
  pwr_manager_read_battery_voltage(&pwr_manager);
  pwr_manager_check_for_battery_warning(&pwr_manager);
  battery_state_msg.voltage = pwr_manager.battery_voltage;
  battery_state_msg.present = pwr_manager.is_battery_connected;
  stamp_header(&battery_state_msg.header.stamp);
  RCCHECK(rcl_publish(&battery_state_pub, &battery_state_msg, NULL));
}

void timer_100ms_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;

  if (timer == NULL)
    return;

  // Publish IMU
  bno055_read_quaternion(&imu);
  bno055_read_angular_velocity(&imu);
  bno055_read_linear_acceleration(&imu);

  imu_msg.header.frame_id = micro_ros_string_utilities_init("imu");
  imu_msg.orientation = *imu.orientation;
  imu_msg.angular_velocity = *imu.angular_velocity;
  imu_msg.linear_acceleration = *imu.linear_acceleration;

  stamp_header(&imu_msg.header.stamp);
  RCCHECK(rcl_publish(&imu_pup, &imu_msg, NULL));

  // Publish encoders
  encoders_msg.header.frame_id = micro_ros_string_utilities_init("encoders");
  encoders_msg.encoders.front_left = encoder_1.counter;
  encoders_msg.encoders.front_right = encoder_2.counter;
  encoders_msg.encoders.rear_left = encoder_3.counter;
  encoders_msg.encoders.rear_right = encoder_4.counter;

  stamp_header(&encoders_msg.header.stamp);
  RCCHECK(rcl_publish(&encoders_pup, &encoders_msg, NULL));
}

void timer_250ms_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;

  if (timer == NULL)
    return;

  // Publish ultrasonic sensors
  hcsr04_measure(&ultra_1);
  ultra_ranges_msg.front_left.range = ultra_1.range;
  stamp_header(&ultra_ranges_msg.front_left.header.stamp);
  hcsr04_measure(&ultra_2);
  ultra_ranges_msg.front_right.range = ultra_2.range;
  stamp_header(&ultra_ranges_msg.front_right.header.stamp);
  hcsr04_measure(&ultra_3);
  ultra_ranges_msg.rear_left.range = ultra_3.range;
  stamp_header(&ultra_ranges_msg.rear_left.header.stamp);
  hcsr04_measure(&ultra_4);
  ultra_ranges_msg.rear_right.range = ultra_4.range;
  stamp_header(&ultra_ranges_msg.rear_right.header.stamp);
  hcsr04_measure(&ultra_5);
  ultra_ranges_msg.center_left.range = ultra_5.range;
  stamp_header(&ultra_ranges_msg.center_left.header.stamp);
  hcsr04_measure(&ultra_6);
  ultra_ranges_msg.center_right.range = ultra_6.range;
  stamp_header(&ultra_ranges_msg.center_right.header.stamp);
  RCCHECK(rcl_publish(&ultra_ranges_pup, &ultra_ranges_msg, NULL));
}

// Service callbacks
void imu_get_calib_status_callback(const void *imu_get_calib_status_req, void *imu_get_calib_status_res) {
  // Cast messages to expected types
  // mobi_interfaces__srv__GetImuCalibStatus_Request *req =
  //     (mobi_interfaces__srv__GetImuCalibStatus_Request *)imu_get_calib_status_req;
  mobi_interfaces__srv__GetImuCalibStatus_Response *res =
    (mobi_interfaces__srv__GetImuCalibStatus_Response *)imu_get_calib_status_res;

  // Handle request message and set the response message values
  RCUTILS_LOG_DEBUG_NAMED(LOGGER_NAME, "Client requested IMU calibration status.");

  bno055_read_calibration_state(&imu);
  while (!imu.reading_device == BNO055_DEVICE_NONE) {
  }

  mobi_interfaces__srv__GetImuCalibStatus_Response__copy(imu.calib_status, res);
}

void imu_get_calib_data_callback(const void *imu_get_calib_data_req, void *imu_get_calib_data_res) {
  // Cast messages to expected types
  // mobi_interfaces__srv__GetImuCalibData_Request *req =
  //     (mobi_interfaces__srv__GetImuCalibData_Request *)imu_get_calib_data_req;
  mobi_interfaces__srv__GetImuCalibData_Response *res =
    (mobi_interfaces__srv__GetImuCalibData_Response *)imu_get_calib_data_res;

  // Handle request message and set the response message values
  RCUTILS_LOG_DEBUG_NAMED(LOGGER_NAME, "Client requested IMU calibration data.");

  bno055_read_calibration_data(&imu);
  while (!imu.reading_device == BNO055_DEVICE_NONE) {
  }

  mobi_interfaces__srv__GetImuCalibData_Response__copy(imu.calib_data, res);
}

void imu_set_calib_data_callback(const void *imu_set_calib_data_req, void *imu_set_calib_data_res) {
  // Cast messages to expected types
  mobi_interfaces__srv__SetImuCalibData_Request *req =
    (mobi_interfaces__srv__SetImuCalibData_Request *)imu_set_calib_data_req;
  mobi_interfaces__srv__SetImuCalibData_Response *res =
    (mobi_interfaces__srv__SetImuCalibData_Response *)imu_set_calib_data_res;

  // Handle request message and set the response message values
  RCUTILS_LOG_DEBUG_NAMED(LOGGER_NAME, "Client set IMU calibration data.");

  bno055_set_calibration_data(&imu, req);

  res->success = true;
  res->message = micro_ros_string_utilities_init("Set IMU calibration data successfully.");
}

void boot_bootlaoder_callback(const void *boot_bootlaoder_req, void *boot_bootlaoder_res) {
  // Cast messages to expected types
  // std_srvs__srv__Trigger_Request *req = (std_srvs__srv__Trigger_Request *)boot_bootlaoder_req;
  std_srvs__srv__Trigger_Response *res = (std_srvs__srv__Trigger_Response *)boot_bootlaoder_res;

  should_jump_to_bootloader = true;
  res->message = micro_ros_string_utilities_init("Booting into bootloader!");
  res->success = true;
}

void pozyx_set_pwr_callback(const void *pozyx_set_pwr_req, void *pozyx_set_pwr_res) {
  // Cast messages to expected types
  mobi_interfaces__srv__SetPower_Request *req = (mobi_interfaces__srv__SetPower_Request *)pozyx_set_pwr_req;
  mobi_interfaces__srv__SetPower_Response *res = (mobi_interfaces__srv__SetPower_Response *)pozyx_set_pwr_res;

  res->old_state = pwr_manager_get_power_pozyx();
  pwr_manager_set_power_pozyx(req->state);
}

void cmd_vel_callback(const void *msgin) {
  // Cast received message to used type
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // Process message
  RCUTILS_LOG_DEBUG_NAMED(LOGGER_NAME, "CMD_VEL --> x: %f, y: %f, phi: %f", msg->linear.x, msg->linear.y,
                          msg->angular.z);

  int16_t speed = 500;
  int16_t rot_speed = 2000;

  // Flip x and y axis to comply with ROS spec.
  // https://www.ros.org/reps/rep-0103.html#id21
  // x forward, y left, z up
  // but for the motorcontroler the y axis is forward
  int16_t vx = speed * -msg->linear.y;
  int16_t vy = speed * msg->linear.x;
  int16_t vphi = rot_speed * msg->angular.z;

  HAL_StatusTypeDef status = canlib_drive(&can, vx, vy, vphi);
  RCUTILS_LOG_DEBUG_NAMED(LOGGER_NAME, "Status: %d", status);
}
/* USER CODE END Application */
