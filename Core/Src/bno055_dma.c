/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: bno055_dma.c
 * Created Date: Monday, February 26th 2024, 2:37:05 pm
 * Author: Florian Hye
 * Description: This file implements helpers for the BNO055 IMU using I2C via DMA.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#include "bno055_dma.h"

#include <micro_ros_utilities/string_utilities.h>
#include <rcl/rcl.h>

#include "i2c.h"
#include "stdio.h"
#include "utils.h"

void transform_vec(uint8_t buf[8], bno055_vector_t *vec, double scale, bool is_quaternion) {
  if (is_quaternion) {
    vec->w = (int16_t)((buf[1] << 8) | buf[0]) / scale;
    vec->x = (int16_t)((buf[3] << 8) | buf[2]) / scale;
    vec->y = (int16_t)((buf[5] << 8) | buf[4]) / scale;
    vec->z = (int16_t)((buf[7] << 8) | buf[6]) / scale;
  } else {
    vec->x = (int16_t)((buf[1] << 8) | buf[0]) / scale;
    vec->y = (int16_t)((buf[3] << 8) | buf[2]) / scale;
    vec->z = (int16_t)((buf[5] << 8) | buf[4]) / scale;
  }
}

// --------------------------------------------------------------------------------
// Low level functions
// --------------------------------------------------------------------------------

void bno055_init(BNO055_t *imu, I2C_HandleTypeDef *hi2c_device, uint16_t device_address) {
  imu->i2c_handle = hi2c_device;
  imu->device_address = device_address;

  HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(imu->i2c_handle, imu->device_address << 1, 5, 1000);
  if (status != HAL_OK) {
    printf("Error setting up IMU!\n");
  }

  imu->temperature = sensor_msgs__msg__Temperature__create();
  imu->temperature->header.frame_id = micro_ros_string_utilities_init("imu");
  imu->temperature->variance = 0.0;

  imu->orientation = geometry_msgs__msg__Quaternion__create();
  imu->linear_acceleration = geometry_msgs__msg__Vector3__create();
  imu->angular_velocity = geometry_msgs__msg__Vector3__create();

  imu->test = 0xffff;

  imu->calib_status = mobi_interfaces__srv__GetCalibStatus_Response__create();
  imu->calib_data = mobi_interfaces__srv__GetImuCalibData_Response__create();

  bno055_write_DMA(imu, BNO055_SYS_TRIGGER, 0x20); // reset imu

  bno055_set_page(imu, 0);
  bno055_write_DMA(imu, BNO055_SYS_TRIGGER, 0x0);

  // Set operating mode to config
  bno055_set_operation_mode(imu, BNO055_OPERATION_MODE_CONFIG);

  // Set op mode NDOF
  bno055_set_operation_mode(imu, BNO055_OPERATION_MODE_NDOF);
}

void bno055_write_DMA(BNO055_t *imu, uint8_t reg_addr, uint8_t data) {
  imu->tx_buf[0] = reg_addr;
  imu->tx_buf[1] = data;

  while (imu->i2c_handle->State != HAL_I2C_STATE_READY) {
  }

  HAL_StatusTypeDef status =
    HAL_I2C_Master_Transmit_DMA(imu->i2c_handle, imu->device_address << 1, imu->tx_buf, sizeof(imu->tx_buf));
  i2c_check_status(imu->i2c_handle, status);
}

void bno055_read_DMA(BNO055_t *imu, uint8_t reg_addr, uint8_t len) {
  imu->tx_buf[0] = reg_addr;
  imu->tx_buf[1] = 0;

  bno055_write_DMA(imu, reg_addr, 0);

  while (imu->i2c_handle->State != HAL_I2C_STATE_READY) {
  }

  HAL_StatusTypeDef status =
    HAL_I2C_Master_Receive_DMA(imu->i2c_handle, imu->device_address << 1, (uint8_t *)imu->rx_buf, len);
  i2c_check_status(imu->i2c_handle, status);
}

void bno055_read_DMA_complete(BNO055_t *imu) {
  bno055_vector_t vec = {.w = 0, .x = 0, .y = 0, .z = 0};

  switch (imu->reading_device) {
  case BNO055_DEVICE_TEMP:
    imu->temperature->temperature = imu->rx_buf[0];
    break;

  case BNO055_DEVICE_QUATERNION:
    transform_vec(imu->rx_buf, &vec, BNO055_QUAT_SCALE, true);

    imu->orientation->w = vec.w;
    imu->orientation->x = vec.x;
    imu->orientation->y = vec.y;
    imu->orientation->z = vec.z;
    break;

  case BNO055_DEVICE_LINEARACCEL:
    transform_vec(imu->rx_buf, &vec, BNO055_ACCEL_SCALE, false);

    imu->linear_acceleration->x = vec.x;
    imu->linear_acceleration->y = vec.y;
    imu->linear_acceleration->z = vec.z;
    break;

  case BNO055_DEVICE_GYROSCOPE:
    transform_vec(imu->rx_buf, &vec, BNO055_ANGULAR_RATE_SCALE, false);

    imu->angular_velocity->x = vec.x;
    imu->angular_velocity->y = vec.y;
    imu->angular_velocity->z = vec.z;
    break;

  case BNO055_CALIB_STAT:
    imu->calib_status->system = (imu->rx_buf[0] >> 6) & 0x03;
    imu->calib_status->gyro = (imu->rx_buf[0] >> 4) & 0x03;
    imu->calib_status->accel = (imu->rx_buf[0] >> 2) & 0x03;
    imu->calib_status->mag = imu->rx_buf[0] & 0x03;
    break;

  case BNO055_ACC_OFFSET_X_LSB: // Calib Data
    // Assumes little endian processor

    bno055_vector_t vec = {.w = 0, .x = 0, .y = 0, .z = 0};

    transform_vec(imu->rx_buf, &vec, 1, false);
    imu->calib_data->offset_accelerometer_x = vec.x;
    imu->calib_data->offset_accelerometer_y = vec.y;
    imu->calib_data->offset_accelerometer_z = vec.z;

    transform_vec(imu->rx_buf + 6, &vec, 1, false);
    imu->calib_data->offset_magnetometer_x = vec.x;
    imu->calib_data->offset_magnetometer_y = vec.y;
    imu->calib_data->offset_magnetometer_z = vec.z;

    transform_vec(imu->rx_buf + 12, &vec, 1, false);
    imu->calib_data->offset_gyroscope_x = vec.x;
    imu->calib_data->offset_gyroscope_y = vec.y;
    imu->calib_data->offset_gyroscope_z = vec.z;

    imu->calib_data->radius_accelerometer = (int16_t)((imu->rx_buf[19] << 8) | imu->rx_buf[18]);

    imu->calib_data->radius_magnetometer = (int16_t)((imu->rx_buf[21] << 8) | imu->rx_buf[20]);

    // Set the IMU back into NDOF mode.
    bno055_set_operation_mode(imu, BNO055_OPERATION_MODE_NDOF);
    break;

  default:
    break;
  }

  imu->reading_device = BNO055_DEVICE_NONE;
}

// --------------------------------------------------------------------------------
// Wrapper functions
// --------------------------------------------------------------------------------

void bno055_set_page(BNO055_t *imu, uint8_t page) { bno055_write_DMA(imu, BNO055_PAGE_ID, page); }

void bno055_set_operation_mode(BNO055_t *imu, bno055_opmode_t opmode) {
  bno055_write_DMA(imu, BNO055_OPR_MODE, opmode);
}

void bno055_read_temp(BNO055_t *imu) {
  bno055_set_page(imu, 0);
  bno055_read_DMA(imu, BNO055_TEMP, 1);
  imu->reading_device = BNO055_DEVICE_TEMP;
}

void bno055_read_quaternion(BNO055_t *imu) {
  bno055_set_page(imu, 0);
  bno055_read_DMA(imu, BNO055_DEVICE_QUATERNION, 8);
  imu->reading_device = BNO055_DEVICE_QUATERNION;
}

void bno055_read_angular_velocity(BNO055_t *imu) {
  bno055_set_page(imu, 0);
  bno055_read_DMA(imu, BNO055_DEVICE_GYROSCOPE, 6);
  imu->reading_device = BNO055_DEVICE_GYROSCOPE;
}

void bno055_read_linear_acceleration(BNO055_t *imu) {
  bno055_set_page(imu, 0);
  bno055_read_DMA(imu, BNO055_DEVICE_LINEARACCEL, 6);
  imu->reading_device = BNO055_DEVICE_LINEARACCEL;
}

void bno055_read_calibration_state(BNO055_t *imu) {
  bno055_set_page(imu, 0);

  bno055_read_DMA(imu, BNO055_CALIB_STAT, 1);
  imu->reading_device = BNO055_CALIB_STAT;
}

void bno055_read_calibration_data(BNO055_t *imu) {
  bno055_set_operation_mode(imu, BNO055_OPERATION_MODE_CONFIG);

  bno055_set_page(imu, 0);
  bno055_read_DMA(imu, BNO055_ACC_OFFSET_X_LSB, 22);

  imu->reading_device = BNO055_ACC_OFFSET_X_LSB;

  // Setting op mode back to NDOF after receiving the data.
}

void bno055_set_calibration_data(BNO055_t *imu, mobi_interfaces__srv__SetImuCalibData_Request *calib_data) {
  uint8_t buffer[22];
  bno055_set_operation_mode(imu, BNO055_OPERATION_MODE_CONFIG);
  bno055_set_page(imu, 0);

  // Assumes litle endian processor
  buffer[0] = (uint8_t)(calib_data->offset_accelerometer_x & 0xFF);
  buffer[1] = (uint8_t)((calib_data->offset_accelerometer_x >> 8) & 0xFF);
  buffer[2] = (uint8_t)(calib_data->offset_accelerometer_y & 0xFF);
  buffer[3] = (uint8_t)((calib_data->offset_accelerometer_y >> 8) & 0xFF);
  buffer[4] = (uint8_t)(calib_data->offset_accelerometer_z & 0xFF);
  buffer[5] = (uint8_t)((calib_data->offset_accelerometer_z >> 8) & 0xFF);
  buffer[6] = (uint8_t)(calib_data->offset_magnetometer_x & 0xFF);
  buffer[7] = (uint8_t)((calib_data->offset_magnetometer_x >> 8) & 0xFF);
  buffer[8] = (uint8_t)(calib_data->offset_magnetometer_y & 0xFF);
  buffer[9] = (uint8_t)((calib_data->offset_magnetometer_y >> 8) & 0xFF);
  buffer[10] = (uint8_t)(calib_data->offset_magnetometer_z & 0xFF);
  buffer[11] = (uint8_t)((calib_data->offset_magnetometer_z >> 8) & 0xFF);
  buffer[12] = (uint8_t)(calib_data->offset_gyroscope_x & 0xFF);
  buffer[13] = (uint8_t)((calib_data->offset_gyroscope_x >> 8) & 0xFF);
  buffer[14] = (uint8_t)(calib_data->offset_gyroscope_y & 0xFF);
  buffer[15] = (uint8_t)((calib_data->offset_gyroscope_y >> 8) & 0xFF);
  buffer[16] = (uint8_t)(calib_data->offset_gyroscope_z & 0xFF);
  buffer[17] = (uint8_t)((calib_data->offset_gyroscope_z >> 8) & 0xFF);
  buffer[18] = (uint8_t)(calib_data->radius_accelerometer & 0xFF);
  buffer[19] = (uint8_t)((calib_data->radius_accelerometer >> 8) & 0xFF);
  buffer[20] = (uint8_t)(calib_data->radius_magnetometer & 0xFF);
  buffer[21] = (uint8_t)((calib_data->radius_magnetometer >> 8) & 0xFF);

  for (uint8_t i = 0; i < 22; i++) {
    bno055_write_DMA(imu, BNO055_ACC_OFFSET_X_LSB + i, buffer[i]);
    while (!imu->reading_device == BNO055_DEVICE_NONE) {
    }
  }

  bno055_set_operation_mode(imu, BNO055_OPERATION_MODE_NDOF);
}