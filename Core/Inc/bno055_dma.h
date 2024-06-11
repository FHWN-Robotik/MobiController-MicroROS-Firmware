/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: bno055_dma.h
 * Created Date: Monday, February 26th 2024, 2:37:05 pm
 * Author: Florian Hye
 * Description: This file defines the driver for the BNO055 IMU using I2C via DMA.
 *              The basis of this driver is: https://github.com/ivyknob/bno055_stm32
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#ifndef __BNO055_DMA_H__
#define __BNO055_DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "geometry_msgs/msg/quaternion.h"
#include "geometry_msgs/msg/vector3.h"
#include "i2c.h"
#include "mobi_interfaces/srv/get_calib_status.h"
#include "mobi_interfaces/srv/get_imu_calib_data.h"
#include "mobi_interfaces/srv/set_imu_calib_data.h"
#include "sensor_msgs/msg/temperature.h"

#define START_BYTE 0xAA
#define RESPONSE_BYTE 0xBB
#define ERROR_BYTE 0xEE

#define BNO055_I2C_ADDR_HI 0x29
#define BNO055_I2C_ADDR_LO 0x28
// #define BNO055_I2C_ADDR BNO055_I2C_ADDR_LO

#define BNO055_READ_TIMEOUT 100
#define BNO055_WRITE_TIMEOUT 10

#define ERROR_WRITE_SUCCESS 0x01     // Everything working as expected
#define ERROR_WRITE_FAIL 0x03        // Check connection, protocol settings and operation more of BNO055
#define ERROR_REGMAP_INV_ADDR 0x04   // Invalid register address
#define ERROR_REGMAP_WRITE_DIS 0x05  // Register is read-only
#define ERROR_WRONG_START_BYTE 0x06  // Check if the first byte
#define ERROR_BUS_OVERRUN_ERR 0x07   // Resend the command, BNO055 was not able to clear the receive buffer
#define ERROR_MAX_LEN_ERR 0x08       // Split the command, max fire size can be up to 128 bytes
#define ERROR_MIN_LEN_ERR 0x09       // Min length of data is less than 1
#define ERROR_RECV_CHAR_TIMEOUT 0x0A // Decrease the waiting time between sending of two bytes of one frame

#define REG_WRITE 0x00
#define REG_READ 0x01

// Page 0
#define BNO055_ID (0xA0)
#define BNO055_CHIP_ID 0x00       // value: 0xA0
#define BNO055_ACC_ID 0x01        // value: 0xFB
#define BNO055_MAG_ID 0x02        // value: 0x32
#define BNO055_GYRO_ID 0x03       // value: 0x0F
#define BNO055_SW_REV_ID_LSB 0x04 // value: 0x08
#define BNO055_SW_REV_ID_MSB 0x05 // value: 0x03
#define BNO055_BL_REV_ID 0x06     // N/A
#define BNO055_PAGE_ID 0x07
#define BNO055_ACC_DATA_X_LSB 0x08
#define BNO055_ACC_DATA_X_MSB 0x09
#define BNO055_ACC_DATA_Y_LSB 0x0A
#define BNO055_ACC_DATA_Y_MSB 0x0B
#define BNO055_ACC_DATA_Z_LSB 0x0C
#define BNO055_ACC_DATA_Z_MSB 0x0D
#define BNO055_MAG_DATA_X_LSB 0x0E
#define BNO055_MAG_DATA_X_MSB 0x0F
#define BNO055_MAG_DATA_Y_LSB 0x10
#define BNO055_MAG_DATA_Y_MSB 0x11
#define BNO055_MAG_DATA_Z_LSB 0x12
#define BNO055_MAG_DATA_Z_MSB 0x13
#define BNO055_GYR_DATA_X_LSB 0x14
#define BNO055_GYR_DATA_X_MSB 0x15
#define BNO055_GYR_DATA_Y_LSB 0x16
#define BNO055_GYR_DATA_Y_MSB 0x17
#define BNO055_GYR_DATA_Z_LSB 0x18
#define BNO055_GYR_DATA_Z_MSB 0x19
#define BNO055_EUL_HEADING_LSB 0x1A
#define BNO055_EUL_HEADING_MSB 0x1B
#define BNO055_EUL_ROLL_LSB 0x1C
#define BNO055_EUL_ROLL_MSB 0x1D
#define BNO055_EUL_PITCH_LSB 0x1E
#define BNO055_EUL_PITCH_MSB 0x1F
#define BNO055_QUA_DATA_W_LSB 0x20
#define BNO055_QUA_DATA_W_MSB 0x21
#define BNO055_QUA_DATA_X_LSB 0x22
#define BNO055_QUA_DATA_X_MSB 0x23
#define BNO055_QUA_DATA_Y_LSB 0x24
#define BNO055_QUA_DATA_Y_MSB 0x25
#define BNO055_QUA_DATA_Z_LSB 0x26
#define BNO055_QUA_DATA_Z_MSB 0x27
#define BNO055_LIA_DATA_X_LSB 0x28
#define BNO055_LIA_DATA_X_MSB 0x29
#define BNO055_LIA_DATA_Y_LSB 0x2A
#define BNO055_LIA_DATA_Y_MSB 0x2B
#define BNO055_LIA_DATA_Z_LSB 0x2C
#define BNO055_LIA_DATA_Z_MSB 0x2D
#define BNO055_GRV_DATA_X_LSB 0x2E
#define BNO055_GRV_DATA_X_MSB 0x2F
#define BNO055_GRV_DATA_Y_LSB 0x30
#define BNO055_GRV_DATA_Y_MSB 0x31
#define BNO055_GRV_DATA_Z_LSB 0x32
#define BNO055_GRV_DATA_Z_MSB 0x33
#define BNO055_TEMP 0x34
#define BNO055_CALIB_STAT 0x35
#define BNO055_ST_RESULT 0x36
#define BNO055_INT_STATUS 0x37
#define BNO055_SYS_CLK_STATUS 0x38
#define BNO055_SYS_STATUS 0x39
#define BNO055_SYS_ERR 0x3A
#define BNO055_UNIT_SEL 0x3B
#define BNO055_OPR_MODE 0x3D
#define BNO055_PWR_MODE 0x3E
#define BNO055_SYS_TRIGGER 0x3F
#define BNO055_TEMP_SOURCE 0x40
#define BNO055_AXIS_MAP_CONFIG 0x41
#define BNO055_AXIS_MAP_SIGN 0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB 0x67
#define BNO055_ACC_RADIUS_MSB 0x68
#define BNO055_MAG_RADIUS_LSB 0x69
#define BNO055_MAG_RADIUS_MSB 0x6A
//
// BNO055 Page 1
#define BNO055_PAGE_ID 0x07
#define BNO055_ACC_CONFIG 0x08
#define BNO055_MAG_CONFIG 0x09
#define BNO055_GYRO_CONFIG_0 0x0A
#define BNO055_GYRO_CONFIG_1 0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK 0x0F
#define BNO055_INT_EN 0x10
#define BNO055_ACC_AM_THRES 0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION 0x13
#define BNO055_ACC_HG_THRESH 0x14
#define BNO055_ACC_NM_THRESH 0x15
#define BNO055_ACC_NM_SET 0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET 0x18
#define BNO055_GYR_DUR_X 0x19
#define BNO055_GYR_HR_Y_SET 0x1A
#define BNO055_GYR_DUR_Y 0x1B
#define BNO055_GYR_HR_Z_SET 0x1C
#define BNO055_GYR_DUR_Z 0x1D
#define BNO055_GYR_AM_THRESH 0x1E
#define BNO055_GYR_AM_SET 0x1F

// Sclaes
#define BNO055_ACCEL_SCALE 100
#define BNO055_TEMP_SCALE 1
#define BNO055_ANGULAR_RATE_SCALE 16
#define BNO055_EULER_SCALE 16
#define BNO055_MAG_SCALE 16
#define BNO055_QUAT_SCALE (1 << 14) // 2^14

typedef enum { // BNO055 operation modes
  BNO055_OPERATION_MODE_CONFIG = 0x00,
  // Sensor Mode
  BNO055_OPERATION_MODE_ACCONLY,
  BNO055_OPERATION_MODE_MAGONLY,
  BNO055_OPERATION_MODE_GYRONLY,
  BNO055_OPERATION_MODE_ACCMAG,
  BNO055_OPERATION_MODE_ACCGYRO,
  BNO055_OPERATION_MODE_MAGGYRO,
  BNO055_OPERATION_MODE_AMG, // 0x07
                             // Fusion Mode
  BNO055_OPERATION_MODE_IMU,
  BNO055_OPERATION_MODE_COMPASS,
  BNO055_OPERATION_MODE_M4G,
  BNO055_OPERATION_MODE_NDOF_FMC_OFF,
  BNO055_OPERATION_MODE_NDOF // 0x0C
} bno055_opmode_t;

typedef struct {
  double w;
  double x;
  double y;
  double z;
} bno055_vector_t;

typedef enum {
  BNO055_DEVICE_NONE = 0x0,
  BNO055_DEVICE_ACCELEROMETER = 0x08, // Default: m/s²
  BNO055_DEVICE_MAGNETOMETER = 0x0E,  // Default: uT
  BNO055_DEVICE_GYROSCOPE = 0x14,     // Default: rad/s
  BNO055_DEVICE_EULER = 0x1A,         // Default: degrees
  BNO055_DEVICE_QUATERNION = 0x20,    // No units
  BNO055_DEVICE_LINEARACCEL = 0x28,   // Default: m/s²
  BNO055_DEVICE_GRAVITY = 0x2E,       // Default: m/s²
  BNO055_DEVICE_TEMP = 0x34,          // Default: °C
} bno055_devices_t;

typedef struct BNO055_s {
  /* I2C */
  I2C_HandleTypeDef *i2c_handle;
  uint16_t device_address;

  /* DMA */
  uint8_t tx_buf[2];
  volatile uint8_t rx_buf[22]; // Note: 22 bytes for calib data. Biggest data for sensors is 8 bytes for quaternion.
  uint8_t reading_device;

  // Data
  geometry_msgs__msg__Quaternion *orientation;
  geometry_msgs__msg__Vector3 *angular_velocity;
  geometry_msgs__msg__Vector3 *linear_acceleration;

  uint64_t test;
  sensor_msgs__msg__Temperature *temperature;

  mobi_interfaces__srv__GetCalibStatus_Response *calib_status;
  mobi_interfaces__srv__GetImuCalibData_Response *calib_data;

} BNO055_t;

// Low level functions
void bno055_init(BNO055_t *imu, I2C_HandleTypeDef *hi2c_device, uint16_t device_address);
void bno055_write_DMA(BNO055_t *imu, uint8_t reg_addr, uint8_t data);
void bno055_read_DMA(BNO055_t *imu, uint8_t reg_addr, uint8_t len);

void bno055_read_DMA_complete(BNO055_t *imu);

// Wrapper functions
void bno055_set_page(BNO055_t *imu, uint8_t page);
void bno055_set_operation_mode(BNO055_t *imu, bno055_opmode_t opmode);

void bno055_read_temp(BNO055_t *imu);
void bno055_read_quaternion(BNO055_t *imu);
void bno055_read_angular_velocity(BNO055_t *imu);
void bno055_read_linear_acceleration(BNO055_t *imu);

void bno055_read_calibration_state(BNO055_t *imu);
void bno055_read_calibration_data(BNO055_t *imu);
void bno055_set_calibration_data(BNO055_t *imu, mobi_interfaces__srv__SetImuCalibData_Request *calib_data);

#ifdef __cplusplus
}
#endif
#endif /*__BNO055_DMA_H__ */
