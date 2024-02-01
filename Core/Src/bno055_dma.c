#include "bno055_dma.h"

#include <rcl/rcl.h>

#include "i2c.h"
#include "stdio.h"

void check_status(BNO055_t *imu, HAL_StatusTypeDef status) {
  if (status == HAL_OK) {
    return;
  }

  if (status == HAL_ERROR) {
    printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
  } else if (status == HAL_TIMEOUT) {
    printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
  } else if (status == HAL_BUSY) {
    printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
  } else {
    printf("Unknown status data %d", status);
  }

  uint32_t error = HAL_I2C_GetError(imu->i2c_handle);
  if (error == HAL_I2C_ERROR_NONE) {
    return;
  } else if (error == HAL_I2C_ERROR_BERR) {
    printf("HAL_I2C_ERROR_BERR\r\n");
  } else if (error == HAL_I2C_ERROR_ARLO) {
    printf("HAL_I2C_ERROR_ARLO\r\n");
  } else if (error == HAL_I2C_ERROR_AF) {
    printf("HAL_I2C_ERROR_AF\r\n");
  } else if (error == HAL_I2C_ERROR_OVR) {
    printf("HAL_I2C_ERROR_OVR\r\n");
  } else if (error == HAL_I2C_ERROR_DMA) {
    printf("HAL_I2C_ERROR_DMA\r\n");
  } else if (error == HAL_I2C_ERROR_TIMEOUT) {
    printf("HAL_I2C_ERROR_TIMEOUT\r\n");
  }

  HAL_I2C_StateTypeDef state = HAL_I2C_GetState(imu->i2c_handle);
  if (state == HAL_I2C_STATE_RESET) {
    printf("HAL_I2C_STATE_RESET\r\n");
  } else if (state == HAL_I2C_STATE_READY) {
    printf("HAL_I2C_STATE_RESET\r\n");
  } else if (state == HAL_I2C_STATE_BUSY) {
    printf("HAL_I2C_STATE_BUSY\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_TX) {
    printf("HAL_I2C_STATE_BUSY_TX\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_RX) {
    printf("HAL_I2C_STATE_BUSY_RX\r\n");
  } else if (state == HAL_I2C_STATE_LISTEN) {
    printf("HAL_I2C_STATE_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_TX_LISTEN) {
    printf("HAL_I2C_STATE_BUSY_TX_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_RX_LISTEN) {
    printf("HAL_I2C_STATE_BUSY_RX_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_ABORT) {
    printf("HAL_I2C_STATE_ABORT\r\n");
  } else if (state == HAL_I2C_STATE_TIMEOUT) {
    printf("HAL_I2C_STATE_TIMEOUT\r\n");
  } else if (state == HAL_I2C_STATE_ERROR) {
    printf("HAL_I2C_STATE_ERROR\r\n");
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

  bno055_write_DMA(imu, BNO055_SYS_TRIGGER, 0x20);  // reset imu

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

  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_DMA(imu->i2c_handle, imu->device_address << 1, imu->tx_buf, sizeof(imu->tx_buf));
  check_status(imu, status);
}

void bno055_read_DMA(BNO055_t *imu, uint8_t reg_addr, uint8_t len) {
  imu->tx_buf[0] = reg_addr;
  imu->tx_buf[1] = 0;

  bno055_write_DMA(imu, reg_addr, 0);

  while (imu->i2c_handle->State != HAL_I2C_STATE_READY) {
  }

  HAL_StatusTypeDef status = HAL_I2C_Master_Receive_DMA(imu->i2c_handle, imu->device_address << 1, (uint8_t *)imu->rx_buf, len);
  check_status(imu, status);
}

void bno055_read_DMA_complete(BNO055_t *imu) {
  switch (imu->reading_sensor) {
    case BNO055_SENSOR_TEMP:
      rcl_time_point_value_t now;
      // rcutils_steady_time_now(&now);

      sensor_msgs__msg__Temperature temp = {
          .header = {
              .frame_id = "imu",
              // .stamp = now,
          },
          .temperature = imu->rx_buf[0],
          .variance = 0};

      imu->temperature = temp;
      break;

    default:
      break;
  }

  imu->reading_sensor = BNO055_SENSOR_NONE;
}

// --------------------------------------------------------------------------------
// Wrapper functions
// --------------------------------------------------------------------------------

void bno055_set_page(BNO055_t *imu, uint8_t page) {
  bno055_write_DMA(imu, BNO055_PAGE_ID, page);
}

void bno055_set_operation_mode(BNO055_t *imu, BNO055_OPMODE_t opmode) {
  bno055_write_DMA(imu, BNO055_OPR_MODE, opmode);
}

void bno055_read_temp(BNO055_t *imu) {
  bno055_set_page(imu, 0);
  bno055_read_DMA(imu, BNO055_TEMP, 1);
  imu->reading_sensor = BNO055_SENSOR_TEMP;
}