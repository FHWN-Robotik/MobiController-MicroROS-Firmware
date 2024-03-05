##########################################################################################################################
# File automatically-generated by STM32forVSCode
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = micro_ros_firmware


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/adc.c \
Core/Src/bno055_dma.c \
Core/Src/bootloader.c \
Core/Src/can.c \
Core/Src/canlib.c \
Core/Src/dma.c \
Core/Src/encoder.c \
Core/Src/freertos.c \
Core/Src/gpio.c \
Core/Src/i2c.c \
Core/Src/interrupts.c \
Core/Src/main.c \
Core/Src/power_manager.c \
Core/Src/stm32l4xx_hal_msp.c \
Core/Src/stm32l4xx_hal_timebase_tim.c \
Core/Src/stm32l4xx_it.c \
Core/Src/syscalls.c \
Core/Src/system_stm32l4xx.c \
Core/Src/usart.c \
Core/Src/utils.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_can.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.c \
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \
USB_DEVICE/App/usb_device.c \
USB_DEVICE/App/usbd_cdc_if.c \
USB_DEVICE/App/usbd_desc.c \
USB_DEVICE/Target/usbd_conf.c \
micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c \
micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c \
micro_ros_stm32cubemx_utils/extra_sources/microros_time.c \
micro_ros_stm32cubemx_utils/extra_sources/microros_transports/usb_cdc_transport.c


CPP_SOURCES = \


# ASM sources
ASM_SOURCES =  \
startup_stm32l452xx.s



#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
POSTFIX = "
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
GCC_PATH="/home/florian/.config/Code/User/globalStorage/bmd.stm32-for-vscode/@xpack-dev-tools/arm-none-eabi-gcc/12.2.1-1.2.1/.content/bin
ifdef GCC_PATH
CXX = $(GCC_PATH)/$(PREFIX)g++$(POSTFIX)
CC = $(GCC_PATH)/$(PREFIX)gcc$(POSTFIX)
AS = $(GCC_PATH)/$(PREFIX)gcc$(POSTFIX) -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy$(POSTFIX)
SZ = $(GCC_PATH)/$(PREFIX)size$(POSTFIX)
else
CXX = $(PREFIX)g++
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DSTM32L452xx \
-DUSE_FULL_ASSERT \
-DUSE_HAL_DRIVER


# CXX defines
CXX_DEFS =  \
-DSTM32L452xx \
-DUSE_HAL_DRIVER


# AS includes
AS_INCLUDES = \

# C includes
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/CMSIS/Device/ST/STM32L4xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/STM32L4xx_HAL_Driver/Inc \
-IDrivers/STM32L4xx_HAL_Driver/Inc/Legacy \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-IUSB_DEVICE/App \
-IUSB_DEVICE/Target \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/action_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/action_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/action_msgs/srv \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/action_msgs/srv/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/actionlib_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/actionlib_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/builtin_interfaces/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/builtin_interfaces/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/composition_interfaces/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/composition_interfaces/srv \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/composition_interfaces/srv/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/control_msgs/action \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/control_msgs/action/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/control_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/control_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/control_msgs/srv \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/control_msgs/srv/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/diagnostic_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/diagnostic_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/diagnostic_msgs/srv \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/diagnostic_msgs/srv/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/example_interfaces/action \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/example_interfaces/action/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/example_interfaces/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/example_interfaces/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/example_interfaces/srv \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/example_interfaces/srv/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/geometry_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/geometry_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/libyaml_vendor \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/lifecycle_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/lifecycle_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/lifecycle_msgs/srv \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/lifecycle_msgs/srv/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/micro_ros_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/micro_ros_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/micro_ros_utilities \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/mobi_interfaces/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/mobi_interfaces/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/mobi_interfaces/srv \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/mobi_interfaces/srv/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/nav_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/nav_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/nav_msgs/srv \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/nav_msgs/srv/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rcl \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rcl_action \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rcl_interfaces/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rcl_interfaces/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rcl_interfaces/srv \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rcl_interfaces/srv/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rcl_lifecycle \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rcl_logging_interface \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rclc \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rclc_lifecycle \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rclc_parameter \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rcutils \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rcutils/stdatomic_helper/gcc \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rcutils/stdatomic_helper/win32 \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rcutils/testing \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rcutils/types \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rmw \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rmw/events_statuses \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rmw/impl \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rmw/impl/cpp \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rmw_microros \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rmw_microxrcedds_c \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rosgraph_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rosgraph_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rosidl_runtime_c \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rosidl_typesupport_c \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rosidl_typesupport_interface \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rosidl_typesupport_introspection_c \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/rosidl_typesupport_microxrcedds_c \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/sensor_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/sensor_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/sensor_msgs/srv \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/sensor_msgs/srv/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/shape_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/shape_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/statistics_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/statistics_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/std_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/std_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/std_srvs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/std_srvs/srv \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/std_srvs/srv/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/stereo_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/stereo_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/test_msgs \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/test_msgs/action \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/test_msgs/action/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/test_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/test_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/test_msgs/srv \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/test_msgs/srv/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/tf2_msgs/action \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/tf2_msgs/action/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/tf2_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/tf2_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/tf2_msgs/srv \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/tf2_msgs/srv/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/tracetools \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/trajectory_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/trajectory_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/ucdr \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/unique_identifier_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/unique_identifier_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client/core/communication \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client/core/session \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client/core/session/stream \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client/core/type \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client/profile/discovery \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client/profile/multithread \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client/profile/transport/can \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client/profile/transport/custom \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client/profile/transport/ip \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client/profile/transport/ip/tcp \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client/profile/transport/ip/udp \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client/profile/transport/serial \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client/profile/transport/stream_framing \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/uxr/client/util \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/visualization_msgs/msg \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/visualization_msgs/msg/detail \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/visualization_msgs/srv \
-Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include/visualization_msgs/srv/detail



# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CXXFLAGS = $(MCU) $(CXX_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -feliminate-unused-debug-types

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf -ggdb
CXXFLAGS += -g -gdwarf -ggdb
endif

# Add additional flags
CFLAGS += -Wall -fdata-sections -ffunction-sections 
ASFLAGS += -Wall -fdata-sections -ffunction-sections 
CXXFLAGS += 

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32L452RETxP_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = \


# Additional LD Flags from config file
ADDITIONALLDFLAGS = -Wl,--print-memory-usage -specs=nano.specs -u _printf_float micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a 

LDFLAGS = $(MCU) $(ADDITIONALLDFLAGS) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of cpp program objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

# list of C objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
UPPER_CASE_ASM_SOURCES = $(filter %.S,$(ASM_SOURCES))
LOWER_CASE_ASM_SOURCES = $(filter %.s,$(ASM_SOURCES))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(UPPER_CASE_ASM_SOURCES:.S=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(LOWER_CASE_ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.cpp STM32Make.make | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cxx STM32Make.make | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cxx=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.c STM32Make.make | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s STM32Make.make | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.S STM32Make.make | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) STM32Make.make
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# flash
#######################################
flash: $(BUILD_DIR)/$(TARGET).elf
	"/home/florian/.config/Code/User/globalStorage/bmd.stm32-for-vscode/@xpack-dev-tools/openocd/0.12.0-1.1/.content/bin/openocd" -f ./openocd.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

#######################################
# erase
#######################################
erase: $(BUILD_DIR)/$(TARGET).elf
	"/home/florian/.config/Code/User/globalStorage/bmd.stm32-for-vscode/@xpack-dev-tools/openocd/0.12.0-1.1/.content/bin/openocd" -f ./openocd.cfg -c "init; reset halt; stm32l4x mass_erase 0; exit"

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# custom makefile rules
#######################################


	
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***