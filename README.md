# MobiController MicroROS Firmware

This repository contains the firmware for the "Mobi" mobile robot.
It utilizes [microROS](https://micro.ros.org).

The used microcontroller is the `STM32L452RE` on the `NUCLEO-L452RE-P` development board.

## Build/Flash Requirements

The following things are required to build this firmware:

- Probably a non-Windows OS. (The build was tested on Linux and most likely will not work on Windows. It could work in WSL)

For building:

- [Docker](https://www.docker.com)
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
- gcc for ARM (this is included in the "stm32-for-vscode" extension)

For debugging:

- OpenOCD (this is included in the "stm32-for-vscode" extension)

For flashing:

- OpenOCD (this is included in the "stm32-for-vscode" extension)
- [dfu-util](https://dfu-util.sourceforge.net)

## Flashing the firmware

It is possible to flash the firmware either via the ST-Link v2 debug port or the exposed USB-Port via DFU.

### Flashing via DFU-Mode

#### Using a downloaded binary

1. Download the binary from the [release page](https://github.com/FHWN-Robotik/MobiController-MicroROS-Firmware/releases/latest).

2. Reset the STM32 into DFU-Mode.
  
    This can be done in two ways:

    - Via the ROS2 service `/boot_bootloader`.
        The following command reboots the STM32 into DFU-Mode.

        ```bash
        ros2 service call /boot_bootloader std_srvs/srv/Trigger
        ```

    - Or one can hold the **USER_BTN** while pressing the **REST** button on the controller board.

3. Use dfu-util to flash the binary.

    ```bash
    dfu-util -a 0 -w -D ./micro_ros_firmware.bin -s 0x08000000:leave
    ```

#### When building from source

1. Build the firmware following the [How to Build](#how-to-build) section.

2. Follow steps 2 and 3 from above.

### Flash via ST-Link (debug connection)

#### Using a downloaded binary

1. Download the binary from the [release page](https://github.com/FHWN-Robotik/MobiController-MicroROS-Firmware/releases/latest).

2. Download the [openocd.cfg](https://github.com/FHWN-Robotik/MobiController-MicroROS-Firmware/blob/main/openocd.cfg) file from this repository.

3. Copy the two files into the same folder (e.g. ~/Downloads).

4. To flash the binary to the STM32, run the following command from within the folder where the files are located:

    ```bash
    openocd -f ./openocd.cfg -c "program ./micro_ros_firmware.bin verify reset exit"
    ```

#### When building from source

1. Build the firmware following the [How to build](#how-to-build) section.

2. If you're using VS Code with the "stm32-for-vscode" extension, you can run the command `STM32: Build and flash to an STM32 platform` to build and flash the firmware.

    Alteretivly, you can run following cli command to only flash the firmware:

    ```bash
    make -j16 -f STM32Make.make flash
    ```

## How to Build

1. Clone this repository

    ```bash
    git clone https://github.com/FHWN-Robotik/MobiController-MicroROS-Firmware.git
    ```

2. Build the MicroROS static library

    > This steps a taken from the [micro-ROS/micro_ros_stm32cubemx_utils](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/humble) repository.

    Building the static library is done using Docker.

    ```bash
    docker pull microros/micro_ros_static_library_builder:humble
    docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:humble
    ```

3. Build the firmware

    If you're using VS Code with the "stm32-for-vscode" extension, you can run the command `STM32: Build STM32 project`

    Alteretivly, you can run following cli command:

    ```bash
    make -j16 -f STM32Make.make build
    ```

4. Output location

    The compiled binary can now be found at `build/micro_ros_firmware.bin`.
