/*
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 * File: bootloader.h
 * Created Date: Monday, March 4th 2024, 4:36:20 pm
 * Author: Florian Hye
 * Description: This file defines the jump_to_bootloader function.
 *              This allows for rebooting the stm into the bootloader via software.
 * ----------------------------------------------------------------------------------------------------------------------------------------------
 */

#ifndef __BOOTLOADER_H_
#define __BOOTLOADER_H_

#include "stdbool.h"
#include "stm32l452xx.h"

#define BOOT_ADDR 0x1FFF0000 // STM32l452 MCU boot code base address

extern bool should_jump_to_bootloader;

void jump_to_bootloader(void);

#endif /* __BOOTLOADER_H_ */