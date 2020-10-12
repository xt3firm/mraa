/*
 * Author: zzl <zzl@radxa.com>
 * Copyright (c) Radxa Limited.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "mraa_internal.h"

#define MRAA_ROCKPIN10_GPIO_COUNT 27
#define MRAA_ROCKPIN10_I2C_COUNT  3
#define MRAA_ROCKPIN10_SPI_COUNT  1
#define MRAA_ROCKPIN10_UART_COUNT 2
#define MRAA_ROCKPIN10_PWM_COUNT  2
#define MRAA_ROCKPIN10_AIO_COUNT  1
#define MRAA_ROCKPIN10_PIN_COUNT  40

mraa_board_t *
        mraa_rockpin10();

#ifdef __cplusplus
}
#endif