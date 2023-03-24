/*
 * Author: Sanry <zhangxinyu@radxa.com>
 * Copyright (c) Radxa Limited.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "mraa_internal.h"

#define MRAA_RADXA_ROCK_5B_GPIO_COUNT 26
#define MRAA_RADXA_ROCK_5B_I2C_COUNT  5
#define MRAA_RADXA_ROCK_5B_SPI_COUNT  2
#define MRAA_RADXA_ROCK_5B_UART_COUNT 4
#define MRAA_RADXA_ROCK_5B_PWM_COUNT  10
#define MRAA_RADXA_ROCK_5B_ADC_COUNT  1
#define MRAA_RADXA_ROCK_5B_PIN_COUNT  40

mraa_board_t *
        mraa_radxa_rock_5b();

#ifdef __cplusplus
}
#endif
