/*
 * Author: Ken <ken@radxa.com>
 * Copyright (c) Radxa Limited.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "mraa_internal.h"

#define MRAA_RADXA_ROCK3C_GPIO_COUNT 28
#define MRAA_RADXA_ROCK3C_I2C_COUNT  2
#define MRAA_RADXA_ROCK3C_SPI_COUNT  1
#define MRAA_RADXA_ROCK3C_UART_COUNT 4
#define MRAA_RADXA_ROCK3C_PWM_COUNT  7
#define MRAA_RADXA_ROCK3C_ADC_COUNT  0
#define MRAA_RADXA_ROCK3C_PIN_COUNT  40

mraa_board_t *
        mraa_radxa_rock3c();

#ifdef __cplusplus
}
#endif
