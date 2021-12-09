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

#define MRAA_RADXA_ROCK3A_GPIO_COUNT 27
#define MRAA_RADXA_ROCK3A_I2C_COUNT  3
#define MRAA_RADXA_ROCK3A_SPI_COUNT  1
#define MRAA_RADXA_ROCK3A_UART_COUNT 6
#define MRAA_RADXA_ROCK3A_PWM_COUNT  7
#define MRAA_RADXA_ROCK3A_ADC_COUNT  1
#define MRAA_RADXA_ROCK3A_PIN_COUNT  40

mraa_board_t *
        mraa_radxa_rock3a();

#ifdef __cplusplus
}
#endif
