/*
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "mraa_internal.h"

#define MRAA_RADXA_ROCK_5A_GPIO_COUNT 26
#define MRAA_RADXA_ROCK_5A_I2C_COUNT  5
#define MRAA_RADXA_ROCK_5A_SPI_COUNT  2
#define MRAA_RADXA_ROCK_5A_UART_COUNT 4
#define MRAA_RADXA_ROCK_5A_PWM_COUNT  10
#define MRAA_RADXA_ROCK_5A_ADC_COUNT  1
#define MRAA_RADXA_ROCK_5A_PIN_COUNT  40
#define MRAA_RADXA_ROCK_5A_LED_COUNT  1

mraa_board_t *
        mraa_radxa_rock_5a();

#ifdef __cplusplus
}
#endif
