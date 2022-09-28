/*
 * Copyright (c) 2022 Radxa Computer Co.,Ltd
 * Author: Nascs <nascs@vamrs.com>
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "mraa_internal.h"

#define MRAA_RADXA_CM3_IO_GPIO_COUNT 27
#define MRAA_RADXA_CM3_IO_I2C_COUNT  2
#define MRAA_RADXA_CM3_IO_SPI_COUNT  2
#define MRAA_RADXA_CM3_IO_UART_COUNT 1
#define MRAA_RADXA_CM3_IO_PWM_COUNT  5
#define MRAA_RADXA_CM3_IO_AIO_COUNT  1
#define MRAA_RADXA_CM3_IO_PIN_COUNT  40

mraa_board_t *
        mraa_radxa_cm3_io();

#ifdef __cplusplus
}
#endif
