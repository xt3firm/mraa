/*
 * Copyright (c) 2022 Radxa Computer Co.,Ltd
 * Author: Nascs <nascs@vamrs.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include <mraa/common.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "arm/radxa_cm3_io.h"
#include "common.h"

#define DT_BASE "/proc/device-tree"

#define PLATFORM_NAME_RADXA_CM3_IO "Radxa CM3 IO"
#define MAX_SIZE 64

const char* radxa_cm3_io_serialdev[MRAA_RADXA_CM3_IO_UART_COUNT] = { "/dev/ttyS1" };

void
mraa_radxa_cm3_io_pininfo(mraa_board_t* board, int index, int sysfs_pin, mraa_pincapabilities_t pincapabilities_t, char* fmt, ...)
{
    va_list arg_ptr;
    if (index > board->phy_pin_count)
        return;

    mraa_pininfo_t* pininfo = &board->pins[index];
    va_start(arg_ptr, fmt);
    vsnprintf(pininfo->name, MRAA_PIN_NAME_SIZE, fmt, arg_ptr);

    if( pincapabilities_t.gpio == 1 ) {
        va_arg(arg_ptr, int);
        pininfo->gpio.gpio_chip = va_arg(arg_ptr, int);
        pininfo->gpio.gpio_line = va_arg(arg_ptr, int);
    }

    pininfo->capabilities = pincapabilities_t;

    va_end(arg_ptr);
    pininfo->gpio.pinmap = sysfs_pin;
    pininfo->gpio.mux_total = 0;
}

mraa_board_t*
mraa_radxa_cm3_io()
{
    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof(mraa_board_t));
    if (b == NULL) {
        return NULL;
    }

    b->adv_func = (mraa_adv_func_t*) calloc(1, sizeof(mraa_adv_func_t));
    if (b->adv_func == NULL) {
        free(b);
        return NULL;
    }

    // pin mux for buses are setup by default by kernel so tell mraa to ignore them
    b->no_bus_mux = 1;
    b->phy_pin_count = MRAA_RADXA_CM3_IO_PIN_COUNT + 1;

    if (mraa_file_exist(DT_BASE "/model")) {
        // We are on a modern kernel, great!!!!
        if (mraa_file_contains(DT_BASE "/model", PLATFORM_NAME_RADXA_CM3_IO)) {
            b->platform_name = PLATFORM_NAME_RADXA_CM3_IO;
            b->uart_dev[0].device_path = (char*) radxa_cm3_io_serialdev[0];
        }
    }

    // UART
    b->uart_dev_count = MRAA_RADXA_CM3_IO_UART_COUNT;
    b->def_uart_dev = 0;
    b->uart_dev[0].index = 2;

    // I2C
    if (strncmp(b->platform_name, PLATFORM_NAME_RADXA_CM3_IO, MAX_SIZE) == 0) {
        b->i2c_bus_count = MRAA_RADXA_CM3_IO_I2C_COUNT;
        b->def_i2c_bus = 0;
        b->i2c_bus[0].bus_id = 2;
        b->i2c_bus[1].bus_id = 4;
    }

    // SPI
    b->spi_bus_count = MRAA_RADXA_CM3_IO_SPI_COUNT;
    b->def_spi_bus = 0;
    b->spi_bus[0].bus_id = 0;
    b->spi_bus[1].bus_id = 3;

    b->pwm_dev_count = MRAA_RADXA_CM3_IO_PWM_COUNT;
    b->pwm_default_period = 500;
    b->pwm_max_period = 2147483;
    b->pwm_min_period = 1;

    b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * b->phy_pin_count);
    if (b->pins == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }

    b->pins[3].pwm.parent_id = 2;
    b->pins[3].pwm.mux_total = 0;
    b->pins[3].pwm.pinmap = 0;
    b->pins[5].pwm.parent_id = 1;
    b->pins[5].pwm.mux_total = 0;
    b->pins[5].pwm.pinmap = 0;
    b->pins[11].pwm.parent_id = 0;
    b->pins[11].pwm.mux_total = 0;
    b->pins[11].pwm.pinmap = 0;
    b->pins[13].pwm.parent_id = 0;
    b->pins[13].pwm.mux_total = 0;
    b->pins[13].pwm.pinmap = 0;
    b->pins[15].pwm.parent_id = 4;
    b->pins[15].pwm.mux_total = 0;
    b->pins[15].pwm.pinmap = 0;
    b->pins[31].pwm.parent_id = 6;
    b->pins[31].pwm.mux_total = 0;
    b->pins[31].pwm.pinmap = 0;
    b->pins[32].pwm.parent_id =11;
    b->pins[32].pwm.mux_total = 0;
    b->pins[32].pwm.pinmap = 0;
    b->pins[33].pwm.parent_id = 7;
    b->pins[33].pwm.mux_total = 0;
    b->pins[33].pwm.pinmap = 0;
    b->pins[37].pwm.parent_id = 3;
    b->pins[37].pwm.mux_total = 0;
    b->pins[37].pwm.pinmap = 0;

    b->aio_count = MRAA_RADXA_CM3_IO_AIO_COUNT;
    b->adc_raw = 10;
    b->adc_supported = 10;
    b->aio_dev[0].pin = 26;
    b->aio_non_seq = 1;

    mraa_radxa_cm3_io_pininfo(b, 0,   -1, (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "INVALID");
    mraa_radxa_cm3_io_pininfo(b, 1,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_radxa_cm3_io_pininfo(b, 2,   14, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_radxa_cm3_io_pininfo(b, 3,   -1, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "SDA2");
    mraa_radxa_cm3_io_pininfo(b, 4,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_radxa_cm3_io_pininfo(b, 5,   13, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "SCL2");
    mraa_radxa_cm3_io_pininfo(b, 6,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_io_pininfo(b, 7,  125, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_D5");
    mraa_radxa_cm3_io_pininfo(b, 8,   25, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "TXD2");
    mraa_radxa_cm3_io_pininfo(b, 9,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_io_pininfo(b, 10,  24, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "RXD2");
    mraa_radxa_cm3_io_pininfo(b, 11,  23, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWM0_M0");
    mraa_radxa_cm3_io_pininfo(b, 12, 119, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_A7");
    mraa_radxa_cm3_io_pininfo(b, 13,  15, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWM0_M1");
    mraa_radxa_cm3_io_pininfo(b, 14,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_io_pininfo(b, 15,  19, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWM4");
    mraa_radxa_cm3_io_pininfo(b, 16, 124, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_D4");
    mraa_radxa_cm3_io_pininfo(b, 17,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_radxa_cm3_io_pininfo(b, 18, 123, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_D3");
    mraa_radxa_cm3_io_pininfo(b, 19, 138, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI3_MOSI_M0");
    mraa_radxa_cm3_io_pininfo(b, 20,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_io_pininfo(b, 21, 136, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI3_MISO_M0");
    mraa_radxa_cm3_io_pininfo(b, 22, 118, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_C6");
    mraa_radxa_cm3_io_pininfo(b, 23, 139, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI1CLK_M0");
    mraa_radxa_cm3_io_pininfo(b, 24, 134, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI3_CS0_M0");
    mraa_radxa_cm3_io_pininfo(b, 25,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_io_pininfo(b, 26,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,1,0}, "SARADC");
    mraa_radxa_cm3_io_pininfo(b, 27, 140, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "SDA2_M1");
    mraa_radxa_cm3_io_pininfo(b, 28, 141, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "SCL2_M1");
    mraa_radxa_cm3_io_pininfo(b, 29, 137, (mraa_pincapabilities_t){1,1,0,0,1,1,0,0}, "SDO2,SPI3_CLK_M0");
    mraa_radxa_cm3_io_pininfo(b, 30,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_io_pininfo(b, 31,  21, (mraa_pincapabilities_t){1,1,0,0,1,1,0,0}, "SDA6,SPI0_MISO_M0");
    mraa_radxa_cm3_io_pininfo(b, 32, 144, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "PWM11_IR_M1");
    mraa_radxa_cm3_io_pininfo(b, 33,  22, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI0_SC0_M0");
    mraa_radxa_cm3_io_pininfo(b, 34,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_io_pininfo(b, 35, 120, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_D0");
    mraa_radxa_cm3_io_pininfo(b, 36, 135, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "SPI3_SC1_M0");
    mraa_radxa_cm3_io_pininfo(b, 37,  18, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "PWM3_IR");
    mraa_radxa_cm3_io_pininfo(b, 38, 122, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_D2");
    mraa_radxa_cm3_io_pininfo(b, 39,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_cm3_io_pininfo(b, 40, 121, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_D1");

    return b;
}
