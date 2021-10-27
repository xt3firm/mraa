/*
 * Author: Ken <ken@radxa.com>
 * Copyright (c) Radxa Limited.
 *
 * SPDX-License-Identifier: MIT
 */

#include <mraa/common.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include "arm/radxa_zero.h"
#include "common.h"

#define DT_BASE "/proc/device-tree"

#define PLATFORM_NAME_RADXA_ZERO "Radxa Zero"

#define MAX_SIZE 64

const char* radxa_zero_serialdev[MRAA_RADXA_ZERO_UART_COUNT] = { "/dev/ttyAML0", "/dev/ttyAML1", "/dev/ttyAML4" };

void
mraa_radxa_zero_pininfo(mraa_board_t* board, int index, int sysfs_pin, mraa_pincapabilities_t pincapabilities_t, char* fmt, ...)
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
mraa_radxa_zero()
{
    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof(mraa_board_t));
    if (b== NULL) {
        return NULL;
    }

    b->adv_func = (mraa_adv_func_t*) calloc(1, sizeof(mraa_adv_func_t));
    if (b->adv_func == NULL) {
        free(b);
        return NULL;
    }

    // pin mux for buses are setup by default by kernel so tell mraa to ignore them
    b->no_bus_mux = 1;
    b->phy_pin_count = MRAA_RADXA_ZERO_PIN_COUNT + 1;

    if (mraa_file_exist(DT_BASE "/model")) {
        // We are on a modern kernel, great!!!!
        if (mraa_file_contains(DT_BASE "/model", PLATFORM_NAME_RADXA_ZERO)  ) {
            b->platform_name = PLATFORM_NAME_RADXA_ZERO;
            b->uart_dev[0].device_path = (char*) radxa_zero_serialdev[0];
            b->uart_dev[1].device_path = (char*) radxa_zero_serialdev[1];
            b->uart_dev[2].device_path = (char*) radxa_zero_serialdev[2];
        }
    }

    // UART
    b->uart_dev_count = MRAA_RADXA_ZERO_UART_COUNT;
    b->def_uart_dev = 0;
    b->uart_dev[0].index = 0;
    b->uart_dev[1].index = 1;
    b->uart_dev[2].index = 2;

    // I2C
    if (strncmp(b->platform_name, PLATFORM_NAME_RADXA_ZERO, MAX_SIZE) == 0) {
        b->i2c_bus_count = MRAA_RADXA_ZERO_I2C_COUNT ;
        b->def_i2c_bus = 0;
        b->i2c_bus[0].bus_id = 1;
        b->i2c_bus[1].bus_id = 3;
        b->i2c_bus[2].bus_id = 4;
    }

    // SPI
    b->spi_bus_count = MRAA_RADXA_ZERO_SPI_COUNT;
    b->def_spi_bus = 0;
    b->spi_bus[0].bus_id = 0;
    b->spi_bus[1].bus_id = 1;

    // PWM
    b->pwm_dev_count = MRAA_RADXA_ZERO_PWM_COUNT;
    b->pwm_default_period = 500;
    b->pwm_max_period = 2147483;
    b->pwm_min_period = 1;

    b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * b->phy_pin_count);
    if (b->pins == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }

    b->pins[18].pwm.parent_id = 6;
    b->pins[18].pwm.mux_total = 0;
    b->pins[18].pwm.pinmap = 0;
    b->pins[40].pwm.parent_id = 2;
    b->pins[40].pwm.mux_total = 0;
    b->pins[40].pwm.pinmap = 0;

    mraa_radxa_zero_pininfo(b, 0,   -1, (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "INVALID");
    mraa_radxa_zero_pininfo(b, 1,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_radxa_zero_pininfo(b, 2,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_radxa_zero_pininfo(b, 3,  490, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "I2C_EE_M3_SDA");
    mraa_radxa_zero_pininfo(b, 4,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_radxa_zero_pininfo(b, 5,  491, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "I2C_EE_M3_SCL");
    mraa_radxa_zero_pininfo(b, 6,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_zero_pininfo(b, 7,  415, (mraa_pincapabilities_t){1,1,0,0,0,1,0,1}, "I2C_AO_M0_SDA,UART_AO_B_RX");
    mraa_radxa_zero_pininfo(b, 8,  412, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART_AO_A_TXD");
    mraa_radxa_zero_pininfo(b, 9,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_zero_pininfo(b, 10, 413, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART_AO_A_RXD");
    mraa_radxa_zero_pininfo(b, 11, 414, (mraa_pincapabilities_t){1,1,0,0,0,1,0,1}, "I2C_AO_M0_SCL,UART_AO_B_TX");
    mraa_radxa_zero_pininfo(b, 12, 501, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI_A_MISO");
    mraa_radxa_zero_pininfo(b, 13, 503, (mraa_pincapabilities_t){1,1,0,0,1,1,0,0}, "I2C_EE_M1_SCL,SPI_A_SCLK");
    mraa_radxa_zero_pininfo(b, 14,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_zero_pininfo(b, 15,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "SARADC_CH1");
    mraa_radxa_zero_pininfo(b, 16, 502, (mraa_pincapabilities_t){1,1,0,0,1,1,0,0}, "I2C_EE_M1_SDA,SPI_A_SS0");
    mraa_radxa_zero_pininfo(b, 17,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_radxa_zero_pininfo(b, 18, 500, (mraa_pincapabilities_t){1,1,1,0,1,0,0,0}, "SPI_A_MOSI,PWM_C");
    mraa_radxa_zero_pininfo(b, 19, 447, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI_B_MOSI");
    mraa_radxa_zero_pininfo(b, 20,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_zero_pininfo(b, 21, 448, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI_B_MISO");
    mraa_radxa_zero_pininfo(b, 22, 475, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIOC_7");
    mraa_radxa_zero_pininfo(b, 23, 450, (mraa_pincapabilities_t){1,1,0,0,1,1,0,1}, "I2C_EE_M1_SCL,SPI_B_SCLK,UART_EE_C_TX");
    mraa_radxa_zero_pininfo(b, 24, 449, (mraa_pincapabilities_t){1,1,0,0,1,1,0,1}, "UART_EE_C_RX,I2C_EE_M1_SDA，SPI_B_SS0");
    mraa_radxa_zero_pininfo(b, 25,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_zero_pininfo(b, 26,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "SARADC_CH2");
    mraa_radxa_zero_pininfo(b, 27, 415, (mraa_pincapabilities_t){1,1,0,0,0,1,0,1}, "I2C_AO_M0_SDA,UART_AO_B_RX");
    mraa_radxa_zero_pininfo(b, 28, 414, (mraa_pincapabilities_t){1,1,0,0,0,1,0,1}, "I2C_AO_M0_SCL,UART_AO_B_TX");
    mraa_radxa_zero_pininfo(b, 29,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "NC");
    mraa_radxa_zero_pininfo(b, 30,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_zero_pininfo(b, 31,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "NC");
    mraa_radxa_zero_pininfo(b, 32, 416, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIOAO_4");
    mraa_radxa_zero_pininfo(b, 33,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "NC");
    mraa_radxa_zero_pininfo(b, 34,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_zero_pininfo(b, 35, 420, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART_AO_B_TX");
    mraa_radxa_zero_pininfo(b, 36, 451, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIOH_8");
    mraa_radxa_zero_pininfo(b, 37, 421, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART_AO_B_RX");
    mraa_radxa_zero_pininfo(b, 38, 422, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIOAO_10");
    mraa_radxa_zero_pininfo(b, 39,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_zero_pininfo(b, 40, 423, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWMAO_A");

    return b;
}
