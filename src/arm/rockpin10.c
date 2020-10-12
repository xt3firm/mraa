/*
 * Author: zzl <zzl@radxa.com>
 * Copyright (c) Radxa Limited.
 *
 * SPDX-License-Identifier: MIT
 */

#include <mraa/common.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "arm/rockpin10.h"
#include "common.h"

#define DT_BASE "/proc/device-tree"
#define PLATFORM_NAME_ROCK_PI_N10 "Radxa ROCK Pi N10"
#define MAX_SIZE 64

const char* rockpin10_serialdev[MRAA_ROCKPIN10_UART_COUNT] = { "/dev/ttyS2","/dev/ttyS4"};

void
mraa_rockpin10_pininfo(mraa_board_t* board, int index, int sysfs_pin, mraa_pincapabilities_t pincapabilities_t, char* fmt, ...)
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
mraa_rockpin10()
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
    b->phy_pin_count = MRAA_ROCKPIN10_PIN_COUNT + 1;

    if (mraa_file_exist(DT_BASE "/model")) {
        // We are on a modern kernel, great!!!!
        if (mraa_file_contains(DT_BASE "/model", PLATFORM_NAME_ROCK_PI_N10) ) {
            b->platform_name = PLATFORM_NAME_ROCK_PI_N10;
            b->uart_dev[0].device_path = (char*) rockpin10_serialdev[0];
            b->uart_dev[1].device_path = (char*) rockpin10_serialdev[1];
        }
    }

    // UART
    b->uart_dev_count = MRAA_ROCKPIN10_UART_COUNT;
    b->def_uart_dev = 0;
    b->uart_dev[0].index = 2;
    b->uart_dev[1].index = 4;

    // I2C
    if (strncmp(b->platform_name, PLATFORM_NAME_ROCK_PI_N10, MAX_SIZE) == 0) {
        b->i2c_bus_count = MRAA_ROCKPIN10_I2C_COUNT;
        b->def_i2c_bus = 0;
        b->i2c_bus[0].bus_id = 7;
        b->i2c_bus[1].bus_id = 2;
        b->i2c_bus[2].bus_id = 6;
    }

    // SPI
    b->spi_bus_count = MRAA_ROCKPIN10_SPI_COUNT;
    b->def_spi_bus = 0;
    b->spi_bus[0].bus_id = 32766;
    
    //PWM
    b->pwm_dev_count = MRAA_ROCKPIN10_PWM_COUNT;
    b->pwm_default_period = 500;
    b->pwm_max_period = 2147483;
    b->pwm_min_period = 1;

    b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * b->phy_pin_count);
    if (b->pins == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }

    b->pins[11].pwm.parent_id = 0;
    b->pins[11].pwm.mux_total = 0;
    b->pins[11].pwm.pinmap = 0;
    b->pins[13].pwm.parent_id = 1;
    b->pins[13].pwm.mux_total = 0;
    b->pins[13].pwm.pinmap = 0;

    //ADC
    b->aio_count = MRAA_ROCKPIN10_AIO_COUNT;
    b->adc_raw = 10;
    b->adc_supported = 10;
    b->aio_dev[0].pin = 26;
    b->aio_non_seq = 1;

    mraa_rockpin10_pininfo(b, 0,   -1, (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "INVALID");
    mraa_rockpin10_pininfo(b, 1,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_rockpin10_pininfo(b, 2,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_rockpin10_pininfo(b, 3,   71, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "SDA7");
    mraa_rockpin10_pininfo(b, 4,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_rockpin10_pininfo(b, 5,   72, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "SCL7");
    mraa_rockpin10_pininfo(b, 6,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpin10_pininfo(b, 7,   13, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO0_B5");
    mraa_rockpin10_pininfo(b, 8,  148, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "TXD2");
    mraa_rockpin10_pininfo(b, 9,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpin10_pininfo(b, 10, 147, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "RXD2");
    mraa_rockpin10_pininfo(b, 11, 146, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWM0");
    mraa_rockpin10_pininfo(b, 12,   9, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO0_B1");
    mraa_rockpin10_pininfo(b, 13, 150, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWM1");
    mraa_rockpin10_pininfo(b, 14,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpin10_pininfo(b, 15, 149, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO4_C5");
    mraa_rockpin10_pininfo(b, 16,   8, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO0_B0");
    mraa_rockpin10_pininfo(b, 17,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_rockpin10_pininfo(b, 18,  70, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO2_A6");
    mraa_rockpin10_pininfo(b, 19,  40, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI1TX,TXD4");
    mraa_rockpin10_pininfo(b, 20,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpin10_pininfo(b, 21,  39, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI1RX,RXD4");
    mraa_rockpin10_pininfo(b, 22, 124, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_D4");
    mraa_rockpin10_pininfo(b, 23,  41, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI1CLK");
    mraa_rockpin10_pininfo(b, 24,  42, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI1CS");
    mraa_rockpin10_pininfo(b, 25,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpin10_pininfo(b, 26,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,1,0}, "ADC_IN0");
    mraa_rockpin10_pininfo(b, 27,  64, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "SDA2");
    mraa_rockpin10_pininfo(b, 28,  65, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "SCL2");
    mraa_rockpin10_pininfo(b, 29,  74, (mraa_pincapabilities_t){1,1,0,0,1,1,0,0}, "SCL6,SPI2RX");
    mraa_rockpin10_pininfo(b, 30,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpin10_pininfo(b, 31,  73, (mraa_pincapabilities_t){1,1,0,0,1,1,0,0}, "SDA6,SPI2TX");
    mraa_rockpin10_pininfo(b, 32, 125, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_D5");
    mraa_rockpin10_pininfo(b, 33, 157, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO4_D5");
    mraa_rockpin10_pininfo(b, 34,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpin10_pininfo(b, 35,  68, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO2_A4");
    mraa_rockpin10_pininfo(b, 36,  66, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO2_A2");
    mraa_rockpin10_pininfo(b, 37, 126, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_D6");
    mraa_rockpin10_pininfo(b, 38,  67, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO2_A3");
    mraa_rockpin10_pininfo(b, 39,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_rockpin10_pininfo(b, 40,  34, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO1_A2");

    return b;
}
