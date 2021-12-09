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
#include "arm/radxa_rock3a.h"
#include "common.h"

#define DT_BASE "/proc/device-tree"

#define PLATFORM_NAME_RADXA_ROCK_3_MODEL_A "Radxa ROCK 3 Model A"

#define MAX_SIZE 64

const char* radxa_rock3a_serialdev[MRAA_RADXA_ROCK3A_UART_COUNT] = { "/dev/ttyS0", "/dev/ttyS3", "/dev/ttyS5", "/dev/ttyS7", "/dev/ttyS8", "/dev/ttyS9" };

void
mraa_radxa_rock3a_pininfo(mraa_board_t* board, int index, int sysfs_pin, mraa_pincapabilities_t pincapabilities_t, char* fmt, ...)
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
mraa_radxa_rock3a()
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
    b->phy_pin_count = MRAA_RADXA_ROCK3A_PIN_COUNT + 1;

    if (mraa_file_exist(DT_BASE "/model")) {
        // We are on a modern kernel, great!!!!
        if (mraa_file_contains(DT_BASE "/model", PLATFORM_NAME_RADXA_ROCK_3_MODEL_A)  ) {
            b->platform_name = PLATFORM_NAME_RADXA_ROCK_3_MODEL_A;
            b->uart_dev[0].device_path = (char*) radxa_rock3a_serialdev[0];
            b->uart_dev[1].device_path = (char*) radxa_rock3a_serialdev[1];
            b->uart_dev[2].device_path = (char*) radxa_rock3a_serialdev[2];
            b->uart_dev[3].device_path = (char*) radxa_rock3a_serialdev[3];
            b->uart_dev[4].device_path = (char*) radxa_rock3a_serialdev[4];
            b->uart_dev[5].device_path = (char*) radxa_rock3a_serialdev[5];
        }
    }

    // UART
    b->uart_dev_count = MRAA_RADXA_ROCK3A_UART_COUNT;
    b->def_uart_dev = 0;
    b->uart_dev[0].index = 0;
    b->uart_dev[1].index = 3;
    b->uart_dev[2].index = 5;
    b->uart_dev[3].index = 7;
    b->uart_dev[4].index = 8;
    b->uart_dev[5].index = 9;

    // I2C
    if (strncmp(b->platform_name, PLATFORM_NAME_RADXA_ROCK_3_MODEL_A, MAX_SIZE) == 0) {
        b->i2c_bus_count = MRAA_RADXA_ROCK3A_I2C_COUNT ;
        b->def_i2c_bus = 0;
        b->i2c_bus[0].bus_id = 1;
        b->i2c_bus[1].bus_id = 2;
        b->i2c_bus[2].bus_id = 3;
    }

    // SPI
    b->spi_bus_count = MRAA_RADXA_ROCK3A_SPI_COUNT;
    b->def_spi_bus = 0;
    b->spi_bus[0].bus_id = 3;

    //The PWM function cannot be used for the time being, because there is no way to use it one by one.
    // PWM
    b->pwm_dev_count = MRAA_RADXA_ROCK3A_PWM_COUNT;
    b->pwm_default_period = 500;
    b->pwm_max_period = 2147483;
    b->pwm_min_period = 1;

    b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * b->phy_pin_count);
    if (b->pins == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }

    b->pins[15].pwm.parent_id = 0;       //pwm1-m0
    b->pins[15].pwm.mux_total = 0;
    b->pins[15].pwm.pinmap = 0;
    b->pins[7].pwm.parent_id = 0;       //pwm1-m1
    b->pins[7].pwm.mux_total = 0;
    b->pins[7].pwm.pinmap = 0;
    b->pins[22].pwm.parent_id = 1;       //pwm2-m0
    b->pins[22].pwm.mux_total = 0;
    b->pins[22].pwm.pinmap = 0;
    b->pins[16].pwm.parent_id = 1;       //pwm2-m1
    b->pins[16].pwm.mux_total = 0;
    b->pins[16].pwm.pinmap = 0;
    b->pins[18].pwm.parent_id = 1;       //pwm9-m0
    b->pins[18].pwm.mux_total = 0;
    b->pins[18].pwm.pinmap = 0;
    b->pins[21].pwm.parent_id = 1;      //pwm12-m1
    b->pins[21].pwm.mux_total = 0;
    b->pins[21].pwm.pinmap = 0;
    b->pins[24].pwm.parent_id = 1;      //pwm13-m1
    b->pins[24].pwm.mux_total = 0;
    b->pins[24].pwm.pinmap = 0;
    b->pins[11].pwm.parent_id = 1;      //pwm14-m0
    b->pins[11].pwm.mux_total = 0;
    b->pins[11].pwm.pinmap = 0;
    b->pins[23].pwm.parent_id = 1;      //pwm14-m1
    b->pins[23].pwm.mux_total = 0;
    b->pins[23].pwm.pinmap = 0;
    b->pins[13].pwm.parent_id = 1;      //pwm15-m0
    b->pins[13].pwm.mux_total = 0;
    b->pins[13].pwm.pinmap = 0;
    b->pins[19].pwm.parent_id = 1;      //pwm15-m1
    b->pins[19].pwm.mux_total = 0;
    b->pins[19].pwm.pinmap = 0;

    // ADC
    b->aio_count = MRAA_RADXA_ROCK3A_ADC_COUNT;
    b->adc_raw = 10;
    b->adc_supported = 10;
    b->aio_dev[0].pin = 37;
    b->aio_non_seq = 1;
    b->pins[37].aio.pinmap = 5;
    
    mraa_radxa_rock3a_pininfo(b, 0,   -1, (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "INVALID");
    mraa_radxa_rock3a_pininfo(b, 1,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_radxa_rock3a_pininfo(b, 2,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_radxa_rock3a_pininfo(b, 3,   32, (mraa_pincapabilities_t){1,1,0,0,0,1,0,1}, "I2C3_SDA_M0,UART3_RX_M0");
    mraa_radxa_rock3a_pininfo(b, 4,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_radxa_rock3a_pininfo(b, 5,   33, (mraa_pincapabilities_t){1,1,0,0,0,1,0,1}, "I2C3_SCL_M0,UART3_TX_M0");
    mraa_radxa_rock3a_pininfo(b, 6,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock3a_pininfo(b, 7,   13, (mraa_pincapabilities_t){1,1,1,0,0,1,0,0}, "I2C2_SCL_M0,PWM1_M1");
    mraa_radxa_rock3a_pininfo(b, 8,   25, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART2_TXD");
    mraa_radxa_rock3a_pininfo(b, 9,   -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock3a_pininfo(b, 10,  24, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART2_RXD");
    mraa_radxa_rock3a_pininfo(b, 11, 116, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "PWM14_M0,UART7_TX_M1");
    mraa_radxa_rock3a_pininfo(b, 12,  99, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_A3");
    mraa_radxa_rock3a_pininfo(b, 13, 117, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "PWM15_IR_M0,UART7_RX_M1");
    mraa_radxa_rock3a_pininfo(b, 14,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock3a_pininfo(b, 15,  16, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "PWM1_M0,UART0_RX");
    mraa_radxa_rock3a_pininfo(b, 16,  14, (mraa_pincapabilities_t){1,1,1,0,0,1,0,0}, "I2C2_SDA_M0,PWM2_M1");
    mraa_radxa_rock3a_pininfo(b, 17,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_radxa_rock3a_pininfo(b, 18, 106, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWM9_M0");
    mraa_radxa_rock3a_pininfo(b, 19, 147, (mraa_pincapabilities_t){1,1,1,0,1,0,0,0}, "PWM15_IR_M1,SPI3_MOSI_M1");
    mraa_radxa_rock3a_pininfo(b, 20,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock3a_pininfo(b, 21, 149, (mraa_pincapabilities_t){1,1,1,0,1,0,0,1}, "PWM12_M1,SPI3_MISO_M1,UART9_TX_M1");
    mraa_radxa_rock3a_pininfo(b, 22,  17, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "PWM2_M0,UART0_TX");
    mraa_radxa_rock3a_pininfo(b, 23, 146, (mraa_pincapabilities_t){1,1,1,0,1,0,0,0}, "PWM14_M1,SPI3_CLK_M1");
    mraa_radxa_rock3a_pininfo(b, 24, 150, (mraa_pincapabilities_t){1,1,1,0,1,0,0,1}, "PWM13_M1,SPI3_CS0_M1,UART9_RX_M1");
    mraa_radxa_rock3a_pininfo(b, 25,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock3a_pininfo(b, 26, 153, (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI3_CS1_M1");
    mraa_radxa_rock3a_pininfo(b, 27,  12, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "I2C1_SDA");
    mraa_radxa_rock3a_pininfo(b, 28,  11, (mraa_pincapabilities_t){1,1,0,0,0,1,0,0}, "I2C1_SCL");
    mraa_radxa_rock3a_pininfo(b, 29,  95, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART8_TX_M1");
    mraa_radxa_rock3a_pininfo(b, 30,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock3a_pininfo(b, 31,  96, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART8_RX_M1");
    mraa_radxa_rock3a_pininfo(b, 32, 114, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART5_TX_M1");
    mraa_radxa_rock3a_pininfo(b, 33, 115, (mraa_pincapabilities_t){1,1,0,0,0,0,0,1}, "UART5_RX_M1");
    mraa_radxa_rock3a_pininfo(b, 34,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock3a_pininfo(b, 35, 100, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_A4");
    mraa_radxa_rock3a_pininfo(b, 36,  98, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_A2");
    mraa_radxa_rock3a_pininfo(b, 37,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,1,0}, "SARADC_VIN5");
    mraa_radxa_rock3a_pininfo(b, 38, 102, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_A6");
    mraa_radxa_rock3a_pininfo(b, 39,  -1, (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock3a_pininfo(b, 40, 101, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_A5");

    return b;
}
