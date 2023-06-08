/*
 * Author: Sanry <zhangxinyu@radxa.com>
 * Copyright (c) Radxa Limited.
 *
 * SPDX-License-Identifier: MIT
 */

#include <mraa/common.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include "arm/radxa_rock_5b.h"
#include "common.h"

#define DT_BASE "/proc/device-tree"

#define PLATFORM_NAME_RADXA_ROCK_5_MODEL_B   "Radxa ROCK 5B"

#define MAX_SIZE 64

static const char* radxa_rock_5b_serialdev[MRAA_RADXA_ROCK_5B_UART_COUNT] = { "/dev/ttyS2", "/dev/ttyS3", "/dev/ttyS4", "/dev/ttyS7" };
static const char* radxa_rock_5b_led[MRAA_RADXA_ROCK_5B_LED_COUNT] = { "status" };

void
mraa_radxa_rock_5b_pininfo(mraa_board_t* board, int index, int sysfs_pin, mraa_pincapabilities_t pincapabilities_t, char* fmt, ...)
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
mraa_radxa_rock_5b()
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
    b->phy_pin_count = MRAA_RADXA_ROCK_5B_PIN_COUNT + 1;

    if (mraa_file_exist(DT_BASE "/model")) {
        if (mraa_file_contains(DT_BASE "/model", PLATFORM_NAME_RADXA_ROCK_5_MODEL_B)) {
            b->platform_name = PLATFORM_NAME_RADXA_ROCK_5_MODEL_B;
            b->uart_dev[0].device_path = (char*) radxa_rock_5b_serialdev[0];
            b->uart_dev[1].device_path = (char*) radxa_rock_5b_serialdev[1];
            b->uart_dev[2].device_path = (char*) radxa_rock_5b_serialdev[2];
            b->uart_dev[3].device_path = (char*) radxa_rock_5b_serialdev[3];
        }
    }

    // UART
    b->uart_dev_count = MRAA_RADXA_ROCK_5B_UART_COUNT;
    b->def_uart_dev = 0;
    b->uart_dev[0].index = 2;
    b->uart_dev[1].index = 3;
    b->uart_dev[2].index = 4;
    b->uart_dev[3].index = 7;

    // I2C
    if (strncmp(b->platform_name, PLATFORM_NAME_RADXA_ROCK_5_MODEL_B, MAX_SIZE) == 0) {
        b->i2c_bus_count = MRAA_RADXA_ROCK_5B_I2C_COUNT ;
        b->def_i2c_bus = 0;
        b->i2c_bus[0].bus_id = 0;
        b->i2c_bus[1].bus_id = 1;
        b->i2c_bus[2].bus_id = 3;
        b->i2c_bus[3].bus_id = 7;
        b->i2c_bus[4].bus_id = 8;
    }

    // SPI
    b->spi_bus_count = MRAA_RADXA_ROCK_5B_SPI_COUNT;
    b->def_spi_bus = 0;
    b->spi_bus[0].bus_id = 0;
    b->spi_bus[1].bus_id = 1;

    // PWM
    b->pwm_dev_count = MRAA_RADXA_ROCK_5B_PWM_COUNT;
    b->pwm_default_period = 500;
    b->pwm_max_period = 2147483;
    b->pwm_min_period = 1;

    b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * b->phy_pin_count);
    if (b->pins == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }

    b->pins[36].pwm.parent_id = 2;       //pwm2-m1
    b->pins[36].pwm.mux_total = 0;
    b->pins[36].pwm.pinmap = 0;

    b->pins[38].pwm.parent_id = 3;       //pwm3-m1
    b->pins[38].pwm.mux_total = 0;
    b->pins[38].pwm.pinmap = 0;

    b->pins[18].pwm.parent_id = 5;       //pwm5-m2
    b->pins[18].pwm.mux_total = 0;
    b->pins[18].pwm.pinmap = 0;

    b->pins[28].pwm.parent_id = 6;       //pwm6-m2
    b->pins[28].pwm.mux_total = 0;
    b->pins[28].pwm.pinmap = 0;

    b->pins[27].pwm.parent_id = 7;       //pwm7-m3
    b->pins[27].pwm.mux_total = 0;
    b->pins[27].pwm.pinmap = 0;

    b->pins[33].pwm.parent_id = 8;       //pwm8-m0
    b->pins[33].pwm.mux_total = 0;
    b->pins[33].pwm.pinmap = 0;

    b->pins[12].pwm.parent_id = 12;      //pwm12-m0
    b->pins[12].pwm.mux_total = 0;
    b->pins[12].pwm.pinmap = 0;

    b->pins[35].pwm.parent_id = 13;      //pwm13-m0
    b->pins[35].pwm.mux_total = 0;
    b->pins[35].pwm.pinmap = 0;
    b->pins[31].pwm.parent_id = 13;      //pwm13-m2
    b->pins[31].pwm.mux_total = 0;
    b->pins[31].pwm.pinmap = 0;

    b->pins[32].pwm.parent_id = 14;      //pwm14-m0
    b->pins[32].pwm.mux_total = 0;
    b->pins[32].pwm.pinmap = 0;
    b->pins[5].pwm.parent_id = 14;       //pwm14-m1
    b->pins[5].pwm.mux_total = 0;
    b->pins[5].pwm.pinmap = 0;

    b->pins[7].pwm.parent_id = 15;       //pwm15-m0
    b->pins[7].pwm.mux_total = 0;
    b->pins[7].pwm.pinmap = 0;
    b->pins[3].pwm.parent_id = 15;       //pwm15-m1
    b->pins[3].pwm.mux_total = 0;
    b->pins[3].pwm.pinmap = 0;
    b->pins[29].pwm.parent_id = 15;      //pwm15-m3
    b->pins[29].pwm.mux_total = 0;
    b->pins[29].pwm.pinmap = 0;

    // ADC
    b->aio_count = MRAA_RADXA_ROCK_5B_ADC_COUNT;
    b->adc_raw = 10;
    b->adc_supported = 10;
    b->aio_dev[0].pin = 22;
    b->aio_non_seq = 1;
    b->pins[22].aio.pinmap = 4;
    
    mraa_radxa_rock_5b_pininfo(b, 0,   -1,  (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "INVALID");
    mraa_radxa_rock_5b_pininfo(b, 1,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_radxa_rock_5b_pininfo(b, 2,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_radxa_rock_5b_pininfo(b, 3,   139, (mraa_pincapabilities_t){1,1,1,0,0,1,0,0}, "I2C7_SDA_M3, PWM15_IR_M1");
    mraa_radxa_rock_5b_pininfo(b, 4,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_radxa_rock_5b_pininfo(b, 5,   138, (mraa_pincapabilities_t){1,1,1,0,0,1,0,0}, "I2C7_SCL_M3, PWM14_M1");
    mraa_radxa_rock_5b_pininfo(b, 6,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5b_pininfo(b, 7,   115, (mraa_pincapabilities_t){1,1,1,0,1,1,0,0}, "SPI1_CS1_M1, I2C8_SDA_M4, PWM15_IR_M0");
    mraa_radxa_rock_5b_pininfo(b, 8,   13,  (mraa_pincapabilities_t){1,1,0,0,0,1,0,1}, "UART2_TX_M0, I2C1_SCL_M0");
    mraa_radxa_rock_5b_pininfo(b, 9,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5b_pininfo(b, 10,  14,  (mraa_pincapabilities_t){1,1,0,0,0,1,0,1}, "UART2_RX_M0, I2C1_SDA_M0");
    mraa_radxa_rock_5b_pininfo(b, 11,  113, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI1_CLK_M1, UART7_RX_M1");
    mraa_radxa_rock_5b_pininfo(b, 12,  109, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "PWM12_M0, UART3_TX_M1");
    mraa_radxa_rock_5b_pininfo(b, 13,  111, (mraa_pincapabilities_t){1,1,0,0,1,1,0,0}, "SPI1_MOSI_M1, I2C3_SCL_M1");
    mraa_radxa_rock_5b_pininfo(b, 14,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5b_pininfo(b, 15,  112, (mraa_pincapabilities_t){1,1,0,0,1,1,0,1}, "SPI1_MISO_M1, I2C3_SDA_M1, UART7_TX_M1");
    mraa_radxa_rock_5b_pininfo(b, 16,  100, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_A4");
    mraa_radxa_rock_5b_pininfo(b, 17,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_radxa_rock_5b_pininfo(b, 18,  148, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWM5_M2");
    mraa_radxa_rock_5b_pininfo(b, 19,  42,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "UART4_RX_M2, SPI0_MOSI_M2");
    mraa_radxa_rock_5b_pininfo(b, 20,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5b_pininfo(b, 21,  41,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI0_MISO_M2");
    mraa_radxa_rock_5b_pininfo(b, 22,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,1,0}, "SARADC_IN4");
    mraa_radxa_rock_5b_pininfo(b, 23,  43,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "UART4_TX_M2, SPI0_CLK_M2");
    mraa_radxa_rock_5b_pininfo(b, 24,  44,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI0_CS0_M2, UART7_RX_M2");
    mraa_radxa_rock_5b_pininfo(b, 25,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5b_pininfo(b, 26,  45,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI0_CS1_M2, UART7_TX_M2");
    mraa_radxa_rock_5b_pininfo(b, 27,  150, (mraa_pincapabilities_t){1,1,1,0,0,1,0,0}, "PWM7_IR_M3, I2C0_SDA_M1");
    mraa_radxa_rock_5b_pininfo(b, 28,  149, (mraa_pincapabilities_t){1,1,1,0,0,1,0,0}, "I2C0_SCL_M1, PWM6_M2");
    mraa_radxa_rock_5b_pininfo(b, 29,  63,  (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWM15_IR_M3");
    mraa_radxa_rock_5b_pininfo(b, 30,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5b_pininfo(b, 31,  47,  (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWM13_M2");
    mraa_radxa_rock_5b_pininfo(b, 32,  114, (mraa_pincapabilities_t){1,1,1,0,1,1,0,0}, "PWM14_M0, I2C8_SCL_M4, SPI1_CS0_M1");
    mraa_radxa_rock_5b_pininfo(b, 33,  103, (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PWM8_M0");
    mraa_radxa_rock_5b_pininfo(b, 34,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5b_pininfo(b, 35,  110, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "UART3_RX_M1, PWM13_M0");
    mraa_radxa_rock_5b_pininfo(b, 36,  105, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "PWM2_M1, UART2_TX_M2");
    mraa_radxa_rock_5b_pininfo(b, 37,  -1,  (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "INVALID");
    mraa_radxa_rock_5b_pininfo(b, 38,  106, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "PWM3_IR_M1, UART2_RX_M2");
    mraa_radxa_rock_5b_pininfo(b, 39,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5b_pininfo(b, 40,  107, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "GPIO3_B3");

    /* set leds */
    b->led_dev[0].name = (char*) radxa_rock_5b_led[0];
    b->led_dev_count   = MRAA_RADXA_ROCK_5B_LED_COUNT;

    return b;
}
