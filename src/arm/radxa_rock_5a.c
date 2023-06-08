/*
 *
 * SPDX-License-Identifier: MIT
 */

#include <mraa/common.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include "arm/radxa_rock_5a.h"
#include "common.h"

#define DT_BASE "/proc/device-tree"

#define PLATFORM_NAME_RADXA_ROCK_5_MODEL_A   "Radxa ROCK 5A"

#define MAX_SIZE 64

static const char* radxa_rock_5a_serialdev[MRAA_RADXA_ROCK_5A_UART_COUNT] = { "/dev/ttyS2", "/dev/ttyS3", "/dev/ttyS4", "/dev/ttyS7" };
static const char* radxa_rock_5a_led[MRAA_RADXA_ROCK_5A_LED_COUNT] = { "status" };

void
mraa_radxa_rock_5a_pininfo(mraa_board_t* board, int index, int sysfs_pin, mraa_pincapabilities_t pincapabilities_t, char* fmt, ...)
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
mraa_radxa_rock_5a()
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
    b->phy_pin_count = MRAA_RADXA_ROCK_5A_PIN_COUNT + 1;

    if (mraa_file_exist(DT_BASE "/model")) {
        if (mraa_file_contains(DT_BASE "/model", PLATFORM_NAME_RADXA_ROCK_5_MODEL_A)) {
            b->platform_name = PLATFORM_NAME_RADXA_ROCK_5_MODEL_A;
            b->uart_dev[0].device_path = (char*) radxa_rock_5a_serialdev[0];
            b->uart_dev[1].device_path = (char*) radxa_rock_5a_serialdev[1];
            b->uart_dev[2].device_path = (char*) radxa_rock_5a_serialdev[2];
            b->uart_dev[3].device_path = (char*) radxa_rock_5a_serialdev[3];
        }
    }

    // UART
    b->uart_dev_count = MRAA_RADXA_ROCK_5A_UART_COUNT;
    b->def_uart_dev = 0;
    b->uart_dev[0].index = 2;
    b->uart_dev[1].index = 3;
    b->uart_dev[2].index = 4;
    b->uart_dev[3].index = 7;

    // I2C
    if (strncmp(b->platform_name, PLATFORM_NAME_RADXA_ROCK_5_MODEL_A, MAX_SIZE) == 0) {
        b->i2c_bus_count = MRAA_RADXA_ROCK_5A_I2C_COUNT ;
        b->def_i2c_bus = 0;
        b->i2c_bus[0].bus_id = 0;
        b->i2c_bus[1].bus_id = 2;
        b->i2c_bus[2].bus_id = 7;
        b->i2c_bus[3].bus_id = 8;
        b->i2c_bus[4].bus_id = 9;
    }

    // SPI
    b->spi_bus_count = MRAA_RADXA_ROCK_5A_SPI_COUNT;
    b->def_spi_bus = 0;
    b->spi_bus[0].bus_id = 0;
    b->spi_bus[1].bus_id = 1;

    // PWM
    b->pwm_dev_count = MRAA_RADXA_ROCK_5A_PWM_COUNT;
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

#if 0
    // ADC (v1.1)
    b->aio_count = MRAA_RADXA_ROCK_5A_ADC_COUNT;
    b->adc_raw = 10;
    b->adc_supported = 10;
    b->aio_dev[0].pin = 22;
    b->aio_non_seq = 1;
    b->pins[22].aio.pinmap = 4;
    
    // V1.1 pin info
    mraa_radxa_rock_5a_pininfo(b, 0,   -1,  (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "INVALID");
    mraa_radxa_rock_5a_pininfo(b, 1,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_radxa_rock_5a_pininfo(b, 2,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_radxa_rock_5a_pininfo(b, 3,   32,  (mraa_pincapabilities_t){1,1,1,0,0,1,0,0}, "I2C2_SDA_M4, SPI4_MISO_M2, UART6_RX_M1");
    mraa_radxa_rock_5a_pininfo(b, 4,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_radxa_rock_5a_pininfo(b, 5,   33,  (mraa_pincapabilities_t){1,1,1,0,0,1,0,0}, "I2C2_SCL_M4, SPI4_MOSI_M2, UART6_TX_M1");
    mraa_radxa_rock_5a_pininfo(b, 6,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 7,   43,  (mraa_pincapabilities_t){1,1,1,0,1,1,0,0}, "SPI0_CLK_M2, UART4_TX_M2, PDMI_CLK1_M1");
    mraa_radxa_rock_5a_pininfo(b, 8,   13,  (mraa_pincapabilities_t){1,1,0,0,0,1,0,1}, "UART2_TX_M0, I2C1_SCL_M0, I2S1_MCLK_M1");
    mraa_radxa_rock_5a_pininfo(b, 9,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 10,  14,  (mraa_pincapabilities_t){1,1,0,0,0,1,0,1}, "UART2_RX_M0, I2C1_SDA_M0, I2S1_SCLK_M1");
    mraa_radxa_rock_5a_pininfo(b, 11,  34,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "I2C4_SDA_M3, SPI4_CLK_M2, UART6_RTSN_M1, PWM0_M2");
    mraa_radxa_rock_5a_pininfo(b, 12,  129, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "SPI0_MOSI_M1, UART9_CTSN_M1, I2S1_SCLK_M0");
    mraa_radxa_rock_5a_pininfo(b, 13,  35,  (mraa_pincapabilities_t){1,1,0,0,1,1,0,0}, "I2C4_SCL_M3, SPI4_CS0_M2, UART6_CTSN_M1, PWM1_M2");
    mraa_radxa_rock_5a_pininfo(b, 14,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 15,  140, (mraa_pincapabilities_t){1,1,0,0,1,1,0,1}, "UART9_TX_M1, PWM11_IR_M1, SPDIF0_TX_M1, I2S1_SD03_M0");
    mraa_radxa_rock_5a_pininfo(b, 16,  62,  (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "I2C8_SCL_M2, UART1_RTSN_M1, PWM14_M2");
    mraa_radxa_rock_5a_pininfo(b, 17,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_radxa_rock_5a_pininfo(b, 18,  63,  (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "I2C8_SDA_M2, UART1_CTSN_M1, PWM15_IR_M3");
    mraa_radxa_rock_5a_pininfo(b, 19,  37,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI2_MOSI_M0");
    mraa_radxa_rock_5a_pininfo(b, 20,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 21,  36,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "SPI2_MISO_M0");
    mraa_radxa_rock_5a_pininfo(b, 22,  45,  (mraa_pincapabilities_t){1,0,0,0,0,0,1,0}, "SPI0_CS1_M2, UART7_TX_M2");
    mraa_radxa_rock_5a_pininfo(b, 23,  38,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI2_CLK_M0");
    mraa_radxa_rock_5a_pininfo(b, 24,  39,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI2_CS0_M0, PDM1_SDI0_M1, PWM3_IR_M3");
    mraa_radxa_rock_5a_pininfo(b, 25,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 26,  -1,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SARADC_VIN5");
    mraa_radxa_rock_5a_pininfo(b, 27,  139, (mraa_pincapabilities_t){1,1,1,0,0,1,0,0}, "I2C7_SDA_M3, UART8_CTSN_M0, PWM15_IR_M1, I2S1_SDO2_M0, CAN1_TX_M1");
    mraa_radxa_rock_5a_pininfo(b, 28,  138, (mraa_pincapabilities_t){1,1,1,0,0,1,0,0}, "I2C7_SCL_M3, SPI0_CS0_M1, UART8_RTSN_M0, PWM14_M1, I2S1_SDO1_M0, CAN1_RX_M1");
    mraa_radxa_rock_5a_pininfo(b, 29,  42,  (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "SPI0_MOSI_M2, UART4_RX_M2, PDM1_SDI3_M1");
    mraa_radxa_rock_5a_pininfo(b, 30,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 31,  41,  (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "SPI0_MISO_M2, PDM1_SDI2_M1");
    mraa_radxa_rock_5a_pininfo(b, 32,  136, (mraa_pincapabilities_t){1,1,1,0,1,1,0,0}, "I2C6_SDA_M3, UART8_TX_M0, I2S1_SDI3_M0");
    mraa_radxa_rock_5a_pininfo(b, 33,  44,  (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "SPI0_CS0_M2, UART7_RX_M2, PDM1_CLK0_M1");
    mraa_radxa_rock_5a_pininfo(b, 34,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 35,  128, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "SPI0_MISO_M1, UART9_RTSN_M1, I2S1_MCLK_M0");
    mraa_radxa_rock_5a_pininfo(b, 36,  130, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "SPI0_CLK_M1, I2S1_LRCK_M0");
    mraa_radxa_rock_5a_pininfo(b, 37,  40,  (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "SPI2_CS1_M0, PDM1_SDI1_M1");
    mraa_radxa_rock_5a_pininfo(b, 38,  133, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "I2C3_SDA_M2, UART3_TX_M2, I2S1_SDI0_M0");
    mraa_radxa_rock_5a_pininfo(b, 39,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 40,  137, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "I2C6_SCL_M3, SPI0_CS1_M1, UART8_RX_M0, SPDIF1_TX_M1, I2S1_SDO0_M0");
#endif
    // ADC (v1.2)
    b->aio_count = MRAA_RADXA_ROCK_5A_ADC_COUNT;
    b->adc_raw = 10;
    b->adc_supported = 10;
    b->aio_dev[0].pin = 37;
    b->aio_non_seq = 1;
    b->pins[37].aio.pinmap = 4;
    
    // V1.2 pin info
    mraa_radxa_rock_5a_pininfo(b, 0,   -1,  (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "INVALID");
    mraa_radxa_rock_5a_pininfo(b, 1,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_radxa_rock_5a_pininfo(b, 2,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_radxa_rock_5a_pininfo(b, 3,   63,  (mraa_pincapabilities_t){1,1,1,0,0,1,0,0}, "I2C8_SDA_M2, UART1_CTSN_M1, PWM15_IR_M3");
    mraa_radxa_rock_5a_pininfo(b, 4,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "5V");
    mraa_radxa_rock_5a_pininfo(b, 5,   62,  (mraa_pincapabilities_t){1,1,1,0,0,1,0,0}, "I2C8_SCL_M2, UART1_RTSN_M1, PWM14_M2");
    mraa_radxa_rock_5a_pininfo(b, 6,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 7,   43,  (mraa_pincapabilities_t){1,1,1,0,1,1,0,0}, "PDM1_CLK1_M1, SPI0_CLK_M2, UART4_TX_M2");
    mraa_radxa_rock_5a_pininfo(b, 8,   13,  (mraa_pincapabilities_t){1,1,0,0,0,1,0,1}, "UART2_TX_M0, I2C1_SCL_M0, I2S1_MCLK_M1");
    mraa_radxa_rock_5a_pininfo(b, 9,   -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 10,  14,  (mraa_pincapabilities_t){1,1,0,0,0,1,0,1}, "UART2_RX_M0, I2C1_SDA_M0, I2S1_SCLK_M1");
    mraa_radxa_rock_5a_pininfo(b, 11,  139, (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "I2C7_SDA_M3, UART8_CTSN_M0, I2S1_SDO2_M0, PWM15_IR_M1, CAN1_TX_M1");
    mraa_radxa_rock_5a_pininfo(b, 12,  129, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "SPI0_MOSI_M1, UART9_CTSN_M1, I2S1_SCLK_M0");
    mraa_radxa_rock_5a_pininfo(b, 13,  138, (mraa_pincapabilities_t){1,1,0,0,1,1,0,0}, "I2C7_SCL_M3, SPI0_CS0_M1, UART8_RTSN_M0, I2S1_SDO1_M0, PWM14_M1, CAN1_RX_M1");
    mraa_radxa_rock_5a_pininfo(b, 14,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 15,  140, (mraa_pincapabilities_t){1,1,0,0,1,1,0,1}, "UART9_TX_M1, PWM11_IR_M1, SPDIF0_TX_M1, I2S1_SD03_M0");
    mraa_radxa_rock_5a_pininfo(b, 16,  37,  (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "SPI2_MOSI_M0");
    mraa_radxa_rock_5a_pininfo(b, 17,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "3V3");
    mraa_radxa_rock_5a_pininfo(b, 18,  40,  (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PDM1_SDI1_M1, SPI2_CS1_M0");
    mraa_radxa_rock_5a_pininfo(b, 19,  33,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "I2C2_SCL_M4, SPI4_MOSI_M2, UART6_TX_M1");
    mraa_radxa_rock_5a_pininfo(b, 20,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 21,  32,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,0}, "I2C2_SDA_M4, SPI4_MISO_M2, UART6_RX_M1");
    mraa_radxa_rock_5a_pininfo(b, 22,  45,  (mraa_pincapabilities_t){1,0,0,0,0,0,1,0}, "SPI0_CS1_M2, UART7_TX_M2");
    mraa_radxa_rock_5a_pininfo(b, 23,  34,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "I2C4_SDA_M3, SPI4_CLK_M2, UART6_RTSN_M1, PWM0_M2");
    mraa_radxa_rock_5a_pininfo(b, 24,  35,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "I2C4_SCL_M3, SPI4_CS0_M2, UART6_CTSN_M1, PWM1_M2");
    mraa_radxa_rock_5a_pininfo(b, 25,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 26,  36,  (mraa_pincapabilities_t){1,1,0,0,1,0,0,1}, "SPI2_MISO_M0");
    mraa_radxa_rock_5a_pininfo(b, 27,  23,  (mraa_pincapabilities_t){1,1,1,0,0,1,0,0}, "PWM6_M0, UART1_RTSN_M2, PDM0_SDI0_M1, I2C6_SDA_M0, I2S1_SDI2_M1, SPI0_MISO_M0");
    mraa_radxa_rock_5a_pininfo(b, 28,  24,  (mraa_pincapabilities_t){1,1,1,0,0,1,0,0}, "PWM7_IR_M0, UART1_CTSN_M2, PDM0_SDI1_M1, I2C6_SCL_M0, I2S1_SDI3_M1, SPI3_MISO_M2");
    mraa_radxa_rock_5a_pininfo(b, 29,  42,  (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PDM1_SDI3_M1, SPI0_MOSI_M2, UART4_RX_M2");
    mraa_radxa_rock_5a_pininfo(b, 30,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 31,  41,  (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PDM1_SDI2_M1, SPI0_MISO_M2");
    mraa_radxa_rock_5a_pininfo(b, 32,  136, (mraa_pincapabilities_t){1,1,1,0,1,1,0,0}, "I2C6_SDA_M3, SPI2_CS1_M1, UART8_TX_M0, I2S1_SDI3_M0");
    mraa_radxa_rock_5a_pininfo(b, 33,  44,  (mraa_pincapabilities_t){1,1,1,0,0,0,0,0}, "PDM1_CLK0_M1, SPI0_CS0_M2, UART7_RX_M2");
    mraa_radxa_rock_5a_pininfo(b, 34,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 35,  128, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "SPI0_MISO_M1, UART9_RTSN_M1, I2S1_MCLK_M0");
    mraa_radxa_rock_5a_pininfo(b, 36,  130, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "SPI0_CLK_M1, I2S1_LRCK_M0");
    mraa_radxa_rock_5a_pininfo(b, 37,  -1,  (mraa_pincapabilities_t){0,0,0,0,0,0,0,0}, "SARADC_VIN2");
    mraa_radxa_rock_5a_pininfo(b, 38,  133, (mraa_pincapabilities_t){1,1,1,0,0,0,0,1}, "I2C3_SDA_M2, UART3_TX_M2, I2S1_SDI0_M0");
    mraa_radxa_rock_5a_pininfo(b, 39,  -1,  (mraa_pincapabilities_t){1,0,0,0,0,0,0,0}, "GND");
    mraa_radxa_rock_5a_pininfo(b, 40,  137, (mraa_pincapabilities_t){1,1,0,0,0,0,0,0}, "I2C6_SCL_M3, SPI0_CS1_M1, UART8_RX_M0, I2S1_SDO0_M0, SPDIF1_TX_M1");

    /* set leds */
    b->led_dev[0].name = (char*) radxa_rock_5a_led[0];
    b->led_dev_count   = MRAA_RADXA_ROCK_5A_LED_COUNT;

    return b;
}
