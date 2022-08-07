/*

SX150x - A library for the Semtech SX150x family of i2c GPIO expanders.
Copyright (c) 2019 J. Ian Lindsay

SX150x ESP-IDF library porting.
Copyright (c) 2022 Simone Tollardo <simonetollardo@gmail.com>

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

This permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

*/

#ifndef __ESP32_SX1503_H__
#define __ESP32_SX1503_H__

#include <stdint.h>
#include "driver/i2c.h"
#include <esp_err.h>

extern uint8_t SX_SDA_PIN;
extern uint8_t SX_SCL_PIN;
extern uint8_t SX_I2C_PORT;

//TODO: implement PLD (Programmable Logic Functions)

#define SX1503_I2C_ADDR             0x20  // Not configurable for SX1503.

/* Register addresses */
#define SX1503_REG_DATA_B           0x00  //
#define SX1503_REG_DATA_A           0x01  //
#define SX1503_REG_DIR_B            0x02  //
#define SX1503_REG_DIR_A            0x03  //
#define SX1503_REG_PULLUP_B         0x04  //
#define SX1503_REG_PULLUP_A         0x05  //
#define SX1503_REG_PULLDOWN_B       0x06  //
#define SX1503_REG_PULLDOWN_A       0x07  //
#define SX1503_REG_IRQ_MASK_B       0x08  //
#define SX1503_REG_IRQ_MASK_A       0x09  //
#define SX1503_REG_SENSE_H_B        0x0A  //
#define SX1503_REG_SENSE_H_A        0x0B  //
#define SX1503_REG_SENSE_L_B        0x0C  //
#define SX1503_REG_SENSE_L_A        0x0D  //
#define SX1503_REG_IRQ_SRC_B        0x0E  //
#define SX1503_REG_IRQ_SRC_A        0x0F  //
#define SX1503_REG_EVENT_STAT_B     0x10  //
#define SX1503_REG_EVENT_STAT_A     0x11  //
#define SX1503_REG_PLD_MODE_B       0x20  //
#define SX1503_REG_PLD_MODE_A       0x21  //
#define SX1503_REG_PLD_TABLE_0B     0x22  //
#define SX1503_REG_PLD_TABLE_0A     0x23  //
#define SX1503_REG_PLD_TABLE_1B     0x24  //
#define SX1503_REG_PLD_TABLE_1A     0x25  //
#define SX1503_REG_PLD_TABLE_2B     0x26  //
#define SX1503_REG_PLD_TABLE_2A     0x27  //
#define SX1503_REG_PLD_TABLE_3B     0x28  //
#define SX1503_REG_PLD_TABLE_3A     0x29  //
#define SX1503_REG_PLD_TABLE_4B     0x2A  //
#define SX1503_REG_PLD_TABLE_4A     0x2B  //
#define SX1503_REG_ADVANCED         0xAD  //

typedef enum {
    SX_MODE_OUTPUT,
    SX_MODE_INPUT
} sx1503_gpio_mode_t;

typedef enum {
    SX_INT_NONE,
    SX_INT_RISING,
    SX_INT_FALLING,
    SX_INT_BOTH
} sx1503_int_mode_t;

/**
 *  @brief  Struct that stores the states of the SX1503.
 */
typedef struct{
    i2c_config_t conf;
    bool init;
    bool autoclear_on_read;
    bool boost_mode;
    uint8_t irq_pin;
    uint8_t rst_pin;
} ESP32_SX1503;


void SX_init(ESP32_SX1503* SX, uint8_t irq_pin, uint8_t reset_pin);
void SX_deinit();
void SX_reset(ESP32_SX1503* SX);

// Basic usage as pins...
void SX_gpioMode(ESP32_SX1503* SX, uint8_t pin, sx1503_gpio_mode_t mode);
void SX_gpioPullup(ESP32_SX1503* SX, uint8_t pin, bool enable);
void SX_gpioPulldown(ESP32_SX1503* SX, uint8_t pin, bool enable);
void SX_digitalWrite(ESP32_SX1503* SX, uint8_t pin, bool value);
int8_t SX_digitalRead(ESP32_SX1503* SX, uint8_t pin);

// Fast GPIO write/read functions (write/read all pins at once)
void SX_fast_gpioRead(ESP32_SX1503* SX, uint8_t buffer[2]);
void SX_fast_gpioWrite(ESP32_SX1503* SX, uint8_t buffer[2]);

// Interrupt functions
void SX_enableInt(ESP32_SX1503* SX, uint8_t pin, sx1503_int_mode_t int_mode, bool enable);
int8_t SX_readInt(ESP32_SX1503* SX, uint8_t pin);
void SX_clearInt(ESP32_SX1503* SX, uint8_t pin);

// Fast interrupt functions (write/read all pins at once)
void SX_fast_enableInt(ESP32_SX1503* SX, uint8_t bufferInt[2], uint8_t bufferMode[4]);
void SX_fast_readInt(ESP32_SX1503* SX, uint8_t buffer[2]);
void SX_fast_clearInt(ESP32_SX1503* SX);

// Others
// void SX_enableBoostMode(ESP32_SX1503* SX, bool enable);
// void SX_setAutoClearOnRead(ESP32_SX1503* SX, bool enable);
// void SX_setPLD();  // TODO: Define API for this feature.

#endif   // __SX1503_DRIVER_H__
