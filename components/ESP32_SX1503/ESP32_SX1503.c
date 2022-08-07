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

#include "ESP32_SX1503.h"
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>

static const char* TAG="ESP32_SX1503";

//TODO: should I check if configuration is same for skip writing or is it only 
// overhead?

// ---------------- I2C AUXILIARY FUNCTIONS ----------------

/**
 *  @brief  write an 8 bit value over I2C
 *  @param  reg
 *  @param  value
 */
static void write8(uint8_t reg, uint8_t value) {
  
    uint8_t buffer[2] = {reg, value};
    ESP_ERROR_CHECK(i2c_master_write_to_device(SX_I2C_PORT, SX1503_I2C_ADDR, buffer, 2, 1000 / portTICK_PERIOD_MS));
}

/**
 *  @brief  write a 16 bit value over I2C
 *  @param  reg
 *  @param  value
 */
static void write16(uint8_t reg, uint16_t value) {
  
    uint8_t buffer[3] = {reg, (uint8_t)(value), (uint8_t)(value >> 8)};
    ESP_ERROR_CHECK(i2c_master_write_to_device(SX_I2C_PORT, SX1503_I2C_ADDR, buffer, 3, 1000 / portTICK_PERIOD_MS));
}

/**
 *  @brief  Reads <SIZE> bytes over I2C
 *  @param  reg
 *  @param  data pointer to the data buffer
 *  @param  size number of bytes to read
 *  @return value
 */
static void read(uint8_t reg, uint8_t *data, size_t size) {
    uint8_t buffer[1] = {reg};
    ESP_ERROR_CHECK(i2c_master_write_to_device(SX_I2C_PORT, SX1503_I2C_ADDR, buffer, 1, 1000 / portTICK_PERIOD_MS));
    ESP_ERROR_CHECK(i2c_master_read_from_device(SX_I2C_PORT, SX1503_I2C_ADDR, data, size, 1000 / portTICK_PERIOD_MS));
}

/**
 * @brief Write bits in a register
 * 
 * @param reg register to write to
 * @param mask mask to apply
 * @param value value to write
 */
static void writeBits(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t tmp;
    read(reg, &tmp, 1);
    tmp &= ~mask;
    tmp |= (value & mask);
    write8(reg, tmp);
}

/*******************************************************************************
* ----------- Init/Delete, initialization functions and so-forth... ------------
*******************************************************************************/

/**
 * @brief Initialize the SX1503 driver
 * 
 * @param SX pointer to the SX1503 driver structure
 * @param irq_pin the pin connected to the IRQ pin of the SX1503
 * @param reset_pin the pin connected to the RESET pin of the SX1503
 * @return void
 */
void SX_init(ESP32_SX1503* SX, const uint8_t irq_pin, const uint8_t reset_pin){

    //Load default values
    SX->irq_pin = irq_pin;
    SX->rst_pin = reset_pin;

    //Set I2C configuration
    SX->conf.mode = I2C_MODE_MASTER;
    SX->conf.sda_io_num = SX_SDA_PIN;
    SX->conf.scl_io_num = SX_SCL_PIN;
    SX->conf.sda_pullup_en = GPIO_PULLUP_DISABLE;     //disable if you have external pullup
    SX->conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    SX->conf.master.clk_speed = 400000;               //I2C Full Speed

    ESP_ERROR_CHECK(gpio_set_direction(SX->irq_pin, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(SX->rst_pin, GPIO_MODE_INPUT));

    ESP_ERROR_CHECK(i2c_param_config(SX_I2C_PORT, &(SX->conf))); //set I2C Config
    ESP_ERROR_CHECK(i2c_driver_install(SX_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    SX->init = true;
}

/**
 * @brief Delete the SX1503
 */
void SX_deinit() {
    ESP_ERROR_CHECK(i2c_driver_delete(SX_I2C_PORT));
}

/**
 * @brief Reset the SX1503
 */
void reset(ESP32_SX1503* SX) {
    // If rst_pin is already low, it means that the SX150x is not initalized.
    if (gpio_get_level(SX->rst_pin) == 0) {
        vTaskDelay(15/portTICK_PERIOD_MS);   // Datasheet says 7ms.
        //Wait for the SX1503 to be ready
        while(gpio_get_level(SX->rst_pin) == 0) {
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
    }
    else{
        ESP_ERROR_CHECK(gpio_set_direction(SX->rst_pin, GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_level(SX->rst_pin, 0));
        vTaskDelay(1/portTICK_PERIOD_MS);   // Datasheet says 300ns.
        ESP_ERROR_CHECK(gpio_set_direction(SX->rst_pin, GPIO_MODE_INPUT));
        while(gpio_get_level(SX->rst_pin) == 0){
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
    }
}

// ----------------- GPIO BASIC FUNCTIONS -----------------

/**
 * @brief  Set the direction of a GPIO pin
 * @param  SX pointer to the SX1503 structure
 * @param  pin GPIO pin number (0-15)
 * @param  mode
 */
void SX_gpioMode(ESP32_SX1503* SX, uint8_t pin, sx1503_gpio_mode_t mode) {
    if (pin < 16) {
        uint8_t reg = (pin < 8) ? SX1503_REG_DIR_A : SX1503_REG_DIR_B;

        // Restrict to 8-bits.
        uint8_t pin_shift = pin & 0x07;
        //pin 0 becomes 0x01, pin 1 becomes 0x02, etc.
        uint8_t pin_hex = 0x01 << pin_shift;
        writeBits(reg, pin_hex, mode ? pin_hex : 0x00);
    }
    else{
        ESP_LOGE(TAG, "SX_gpioMode: pin %d out of range", pin);
    }
}

void SX_gpioPullup(ESP32_SX1503* SX, uint8_t pin, bool enable) {
    if (pin < 16) {
        uint8_t reg = (pin < 8) ? SX1503_REG_PULLUP_A : SX1503_REG_PULLUP_B;

        // Restrict to 8-bits.
        uint8_t pin_shift = pin & 0x07;
        //pin 0 becomes 0x01, pin 1 becomes 0x02, etc.
        uint8_t pin_hex = 0x01 << pin_shift;
        writeBits(reg, pin_hex, enable ? pin_hex : 0x00);
    }
    else{
        ESP_LOGE(TAG, "SX_gpioPullup: pin %d out of range", pin);
    }
}

void SX_gpioPulldown(ESP32_SX1503* SX, uint8_t pin, bool enable) {
    if (pin < 16) {
        uint8_t reg = (pin < 8) ? SX1503_REG_PULLDOWN_A : SX1503_REG_PULLDOWN_B;

        // Restrict to 8-bits.
        uint8_t pin_shift = pin & 0x07;
        //pin 0 becomes 0x01, pin 1 becomes 0x02, etc.
        uint8_t pin_hex = 0x01 << pin_shift;
        writeBits(reg, pin_hex, enable ? pin_hex : 0x00);
    }
    else{
        ESP_LOGE(TAG, "SX_gpioPulldown: pin %d out of range", pin);
    }
}


//TODO: Implement open-drain.
/**
 * @brief Set value of an Ouput GPIO
 * 
 * @param SX SX1503 structure pointer
 * @param pin pin number (0-15)
 * @param value bit value (0 or 1)
 */
void SX_digitalWrite(ESP32_SX1503* SX, uint8_t pin, bool value) {
    if (pin < 16) {
        uint8_t reg = (pin < 9) ? SX1503_REG_DATA_A : SX1503_REG_DATA_B;
        uint8_t pin_shift = pin & 0x07; // Restrict to 8-bits.
        uint8_t pin_hex = 0x01 << pin_shift; //pin 0 becomes 0x01, pin 1 becomes 0x02, etc.
        writeBits(reg, pin_hex, value ? pin_hex : 0x00);
    }
    else {
        ESP_LOGE(TAG, "SX_digitalWrite: pin %d is not a valid pin.", pin);
    }
}

/**
 * @brief Get value of an Input GPIO (0 or 1), if pin not valid, return -1;
 * @param SX SX1503 structure pointer
 * @param pin pin number (0-15)
 */
int8_t SX_digitalRead(ESP32_SX1503* SX, uint8_t pin) {
    if (pin < 16) {
        uint8_t buf[1];
        uint8_t reg = (pin < 8) ? SX1503_REG_DATA_A : SX1503_REG_DATA_B;
        pin = pin & 0x07; // Restrict to 8-bits.
        read(reg, buf, 1);
        return (buf[0] >> pin) & 0x01;
    }
    else{
        ESP_LOGE(TAG, "SX_digitalRead: pin %d is not a valid pin.", pin);
        return -1;
    }
}

/**
 * @brief  Read all the GPIO pins register and update the local copy
 * @param  SX pointer to the SX1503 structure
 * @param  buffer buffer to store the read data (pin 0-7, pin 8-15)
 */
void SX_fast_gpioRead(ESP32_SX1503* SX, uint8_t buffer[2]){
    read(SX1503_REG_DATA_B, buffer, 2);
    //reverse array before returning, REG_B is first than REG_A
    uint8_t temp = buffer[0];
    buffer[0] = buffer[1];
    buffer[1] = temp;
}

/**
 * @brief  Write all the GPIO pins register from the buffer and update the local copy
 * @param  SX pointer to the SX1503 structure
 * @param  buffer buffer to write (pin 0-7, pin 8-15)
 */
void SX_fast_gpioWrite(ESP32_SX1503* SX, uint8_t buffer[2]){
    //reverse array, REG_B is first than REG_A
    uint8_t rev_buffer[2] = {buffer[1], buffer[0]};
    write16(SX1503_REG_DATA_B, rev_buffer);
}

// ----------------- INTERRUPT FUNCTIONS -----------------

/**
 * @brief  Enable the interrupt for a GPIO **Input** pin
 * @param  SX pointer to the SX1503 structure
 * @param  pin GPIO pin number (0-15)
 * @param  mode edge sensitivity mode (none, rising, falling, both)
 */
void SX_enableInt(ESP32_SX1503* SX, uint8_t pin, sx1503_int_mode_t int_mode, bool enable) {
    if (pin < 16) {

        // Set interrupt mask
        uint8_t reg = (pin < 8) ? SX1503_REG_IRQ_MASK_A : SX1503_REG_IRQ_MASK_B;

        uint8_t pin_shift = pin & 0x07; // Restrict to 8-bits.
        //pin 0 becomes 0x01, pin 1 becomes 0x02, etc.
        uint8_t pin_hex = 0x01 << pin_shift;
        writeBits(reg, pin_hex, enable ? pin_hex : 0x00);

        // Set the interrupt mode
        if(pin<16 && pin>11){
            reg = SX1503_REG_SENSE_H_B;
        }
        else if(pin>7 && pin<12){
            reg = SX1503_REG_SENSE_L_B;
        }
        else if(pin>3 && pin <8){
            reg = SX1503_REG_SENSE_H_A;
        }
        else{
            reg = SX1503_REG_SENSE_L_A;
        }

        pin_shift = pin & 0x03; // Restrict to 4-bits.
        //two bits for every pin configuration
        pin_hex = 0x11 << pin_shift;
        writeBits(reg, pin_hex, (uint8_t)int_mode);
    }
    else{
        ESP_LOGE(TAG, "SX_enableInt: pin %d is not a valid pin.", pin);
    }
}

int8_t SX_readInt(ESP32_SX1503* SX, uint8_t pin){
    if (pin < 16) {
        uint8_t buf[1];

        // Set interrupt mask
        uint8_t reg = (pin < 8) ? SX1503_REG_IRQ_SRC_A : SX1503_REG_IRQ_SRC_B;

        uint8_t pin_shift = pin & 0x07; // Restrict to 8-bits.
        read(reg, buf, 1);
        return (buf[0] >> pin) & 0x01;
    }
    else{
        ESP_LOGE(TAG, "SX_readInt: pin %d is not a valid pin.", pin);
        return -1;
    }
}

/**
 * @brief Clear interrupt status for the selected pin.
 * It clears the pin in the REGInterruptSource and also in the RegEventStatus 
 * automatically (see DS).
 * Note that NINT bit is cleared only if all interrrupt source registers are 
 * cleared.
 * 
 * @param SX Pointer to the SX1503 structure
 * @param pin pin number (0-15)
 * 
 */
void SX_clearInt(ESP32_SX1503* SX, uint8_t pin){
    if (pin < 16) {

        // Set interrupt mask
        uint8_t reg = (pin < 8) ? SX1503_REG_IRQ_SRC_A : SX1503_REG_IRQ_SRC_B;

        uint8_t pin_shift = pin & 0x07; // Restrict to 8-bits.
        //pin 0 becomes 0x01, pin 1 becomes 0x02, etc.
        uint8_t pin_hex = 0x01 << pin_shift;
        writeBits(reg, pin_hex, pin_hex);
    }
    else{
        ESP_LOGE(TAG, "SX_clearInt: pin %d is not a valid pin.", pin);
    }
}

/**
 * @brief Fast enable/disable interrupt and set the interrupt mode for all pins
 * (write all registers at once)
 * 
 * @param SX Pointer to the SX1503 structure
 * 
 * @param bufferInt Buffer containing the interrupt enable register for all pins 
 * (pin 0 to pin 15)
 * 
 * @param bufferMode Buffer containing the interrupt mode register for all pins
 * (pin 0 to pin 15, 2 bits per pin) -> bufferMode[0] = pins 0 to 3, 
 * bufferMode[1] = pins 4 to 7, bufferMode[2] = pins 8 to 11, 
 * bufferMode[3] = pins 12 to 15.
 */
void SX_fast_enableInt(ESP32_SX1503* SX, uint8_t bufferInt[2], uint8_t bufferMode[4]){
    //reverse array, REG_B is first than REG_A
    uint8_t rev_bufferInt[2] = {bufferInt[1], bufferInt[0]};
    write16(SX1503_REG_IRQ_MASK_B, rev_bufferInt);

    //reverse and move in two arrays, reg order is REG_H_B, REG_H_A, REG_L_B, REG_L_A
    uint8_t rev_bufferMode_H[2] = {bufferMode[4], bufferMode[2]};
    uint8_t rev_bufferMode_L[2] = {bufferMode[3], bufferMode[1]};
    write16(SX1503_REG_SENSE_H_B, rev_bufferMode_H);
    write16(SX1503_REG_SENSE_L_B, rev_bufferMode_L);
}

/**
 * @brief Fast read interrupt status for all pins (read all registers at once)
 * 
 * @param SX Pointer to the SX1503 structure
 * 
 * @param buffer Buffer containing the interrupt status register for all pins
 * (pin 0 to pin 15)
 */
void SX_fast_readInt(ESP32_SX1503* SX, uint8_t buffer[2]){
    read(SX1503_REG_IRQ_SRC_B, buffer, 2);
    //reverse array before returning, REG_B is first than REG_A
    uint8_t temp = buffer[0];
    buffer[0] = buffer[1];
    buffer[1] = temp;
}

/**
 * @brief Clear interrupt status for all pins (write all registers at once).
 * It clears REGInterruptSource and also RegEventStatus automatically (see DS).
 * Note that NINT bit is cleared only if all interrrupt source registers are 
 * cleared
 * 
 * @param SX Pointer to the SX1503 structure
 * 
 */
void SX_fast_clearInt(ESP32_SX1503* SX){
    write16(SX1503_REG_IRQ_SRC_B, 0xFFFF);
}