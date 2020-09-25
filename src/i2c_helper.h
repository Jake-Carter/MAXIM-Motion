/*
 * Generic i2c helper functions.
 */

#ifndef I2C_HELPER_H
#define I2C_HELPER_H

#include <stdint.h>

void i2c_init();
uint8_t i2c_read_byte(uint8_t addr, uint8_t reg);
int i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* out);
int i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t value);
int i2c_write_bytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* values);

#endif
