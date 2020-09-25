/*
 * A simple I2C master implementation of i2c_helper.h for the MAX32666
 */

#include "i2c.h"
#include "i2c_helper.h"
#include <stdio.h>

//////////////////////////////////
// CONFIG
//////////////////////////////////

#define I2C_BUS 		MXC_I2C0_BUS0
#define I2C_SPEED		I2C_STD_MODE

//////////////////////////////////
// i2c_helper.h FUNCTIONS
//////////////////////////////////

void i2c_init() {
	// Configure I2C bus
	I2C_Shutdown(I2C_BUS);
	I2C_Init(I2C_BUS, I2C_SPEED, 0);
	NVIC_EnableIRQ(I2C0_IRQn);
}

uint8_t i2c_read_byte(uint8_t addr, uint8_t reg) {
	uint8_t tx = reg;
	uint8_t rx = 0x0;

	int w_status = I2C_MasterWrite(I2C_BUS, (addr << 1), &tx, 1, 1); // Write the register addr to the slave w/ a repeated start
	int r_status = I2C_MasterRead(I2C_BUS, (addr << 1), &rx, 1, 0); // Read 1 byte from the slave

	return rx;
}

int i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* out) {
	uint8_t tx = reg;
	for (int i = 0; i < count; i++) {
		out[i] = 0x0;
	}

	int w_status = I2C_MasterWrite(I2C_BUS, (addr << 1), &tx, 1, 1); // Write the register addr to the slave w/ a repeated start
	int r_status = I2C_MasterRead(I2C_BUS, (addr << 1), out, count, 0); // Read [count] bytes from the slave

	if (w_status != 1 || r_status != count) { return 0; }
	else { return 1; }
}

int i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t value) {
	// Write the register address, then the byte value
	uint8_t tx[2] = { reg, value };
	int w_status = I2C_MasterWrite(I2C_BUS, (addr << 1), tx, 2, 0);

	if (w_status != 2) { return 0; }
	else { return 1; }
}

int i2c_write_bytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* values) {
	// Insert the register addr to the beginning of the byte sequence, then write the byte values
	uint8_t tx[1 + count];
	tx[0] = reg;
	for (int i = 1; i < 1 + count; i++) {
		tx[i] = values[i - 1];
	}

	int w_status = I2C_MasterWrite(I2C_BUS, (addr << 1), tx, 1 + count, 0);

	if (w_status != 1 + count) { return 0; }
	else { return 1; }
}
