/*
 * A simple I2C master implementation of i2c_helper.h for the MAX32655
 */

#include "i2c.h"
#include "i2c_helper.h"
#include <stdio.h>

//////////////////////////////////
// CONFIG
//////////////////////////////////

#define I2C_BUS 		MXC_I2C0
#define I2C_SPEED		100000
#define I2C_BUFFER_SIZE 32

//////////////////////////////////
// GLOBALS
//////////////////////////////////

mxc_i2c_req_t req;
uint8_t tx_buffer[I2C_BUFFER_SIZE];
uint8_t rx_buffer[I2C_BUFFER_SIZE];
int error;

//////////////////////////////////
// FUNCTIONS
//////////////////////////////////

void wipe_buffers() {
	for (int i = 0; i < I2C_BUFFER_SIZE; i++) {
		tx_buffer[i] = 0;
		rx_buffer[i] = 0;
	}
	return;
}

void i2c_init() {
	// Configure I2C bus
	MXC_I2C_Init(I2C_BUS, 1, 0);
	MXC_I2C_SetFrequency(I2C_BUS, I2C_SPEED);

	// Initialize request struct
	req.i2c = I2C_BUS;
	req.addr = 0;
	req.tx_buf = tx_buffer;
	req.rx_buf = rx_buffer;
	req.tx_len = 0;
	req.rx_len = 0;
	req.restart = 0;

	wipe_buffers();
}

uint8_t i2c_read_byte(uint8_t addr, uint8_t reg) {
	req.addr = addr;
	req.restart = 0;
	req.tx_len = 1;
	req.rx_len = 1;
	tx_buffer[0] = reg;

	if((error =MXC_I2C_MasterTransaction(&req)) != 0) {
		printf("i2c_read_byte(%d, %d) error!  Error code :%d\n", addr, reg, error);
		return 0;
	}

	return rx_buffer[0];
}

int i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* out) {
	req.addr = addr;
	req.tx_len = 1;
	req.rx_len = count;
	req.restart = 0;
	tx_buffer[0] = reg;

	// Clear output array
	for (int i = 0; i < count; i++) {
		out[i] = 0x0;
	}

	if((error = MXC_I2C_MasterTransaction(&req)) != 0) {
		printf("i2c_read_bytes(%d, %d, %d, %d) error!  Error code :%d\n", addr, reg, count, out, error);
		return 0;
	} else {
		// Write to output array
		for (int i = 0; i < count; i++) {
			out[i] = rx_buffer[i];
		}

		return 1;
	}
}

int i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t value) {
	req.addr = addr;
	req.tx_len = 2;
	req.rx_len = 0;
	req.restart = 0;
	tx_buffer[0] = reg;
	tx_buffer[1] = value;

	if((error = MXC_I2C_MasterTransaction(&req)) != 0) {
		printf("i2c_write_byte(%d, %d, %d) error!  Error code :%d\n", addr, reg, value, error);
		return 0;
	}

	return 1;
}

int i2c_write_bytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* values) {
	req.addr = addr;
	req.tx_len = count;
	req.rx_len = 0;
	req.restart = 0;
	tx_buffer[0] = reg;
	for (int i = 1; i < count+1 && i < I2C_BUFFER_SIZE; i++) {
		tx_buffer[i] = values[i - 1];
	}

	if((error = MXC_I2C_MasterTransaction(&req)) != 0) {
		printf("i2c_write_bytes(%d, %d, %d, %d) error!  Error code :%d\n", addr, reg, count, values, error);
		return 0;
	}

	return 1;
}
