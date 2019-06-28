#pragma once

#include "driver/i2c.h"

#define I2C_WRITE 0
#define I2C_READ 1
#define I2C_CHECK_ACK 0
#define I2C_ACK_VAL 0x0
#define I2C_NACK_VAL 0x1

/**
 * Read `count` registers starting at `address` and put the results into `buf`
 */
int i2c_read(i2c_port_t port, int8_t dev_addr, uint8_t reg_addr, int count, uint8_t * buf);

/**
 * Write `count` registers starting at `address` using the contents of `buf`
 */
int i2c_write(i2c_port_t port, int8_t dev_addr, uint8_t reg_addr, int count, uint8_t * buf);