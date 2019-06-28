#include "i2c.h"

int i2c_read(i2c_port_t port, int8_t dev_addr, uint8_t reg_addr, int count, uint8_t buf[]) {
    if (count == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // Start read transaction, send address byte
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_WRITE, I2C_CHECK_ACK);
    // Write word byte
    i2c_master_write_byte(cmd, reg_addr, I2C_CHECK_ACK);
    i2c_master_start(cmd);
    // Send address byte again
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_READ, I2C_CHECK_ACK);
    // Actually do read
    if (count > 1) {
        i2c_master_read(cmd, buf, count - 1, I2C_ACK_VAL);
    }
    i2c_master_read_byte(cmd, buf + count - 1, I2C_NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int i2c_write(i2c_port_t port, int8_t dev_addr, uint8_t reg_addr, int count, uint8_t buf[]) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // Begin write transaction
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_WRITE, I2C_CHECK_ACK);
    // Set address
    i2c_master_write_byte(cmd, reg_addr, I2C_CHECK_ACK);
    // Write
    i2c_master_write(cmd, buf, count, I2C_CHECK_ACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1);
    i2c_cmd_link_delete(cmd);
    return ret;
}