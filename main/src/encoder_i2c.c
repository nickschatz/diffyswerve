#include "encoder_i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

SemaphoreHandle_t i2cAccSemaphore = NULL;

void encoder_i2c_init() {
    i2cAccSemaphore = xSemaphoreCreateMutex();
}

void encoder_configure(Encoder encoder, encoder_conf settings) {
    uint8_t confl = settings.pm & 0b11;
    confl |= (settings.hyst & 0b11) << 2;
    confl |= (settings.outs & 0b11) << 4;
    confl |= (settings.pwmf & 0b11) << 6;
    uint8_t confh = settings.sf & 0b11;
    confh |= (settings.fth & 0b111) << 2;
    confh |= (settings.wd & 0b1) << 5;
    uint8_t buf[] = {confh, confl};
    int err = encoder_write(encoder, ENCODER_REGISTER_CONF, 2, buf);
    if (err != ESP_OK) {
        printf("Encoder %d: Error writing conf: %s\n", encoder.mux_channel, esp_err_to_name(err));
    }
}

float encoder_read_angle(Encoder encoder) {
    uint8_t buf[2];
    int err = encoder_read(encoder, ENCODER_REGISTER_ANGLE, 2, buf);
    if (err != ESP_OK) {
        printf("Encoder %d: Error reading angle: %s\n", encoder.mux_channel, esp_err_to_name(err));
    }
    short angle_int = (buf[0] << 8) | buf[1];
    return 2 * M_PI * angle_int / 4096;
}

int encoder_read_status(Encoder encoder) {
    uint8_t buf[1];
    buf[0] = 0;
    int err = encoder_read(encoder, ENCODER_REGISTER_STATUS, 1, buf);
    if (err != ESP_OK) {
        printf("Encoder %d: Error reading status: %s\n", encoder.mux_channel, esp_err_to_name(err));
    }
    if ((buf[0] >> 3) & 1) { // MH
        return ENCODER_MAGNET_CLOSE;
    }
    else if ((buf[0] >> 4) & 1) { // ML
        return ENCODER_MAGNET_FAR;
    }
    else if ((buf[0] >> 5) & 1) { // MD
        return ENCODER_MAGNET_OK;
    }
    return -1;
}

int encoder_read_agc(Encoder encoder) {
    uint8_t buf[1];
    buf[0] = 0;
    int err = encoder_read(encoder, ENCODER_REGISTER_AGC, 1, buf);
    if (err != ESP_OK) {
        printf("Encoder %d: Error reading AGC: %s\n", encoder.mux_channel, esp_err_to_name(err));
    }
    return buf[0];
}

void encoder_set_hysteresis(Encoder encoder, char hyst_mode) {
    // Read conf register
    uint8_t conf[2];
    encoder_read(encoder, ENCODER_REGISTER_CONF, 2, conf);
    // Set bits
    if (hyst_mode & 1) {
        conf[1] |= 1<<2; 
    }
    else {
        conf[1] &= ~(1<<2);
    }
    if ((hyst_mode>>1) & 1) {
        conf[1] |= 1<<3; 
    }
    else {
        conf[1] &= ~(1<<3);
    }
    // Write conf register back
    encoder_write(encoder, ENCODER_REGISTER_CONF, 2, conf);
}

int encoder_read(Encoder encoder, uint8_t reg_addr, int count, uint8_t * buf) {
    if (xSemaphoreTake(i2cAccSemaphore, 5) == pdTRUE) {
        mux_select_encoder(encoder);
        int ret = i2c_read(encoder.port, ENCODER_ADDR, reg_addr, count, buf);
        xSemaphoreGive(i2cAccSemaphore);
        return ret;
    }
    else {
        return ESP_ERR_TIMEOUT;
    }
}

int encoder_write(Encoder encoder, uint8_t reg_addr, int count, uint8_t * buf) {
    if (xSemaphoreTake(i2cAccSemaphore, 5) == pdTRUE) {
        mux_select_encoder(encoder);
        int ret = i2c_write(encoder.port, ENCODER_ADDR, reg_addr, count, buf);
        xSemaphoreGive(i2cAccSemaphore);
        return ret;
    }
    else {
        return ESP_ERR_TIMEOUT;
    }
}

void mux_select_encoder(Encoder encoder) {
    // Select encoder on mux
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // Begin write transaction
    i2c_master_write_byte(cmd, (MUX_ADDR << 1) | I2C_WRITE, I2C_CHECK_ACK);
    // Set mux
    i2c_master_write_byte(cmd, 1 << encoder.mux_channel, I2C_CHECK_ACK);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(encoder.port, cmd, 1);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        printf("Error writing mux select: %s\n", esp_err_to_name(ret));
    }
}