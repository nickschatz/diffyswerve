#pragma once

#include "i2c.h"

#define ENCODER_ADDR 0x36
#define ENCODER_MAGNET_OK 0
#define ENCODER_MAGNET_CLOSE 1
#define ENCODER_MAGNET_FAR 2

#define MUX_ADDR 0x70

// Registers
// CONF is two wide
#define ENCODER_REGISTER_CONF 0x07
// ANGLE is two wide
#define ENCODER_REGISTER_ANGLE 0x0E
#define ENCODER_REGISTER_STATUS 0x0B
#define ENCODER_REGISTER_AGC 0x1A

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

typedef struct {
    i2c_port_t port;
    uint8_t mux_channel;
} Encoder;

typedef struct {
    char pm;
    char hyst;
    char outs;
    char pwmf;
    char sf;
    char fth;
    char wd;
} encoder_conf;

/**
 * Write a configuration to the encoder
 */
void encoder_configure(Encoder encoder, encoder_conf settings);

/**
 * Returns the angle of the sensor from 0 to 2pi
 */
float encoder_read_angle(Encoder encoder);

/**
 * Returns the status of the encoder (OK, CLOSE, or FAR)
 */
int encoder_read_status(Encoder encoder);

/**
 * Returns the value of the AGC. The AGC should be in the middle of its range (~64 for 3.3v)
 */
int encoder_read_agc(Encoder encoder);

/**
 * Set the hysteresis mode
 */
void encoder_set_hysteresis(Encoder encoder, char hyst_mode);

/**
 * Read `count` registers starting at `address` and put the results into `buf`
 */
int encoder_read(Encoder encoder, uint8_t reg_addr, int count, uint8_t * buf);

/**
 * Write `count` registers starting at `address` using the contents of `buf`
 */
int encoder_write(Encoder encoder, uint8_t reg_addr, int count, uint8_t * buf);

void mux_select_encoder(Encoder encoder);

void encoder_i2c_init();