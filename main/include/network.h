#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdlib.h>

#define AP_SSID "GopherSwerve"
#define AP_PASS "12345"
#define PORT_NUMBER 4357
#define NET_TAG "NETWORK"

#define PACKET_LEN 4
typedef struct {
    int8_t commandx;
    int8_t commandy;
    int8_t commandrot;
    int8_t commandother;
    TickType_t timestamp;
} Packet;

QueueHandle_t xNetQueue;

void init_ap();

void vTaskServer(void * params);