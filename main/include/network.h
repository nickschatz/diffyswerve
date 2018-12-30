#pragma once

#define AP_SSID "GopherSwerve"
#define AP_PASS "12345"
#define PORT_NUMBER 4357
#define NET_TAG "NETWORK"

#define PACKET_LEN 4
typedef struct {
    char commandx;
    char commandy;
    char commandrot;
    char commandother;
    TickType_t timestamp;
} Packet;

QueueHandle_t xNetQueue;

void init_ap();

void vTaskServer(void * params);