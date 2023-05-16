#pragma once

#include <stdint.h>
#include "stdio.h"

#include "cpx.h"
#include "pmsis.h"

#include "bsp/bsp.h"
#include "bsp/camera/himax.h"
#include "bsp/buffer.h"

#define MS_2_US 1000
#define IMG_ORIENTATION 0x0101
#define CAM_WIDTH 40
#define CAM_HEIGHT 40

typedef enum {
  WIFI_CTRL_SET_SSID                = 0x10,
  WIFI_CTRL_SET_KEY                 = 0x11,

  WIFI_CTRL_WIFI_CONNECT            = 0x20,

  WIFI_CTRL_STATUS_WIFI_CONNECTED   = 0x31,
  WIFI_CTRL_STATUS_CLIENT_CONNECTED = 0x32,
} __attribute__((packed)) WiFiCTRLType;

typedef struct {
  WiFiCTRLType cmd;
  uint8_t data[50];
} __attribute__((packed)) WiFiCTRLPacket_t;


static CPXPacket_t txp; // Transfer packet
static CPXPacket_t rxp; // Receive packet

void setupWiFi(void) {
    static char ssid[] = "WiFi streaming example";
    printf("Setting up WiFi AP\n");
    // Set up the routing for the WiFi CTRL packets
    txp.route.destination = CPX_T_ESP32;
    rxp.route.source = CPX_T_GAP8;
    txp.route.function = CPX_F_WIFI_CTRL;
    WiFiCTRLPacket_t * wifiCtrl = (WiFiCTRLPacket_t*) txp.data;
    
    wifiCtrl->cmd = WIFI_CTRL_SET_SSID;
    memcpy(wifiCtrl->data, ssid, sizeof(ssid));
    txp.dataLength = sizeof(ssid);
    cpxSendPacketBlocking(&txp);
    
    wifiCtrl->cmd = WIFI_CTRL_WIFI_CONNECT;
    wifiCtrl->data[0] = 0x01;
    txp.dataLength = 2;
    cpxSendPacketBlocking(&txp);
}


static int wifiConnected = 0;
static int wifiClientConnected = 0;

void rx_task(void *parameters)
{
  while (1)
  {
    cpxReceivePacketBlocking(CPX_F_WIFI_CTRL, &rxp);

    WiFiCTRLPacket_t * wifiCtrl = (WiFiCTRLPacket_t*) rxp.data;

    switch (wifiCtrl->cmd)
    {
      case WIFI_CTRL_STATUS_WIFI_CONNECTED:
        printf("Wifi connected (%u.%u.%u.%u)\n",
                          wifiCtrl->data[0], wifiCtrl->data[1],
                          wifiCtrl->data[2], wifiCtrl->data[3]);
        wifiConnected = 1;
        break;
      case WIFI_CTRL_STATUS_CLIENT_CONNECTED:
        printf("Wifi client connection status: %u\n", wifiCtrl->data[0]);
        wifiClientConnected = wifiCtrl->data[0];
        break;
      default:
        break;
    }
  }
}



typedef struct
{
  uint8_t magic;
  uint16_t width;
  uint16_t height;
  uint8_t depth;
  uint8_t type;
  uint32_t size;
} __attribute__((packed)) img_header_t;



pi_buffer_t header;
uint32_t headerSize;
pi_buffer_t footer;
uint32_t footerSize;


void createImageHeaderPacket(CPXPacket_t * packet, uint32_t imgSize) {
  img_header_t *imgHeader = (img_header_t *) packet->data;
  imgHeader->magic = 0xBC;
  imgHeader->width = CAM_WIDTH;
  imgHeader->height = CAM_HEIGHT;
  imgHeader->depth = 1;
  imgHeader->type = 0;
  imgHeader->size = imgSize;
  packet->dataLength = sizeof(img_header_t);
}

void sendBufferViaCPX(CPXPacket_t * packet, uint8_t * buffer, uint32_t bufferSize) {
  uint32_t offset = 0;
  uint32_t size = 0;
  do {
    size = sizeof(packet->data);
    if (offset + size > bufferSize)
    {
      size = bufferSize - offset;
    }
    memcpy(packet->data, &buffer[offset], sizeof(packet->data));
    packet->dataLength = size;
    cpxSendPacketBlocking(packet);
    offset += size;
  } while (size == sizeof(packet->data));
}
