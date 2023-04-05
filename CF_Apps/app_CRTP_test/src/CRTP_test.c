#include "app.h"
#include "app_channel.h"


#include "debug.h"
#define DEBUG_MODULE "CRTP_TEST"


struct testPacketRX{
  float x;
  float y;
  float z;
} __attribute__((packed));

struct testPacketTX{
  float sum;
} __attribute__((packed));

void appMain() {
  DEBUG_PRINT("Waiting for activation... \n");

  struct testPacketRX rxPacket;
  struct testPacketTX txPacket;
  
  while(1)
  {
    if (appchannelReceiveDataPacket(&rxPacket,sizeof(rxPacket),APPCHANNEL_WAIT_FOREVER))
    {
      DEBUG_PRINT("App channel received x: %f, y: %f, z:%f\n",(double)rxPacket.x, (double)rxPacket.y, (double)rxPacket.z);

      txPacket.sum = rxPacket.x + rxPacket.y + rxPacket.z;

      appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
    }
    
  }
  
}
