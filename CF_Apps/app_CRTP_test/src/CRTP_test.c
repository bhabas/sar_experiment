#include "app.h"
#include "app_channel.h"


#include "debug.h"
#define DEBUG_MODULE "CRTP_TEST"


struct testPacketRX{
    uint16_t cmd_id;
    uint8_t cmd_type; 
    float cmd_val1;
    float cmd_val2;
    float cmd_val3;
    float cmd_flag;
} __attribute__((packed));

void appMain() {
  DEBUG_PRINT("Waiting for activation... \n");

  struct testPacketRX rxPacket;
  
  while(1)
  {
    if (appchannelReceiveDataPacket(&rxPacket,sizeof(rxPacket),APPCHANNEL_WAIT_FOREVER))
    {
        // uint8_t* rxPacketBytes = (uint8_t*)&rxPacket;
        // for (size_t i = 0; i < sizeof(rxPacket); i++) {
        //         consolePrintf("%02X", rxPacketBytes[i]); // Print each byte in hexadecimal format
        //     }
        // consolePrintf("\n");

        consolePrintf("App channel received x: %.3f\n",(double)rxPacket.cmd_val3);

    }
    
  }
  
}
