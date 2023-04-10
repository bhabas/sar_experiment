#ifndef SHARED_LIB_H
#define SHARED_LIB_H


// CF LIBARARIES
#include "math3d.h"
#include "stabilizer_types.h"


struct GTC_CmdPacket{
    uint8_t cmd_type; 
    float cmd_val1;
    float cmd_val2;
    float cmd_val3;
    float cmd_flag;
    bool  cmd_rx;
} __attribute__((packed));

extern struct GTC_CmdPacket GTC_Cmd;

extern float value_1;
extern float value_2;


#endif /* SHARED_VARIABLES_H */
