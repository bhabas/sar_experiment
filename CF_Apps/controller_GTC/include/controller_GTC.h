/**
 *
 * controller_gtc.h - Geometric Tracking Controller Interface
 *
 */

#ifndef __CONTROLLER_GTC_H__
#define __CONTROLLER_GTC_H__

#ifdef GAZEBO_SIM
#define consolePrintf printf
#define DEBUG_PRINT printf
#endif

// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>


// CF LIBARARIES
#include "math3d.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"
#include "led.h"
#include "app.h"
#include "app_channel.h"

#include "controller.h"

// CUSTOM LIBRARIES
#include "shared_lib.h"

#define DEBUG_MODULE "GTC_CONTROLLER"






#endif //__CONTROLLER_GTC_H__
