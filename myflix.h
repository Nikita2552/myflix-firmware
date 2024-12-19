#ifndef MYFLIX_H
#define MYFLIX_H

#include "globals.h"
#include "imu.h"
#include "looptimer.h"
#include "wifiaccesspoint.h"
#include "escmotors.h"
#include "estimate.h"
#include "control.h"

#ifdef DBG
  #include "debug.h"
#endif

void setup();
void loop();

#endif