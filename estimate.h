#ifndef ESTIM_H
#define ESTIM_H

#include <math.h>
#include "imu.h"
#include "globals.h"
#include "looptimer.h"
#include "lowpassfilter.h"
#include "arrayfilter.h"
#include "quaterniondigit.h"
#include "debug.h"

using namespace quatlib;

typedef enum
{
    LPFENABLE = true,
    LPFDISABLE  = false
} LPFSWITCH;

void setupEST();
void setEST();

void setAccel();
void setGyro();
void setGyroSec();
void result();
void resultSec();
void callibrate();

#ifdef DBG
void printESTCal();
#endif

#endif
