#ifndef IMU_H
#define IMU_H

#include "FastIMU.h"
#include <Wire.h>
#include "globals.h"

#ifdef DBG
  #include "debug.h"
#endif

void setupIMU();
void configureIMU();
void readIMU();

void callibrateGyro();
void callibrateAccel();
void callibrateAccel_old();
void callibrateAccelOnce();

float translateRegDatatoAngle_Gyro(float regdata);
float translateRegDatatoAngle_Accel(float regdata);

#ifdef DBG
void printIMUCal();
#endif

#endif