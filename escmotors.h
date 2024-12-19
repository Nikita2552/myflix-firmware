#ifndef ESC_H
#define ESC_H

#include <ESP32Servo.h>
#include "control.h"
#include "globals.h"
#include "debug.h"

void setupESC();
void setESC();
void testESC();

void initChannel(Servo* ESC, int pin);
void setValue(Servo* ESC, int pin, int Value);

#ifdef DBG
void printESCCal();
#endif

#endif