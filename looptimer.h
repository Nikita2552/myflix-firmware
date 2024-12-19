#ifndef LOOPTIMER_H
#define LOOPTIMER_H

#include <Timer.h>
#include "debug.h";

void startTimer();
void stopTimer();

void getTime();
void updateTimerLoop();

#ifdef DBG
void printTimerCal();
#endif

#endif;