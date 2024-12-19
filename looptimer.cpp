
#include "looptimer.h"

Timer *timer = new Timer(MICROS);

float now = 0;
float t = 0;
float dt = 1/15600;
float loopFreq; // loop frequency, Hz

void startTimer()
{
  timer->start();
}

void stopTimer()
{
  timer->stop();
}

void getTime()
{  
  float now = micros() / MICROSECLIMIT;
	dt = now - t;
	t = now;

	if (!(dt > 0)) {
		dt = 0; // assume dt to be zero on first step and on reset
	}
  updateTimerLoop();
}

void updateTimerLoop()
{
  static float windowStart = 0;
	static uint32_t freq = 0;
	freq++;
	if (t - windowStart >= 1) { // 1 second window
		loopFreq = freq;
		windowStart = t;
		freq = 0;
	}
}

#ifdef DBG
void printTimerCal()
{
  printDEBUG("timer::");
  printDEBUG(" dt: ");
  //printDEBUG(dt);
  Serial.print(dt, 8);
    
  printDEBUG(" ");
}
#endif