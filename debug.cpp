
#include "debug.h"

#ifdef DBG

void setupDEBUG()
{
  Serial.begin(SERIALSPEED);
  while (!Serial) {};
}

#endif