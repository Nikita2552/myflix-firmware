
#include "myflix.h"

void setup()
{
#ifdef DBG
  setupDEBUG();
#endif
  
  setupIMU();
  setupEST();
  setupCONT();
  setupESC();

#ifdef WIFI
  setupWIFI();
#endif

  startTimer();
}

void loop()
{
  getTime();
  
  readIMU();
  setEST();
  setCONT();
  setESC();

#ifdef WIFI
  readWIFI();
#endif 

#ifdef DBG
  //printTimerCal();
  //printIMUCal();
  //printESTCal();
  //printCONTCal();
  printESCCal();  
  //printlnDEBUG("hello world");
#endif
}
