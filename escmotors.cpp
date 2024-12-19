#include "escmotors.h"

Servo ESC0; 
Servo ESC1;     
Servo ESC2;     
Servo ESC3;

extern vector <float> motors;

void setupESC()
{
  initChannel(&ESC0, SERVOPIN[0]);
  initChannel(&ESC1, SERVOPIN[1]);
  initChannel(&ESC2, SERVOPIN[2]);
  initChannel(&ESC3, SERVOPIN[3]);

  /* Тестирование моторов */
#ifdef WITHESCTEST  
  testESC();
 #endif
}

void setESC()
{
  setValue(&ESC0, SERVOPIN[0], (int) (motors[0]*MAXPWMIMPULSE));
  setValue(&ESC1, SERVOPIN[1], (int) (motors[1]*MAXPWMIMPULSE));
  setValue(&ESC2, SERVOPIN[2], (int) (motors[2]*MAXPWMIMPULSE));
  setValue(&ESC3, SERVOPIN[3], (int) (motors[3]*MAXPWMIMPULSE));
}

void testESC()
{
  delay(5000);

  setValue(&ESC0, SERVOPIN[0], 200);
  delay(1000);
  setValue(&ESC0, SERVOPIN[0], 0);

  setValue(&ESC1, SERVOPIN[0], 200);
  delay(1000);
  setValue(&ESC1, SERVOPIN[0], 0);

  setValue(&ESC2, SERVOPIN[0], 200);
  delay(1000);
  setValue(&ESC2, SERVOPIN[0], 0);

  setValue(&ESC3, SERVOPIN[0], 200);
  delay(1000);
  setValue(&ESC3, SERVOPIN[0], 0);
}

void initChannel(Servo* ESC, int pin)
{
  // Attach the ESC on pin 9
  ESC->attach(pin, MINPWMIMPULSE, MAXPWMIMPULSE); // (pin, min pulse width, max pulse width in microseconds)
  delay(100);

   //potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
  int potValue = 0;
  potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  ESC->write(potValue);    // Send the signal to the ESC
  delay(500);
}

void setValue(Servo* ESC, int pin, int Value)
{
  int potValue = map(Value, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  ESC->write(potValue);    // Send the signal to the ESC

}

#ifdef DBG
void printESCCal()
{
  printDEBUG("motors:");
  printDEBUG("\t");

  printDEBUG(motors[0]);
  printDEBUG("\t");
  printDEBUG(motors[1]);
  printDEBUG("\t");
  printDEBUG(motors[2]);
  printDEBUG("\t");
  printDEBUG(motors[3]);
  printDEBUG("\t"); 
 
  printlnDEBUG("");
}
#endif