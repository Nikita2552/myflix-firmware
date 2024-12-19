
#include "estimate.h"

extern AccelData accelData;    //Sensor data
extern GyroData gyroData;

extern float accelOffset[];
extern float gyroOffset[];

QuaternionDigit speedAngle;
QuaternionDigit speedAngleSec;
QuaternionDigit accelAngle;

vector<float> lpfgyroData;
vector<float> rates;
LowpassFilter<float> lpfX(LPFCOEFF, LPFENABLE);
LowpassFilter<float> lpfY(LPFCOEFF, LPFENABLE);
LowpassFilter<float> lpfZ(LPFCOEFF, LPFENABLE);

ArrayFilter<float> arsecX(1000);
ArrayFilter<float> arsecY(1000);
ArrayFilter<float> arsecZ(1000);

vector<float> resultAngle{0, 0, 1};
vector<float> resultAngleSec{1, 0, 0};
vector<float> resultAngleSen{1, 0, 0};
vector<float> resultAngleArr{1, 0, 0};

extern float dt;

void setupEST()
{
  speedAngle = converttoquat(resultAngle);
  speedAngleSec = converttoquat(resultAngleSec);
}

/*estimate - комплиментарный фильтр */
void setEST()
{
   setGyro();
   setAccel();
   setGyroSec();
   result();
   resultSec();  
}

void setGyro()
{
  rates.clear();
  rates.push_back(lpfX.update(gyroData.gyroX - gyroOffset[0]));
  rates.push_back(lpfY.update(gyroData.gyroY - gyroOffset[1]));
  rates.push_back(lpfZ.update(gyroData.gyroZ - gyroOffset[2]));

  lpfgyroData.clear();
  lpfgyroData.push_back(dt * rates[0]);
  lpfgyroData.push_back(dt * rates[1]);
  lpfgyroData.push_back(dt * rates[2]);  

  qtoangle(&speedAngle, lpfgyroData);
  speedAngle.normalize();
}

void setGyroSec()
{
  qtoangle(&speedAngleSec, lpfgyroData);
  speedAngleSec.normalize();  
}

void setAccel()
{
  accelAngle.setW(0);
  accelAngle.setX(accelData.accelX - accelOffset[0]);
  accelAngle.setY(accelData.accelY - accelOffset[1]);
  accelAngle.setZ(-(accelData.accelZ - accelOffset[2]));
  accelAngle.normalize();  

  speedAngle = correctgyrofromaccel(speedAngle, accelAngle);
  speedAngle.normalize(); 
}

void result()
{
  resultAngle = converttovect(speedAngle);  
}

void resultSec()
{  
  resultAngleSen = converttovect(speedAngleSec);

  //Фильтры не работают. Подумать ещё!
  resultAngleArr = floorceil(resultAngleSec, resultAngleSen);

  resultAngleSec.clear();
  resultAngleSec.push_back(arsecX.update(resultAngleArr[0]));
  resultAngleSec.push_back(arsecY.update(resultAngleArr[1]));
  resultAngleSec.push_back(arsecZ.update(resultAngleArr[2]));
}

void callibrate()
{
  callibrateGyro();
  callibrateAccel_old();
}

#ifdef DBG
void printESTCal()
{ 
  // printDEBUG(" lpfgyroData::X:");
  // printDEBUG(lpfgyroData[0]);  
  // printDEBUG(",Y:");
  // printDEBUG(lpfgyroData[1]);    
  // printDEBUG(",Z:");
  // printDEBUG(lpfgyroData[2]);
  
  // printDEBUG(" accelAngle::X:");
  // printDEBUG(accelAngle.getX());
  // printDEBUG(",Y:");
  // printDEBUG(accelAngle.getY());
  // printDEBUG(",Z:");
  // printDEBUG(accelAngle.getZ());
  
  printDEBUG(" resultAngle::X:");
  printDEBUG(resultAngle[0]);
  printDEBUG(", Y:");
  printDEBUG(resultAngle[1]);
  printDEBUG(", Z:");
  printDEBUG(resultAngle[2]);

  // printDEBUG(" resultAngleSen::X:");
  // printDEBUG(resultAngleSen[0]);
  // printDEBUG(", Y:");
  // printDEBUG(resultAngleSen[1]);
  // printDEBUG(", Z:");
  // printDEBUG(resultAngleSen[2]);

  // printDEBUG(" resultAngleArr::X:");
  // printDEBUG(resultAngleArr[0]);
  // printDEBUG(", Y:");
  // printDEBUG(resultAngleArr[1]);
  // printDEBUG(", Z:");
  // printDEBUG(resultAngleArr[2]); 

  // printDEBUG(" resultAngleSec::X:");
  // printDEBUG(resultAngleSec[0]);
  // printDEBUG(", Y:");
  // printDEBUG(resultAngleSec[1]);
  // printDEBUG(", Z:");
  // printDEBUG(resultAngleSec[2]);  

  // printlnDEBUG("");
}
#endif