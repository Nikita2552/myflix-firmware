#include "imu.h"

MPU6500 IMU;

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;

float accelOffset[3] = {0.10, 0.09, -0.2};
float gyroOffset[3] = {4.25, -1.55, 0.75};

#ifdef MAG
  MagData magData;
#endif

void setupIMU()
{
  Wire.begin();
  Wire.setClock(SPICLOCK);  

  int err = IMU.init(calib, IMU_ADDRESS);
  
#ifdef DBG
  if (!err)
  {
    printlnDEBUG("Initializing IMU is done");
  }
  else  
  {
    printDEBUG("Error initializing IMU: ");
    printDEBUG(err);
    printlnDEBUG("");
    while (true) {};
  }
#endif
}

void configureIMU()
{
  IMU.setAccelRange(ACCEL_RANGE_4G);
  IMU.setGyroRange(GYRO_RANGE_2000DPS);
}

void readIMU()
{
  IMU.update();

  IMU.getAccel(&accelData);
  
  IMU.getGyro(&gyroData);  

#ifdef MAG
  if (IMU.hasMagnetometer())
  {
    IMU.getMag(&magData);
  }
#endif
}

void callibrateGyro()
{
  int i = 0; 

  while (i < CALLIBRATETRIES)
  {
    IMU.getGyro(&gyroData);
    gyroOffset[0] += gyroData.gyroX;
    gyroOffset[1] += gyroData.gyroY;
    gyroOffset[2] += gyroData.gyroZ;
    i++;
  }  

  gyroOffset[0] /= CALLIBRATETRIES;
  gyroOffset[1] /= CALLIBRATETRIES;
  gyroOffset[2] /= CALLIBRATETRIES;  
}

void callibrateAccel()
{
  
}

void callibrateAccel_old()
{
  int i = 0;
  while (i < CALLIBRATETRIES)
  {
    IMU.getAccel(&accelData);
    accelOffset[0] += accelData.accelX;
    accelOffset[1] += accelData.accelY;
    accelOffset[2] += accelData.accelZ;
    i++;
  }

    accelOffset[0] /= CALLIBRATETRIES;
    accelOffset[1] /= CALLIBRATETRIES;
    accelOffset[2] /= CALLIBRATETRIES;
}

void callibrateAccelOnce()
{

}

float translateRegDatatoAngle_Gyro(float regdata)
{
  return regdata/32768*GYRO_RANGE_2000DPS;
}

float translateRegDatatoAngle_Accel(float regdata)
{
  return regdata/32768*ACCEL_RANGE_4G;
}

#ifdef DBG
void printIMUCal()
{ 
  printDEBUG("Gyro::");
  printDEBUG(" X:");
  printDEBUG(gyroData.gyroX);
  printDEBUG(" Y:");
  printDEBUG(gyroData.gyroY);
  printDEBUG(" Z:");
  printDEBUG(gyroData.gyroZ);
  printDEBUG("; Accel::X:");
  printDEBUG(accelData.accelX);
  printDEBUG(", Y:");
  printDEBUG(accelData.accelY);
  printDEBUG(", Z:");
  printDEBUG(accelData.accelZ);

// #ifdef MAG  
//   if (IMU.hasMagnetometer()) {
//     printDEBUG("\t");
//     printDEBUG(magData.magX);
//     printDEBUG("\t");
//     printDEBUG(magData.magY);
//     printDEBUG("\t");
//     printDEBUG(magData.magZ);
//   }
// #endif
// #ifdef TEMP
//   if (IMU.hasTemperature()) {
// 	  printDEBUG("\t");
// 	  printDEBUG(IMU.getTemp());
//   }
// #endif

//   printDEBUG(" Offsets:");
//   printDEBUG("Gyro::X:");
//   printDEBUG(gyroOffset[0]);
//   printDEBUG(", Y:");
//   printDEBUG(gyroOffset[1]);
//   printDEBUG(", Z:");
//   printDEBUG(gyroOffset[2]);
//   printDEBUG("; Accel::X:");
//   printDEBUG(accelOffset[0]);
//   printDEBUG(", Y:");
//   printDEBUG(accelOffset[1]);
//   printDEBUG(", Z:");
//   printDEBUG(accelOffset[2]);

 printDEBUG(" ");
}
#endif