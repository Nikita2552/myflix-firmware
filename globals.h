#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

/* Блок глобальных настроек */
#define DBG

/* Настройки SERIAL */
#define SERIALSPEED 115200

/* Настройки IMU */
#define SPICLOCK 400000 //400khz clock
#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
#define GYRO_RANGE_2000DPS 2000
#define ACCEL_RANGE_4G 4
//#define MAG
//#define TEMP
#define MICROSECLIMIT 1000000.0

/* Настройки ESC */
const int SERVOPIN[4] = {16, 17, 5, 18};
#define MINPWMIMPULSE 1000
#define MAXPWMIMPULSE 2000
#define WITHESCTEST

/* Настройки WIFI */
#define WIFI
#define RCNUM 4
#define SERVERTIMEOUT 5000

/* Настройки EST */
#define CALLIBRATETRIES 100
#define LPFCOEFF 0.02
#define COMPLIMENTARYCOEFF 0.02
#define MAXPITCHANDROLLANGLE 30 //градусы
#define MAXYAWANGLE 359 //градусы
#define MINPITCHANDROLLANGLE 0 //градусы
#define MINYAWANGLE 0 //градусы
#define BIG_G 9.8

/* Настройки CONT */
#define RATEPIDROLL_P 0.8 //Пропорциональный коэффициент
#define RATEPIDROLL_I 0.05 //Интегральный - потом поменять
#define RATEPIDROLL_D 0.05 //Дифференциальный - потом поменять
#define RATEPIDROLL_LIM 0.3
#define RATEPIDPITCH_P RATEPIDROLL_P
#define RATEPIDPITCH_I RATEPIDROLL_I
#define RATEPIDPITCH_D RATEPIDROLL_D
#define RATEPIDPITCH_LIM RATEPIDROLL_LIM
#define RATEPIDYAW_P 0
#define RATEPIDYAW_I 0
#define RATEPIDYAW_D 0
#define RATEPIDYAW_LIM 0
/* *** */
#define TORGPIDROLL_P 0.01 //Пропорциональный коэффициент
#define TORGPIDROLL_I 0.005 //Интегральный - потом поменять
#define TORGPIDROLL_D 0.005 //Дифференциальный - потом поменять
#define TORGPIDROLL_LIM 0.03
#define TORGPIDPITCH_P TORGPIDROLL_P
#define TORGPIDPITCH_I TORGPIDROLL_I
#define TORGPIDPITCH_D TORGPIDROLL_D
#define TORGPIDPITCH_LIM TORGPIDROLL_LIM
#define TORGPIDYAW_P 0
#define TORGPIDYAW_I 0
#define TORGPIDYAW_D 0
#define TORGPIDYAW_LIM 0
/* --- */

template <typename T>
T mapf(T value, T fromLow, T fromHigh, T toLow, T toHigh)
{
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

#endif
