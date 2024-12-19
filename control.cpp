
#include "control.h"

extern vector<float> resultAngle;
extern vector<float> resultAngleSec;

extern float dt;

vector<float> angles{0, 0, 0};
vector<float> targets{0, 0, 0};

extern vector<float> rates;
vector<float> ratetargets{0, 0, 0};

vector<float> error{0, 0, 0};
vector<float> erroreu{0, 0, 0};
vector<float> errorto{0, 0, 0};

float rolltorg = 0;
float pitchtorg = 0;
float yawtorg = 0;

PidController rateRoll(RATEPIDROLL_P, RATEPIDROLL_I, RATEPIDROLL_D, RATEPIDROLL_LIM);
PidController ratePitch(RATEPIDPITCH_P, RATEPIDPITCH_I, RATEPIDPITCH_D, RATEPIDPITCH_LIM);
PidController rateYaw(RATEPIDYAW_P, RATEPIDYAW_I, RATEPIDYAW_D, RATEPIDYAW_LIM);

PidController torgueRoll(TORGPIDROLL_P, TORGPIDROLL_I, TORGPIDROLL_D, TORGPIDROLL_LIM);
PidController torguePitch(TORGPIDPITCH_P, TORGPIDPITCH_I, TORGPIDPITCH_D, TORGPIDPITCH_LIM);
PidController torgueYaw(TORGPIDYAW_P, TORGPIDYAW_I, TORGPIDYAW_D, TORGPIDYAW_LIM);

vector<float> motors{0, 0, 0, 0};

vector<float> errortooffcet{-0.05, -0.08, 0};

extern vector<int> rc;
vector<float> targetAngle {0, 0, 1};
float thrust = 0;

void setupCONT()
{

}

void setCONT()
{
  interpretrc();
  
  error = getanglesbetweenvecs(resultAngle, targetAngle);
  
  QuaternionDigit dig0 = converttoquat(resultAngle);
  QuaternionDigit dig1 = converttoquat(targetAngle);

  //Получаем углы поворота roll, pitch, yaw
  //чтобы излечь yaw
  angles = converquattoeuler(dig0);
  targets = converquattoeuler(dig1);

  erroreu = getdifference(angles, targets);

  // Получаем скорости по осям, должны быть от -GYRO_RANGE_2000DPS до GYRO_RANGE_2000DPS //Тут сомневаюсь, надо проверять.
  ratetargets.clear();
  
  ratetargets.push_back(rateRoll.update(error[0], dt)); //error[0] - верно
  ratetargets.push_back(ratePitch.update(error[1], dt)); //error[1] - верно
  ratetargets.push_back(rateYaw.update(erroreu[2], dt)); //TODO: как запрограммировать рысканье? erroreu[2])); //Тут сомневаюсь, надо проверять.

  errorto = getdifference(rates, ratetargets); //Справиться со смещением и накоплением ошибки  

  //Получаем тяги по осям, должны быть от -1 до 1
  rolltorg = tolimit(torgueRoll.update(errorto[0], dt));
  pitchtorg = tolimit(torguePitch.update(errorto[1], dt));
  yawtorg = tolimit(torgueYaw.update(errorto[2], dt)); //Тут сомневаюсь, надо проверять.

  motors.clear();

  //Выставление состояния моторов
  //                тяга    тангаж     крен        рыскание
  motors.push_back(thrust + rolltorg - pitchtorg); // + yawtorg);
  motors.push_back(thrust - rolltorg - pitchtorg); // - yawtorg);
  motors.push_back(thrust + rolltorg + pitchtorg); // - yawtorg);
  motors.push_back(thrust - rolltorg + pitchtorg); // + yawtorg);

  //Чтобы совершился тангаж, нужно для моторов ноль и два увеличить тягу,
  //для один и три уменьшить
  //Для крена увеличить для 2 и 3, уменьшить для 0 и 1
  //Для рысканья увеличить для 0 и 3, уменьшить для 1 и 2

  for(int i = 0; i < motors.size(); i++)
  {
      if (motors[i] > 1)
          motors[i] = 1;
      else if (motors[i] < 0)
          motors[i] = 0;
  }
}

vector<float> getdifference(vector<float>& one, vector<float>& two)
{
  vector<float> ret;

  ret.push_back(one[0] - two[0]);
  ret.push_back(one[1] - two[1]);
  ret.push_back(one[2] - two[2]);

  return ret;
}

void interpretrc()
{
  thrust = (float) rc[3] / 100;

  vector<float> rot;
  rot.push_back((float) rc[2] / 100 * MAXPITCHANDROLLANGLE);
  rot.push_back((float) rc[1] / 100 * MAXPITCHANDROLLANGLE);
  rot.push_back((float) rc[0] / 100 * MAXPITCHANDROLLANGLE); 

  rc[2] = 0;
  rc[1] = 0;
  rc[0] = 0;
  
  QuaternionDigit dig1 = converttoquat(targetAngle);
  qtoangle(&dig1, rot);

  targetAngle = converttovect(dig1);
}

float tolimit(float ans)
{
  if (ans > 1)
    ans = 1;
  else if (ans < -1)
    ans = 0;

  return ans;
}

void printCONTCal()
{
  // printDEBUG(" angles::roll:");
  // printDEBUG(angles[0]);
  // printDEBUG(", pitch:");
  // printDEBUG(angles[1]);
  // printDEBUG(", yaw:");
  // printDEBUG(angles[2]);

  // printDEBUG(" targets::roll:");
  // printDEBUG(targets[0]);
  // printDEBUG(", pitch:");
  // printDEBUG(targets[1]);
  // printDEBUG(", yaw:");
  // printDEBUG(targets[2]);  

  // printDEBUG(" torgs:: rolltorg:");
  // printDEBUG(rolltorg);
  // printDEBUG(", pitchtorg ");
  // printDEBUG(pitchtorg);
  // printDEBUG(", yawtorg");
  // printDEBUG(yawtorg);

  // printDEBUG(" rc:: :");
  // printDEBUG(rc[0]);
  // printDEBUG(", :");
  // printDEBUG(rc[1]);
  // printDEBUG(", :");
  // printDEBUG(rc[2]);
  // printDEBUG(", :");
  // printDEBUG(rc[3]);

  printDEBUG(" targetAngle::X:");
  printDEBUG(targetAngle[0]);
  printDEBUG(", Y:");
  printDEBUG(targetAngle[1]);
  printDEBUG(", Z:");
  printDEBUG(targetAngle[2]);

  printDEBUG(" error:: ");
  printDEBUG(error[0]);
  printDEBUG(", ");
  printDEBUG(error[1]);
  // printDEBUG(", ");
  // printDEBUG(erroreu[2]);

  printDEBUG(" rates::X:");
  printDEBUG(rates[0]);
  printDEBUG(", Y:");
  printDEBUG(rates[1]);
  printDEBUG(", Z:");
  printDEBUG(rates[2]);

  printDEBUG(" ratetargets::X:");
  printDEBUG(ratetargets[0]);
  printDEBUG(", Y:");
  printDEBUG(ratetargets[1]);
  // printDEBUG(", Z:");
  // printDEBUG(ratetargets[2]);

  printDEBUG(" errorto:: ");
  printDEBUG(errorto[0]);
  printDEBUG(", ");
  printDEBUG(errorto[1]);
  // printDEBUG(", ");
  // printDEBUG(errorto[2]);

  printDEBUG(" torgs:: rolltorg:");
  printDEBUG(rolltorg);
  printDEBUG(", pitchtorg ");
  printDEBUG(pitchtorg);
  // printDEBUG(", yawtorg");
  // printDEBUG(yawtorg);

  printDEBUG(" thrust:: ");
  printDEBUG(thrust);
  
  printlnDEBUG("");
}
