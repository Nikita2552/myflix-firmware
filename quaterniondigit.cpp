#include "quaterniondigit.h"

float quatlib::QuaternionDigit::getW()
{
    return w;
}

float quatlib::QuaternionDigit::getX()
{
    return x;
}

float quatlib::QuaternionDigit::getY()
{
    return y;
}

float quatlib::QuaternionDigit::getZ()
{
    return z;
}

void quatlib::QuaternionDigit::setW(float sw)
{
      w = sw;
}

void quatlib::QuaternionDigit::setX(float sx)
{
      x = sx;
}

void quatlib::QuaternionDigit::setY(float sy)
{
      y = sy;
}

void quatlib::QuaternionDigit::setZ(float sz)
{
      z = sz;
}

// void quatlib::QuaternionDigit::print()
// {
//     cout << "q = " << w << " + " << x << "*i" << " + " << y << "*j" << " + " << z << "*k" << endl;
// }

quatlib::QuaternionDigit quatlib::QuaternionDigit::operator+(const QuaternionDigit &dig)
{
    return QuaternionDigit{w + dig.w, x + dig.x, y + dig.y, z + dig.z};
}

quatlib::QuaternionDigit quatlib::QuaternionDigit::operator-(const QuaternionDigit &dig)
{
    return QuaternionDigit{w - dig.w, x - dig.x, y - dig.y, z - dig.z};
}

quatlib::QuaternionDigit quatlib::QuaternionDigit::operator*(const QuaternionDigit &dig)
{
    return QuaternionDigit
    {
        w*dig.w - x*dig.x - y*dig.y - z*dig.z,
        w*dig.x + dig.w*x + y*dig.z - dig.y*z,
        w*dig.y + dig.w*y + z*dig.x - dig.z*x,
        w*dig.z + dig.w*z + x*dig.y - dig.x*y
    };
}

quatlib::QuaternionDigit quatlib::QuaternionDigit::operator*=(const QuaternionDigit &dig)
{
    return QuaternionDigit
    {
        w*dig.w - x*dig.x - y*dig.y - z*dig.z,
        w*dig.x + dig.w*x + y*dig.z - dig.y*z,
        w*dig.y + dig.w*y + z*dig.x - dig.z*x,
        w*dig.z + dig.w*z + x*dig.y - dig.x*y
    };
}

quatlib::QuaternionDigit quatlib::QuaternionDigit::operator/(const float &dig)
{
    return QuaternionDigit{w/dig, x/dig, y/dig, z/dig};
}

quatlib::QuaternionDigit quatlib::QuaternionDigit::operator*(const float &dig)
{
    return QuaternionDigit{w*dig, x*dig, y*dig, z*dig};
}

quatlib::QuaternionDigit quatlib::QuaternionDigit::operator*=(const float &dig)
{
    return QuaternionDigit{w*dig, x*dig, y*dig, z*dig};
}

/* Получение нормали */
float quatlib::QuaternionDigit::norm()
{
    return w*w+x*x+y*y+z*z;
}

/* Получение магнитуды */
float quatlib::QuaternionDigit::mag()
{
    return sqrt(norm());
}

/* Получение сопряженного кватерниона */
quatlib::QuaternionDigit quatlib::QuaternionDigit::conj()
{
    return QuaternionDigit{w, -x, -y, -z};
}

/* Окургление значений кватерниона по осям */
void quatlib::QuaternionDigit::toround()
{
    auto toroundl {
        [](float ret) -> float
        {
            float delta = abs(ret) - floor(abs(ret));
            if (delta < 0.01)
                return floor(ret);
            else
                return ret;
        }
    };

    w = toroundl(w);
    x = toroundl(x);
    y = toroundl(y);
    z = toroundl(z);
}

std::vector<float> quatlib::QuaternionDigit::converttovector()
{
    return vector<float>{x, y, z};
}

quatlib::QuaternionDigit quatlib::QuaternionDigit::getcleanquat()
{
    return QuaternionDigit{0, x, y, z};
}

quatlib::QuaternionDigit quatlib::QuaternionDigit::getrealquat()
{
    return QuaternionDigit{w, 0, 0, 0};
}

void quatlib::QuaternionDigit::normalize()
{
    float mg = mag();

    if (mg)
    {
      w = w/mg;
      x = x/mg;
      y = y/mg;
      z = z/mg;
    }
}

std::vector<float> quatlib::QuaternionDigit::geteuleryawpitchroll()
{
    // https://github.com/ros/geometry2/blob/589caf083cae9d8fae7effdb910454b4681b9ec1/tf2/include/tf2/impl/utils.h#L87
    vector<float> euler;
    float sqx = x * x;
    float sqy = y * y;
    float sqz = z * z;
    float sqw = w * w;
    // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
    float sarg = -2 * (x * z - w * y) / (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */
    if (sarg <= -0.99999) {
        euler.push_back(0); //yaw
        euler.push_back(-0.5 * M_PI); //pitch
        euler.push_back(-2 * atan2(y, x)); //roll
    } else if (sarg >= 0.99999) {
        euler.push_back(0);
        euler.push_back(0.5 * M_PI);
        euler.push_back(2 * atan2(y, x));
    } else {
        euler.push_back(atan2(2 * (y * z + w * x), sqw - sqx - sqy + sqz));
        euler.push_back(asin(sarg));
        euler.push_back(atan2(2 * (x * y + w * z), sqw + sqx - sqy - sqz));
    }
    return euler;
}

/* Создание простого ротора по углу и оси, вокруг которой он должен крутиться */
quatlib::QuaternionDigit quatlib::qsimplerotor(float angle, AXIS ax)
{
   switch (ax)
    {
    case (AXIS_Y):
        return QuaternionDigit{cos(angle), 0, sin(angle), 0};
    case (AXIS_Z):
        return QuaternionDigit{cos(angle), 0, 0, sin(angle)};
    default:
        return QuaternionDigit{cos(angle), sin(angle), 0, 0};
    }
}

quatlib::QuaternionDigit quatlib::rsimplerotor(float rotor, AXIS ax)
{
    
   float angle = asin(rotor);
   float cosrotor = cos(angle);
   float sinrotor = sin(angle);

   switch (ax)
    {
    case (AXIS_Y):
        return QuaternionDigit{cosrotor, 0, sinrotor, 0};
    case (AXIS_Z):
        return QuaternionDigit{cosrotor, 0, 0, sinrotor};
    default:
        return QuaternionDigit{cosrotor, sinrotor, 0, 0};
    }
}

quatlib::QuaternionDigit quatlib::qinvsimplerotor(QuaternionDigit &dig)
{
    return dig.conj()/dig.mag();
}

/* Перемножение кватерниона на ротор по одной оси */
void quatlib::qtoangle(QuaternionDigit *dig, float angle, AXIS ax)
{
    auto degtorad {
        [](float ret) -> float
        {
            return ret * M_PI / 180;
        }
    };

    QuaternionDigit forw = qsimplerotor(degtorad(angle/2), ax);    
    QuaternionDigit inve = qinvsimplerotor(forw);

    QuaternionDigit ret = forw * dig->getcleanquat() * inve;    

    dig->setW(ret.getW());
    dig->setX(ret.getX());
    dig->setY(ret.getY());
    dig->setZ(ret.getZ());
}

void quatlib::rtoangle(QuaternionDigit *dig, float rotor, AXIS ax)
{
    auto degtorad {
        [](float ret) -> float
        {
            return ret * M_PI / 180;
        }
    };

    QuaternionDigit forw = rsimplerotor(rotor, ax);    
    QuaternionDigit inve = qinvsimplerotor(forw);

    QuaternionDigit ret = forw * dig->getcleanquat() * inve;    

    dig->setW(ret.getW());
    dig->setX(ret.getX());
    dig->setY(ret.getY());
    dig->setZ(ret.getZ());
}

/* Перемножение кватерниона на ротор по трём осям */
void quatlib::qtoangle(QuaternionDigit *dig, vector<float> angles, ROTORFLAG rot)
{    
    if (rot == ANGLE)
    {
      qtoangle(dig, angles[AXIS_X]);
      qtoangle(dig, angles[AXIS_Y], AXIS_Y);
      qtoangle(dig, angles[AXIS_Z], AXIS_Z);    
    }
    else if (rot == ROTOR)
    {
      rtoangle(dig, angles[AXIS_X]);
      rtoangle(dig, angles[AXIS_Y], AXIS_Y);
      rtoangle(dig, angles[AXIS_Z], AXIS_Z); 
    }
    // dig->toround();    
}

/* Преобразование из вектора в кватернион */
quatlib::QuaternionDigit quatlib::converttoquat(const vector<float>& vec)
{
    return QuaternionDigit{0, vec[0], vec[1], vec[2]};
}

std::vector<float> quatlib::converttovect(QuaternionDigit &dig)
{
    return vector<float>{dig.getX(), dig.getY(), dig.getZ()};
}

/* Получение углов отклонения пдск по показаниям с акселерометра */
std::vector<float> quatlib::getaccelquat(vector<float> vec)
{    
    QuaternionDigit cdig{0, 0, 0, -1};

    qtoangle(&cdig, vec, ROTOR);

    //QuaternionDigit cdig = converttoquat(vec);
    
    QuaternionDigit odig = qinvsimplerotor(cdig);    

    return converttovect(odig);
}

/* Вычисление угла между векторами по одной оси*/
float quatlib::getanglebetweenvectors(std::vector<float> &z, std::vector<float> &r)
{
    auto radtodeg {
        [](float ret) -> float
        {
            return ret * 180 / M_PI;
        }
    };
    
    //Считаем скалярное произведение
    float scal = z[0]*r[0] + z[1]*r[1] + z[2]*r[2];
    //Находим модули векторов
    float modz = sqrt(z[0]*z[0] + z[1]*z[1] + z[2]*z[2]);
    float modr = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
    //Получаем угол

    scal = scal/(modz*modr);
    if ((scal >= -1) && (scal <= 1))
    {
      return radtodeg(acos(scal));
    }

    return 0;    
}

std::vector<float> quatlib::getanglesbetweenvectorsv1(vector<float> &z, vector<float> &r)
{
    vector<float> ret;
    ret.push_back(r[0] - z[0]);
    ret.push_back(r[1] - z[1]);
    ret.push_back(r[2] - z[2]);

    return ret;
}

/* Вычисление угла между векторами по 3м осям */
std::vector<float> quatlib::getanglesbetweenvectorsv2(vector<float> &z, vector<float> &r)
{
    return
    {
       getanglebetweenvectorsaxis(z, r, AXIS_X),
       getanglebetweenvectorsaxis(z, r, AXIS_Y),
       getanglebetweenvectorsaxis(z, r, AXIS_Z)
    };
}

// std::vector<float> quatlib::getanglesbetweenvectorsmid(vector<float> &z, vector<float> &r)
// {
//     std::vector<float> v1 = getanglesbetweenvectorsv1(z, r);
//     std::vector<float> v2 = getanglesbetweenvectorsv2(z, r);

//     return std::vector<float>{(v1[0]+v2[0])/2, (v1[1]+v2[1])/2, (v1[2]+v2[2])/2};
// }

/* Опускание векторов на плоскости */
float quatlib::getanglebetweenvectorsaxis(std::vector<float> &z, std::vector<float> &r, AXIS ax)
{
    vector<float> az;
    vector<float> ar;
    switch (ax)
    {
        case (AXIS_X):
            az.push_back(0); az.push_back(z[1]); az.push_back(z[2]);
            ar.push_back(0); ar.push_back(r[1]); ar.push_back(r[2]);
            break;
        case (AXIS_Y):
            az.push_back(z[0]); az.push_back(0); az.push_back(z[2]);
            ar.push_back(r[0]); ar.push_back(0); ar.push_back(r[2]);
            break;
        default:
            az.push_back(z[0]); az.push_back(z[1]); az.push_back(0);
            ar.push_back(r[0]); ar.push_back(r[1]); ar.push_back(0);
            break;
    }
    return getanglebetweenvectors(az, ar);
}

/* SLERP в случае больших углов и линейная интерполяция в случае малых */
quatlib::QuaternionDigit quatlib::correctgyrofromaccel(QuaternionDigit &gyro, QuaternionDigit &accel)
{
    auto radtodeg {
        [](float ret) -> float
        {
            return ret * 180 / M_PI;
        }
    };

    //Проверочный дебаг
    //cuatDEBUG(gyro, accel);

    //Проверяем, какую интерпроляцию применить
    float magp = gyro.mag() * accel.mag();

    //Если скалярное произведение оотрицательно, значит интерполяция пойдёт по худшему пути. Ивертируем один изкватернионов
    QuaternionDigit accelp;
    if (magp > 0)
    {
        accelp = accel;
    }
    else
    {
       accelp = qinvsimplerotor(accel);
    }

    float quatAngle = (gyro.getW()*accelp.getW() +
                       gyro.getX()*accelp.getX() +
                       gyro.getY()*accelp.getY() +
                       gyro.getZ()*accelp.getZ())/
                       (gyro.mag() * accelp.mag());

    if ((quatAngle <= 1) && (quatAngle >= -1))
    {
        quatAngle = acos(quatAngle);

        if (abs(quatAngle) >= 0.0001)
        {
            //Для больших углов
            return (gyro*(sin((1 - COMPLIMENTARYCOEFF)*quatAngle)/sin(quatAngle)) + accelp*(sin(COMPLIMENTARYCOEFF*quatAngle)/sin(quatAngle)));
        }
        else
        {
            //Для малых углов
            float rw = gyro.getW() + COMPLIMENTARYCOEFF*(accel.getW() - gyro.getW());
            float rx = gyro.getX() + COMPLIMENTARYCOEFF*(accel.getX() - gyro.getX());
            float ry = gyro.getY() + COMPLIMENTARYCOEFF*(accel.getY() - gyro.getY());
            float rz = gyro.getZ() + COMPLIMENTARYCOEFF*(accel.getZ() - gyro.getZ());

            return QuaternionDigit{rw, rx, ry, rz};
        }
    }
    else
    {
        return accel; // Что лучше в случае ошибки? Гироскоп или акселерометр. Думая, что акс, так как он менее подвержен помехам (???)
    }
}

std::vector<float> quatlib::converquattoeuler(QuaternionDigit quat)
{
    float sqw = quat.getW() * quat.getW();
    float sqx = quat.getX() * quat.getX();
    float sqy = quat.getY() * quat.getY();
    float sqz = quat.getZ() * quat.getZ();

    float sarg = -2 * (quat.getX()*quat.getZ() -
                       quat.getW()*quat.getY()) /
                      (sqx + sqy + sqz + sqw); 
    
    float roll, pitch, yaw;

    if (sarg <= -0.999999)
    {
        roll = 0;
        pitch = -0.5 * M_PI;
        yaw = -2 * atan2(quat.getY(), quat.getX());
    }
    else if (sarg >= 0.999999)
    {
        roll = 0;
        pitch = 0.5 * M_PI;
        yaw = 2 * atan2(quat.getY(), quat.getX());
    }
    else
    {
        roll = atan2(2*(quat.getY()*quat.getZ() + quat.getW()*quat.getX()),
                     sqw - sqx - sqy + sqz);
        pitch = asin(sarg);
        yaw = atan2(2*(quat.getX()*quat.getY() + quat.getW()*quat.getZ()),
                     sqw + sqx - sqy - sqz);
    }

    auto radtodeg {
        [](float ret) -> float
        {
            return ret * 180 / M_PI;
        }
    };

    //return vector<float>{radtodeg(roll), radtodeg(pitch), radtodeg(yaw)};
    return vector<float>{roll, pitch, yaw};
}

std::vector<float> quatlib::getanglesbetweenvecs(std::vector<float>& vstart, std::vector<float>& vfin)
{
  vector<float> ret = cross(vstart, vfin);

  //Для случая, когда угол равен 180
  if (ret[0] == 0 && ret[1] == 0 && ret[2] == 0)
  {
    QuaternionDigit qstart = converttoquat(vstart);
    QuaternionDigit qfin = converttoquat(vfin);

    qfin = correctgyrofromaccel(qfin, qstart);

    vector<float> vfin_ = converttovect(qfin);

    return cross(vstart, vfin_);
  }

  float angle = getanglebetweenvectors(vstart, vfin);

  ret[0] *= angle;
  ret[1] *= angle;
  ret[2] *= angle;

  return ret;
}

std::vector<float> quatlib::cross(std::vector<float>& vstart, std::vector<float>& vfin)
{
  vector<float> ret;

  ret.push_back(vstart[1]*vfin[2] - vstart[2]*vfin[1]);
  ret.push_back(vstart[2]*vfin[0] - vstart[0]*vfin[2]);
  ret.push_back(vstart[0]*vfin[1] - vstart[1]*vfin[0]);
  
  return ret;
}

#define FLOORCOEFF 100

std::vector<float> quatlib::floorceil(vector<float>& target, vector<float>& sensor)
{
  vector<float> ret;

  for (int i = 0; i < 3; i++)
  {
    if (target[i]*FLOORCOEFF == floor(sensor[i]*FLOORCOEFF))
    {
      ret.push_back(floor(target[i]*FLOORCOEFF)/FLOORCOEFF);
    }
    else
    {
      ret.push_back(floor(sensor[i]*FLOORCOEFF)/FLOORCOEFF);
    }
  }

  return ret;
}

void quatlib::cuatDEBUG(QuaternionDigit& gyro, QuaternionDigit& accel)
{
  printDEBUG(" gyro::X:");
  printDEBUG(gyro.getX());
  printDEBUG(", Y:");
  printDEBUG(gyro.getY());
  printDEBUG(", Z:");
  printDEBUG(gyro.getZ());

  printDEBUG(" accel::X:");
  printDEBUG(accel.getX());
  printDEBUG(", Y:");
  printDEBUG(accel.getY());
  printDEBUG(", Z:");
  printDEBUG(accel.getZ());
}