#ifndef QUATERNIONDIGIT_H
#define QUATERNIONDIGIT_H

#include <iostream>
#include <cmath>
#include <vector>
#include "globals.h"
#include "debug.h"

namespace quatlib
{
    using namespace std;

    typedef enum
    {
        AXIS_X  = 00,
        AXIS_Y  = 01,
        AXIS_Z  = 02
    } AXIS;

    typedef enum
    {
        ROTOR  = 00,
        ANGLE  = 01,
        
    } ROTORFLAG;

    // typedef enum
    // {
    //     FORWARD = true, // Увеличиваем угол по часовой стрелке, но против положительного увеличения тригонометрического угла
    //     BACKWARD  = false // Уменьшаем угол против часовой стрелки
    // } DIRECTION;

    class QuaternionDigit
    {
    public:
        QuaternionDigit(): w(0), x(0), y(0), z(0) {};
        QuaternionDigit(float cw, float cx, float cy, float cz): w(cw), x(cx), y(cy), z(cz) {};
        ~QuaternionDigit(){};

        float getW();
        float getX();
        float getY();
        float getZ();

        void setW(float sw);
        void setX(float sx);
        void setY(float sy);
        void setZ(float sz);

        // void print();

        QuaternionDigit operator+(const QuaternionDigit &dig);
        QuaternionDigit operator-(const QuaternionDigit &dig);
        QuaternionDigit operator*(const QuaternionDigit &dig);
        QuaternionDigit operator*=(const QuaternionDigit &dig);
        QuaternionDigit operator*(const float &dig);
        QuaternionDigit operator*=(const float &dig);
        QuaternionDigit operator/(const float &dig);

        float norm();
        float mag();

        QuaternionDigit conj();
        void toround();
        vector<float> converttovector();
        QuaternionDigit getcleanquat();
        QuaternionDigit getrealquat();

        void normalize();

        vector<float> geteuleryawpitchroll();

    private:
        float w;
        float x;
        float y;
        float z;
    };

    QuaternionDigit qsimplerotor(float angle, AXIS ax = AXIS_X);
    QuaternionDigit rsimplerotor(float rotor, AXIS ax = AXIS_X);
    QuaternionDigit qinvsimplerotor(QuaternionDigit& dig);    
    void qtoangle(QuaternionDigit *dig, float angle = 0, AXIS ax = AXIS_X);
    void rtoangle(QuaternionDigit *dig, float rotor = 0, AXIS ax = AXIS_X);
    void qtoangle(QuaternionDigit *dig, vector<float> angles = {0, 0, 0}, ROTORFLAG rot = ANGLE);
    QuaternionDigit converttoquat(const vector<float>& vec);
    vector<float> converttovect(QuaternionDigit& dig);
    vector<float> getaccelquat(vector<float> vec);
    float getanglebetweenvectors(vector<float>& z, vector<float>& r);
    vector<float> getanglesbetweenvectorsv1(vector<float>& z, vector<float>& r); //???
    vector<float> getanglesbetweenvectorsv2(vector<float>& z, vector<float>& r); //???
    // vector<float> getanglesbetweenvectorsmid(vector<float>& z, vector<float>& r); //???
    float getanglebetweenvectorsaxis(vector<float>& z, vector<float>& r, AXIS ax = AXIS_Z);
    QuaternionDigit correctgyrofromaccel(QuaternionDigit& gyro, QuaternionDigit& accel); //???
    void cuatDEBUG(QuaternionDigit& gyro, QuaternionDigit& accel);

    vector<float> converquattoeuler(QuaternionDigit quat);

    vector<float> getanglesbetweenvecs(vector<float>& vstart, vector<float>& vfin);
    vector<float> cross(vector<float>& vstart, vector<float>& vfin);

    vector<float> floorceil(vector<float>& target, vector<float>& sensor);  
}

#endif // QUATERNIONDIGIT_H
