#include "pidcontroller.h"

PidController::PidController(float K_p, float K_i, float K_d, float K_l):
Kp(K_p), Ki(K_i), Kd(K_d), integralError(0), prevError(0), Lim(K_l)
{

}

PidController::~PidController()
{

}

float PidController::update(float error, float dt)
{
    integralError += error *dt;

    float derivativeError = (error - prevError) / dt; //???
    prevError = error;

    auto constr {
        [](float ret, float lim) -> float
        {
            if ((-lim < ret) && (ret < lim))            
              return ret;
            else if (ret >= lim)
              return lim;
            else if (ret <= -lim)
              return -lim;
        }
    };

    float outputData  = Kp*error + constr(Ki*integralError, Lim) + Kd*derivativeError; //???

    return outputData;
}
