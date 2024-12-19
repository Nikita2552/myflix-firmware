#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H


class PidController
{
public:
    PidController(float K_p, float K_i, float K_d, float K_l);
    ~PidController();

    float update(float error, float dt);

private:
    float Kp;
    float Ki;
    float Kd;

    float integralError;
    float prevError;

    float Lim;
};

#endif // PIDCONTROLLER_H
