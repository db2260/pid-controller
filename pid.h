#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct
{
    //Controller gains
    float Kp;
    float Ki;
    float Kd;

    //Derivative low-pass filter time constant
    float tau;

    //Output limits
    float limMin;
    float limMax;

    //Integrator limits
    float limMinInt;
    float limMaxInt;

    //Sample time
    float T;

    //Controller 'memory'
    float integrator;
    float prevError;
    float differentiator;
    float prevMeasurement;

    float out;

} PIDController;

void PIDControllerInit(PIDController *pid);
float PIDControllerUpdate(PIDController *pid, float setpoint, float measurement);

#endif