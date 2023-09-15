#include <stdio.h>
#include <stdlib.h>

#include "pid.h"

//Controller parameters

#define PID_KP 2.0f
#define PID_KI 0.5f
#define PID_KD 0.25f
#define PID_TAU 0.02f
#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX 10.0f
#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT 5.0f
#define PID_SAMPLE_TIME 0.01f

//Maximum run-time of simulation

#define SIMULATION_TIME_MAX 4.0f

//Simulated dynamical system (first-order)

float TestSystemUpdate(float input);

int main()
{
    PIDController pid = {PID_KP, PID_KI, PID_KD, PID_TAU, PID_LIM_MIN, PID_LIM_MAX, PID_LIM_MIN_INT, PID_LIM_MAX_INT, PID_SAMPLE_TIME};

    PIDControllerInit(&pid);

    float setpoint = 1.0f;

    printf("Time (s)\tSystem Output\tController Output\r\n");
    
    for(float t=0.0f; t<=SIMULATION_TIME_MAX; t+=PID_SAMPLE_TIME)
    {
        float measurement = TestSystemUpdate(pid.out);
        PIDControllerUpdate(&pid, setpoint, measurement);
        printf("%f\t%f\t%f\r\n", t, measurement, pid.out);
    }

    return 0;
}

float TestSystemUpdate(float input)
{
    static float output = 0.0f;
    static const float alpha = 0.02f;

    output = (PID_SAMPLE_TIME * input + output) / (1.0f + alpha * PID_SAMPLE_TIME);

    return output;
}


