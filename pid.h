#pragma once

#include <vector>

class pidcontroller {
    public:
        float Kp, Ki, Kd;
        float tau;  // derivative LPF time constant
        float T;    // sampling time of controller in seconds

        float maxlimit, minlimit;   // output limits

        // in memory
        float integrator, differentiator;
        //float preverror, prevmeasurement;

        float output;

        void pid_initialization(pidcontroller* pid);
        void pid_set_K(pidcontroller* pid, float Kp, float Ki, float Kd);
        void pid_set_limits(pidcontroller* pid, float maxlim, float minlim);
        void pid_set_var(pidcontroller* pid, float T, float tau);
        
        float pid_update(pidcontroller* pid, float setpoint, float measurement);

        // DC motor
        float M1, M2, M3, M4, M5, M6, ac;
        float A[3][3];
        float B[3][1];
        std::vector<float> X;

        void initialize_X(pidcontroller* mp);
        void set_M(pidcontroller* mp, float hc);    // set Mi and ac
        void set_AB(pidcontroller* mp);
        void update_X(pidcontroller* mp);
    private:
        float preverror, prevmeasurement;
};
