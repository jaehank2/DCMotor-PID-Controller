#include "pid.h"

void pidcontroller::pid_set_K(pidcontroller* pid, float Kp, float Ki, float Kd){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void pidcontroller::pid_set_limits(pidcontroller* pid, float maxlim, float minlim){
    pid->maxlimit = maxlim;
    pid->minlimit = minlim;
}

void pidcontroller::pid_set_var(pidcontroller* pid, float T, float tau){
    pid->T = T;
    pid->tau = tau;
}

void pidcontroller::pid_initialization(pidcontroller* pid){
    pid->integrator = 0.0f;
    pid->differentiator = 0.0f;
    //pid->preverror = 0.0f;
    preverror = 0;
    //pid->prevmeasurement = 0.0f;
    prevmeasurement = 0;
    pid->output = 0.0f;
}

float pidcontroller::pid_update(pidcontroller* pid, float setpoint, float measurement){
    float error = setpoint - measurement;
    float proportional = (pid->Kp) * error;    // p[n]

    float temp_integrator = pid->integrator + pid->Ki * pid->T * 0.5f * (error + preverror); // i[n]

    // avoid derivative kick during setpoint by doing derivative on measurement
    pid->differentiator = -1.0f*(2.0f * pid->Kd * (measurement - prevmeasurement) +
                          (2.0f * pid->tau - pid->T)*pid->differentiator) / 
                          (2.0f * pid->tau + pid->T);
    
    pid->output = proportional + temp_integrator + pid->differentiator;

    bool clamp = false;
    bool samesign = false;

    if ((pid->output > 0.0f && error > 0.0f) || (pid->output < 0.0f && error < 0.0f)){
        samesign = true;
    }

    // clamp output for integral anti windup
    if (pid->output > pid->maxlimit){
        pid->output = pid->maxlimit;
        if (samesign == true){
            clamp = true;
        }
    }
    else if (pid->output < pid->minlimit){
        pid->output = pid->minlimit;
        if (samesign == true){
            clamp = true;
        }
    }

    // if no need for clamping, set integrator as is
    if (clamp == false){
        pid->integrator = temp_integrator;
    }
    // else if clamp is true, do not update pid->integrator

    preverror = error;
    prevmeasurement = measurement;

    return pid->output;
}

// DC Motor

void pidcontroller::initialize_X(pidcontroller* mp){
    for (int i=0; i<3; i++){
        mp->X.push_back(0.0f);
    }
}

void pidcontroller::set_M(pidcontroller* mp, float hc){
    mp->M1 = -hc;
    mp->M2 = 1.0f + hc*(3.5077f / 3.2284f);
    mp->M3 = -hc*(0.0274f / 0.0000032284f);
    mp->M4 = hc*(0.0274f / 0.00000275f);
    mp->M5 = 1.0f + hc*(4.0f / 0.00000275f);
    mp->M6 = -hc*(1.0f / 0.00000275f);
    mp->ac = 1.0f / ((mp->M2 * mp->M5) - (mp->M3 * mp->M4));
}

void pidcontroller::set_AB(pidcontroller* mp){
    mp->A[0][0] = mp->ac * ((mp->M2 * mp->M5) - (mp->M3 * mp->M4));
    mp->A[0][1] = mp->ac * (-mp->M1 * mp->M5);
    mp->A[0][2] = mp->ac * (mp->M1 * mp->M3);
    mp->B[0][0] = mp->ac * (-mp->M1 * mp->M3 * mp->M6);

    mp->A[1][0] = 0.0f;
    mp->A[1][1] = mp->ac * (mp->M5);
    mp->A[1][2] = mp->ac * (-mp->M3);
    mp->B[1][0] = mp->ac * (mp->M3 * mp->M6);

    mp->A[2][0] = 0.0f;
    mp->A[2][1] = mp->ac * (-mp->M4);
    mp->A[2][2] = mp->ac * (mp->M2);
    mp->B[2][0] = mp->ac * (-mp->M2 * mp->M6);
}

void pidcontroller::update_X(pidcontroller* mp){
    mp->X[0] = (mp->A[0][0] * mp->X[0]) + (mp->A[0][1] * mp->X[1]) + (mp->A[0][2] * mp->X[2]) + (mp->B[0][0] * mp->output);
    mp->X[1] = (mp->A[1][0] * mp->X[0]) + (mp->A[1][1] * mp->X[1]) + (mp->A[1][2] * mp->X[2]) + (mp->B[1][0] * mp->output);
    mp->X[2] = (mp->A[2][0] * mp->X[0]) + (mp->A[2][1] * mp->X[1]) + (mp->A[2][2] * mp->X[2]) + (mp->B[2][0] * mp->output);
}