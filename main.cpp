#include <iostream>
#include "pid.h"

using namespace std;

int main(){

    pidcontroller u;
    pidcontroller* temp = &u;
    u.pid_initialization(temp);
    u.pid_set_K(temp, 21, 500, 0.15);    // Kp, Ki, Kd
    u.pid_set_var(temp, 0.01, 100);   // T and tau
    u.pid_set_limits(temp, 1000, -1000);   // max and min

    u.initialize_X(temp);   // state space
    u.set_M(temp, 0.01); // hc
    u.set_AB(temp);
    
    float output = 0;

    for (int i=0; i<40; i++){
        output = u.pid_update(temp, 1, u.X[0]);
        //cout << "PID output = " << output << endl;
        u.update_X(temp);
        cout << "theta output = " << u.X[0] << endl;
    }
    
    return 0;
}