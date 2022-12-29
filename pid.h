#pragma once

#include <math.h>

class Pid 
{
    double min_ctrl_value;
    double max_ctrl_value;


    int max_e;
    double dt;
    double kp;
    double ki;
    double kd;
    double error;
    double error_sum;
    double previus_error;

    

    public:
    
    Pid(double dt, double min_ctrl_value, double max_ctrl_value, int max_e);

    void set_error_sum(double es);
    void set_ki(double ki);
    void set_kd(double kd);
    void set_kp(double kp);
    
    double squash(double value);
    void update(double set_value, double current_value, double *ctrl_value, double integration_threshold);

};
