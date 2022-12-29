#include <Arduino.h>
#include "pid.h"
#include <Wire.h>

Pid::Pid(double dt, double min_ctrl_value, double max_ctrl_value, int max_e)
{
    this->max_e = max_e;
    this->dt = dt;
    this->min_ctrl_value = min_ctrl_value;
    this->max_ctrl_value = max_ctrl_value;
    this->error_sum = 0;
    this->previus_error = 0;
    Serial.begin(115200);
    
}

void Pid::set_error_sum(double es)
{
    this->error_sum = es;
}

void Pid::set_kp(double kp)
{
    this->kp = kp;
}

void Pid::set_ki(double ki)
{
    this->ki = ki;
}

void Pid::set_kd(double kd)
{
    this->kd = kd;
}

double Pid::squash(double value)
{
    return (value < min_ctrl_value) ? min_ctrl_value : ((value > max_ctrl_value) ? max_ctrl_value : value);    
}

void Pid::update(double set_value, double current_value, double *ctrl_value, double integration_threshold)
{
    long double kp_val, ki_val, kd_val, ctrl; 
    
    error = (set_value - current_value);

    if (error > max_e){
        error = max_e;
    } else if(error < -max_e){
        error = -max_e;
    }
        
    if (fabs(error) < integration_threshold)
        error_sum += error;

    kp_val = error;
    ki_val = error_sum * dt;
    kd_val = (previus_error - error) / (dt);
    
    previus_error = error;
    ctrl = kp*kp_val + ki*ki_val + kd*kd_val;
    *ctrl_value = squash(ctrl);
}

