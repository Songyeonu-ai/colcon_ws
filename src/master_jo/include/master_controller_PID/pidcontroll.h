#ifndef PIDCONTROLL_H
#define PIDCONTROLL_H

#include <iostream>

class PIDControll
{
    double Error[3];
    double K[3];
    double Kp, Ki, Kd;
    double Ka; //antiwindup gain
    double Control_Output[2]; //controller output
    double us; //saturation output
    double Limit; //saturation limit

public:
    PIDControll(double Kp, double Ki, double Kd, double Kl, double Ka);
    void print();
    void setParams(double Kp, double Ki, double Kd, double Ka);
    double controller(double Current_Error);
    double saturation(double input);
    double antiWindup(double u1, double u2);
};

#endif // PIDCONTROLL_H
