#include "pidcontroll.h"

using namespace std;

PIDControll::PIDControll(double Kp, double Ki, double Kd, double Kl, double Ka)
{
    //Kp: 비례제어 게인
    //Ki: 적분제어 게인
    //Kd: 미분제어 게인
    //Kl: 포화 제한값
    //Ka: 안티-와인드업 게인 ->구현 X

    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    K[0] = Kp + Ki + Kd;
    K[1] = -Kp -2*Kd;
    K[2] = Kd;

    this->Limit = Kl;
    this->Ka = Ka;
}

void PIDControll::print()
{
    cout << endl << endl;

    cout << "PID CONTROLLER DEBUG PRINTING" << endl << endl;

    cout << "ERROR 0:           " << Error[0] << endl;
    cout << "ERROR 1:           " << Error[1] << endl;
    cout << "ERROR 2:           " << Error[2] << endl;

    cout << "K 0:               " << K[0] << endl;
    cout << "K 1:               " << K[1] << endl;
    cout << "K 2:               " << K[2] << endl;

    cout << "Control_Output 0: " << Control_Output[0] << endl;
    cout << "Control_Output 1: " << Control_Output[1] << endl;

    cout << "CONTROL:          " << Error[0] * K[0] + Error[1] * K[1] + Error[2] * K[2] << endl;

    cout << endl << endl;
}

void PIDControll::setParams(double Kp, double Ki, double Kd, double Ka = 1)
{
    K[0] = Kp + Ki + Kd;
    K[1] = -Kp -2*Kd;
    K[2] = Kd;

    this->Ka = Ka;
}

double PIDControll::controller(double Current_Error)
{
    Control_Output[0] = Control_Output[1];
    Error[2] = Error[1];
    Error[1] = Error[0];
    Error[0] = Current_Error;

//    Control_Output[1] = Control_Output[0] + Ki*Ka*us +
//    Error[0] * K[0] +
//    Error[1] * K[1] +
//    Error[2] * K[2];

//    Control_Output[1] = Control_Output[1] / (1+Ka*Ki);

//    us = saturation(Control_Output[1]);

    Control_Output[1] = Control_Output[0] +
    Error[0] * K[0] +
    Error[1] * K[1] +
    Error[2] * K[2];

    Control_Output[1] = saturation(Control_Output[1]);


    return Control_Output[1];
}

double PIDControll::saturation(double input)
{
    double output;

    if(input >= Limit) output = Limit;
    else if(input <= -Limit) output = -Limit;
    else output = input;

    return output;
}



