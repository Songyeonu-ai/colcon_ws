#ifndef ROBOT_H
#define ROBOT_H

#define REAL_MAP_SIZE_X 1100
#define REAL_MAP_SIZE_Y 800

#define DEG2RAD (M_PI / 180)
#include <ctime>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

class ROBOT
{
public:
    //For Init
    //객채 생성자 함수, x y 값 입력시 해당 값으로 설정, 입력 안할 시 0,0으로 설정됨
    ROBOT(){}
    ROBOT(float x, float y)
    {
        this->x = x;
        this->y = y;
    }

    int state = 0;

    float x = 0.0;//REAL_MAP_SIZE_X / 2.0;
    float y = 0.0;//REAL_MAP_SIZE_Y / 2.0;
    float z = 0.0;

    float b = 0.0;

    //오도메트리값
    float odom_fx = 0.30;
    float odom_bx = 0.30;

    float odom_ly = 0.30;
    float odom_ry = 0.30;

    //현재 들어오는 ik파라미터 값
    float now_ik_param_x = 0;
    float now_ik_param_y = 0;

    struct POINT
    {
        //로봇이 지나간 경로를 그리기 위한 구조체
        unsigned int TIME;
        int X;
        int Y;
    };
    POINT point;
    vector<POINT> TIME_STAMP;//TIME_STAMP를 저장할 벡터 컨테이너

    double weight; //파티클이 가지는 가중치값, 파티클에만 사용되는 값이고 로봇에는 사용되지 않음

private:


public:
    void create_time_stamp(double Xmoved, double Ymoved)
    {
        //PRE CONDITION : Xmoved, Ymoved
        //POST CONDITION : TIME_STAMP
        //PURPOSE : 로봇에 Xmoved 혹은 Ymoved값이 들어오면 이동 경로를 저장

        if(Xmoved != 0 || Ymoved != 0)
        //Xmoved 혹은 Ymoved값이 입력 될 경우 실행
        {
            time_t timer; //타이머 생성
            timer = time(NULL);
            point.TIME = timer; point.X = this->x; point.Y = this->y; //point 구조체에 시간, X, Y 값을 저장
            TIME_STAMP.push_back(point); //해당 포인트를 TIME_STAMP 벡터 컨테이너에 저장
            if(TIME_STAMP.size() > 10){TIME_STAMP.erase(TIME_STAMP.begin());} //TIME_STAMP 벡터 컨테이너의 크기가 10이 넘으면 초기화
        }
        else
        //Xmoved 혹은 Ymoved값이 입력 되지 않을 경우 TIME_STAMP 벡터 컨테이너 초기화
        {
            TIME_STAMP.clear();
        }
    }

    void move(double Xmoved, double Ymoved)
    {
        //PRE CONDITION : Xmoved, Ymoved
        //POST CONDITION : x, y
        //PURPOSE : 로봇에 Xmoved 혹은 Ymoved값이 들어오면 해당 값으로 이동

        //time_stamp
        create_time_stamp(Xmoved, Ymoved);

        //odom
        this->now_ik_param_x = Xmoved;
        this->now_ik_param_y = Ymoved;
        if(Xmoved > 0){Xmoved *= odom_fx;}
        else{Xmoved *= odom_bx;}

        if(Ymoved > 0){Ymoved *= odom_ly;}
        else{Ymoved *= odom_ry;}


        //x및 y값 계산 및 초기화
        this->x += Xmoved * cos((this->z + 90) * DEG2RAD) - Ymoved * sin((this->z + 90) * DEG2RAD);
        this->y -= Xmoved * sin((this->z + 90) * DEG2RAD) + Ymoved * cos((this->z + 90) * DEG2RAD);

        if(this->x < 100){this->x = 100;}
        if(this->x > 1000){this->x = 1000;}
        if(this->y < 100){this->y = 100;}
        if(this->y > 700){this->y = 700;}
    }
    void random_point(int x, int y, float particle_range)
    {
        //PRE CONDITION :  x, y, particle_range
        //POST CONDITION : x, y
        //PURPOSE : 가우시안 분포로 x, y값 변경. 파티클에서만 사용
        this->x = x + gaussianRandom(0, particle_range);//30.0);
        this->y = y + gaussianRandom(0, particle_range);//30.0);
    }




private:
    double gaussianRandom(double average, double stdev)
    {
        //PRE CONDITION :  average, stdev
        //POST CONDITION : temp(noise)
        //PURPOSE : 가우시안 노이즈 생성

        double v1, v2, s, temp;
        do{
            v1 = 2 * ((double) rand() / RAND_MAX) - 1;      // -1.0 ~ 1.0 까지의 값
            v2 = 2 * ((double) rand() / RAND_MAX) - 1;      // -1.0 ~ 1.0 까지의 값
            s = v1 * v1 + v2 * v2;
        }while (s >= 1 || s == 0);
        s = sqrt((-2 * log(s)) / s);
        temp = v1 * s;
        temp = (stdev * temp) + average;
        return temp;
    }

};


#endif // ROBOT_H
