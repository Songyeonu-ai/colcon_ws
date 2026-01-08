#ifndef LINE_H
#define LINE_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define DEG2RAD (M_PI / 180)

using namespace std;
using namespace cv;

class LINE
{
public:
    // For Init
    LINE()
    {
    }

public:
    struct VISION_POINT
    {
        // 비전에서 찾은 포인트 데이터를 저장하는 구조체

        double CONFIDENCE; // 포인트 데이터의 신뢰도
        double DISTANCE;   // 포인트 데이터의 거리

        // 특징점과 로봇 사이의 거리
        int POINT_VEC_X;
        int POINT_VEC_Y;

        // 로봇의 좌표
        int STD_X;
        int STD_Y;
    };
    VISION_POINT vision_point;
    vector<VISION_POINT> vision_point_vect; // VISION_POINT 를 저장할 벡터 컨테이너

    Point CIRCLE_CENTER = Point(0, 0);
    double CIRCLE_R = 0;

    // 특징점의 로컬 좌표
    int Local_point_x[27] = {100, 550, 1000, 100, 300, 800, 1000, 100, 200, 900, 1000, 550, 250, 550, 850, 550, 100, 200, 900, 1000, 100, 300, 800, 1000, 100, 550, 1000};
    int Local_point_y[27] = {100, 100, 100, 150, 150, 150, 150, 250, 250, 250, 250, 325, 400, 400, 400, 475, 550, 550, 550, 550, 650, 650, 650, 650, 700, 700, 700};

    // 특징점 활성화 변수
    int Local_point_on_off[27] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int Local_point_check[27] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

private:
    Mat Likelihood_mat;

public:
    void set_circle(double yaw, vector<VISION_POINT> &vect)
    {
        // PRE CONDITION : YAW, VECTOR<VISION_POINT>
        // POST CONDITION :
        // PURPOSE : 로컬의 특징점 범위를 설정

        double X_MAX = -9999, X_MIN = 9999;
        double Y_MAX = -9999, Y_MIN = 9999;

        for (int i = 0; i < vect.size(); i++)
        {
            // 포착된 포인트 중 최대 거리와 최소 거리를 계산
            int transformation_x = (vect[i].POINT_VEC_X) * cos((-1) * yaw * DEG2RAD) - (vect[i].POINT_VEC_Y) * sin((-1) * yaw * DEG2RAD);
            int transformation_y = (vect[i].POINT_VEC_X) * sin((-1) * yaw * DEG2RAD) + (vect[i].POINT_VEC_Y) * cos((-1) * yaw * DEG2RAD);
            if (transformation_x > X_MAX)
            {
                X_MAX = transformation_x;
            }
            if (transformation_x < X_MIN)
            {
                X_MIN = transformation_x;
            }
            if (transformation_y > Y_MAX)
            {
                Y_MAX = transformation_y;
            }
            if (transformation_y < Y_MIN)
            {
                Y_MIN = transformation_y;
            }
        }

        // 로봇의 좌표에서 최대 거리 값과 최소 거리 값의 평균 값을 더한 후 해당 값을 범위의 중앙으로 설정
        CIRCLE_CENTER = Point((int)(vect[0].STD_X + ((X_MAX + X_MIN) / 2) * cos(yaw * DEG2RAD) - ((Y_MAX + Y_MIN) / 2) * sin(yaw * DEG2RAD)), (int)(vect[0].STD_Y + ((X_MAX + X_MIN) / 2) * sin(yaw * DEG2RAD) + ((Y_MAX + Y_MIN) / 2) * cos(yaw * DEG2RAD)));

        // 로봇의 좌표에서 최대 거리 값과 최소 거리 값의 평균 값을 뺀 후 해당 값에서 +50 한 값을 범위의 반지름으로 설정
        CIRCLE_R = 0;

        for (int i = 0; i < vect.size(); i++)
        {
            // cout << "CIRCLE_CENTER.x : " << CIRCLE_CENTER.x << endl;
            // cout << "CIRCLE_CENTER.y : " << CIRCLE_CENTER.y << endl;
            // cout << "vect[i].POINT_VEC_X : " << vect[i].POINT_VEC_X << endl;
            // cout << "vect[i].POINT_VEC_Y : " << vect[i].POINT_VEC_Y << endl;
            // cout << "vect[i].STD_X : " << vect[i].STD_X << endl;
            // cout << "vect[i].STD_Y : " << vect[i].STD_Y << endl;

            double dis = sqrt(pow(CIRCLE_CENTER.x - vect[i].POINT_VEC_X - vect[i].STD_X, 2) + pow(CIRCLE_CENTER.y - vect[i].POINT_VEC_Y - vect[i].STD_Y, 2));
            if (CIRCLE_R < dis)
            {
                CIRCLE_R = dis;
            }
        }

        CIRCLE_R += 50;
    }
    void on_local_point(int PT_X, int PT_Y, int NOW_X, int NOW_Y)
    {
        // PRE CONDITION : PT_X, PT_Y, ROBOT_X, ROBOT_Y
        // POST CONDITION : Local_point_on_off
        // PURPOSE : 파티클의 x y와 로봇의 x y 값을 통해 활성화 할 특징점 계산

        for (int i = 0; i < 27; i++)
        {
            if (pow(CIRCLE_R, 2) > pow(PT_X - NOW_X + CIRCLE_CENTER.x - Local_point_x[i], 2) + pow(PT_Y - NOW_Y + CIRCLE_CENTER.y - Local_point_y[i], 2))
            {
                Local_point_on_off[i] = 1;
            }
            else
            {
                Local_point_on_off[i] = 0;
            }
        }
    }

    void check_local_point(int ago_point_cnt, double yaw, vector<VISION_POINT> &vect)
    {
        // PRE CONDITION : ago_point_cnt = 50, yaw, vect<VISION_POINT> = VISION_POINT_VECT
        // POST CONDITION : Local_point_check
        // PURPOSE : 로봇의 yaw값을 통해 파티클 필터 계산에 사용될 특징점 활성화

        double X_MAX = -9999, X_MIN = 9999;
        double Y_MAX = -9999, Y_MIN = 9999;
        for (int i = ago_point_cnt; i < vect.size(); i++)
        {
            // 포착된 포인트 중 최대 거리와 최소 거리를 계산, 이때 벡터 컨테이너에서 50번째 이후 요소부터 계산
            int transformation_x = (vect[i].POINT_VEC_X) * cos((-1) * yaw * DEG2RAD) - (vect[i].POINT_VEC_Y) * sin((-1) * yaw * DEG2RAD);
            int transformation_y = (vect[i].POINT_VEC_X) * sin((-1) * yaw * DEG2RAD) + (vect[i].POINT_VEC_Y) * cos((-1) * yaw * DEG2RAD);
            if (transformation_x > X_MAX)
            {
                X_MAX = transformation_x;
            }
            if (transformation_x < X_MIN)
            {
                X_MIN = transformation_x;
            }
            if (transformation_y > Y_MAX)
            {
                Y_MAX = transformation_y;
            }
            if (transformation_y < Y_MIN)
            {
                Y_MIN = transformation_y;
            }
        }

        // 로봇의 좌표에서 최대 거리 값과 최소 거리 값의 평균 값을 더한 후 해당 값을 범위의 중앙으로 설정
        CIRCLE_CENTER = Point((int)(vect[0].STD_X + ((X_MAX + X_MIN) / 2) * cos(yaw * DEG2RAD) - ((Y_MAX + Y_MIN) / 2) * sin(yaw * DEG2RAD)), (int)(vect[0].STD_Y + ((X_MAX + X_MIN) / 2) * sin(yaw * DEG2RAD) + ((Y_MAX + Y_MIN) / 2) * cos(yaw * DEG2RAD)));

        // 로봇의 좌표에서 최대 거리 값과 최소 거리 값의 평균 값을 뺀 후 해당 값에서 +50 한 값을 범위의 반지름으로 설정
        CIRCLE_R = 0;
        for (int i = ago_point_cnt; i < vect.size(); i++)
        {
            double dis = sqrt(pow(CIRCLE_CENTER.x - vect[i].POINT_VEC_X - vect[i].STD_X, 2) + pow(CIRCLE_CENTER.y - vect[i].POINT_VEC_Y - vect[i].STD_Y, 2));
            if (CIRCLE_R < dis)
            {
                CIRCLE_R = dis;
            }
        }
        CIRCLE_R += 50;
        for (int i = 0; i < 27; i++)
        {
            if (pow(CIRCLE_R, 2) > pow(CIRCLE_CENTER.x - Local_point_x[i], 2) + pow(CIRCLE_CENTER.y - Local_point_y[i], 2))
            {
                Local_point_check[i] = 1;
            }
            else
            {
                Local_point_check[i] = 0;
            }
        }
    }
    double sence(int PT_X, int PT_Y, int NOW_X, int NOW_Y, vector<VISION_POINT> &vect)
    {
        // !!!!!!!제일 중요한 부분!!!!!!!
        // PRE CONDITION : PT_X, PT_Y, ROBOT_X, ROBOT_Y, VECTOR<VISION_POINT> = VISION_POINT_VECT
        // POST CONDITION : weight
        // PURPOSE : 가중치를 구하는 함수

        on_local_point(PT_X, PT_Y, NOW_X, NOW_Y); // 활성화 할 특징점 포인트 계산
        double weight = 0.0;
        int std_R = 50;

        for (int i = 0; i < vect.size(); i++)
        // 비전에서 포착한 포인트 수 많큼 연산
        {
            // pt = 파티클 좌표 + 계산에 사용된 로봇의 좌표 - 현 상태의 로봇 좌표 + 특징점과 로봇 사이의 거리
            int ptx = PT_X + vect[i].STD_X - NOW_X + vect[i].POINT_VEC_X;
            int pty = PT_Y + vect[i].STD_Y - NOW_Y + vect[i].POINT_VEC_Y;

            double min_dis = 99999999;

            for (int j = 0; j < 27; j++)
            {
                // 특징점 수 만큼 연산
                double dis = 99999999;
                if (Local_point_on_off[j] == 1) // 해당 특징점이 활성화 시 실행
                {
                    // 특징점의 로컬 좌표와 pt 사이의 거리
                    dis = sqrt(pow(Local_point_x[j] - ptx, 2) + pow(Local_point_y[j] - pty, 2));
                }
                // 해당 값이 전체 특징점 가운데서 가장 가까우면 해당 값의 거리 값을 저장
                if (min_dis > dis)
                {
                    min_dis = dis;
                }
            }
            if (min_dis <= 10)
            {
                min_dis = 10;
            }
            //            else if(min_dis >= 50){min_dis = (-1)*min_dis;}
            // 해당 값과 VISION_POINT의 신뢰도, 거리값을 통해 가중치 계산
            weight += (10 / min_dis) * vect[i].CONFIDENCE * abs(1 - vect[i].DISTANCE / 10000);
            // cout << "i : " << i << endl;
            // cout << "min_dis : " << min_dis << endl;
            // cout << "vect[i].CONFIDENCE : " << vect[i].CONFIDENCE << endl;
            // cout << "vect[i].DISTANCE : " << vect[i].DISTANCE << endl;
            // cout << "weight : " << weight << endl;
        }
        return weight;
    }
    //    double sence(int PT_X, int PT_Y, int NOW_X, int NOW_Y, vector<VISION_POINT> &vect)
    //    {
    //        on_local_point(PT_X, PT_Y, NOW_X, NOW_Y);
    //        double weight = 0.0;
    //        int std_R = 50;

    //        for(int i = 0; i < vect.size(); i++)
    //        {
    //            int ptx = PT_X + vect[i].STD_X - NOW_X + vect[i].POINT_VEC_X;
    //            int pty = PT_Y + vect[i].STD_Y - NOW_Y + vect[i].POINT_VEC_Y;
    //            for(int j = 0; j < 27; j++)
    //            {
    //                double dis = pow(Local_point_x[j] - ptx, 2) + pow(Local_point_y[j] - pty, 2);

    //                if(pow(std_R, 2) > dis)
    //                {
    //                    weight += ((-1) * double(sqrt(dis)) / double(std_R) + 1) * vect[i].CONFIDENCE * abs(1 - vect[i].DISTANCE / 10000);
    //                }
    //            }

    //        }
    //        return weight;

    //    }

private:
    int possibility_matching(int NUM, int N_NUM, int X_NUM, int L_NUM, int T_NUM)
    {
        if (NUM == 1)
        {
            return 0;
        }
        else if (NUM == 2)
        {
            if (T_NUM > 0)
            {
                return 1;
            }
        }
        else if (NUM == 3)
        {
            return 0;
        }
        else if (NUM == 4)
        {
            if (X_NUM > 0)
            {
                return 1;
            }
        }
        else if (NUM == 5)
        {
            if (X_NUM > 0 || L_NUM > 0)
            {
                return 1;
            }
        }
        else if (NUM == 6)
        {
            if (X_NUM > 0)
            {
                return 1;
            }
        }
        else if (NUM == 7)
        {
            return 0;
        }
        else if (NUM == 8)
        {
            if (T_NUM > 0)
            {
                return 1;
            }
        }
        else if (NUM == 9)
        {
            return 0;
        }
        return 0;
    }

    Rect get_grid_size(int Num)
    {
        int x = 0, y = 0, w = 0, h = 0;
        if (Num == 1)
        {
            x = 0;
            y = 0;
            w = 400;
            h = 300;
        }
        else if (Num == 2)
        {
            x = 400;
            y = 0;
            w = 300;
            h = 150;
        }
        else if (Num == 3)
        {
            x = 700;
            y = 0;
            w = 400;
            h = 300;
        }
        else if (Num == 4)
        {
            x = 0;
            y = 300;
            w = 400;
            h = 200;
        }
        else if (Num == 5)
        {
            x = 400;
            y = 150;
            w = 300;
            h = 500;
        }
        else if (Num == 6)
        {
            x = 700;
            y = 300;
            w = 400;
            h = 200;
        }
        else if (Num == 7)
        {
            x = 0;
            y = 500;
            w = 400;
            h = 300;
        }
        else if (Num == 8)
        {
            x = 400;
            y = 650;
            w = 300;
            h = 150;
        }
        else if (Num == 9)
        {
            x = 700;
            y = 500;
            w = 400;
            h = 300;
        }
        else
        {
            x = 0;
            y = 0;
            w = 0;
            h = 0;
        }

        Rect rect(x, y, w, h);
        return rect;
    }
    Mat get_grid_likelihood(int Num)
    {
        Rect bounds(0, 0, 1100, 800);
        Rect r = get_grid_size(Num);
        Mat roi = Likelihood_mat(r & bounds);
        return roi;
    }

    int get_grid_index(int X, int Y)
    {
        if (X > 0 && Y > 0 && X <= 400 && Y <= 300)
        {
            return 1;
        }
        else if (X > 400 && Y > 0 && X <= 700 && Y <= 150)
        {
            return 2;
        }
        else if (X > 700 && Y > 0 && X <= 1100 && Y <= 300)
        {
            return 3;
        }
        else if (X > 0 && Y > 300 && X <= 400 && Y <= 500)
        {
            return 4;
        }
        else if (X > 400 && Y > 150 && X <= 700 && Y <= 650)
        {
            return 5;
        }
        else if (X > 700 && Y > 300 && X <= 1100 && Y <= 500)
        {
            return 6;
        }
        else if (X > 0 && Y > 500 && X <= 400 && Y <= 800)
        {
            return 7;
        }
        else if (X > 400 && Y > 650 && X <= 700 && Y <= 800)
        {
            return 8;
        }
        else if (X > 700 && Y > 500 && X <= 1100 && Y <= 800)
        {
            return 9;
        }
        else
        {
            return 0;
        }
    }
};

#endif // LINE_H
