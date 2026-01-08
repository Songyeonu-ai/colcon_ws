/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date August 2024
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/robocup_localization25/main_window.hpp"

#include <QMessageBox>
#include <QtGui>
#include <fstream>
#include <iostream>

#include "/home/robit/colcon_ws/src/robot_config.h"

float particle_range = 0.0;  // is 100

struct MEASUREMENT {
  // 파티클의 가중치 값을 저장, NUM은 pt[i]의 인덱스 (i), WEIGHT는 pt[i]의
  // 가중치값
  int NUM;
  double WEIGHT;
};
MEASUREMENT measurement;
vector<MEASUREMENT>
    particle_weight;  // purpose : MEASUREMENT를 저장하는 벡터 컨테이너

double sort_return(MEASUREMENT x, MEASUREMENT y)
// PRE CONDITION : MEASUREMENT 구조체 두개
// POST CONDITION : 두 구조체 중 WEIGHT멤버가 더 큰 쪽이 리턴
// purpose : MEASUREMENT 구조체를 담는 particle_weight 벡터 컨테이너에서
// sort함수를 사용하기 위한 매개함수
{
  return x.WEIGHT > y.WEIGHT;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindowDesign) {
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  m_Timer = new QTimer(this);
  connect(m_Timer, SIGNAL(timeout()), this, SLOT(main()));
  m_Timer->start(100);

  // 맵 이미지의 절대주소를 저장하는 QString 변수

  QString img_path =
      "src/robocup_localization25/resources/Map/2022_RoboCup_Field.png";
  QImage img(img_path);           // QString 변수를 이용한 QImage 변수
  buf = QPixmap::fromImage(img);  // QPixmap을 저장하는 버퍼
  buf = buf.scaled(825, 600);     // 버퍼 리스케일링

  scene = new QGraphicsScene(this);
  qnode->robot0.x = 550;
  qnode->robot0.y = 400;  // 로봇 시작 포인트, 맵의 중앙

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(featureReceived()), this, SLOT(featureCalc()));
}

void MainWindow::main() {
  // INIT
  setting();       // 공 선택 & 파라미터 불러오기 & UI에서 오도메트리 값 설정
  sel_ball();      // UDP 통신에서 받은 다른 로봇의 공데이터로 공 위치 설정 &
                   // obstacle 위치 계산
  Print_Screen();  // UI에 로봇 공 obstacle 파티클 그리기
  publish_msg();   // 다른 노드로 데이터 publish
}

void MainWindow::sel_ball() {
  // PRE CONDITION : robotX.b, qnode->visionMSG, ball 데이터 - 해당 데이터는
  // 다른 노드에서 통신받아 옴 POST CONDITION : obstacleX, ball 데이터 purpose :
  // 다른 로봇에서 받아 온 공 데이터로 최종 공 위치 설정, 이때 최종 공 위치는 각
  // 로봇의 공 데이터 중 가장 공과 가까운 로봇의 데이터로 설정 & obstacle 의
  // 위치 설정

  // 로봇의 공 데이터 초기화
  qnode->robot1.b -= 0.005;
  qnode->robot2.b -= 0.005;
  qnode->robot3.b -= 0.005;
  qnode->robot4.b -= 0.005;

  // 다른 로봇에서 공 데이터를 받아오지 않으면, 해당 로봇의 데이터 비활성화
  if (qnode->robot1.b <= 0) {
    qnode->robot1.x = 0;
    qnode->robot1.y = 0;
    qnode->robot1.z = 0;
    qnode->robot1.b = 0;
    qnode->robot1.state = 0;
    qnode->ball1.x = 0;
    qnode->ball1.y = 0;
    qnode->ball1.d = 999999;
  }
  if (qnode->robot2.b <= 0) {
    qnode->robot2.x = 0;
    qnode->robot2.y = 0;
    qnode->robot2.z = 0;
    qnode->robot2.b = 0;
    qnode->robot2.state = 0;
    qnode->ball2.x = 0;
    qnode->ball2.y = 0;
    qnode->ball2.d = 999999;
  }
  if (qnode->robot3.b <= 0) {
    qnode->robot3.x = 0;
    qnode->robot3.y = 0;
    qnode->robot3.z = 0;
    qnode->robot3.b = 0;
    qnode->robot3.state = 0;
    qnode->ball3.x = 0;
    qnode->ball3.y = 0;
    qnode->ball3.d = 999999;
  }
  if (qnode->robot4.b <= 0) {
    qnode->robot4.x = 0;
    qnode->robot4.y = 0;
    qnode->robot4.z = 0;
    qnode->robot4.b = 0;
    qnode->robot4.state = 0;
    qnode->ball4.x = 0;
    qnode->ball4.y = 0;
    qnode->ball4.d = 999999;
  }

  // qnode->visionMSG.ROBOT_VEC_X_size 해당 백터 콘테이너의 크기를 판별하여
  // 크기만큼 obstacle 계산 ex) 콘테이너 크기 2 -> qnode->obstacle0,
  // qnode->obstacle1 계산
  if (qnode->visionMSG.ROBOT_VEC_X.size() > 0) {
    qnode->obstacle0.x =
        qnode->robot0.x + (-1) * (qnode->visionMSG.ROBOT_VEC_X[0] / 10 *
                                      cos((qnode->robot0.z) * M_PI / 180) +
                                  qnode->visionMSG.ROBOT_VEC_Y[0] / 10 *
                                      sin((qnode->robot0.z) * M_PI / 180));
    qnode->obstacle0.y =
        qnode->robot0.y + (qnode->visionMSG.ROBOT_VEC_X[0] / 10 *
                               sin((qnode->robot0.z) * M_PI / 180) -
                           qnode->visionMSG.ROBOT_VEC_Y[0] / 10 *
                               cos((qnode->robot0.z) * M_PI / 180));
    qnode->obstacle0.nocnt = 0;
  } else {
    qnode->obstacle0.nocnt += 1;
    if (qnode->obstacle0.nocnt > 20) {
      qnode->obstacle0.nocnt = 20;
      qnode->obstacle0.x = 0;
      qnode->obstacle0.y = 0;
    }
  }
  if (qnode->visionMSG.ROBOT_VEC_X.size() > 1) {
    qnode->obstacle1.x =
        qnode->robot0.x + (-1) * (qnode->visionMSG.ROBOT_VEC_X[1] / 10 *
                                      cos((qnode->robot0.z) * M_PI / 180) +
                                  qnode->visionMSG.ROBOT_VEC_Y[1] / 10 *
                                      sin((qnode->robot0.z) * M_PI / 180));
    qnode->obstacle1.y =
        qnode->robot0.y + (qnode->visionMSG.ROBOT_VEC_X[1] / 10 *
                               sin((qnode->robot0.z) * M_PI / 180) -
                           qnode->visionMSG.ROBOT_VEC_Y[1] / 10 *
                               cos((qnode->robot0.z) * M_PI / 180));
    qnode->obstacle1.nocnt = 0;
  } else {
    qnode->obstacle1.nocnt += 1;
    if (qnode->obstacle1.nocnt > 20) {
      qnode->obstacle1.nocnt = 20;
      qnode->obstacle1.x = 0;
      qnode->obstacle1.y = 0;
    }
  }
  if (qnode->visionMSG.ROBOT_VEC_X.size() > 2) {
    qnode->obstacle2.x =
        qnode->robot0.x + (-1) * (qnode->visionMSG.ROBOT_VEC_X[2] / 10 *
                                      cos((qnode->robot0.z) * M_PI / 180) +
                                  qnode->visionMSG.ROBOT_VEC_Y[2] / 10 *
                                      sin((qnode->robot0.z) * M_PI / 180));
    qnode->obstacle2.y =
        qnode->robot0.y + (qnode->visionMSG.ROBOT_VEC_X[2] / 10 *
                               sin((qnode->robot0.z) * M_PI / 180) -
                           qnode->visionMSG.ROBOT_VEC_Y[2] / 10 *
                               cos((qnode->robot0.z) * M_PI / 180));
    qnode->obstacle2.nocnt = 0;
  } else {
    qnode->obstacle2.nocnt += 1;
    if (qnode->obstacle2.nocnt > 20) {
      qnode->obstacle2.nocnt = 20;
      qnode->obstacle2.x = 0;
      qnode->obstacle2.y = 0;
    }
  }
  if (qnode->visionMSG.ROBOT_VEC_X.size() > 3) {
    qnode->obstacle3.x =
        qnode->robot0.x + (-1) * (qnode->visionMSG.ROBOT_VEC_X[3] / 10 *
                                      cos((qnode->robot0.z) * M_PI / 180) +
                                  qnode->visionMSG.ROBOT_VEC_Y[3] / 10 *
                                      sin((qnode->robot0.z) * M_PI / 180));
    qnode->obstacle3.y =
        qnode->robot0.y + (qnode->visionMSG.ROBOT_VEC_X[3] / 10 *
                               sin((qnode->robot0.z) * M_PI / 180) -
                           qnode->visionMSG.ROBOT_VEC_Y[3] / 10 *
                               cos((qnode->robot0.z) * M_PI / 180));
    qnode->obstacle3.nocnt = 0;
  } else {
    qnode->obstacle3.nocnt += 1;
    if (qnode->obstacle3.nocnt > 20) {
      qnode->obstacle3.nocnt = 20;
      qnode->obstacle3.x = 0;
      qnode->obstacle3.y = 0;
    }
  }

  // qnode->visionMSG.Ball_2d 의 데이터를 통해 비전에서 받아온 공 데이터를 로컬
  // 좌표로 변환
  if (qnode->visionMSG.Ball_2d_X != 0 || qnode->visionMSG.Ball_2d_Y != 0) {
    qnode->ball.noballcnt = 0;
    qnode->ball.x =
        qnode->robot0.x + (-1) * (qnode->visionMSG.Ball_2d_X / 10 *
                                      cos((qnode->robot0.z) * M_PI / 180) +
                                  qnode->visionMSG.Ball_2d_Y / 10 *
                                      sin((qnode->robot0.z) * M_PI / 180));
    qnode->ball.y =
        qnode->robot0.y +
        (qnode->visionMSG.Ball_2d_X / 10 * sin((qnode->robot0.z) * M_PI / 180) -
         qnode->visionMSG.Ball_2d_Y / 10 * cos((qnode->robot0.z) * M_PI / 180));
    qnode->ball.speed_x =
        (qnode->visionMSG.Ball_speed_X) *
            cos((-1) * qnode->robot0.z * DEG2RAD) -
        (qnode->visionMSG.Ball_speed_Y) * sin((-1) * qnode->robot0.z * DEG2RAD);
    qnode->ball.speed_y =
        (qnode->visionMSG.Ball_speed_X) *
            sin((-1) * qnode->robot0.z * DEG2RAD) +
        (qnode->visionMSG.Ball_speed_Y) * cos((-1) * qnode->robot0.z * DEG2RAD);
  }
  // 데이터를 받지 않으면 데이터 비활성화
  else {
    qnode->ball.noballcnt += 1;
    if (qnode->ball.noballcnt >= 1000) {
      qnode->ball.x = 0;
      qnode->ball.y = 0;
      qnode->ball.noballcnt = 1000;
    }
  }

  double min_d = qnode->visionMSG.Ball_D /
                 10;  // 공 최종위치 판별 시 공 위치를 판별할 예외처리 변수 선언
  if (min_d == 0) {
    min_d = 999999;
  }
  int min_i = 0;

  // 각 로봇에서 측정된 공 데이터를 비교하여 해당 공 데이터가 최소값일
  // 경우(측정된 로봇과 가장 가까울 경우) 해당 값 적용
  if (min_d > qnode->ball1.d) {
    min_d = qnode->ball1.d;
    min_i = 1;
  }
  if (min_d > qnode->ball2.d) {
    min_d = qnode->ball2.d;
    min_i = 2;
  }
  if (min_d > qnode->ball3.d) {
    min_d = qnode->ball3.d;
    min_i = 3;
  }
  if (min_d > qnode->ball4.d) {
    min_d = qnode->ball4.d;
    min_i = 4;
  }

  // 최소값인 공 데이터가 해당 로봇이 아닐때 최소값인 공 데이터 적용
  if (min_i != 0 &&
      (qnode->ball.noballcnt == 0 || qnode->ball.noballcnt == 100)) {
    if (min_i == 1) {
      qnode->ball.x = qnode->ball1.x;
      qnode->ball.y = qnode->ball1.y;
    }
    if (min_i == 2) {
      qnode->ball.x = qnode->ball2.x;
      qnode->ball.y = qnode->ball2.y;
    }
    if (min_i == 3) {
      qnode->ball.x = qnode->ball3.x;
      qnode->ball.y = qnode->ball3.y;
    }
    if (min_i == 4) {
      qnode->ball.x = qnode->ball4.x;
      qnode->ball.y = qnode->ball4.y;
    }
    qnode->ball.noballcnt = 25;
  }

  // 공 데이터 예외처리
  if (qnode->ball.x < 100 && qnode->ball.x != 0) {
    qnode->ball.x = 100;
  }
  if (qnode->ball.x > 1000) {
    qnode->ball.x = 1000;
  }
  if (qnode->ball.y < 100 && qnode->ball.y != 0) {
    qnode->ball.y = 100;
  }
  if (qnode->ball.y > 700) {
    qnode->ball.y = 700;
  }

  if (qnode->obstacle0.x < 100 && qnode->obstacle0.x != 0) {
    qnode->obstacle0.x = 100;
  }
  if (qnode->obstacle0.x > 1000) {
    qnode->obstacle0.x = 1000;
  }
  if (qnode->obstacle0.y < 100 && qnode->obstacle0.y != 0) {
    qnode->obstacle0.y = 100;
  }
  if (qnode->obstacle0.y > 700) {
    qnode->obstacle0.y = 700;
  }

  if (qnode->obstacle1.x < 100 && qnode->obstacle1.x != 0) {
    qnode->obstacle1.x = 100;
  }
  if (qnode->obstacle1.x > 1000) {
    qnode->obstacle1.x = 1000;
  }
  if (qnode->obstacle1.y < 100 && qnode->obstacle1.y != 0) {
    qnode->obstacle1.y = 100;
  }
  if (qnode->obstacle1.y > 700) {
    qnode->obstacle1.y = 700;
  }

  if (qnode->obstacle2.x < 100 && qnode->obstacle2.x != 0) {
    qnode->obstacle2.x = 100;
  }
  if (qnode->obstacle2.x > 1000) {
    qnode->obstacle2.x = 1000;
  }
  if (qnode->obstacle2.y < 100 && qnode->obstacle2.y != 0) {
    qnode->obstacle2.y = 100;
  }
  if (qnode->obstacle2.y > 700) {
    qnode->obstacle2.y = 700;
  }

  if (qnode->obstacle3.x < 100 && qnode->obstacle3.x != 0) {
    qnode->obstacle3.x = 100;
  }
  if (qnode->obstacle3.x > 1000) {
    qnode->obstacle3.x = 1000;
  }
  if (qnode->obstacle3.y < 100 && qnode->obstacle3.y != 0) {
    qnode->obstacle3.y = 100;
  }
  if (qnode->obstacle3.y > 700) {
    qnode->obstacle3.y = 700;
  }
}

void MainWindow::setting() {
  // PRE CONDITION : qnode->vision_callback_timer, qnode->set_ball_flag,
  // ball_set_N, setting_flag, 파라미터 저장파일 POST CONDITION : ball_N,
  // qnode->robot0.odom_Nn purpose : 로봇의 초기 세팅 및 입출력스트림을 통해
  // 저장되어있던 오도메트리 값 설정, 볼의 위치를 재설정 하는 경우 볼 위치 변경,
  // ui슬라이더를 통해 오도메트리 값 적용

  qnode->vision_callback_timer += 1;  // vision timer count +1
  if (qnode->set_ball_flag)  // qnode->set_ball_flag가 1일때 공 위치 변경
  {
    qnode->ball.x = qnode->ball.set_x;
    qnode->ball.y = qnode->ball.set_y;
    qnode->ball.noballcnt = 0;
  }
  if (qnode->vision_callback_timer > 10 &&
      qnode->set_ball_flag == 0)  // qnode->set_ball_flag가 비활성화 상태이고
                                  // timer가 10 이상이면 공 위치 비활성화
  {
    qnode->ball.x = 0;
    qnode->ball.y = 0;
  }

  if (setting_flag ==
      0)  // setting_flag가 0이면 입출력스트림을 통해 데이터 로드
  {
    String param = "";
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_1
    param = "src/robocup_localization25/resources/param/PARAM1.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_2
    param = "src/robocup_localization25/resources/param/PARAM2.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_3
    param = "src/robocup_localization25/resources/param/PARAM3.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_4
    param = "src/robocup_localization25/resources/param/PARAM4.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_5
    param = "src/robocup_localization25/resources/param/PARAM5.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_6
    param = "src/robocup_localization25/resources/param/PARAM6.txt";
#endif
    ifstream Last_Index_Num_IN(param);
    if (Last_Index_Num_IN.is_open()) {
      Last_Index_Num_IN >> particle_range;
      Last_Index_Num_IN >> qnode->robot0.odom_fx;
      Last_Index_Num_IN >> qnode->robot0.odom_bx;
      Last_Index_Num_IN >> qnode->robot0.odom_ly;
      Last_Index_Num_IN >> qnode->robot0.odom_ry;
      std::cout << "Odom Load" << std::endl;
    } else {
      std::cout << "No Odom Data" << std::endl;
    }
    Last_Index_Num_IN
        .close();  // setting_flag가 0이면 로드 된 데이터를 ui슬라이더에 적용
    ui->particle_range_value->setValue(particle_range);
    ui->odom_fx_value->setValue(qnode->robot0.odom_fx);
    ui->odom_bx_value->setValue(qnode->robot0.odom_bx);
    ui->odom_ly_value->setValue(qnode->robot0.odom_ly);
    ui->odom_ry_value->setValue(qnode->robot0.odom_ry);
    setting_flag = 1;  // setting_flag set 1
  }
  // ui의 슬라이더 값을 오도메트리 값에 적용
  particle_range = ui->particle_range_value->value();
  qnode->robot0.odom_fx = ui->odom_fx_value->value();
  qnode->robot0.odom_bx = ui->odom_bx_value->value();
  qnode->robot0.odom_ly = ui->odom_ly_value->value();
  qnode->robot0.odom_ry = ui->odom_ry_value->value();
}

void MainWindow::publish_msg() {
  // PRE CONDITION : ball_n, ball_speed_n, qnode->visionMSG.Ball_speed_level,
  // qnode->robot0.n, obstacleX.n POST CONDITION : localizationMsg purpose :
  // 다른 노드로 로컬 데이터를 전송 & 로컬 데이터 ui에 표시

  localizationMsg.ball_x = qnode->ball.x;
  localizationMsg.ball_y = qnode->ball.y;
  localizationMsg.ball_speed_x =
      qnode->ball.x + qnode->ball.speed_x * qnode->visionMSG.Ball_speed_level;
  localizationMsg.ball_speed_y =
      qnode->ball.y + qnode->ball.speed_y * qnode->visionMSG.Ball_speed_level;
  localizationMsg.robot_x = qnode->robot0.x;
  localizationMsg.robot_y = qnode->robot0.y;
  localizationMsg.obstacle0_x = qnode->obstacle0.x;
  localizationMsg.obstacle0_y = qnode->obstacle0.y;
  localizationMsg.obstacle1_x = qnode->obstacle1.x;
  localizationMsg.obstacle1_y = qnode->obstacle1.y;
  localizationMsg.obstacle2_x = qnode->obstacle2.x;
  localizationMsg.obstacle2_y = qnode->obstacle2.y;
  localizationMsg.obstacle3_x = qnode->obstacle3.x;
  localizationMsg.obstacle3_y = qnode->obstacle3.y;
  qnode->localization_pub_->publish(localizationMsg);

  // 로컬 데이터 ui에 표시
  char TEXT[256];
  sprintf(TEXT, "BALL_X : %d", (int)localizationMsg.ball_x);
  ui->BALL_X_value->setText(TEXT);
  sprintf(TEXT, "BALL_Y : %d", (int)localizationMsg.ball_y);
  ui->BALL_Y_value->setText(TEXT);
  sprintf(TEXT, "ROBOT_X : %d", (int)localizationMsg.robot_x);
  ui->ROBOT_X_value->setText(TEXT);
  sprintf(TEXT, "ROBOT_Y : %d", (int)localizationMsg.robot_y);
  ui->ROBOT_Y_value->setText(TEXT);
}

void MainWindow::Print_Screen()  // ui 출력
{
  // PRE CONDITION : robotX.n, obstacleX.n, qnode->ball.n, qnode->Likelihood
  // POST CONDITION : UI
  // purpose : UI출력

  // INIT
  if (scene) {
    delete scene;
  }

  char TEXT[256];
  sprintf(TEXT, "IK_X : %d", (int)qnode->robot0.now_ik_param_x);
  ui->IK_X_value->setText(TEXT);
  sprintf(TEXT, "IK_Y : %d", (int)qnode->robot0.now_ik_param_y);
  ui->IK_Y_value->setText(TEXT);

  scene = new QGraphicsScene(this);
  ui->screen->setScene(scene);
  scene->addPixmap(buf);

  QBrush blueBrush(Qt::blue);
  QBrush redBrush(Qt::red);
  QBrush greenBrush(Qt::green);
  QBrush yellowBrush(Qt::yellow);
  QBrush blackBrush(Qt::black);
  QBrush grayBrush(Qt::gray);
  QPen grayPen(Qt::gray);
  grayPen.setWidth(1);
  QPen redPen(Qt::red);
  redPen.setWidth(2);
  QPen redPen3(Qt::red);
  redPen3.setWidth(3);
  QPen greenPen(Qt::green);
  greenPen.setWidth(2);
  QPen bluePen(Qt::blue);
  bluePen.setWidth(2);
  QPen blackPen1(Qt::black);
  blackPen1.setWidth(1);
  QPen blackPen(Qt::black);
  blackPen.setWidth(2);
  QPen blackPen3(Qt::black);
  blackPen3.setWidth(3);
  QPen LocalPen1(Qt::blue);
  LocalPen1.setWidth(5);
  QPen LocalPen2(Qt::green);
  LocalPen2.setWidth(5);
  QPen LocalPen3(Qt::yellow);
  LocalPen3.setWidth(5);
  QPen LocalPen4(Qt::red);
  LocalPen4.setWidth(5);
  QPen yellowPen(Qt::yellow);
  yellowPen.setWidth(3);
  QPen targetPen(Qt::red);
  targetPen.setWidth(3);
  QPen targetlinePen(Qt::gray);
  targetlinePen.setWidth(2);

  // print obstacle
  if (qnode->obstacle0.x != 0 && qnode->obstacle0.y != 0) {
    scene->addEllipse(cvt_Print_xy(qnode->obstacle0.x) - 35,
                      cvt_Print_xy(qnode->obstacle0.y) - 35, 70, 70, grayPen,
                      grayBrush);
  }
  if (qnode->obstacle1.x != 0 && qnode->obstacle1.y != 0) {
    scene->addEllipse(cvt_Print_xy(qnode->obstacle1.x) - 35,
                      cvt_Print_xy(qnode->obstacle1.y) - 35, 70, 70, grayPen,
                      grayBrush);
  }
  if (qnode->obstacle2.x != 0 && qnode->obstacle2.y != 0) {
    scene->addEllipse(cvt_Print_xy(qnode->obstacle2.x) - 35,
                      cvt_Print_xy(qnode->obstacle2.y) - 35, 70, 70, grayPen,
                      grayBrush);
  }
  if (qnode->obstacle3.x != 0 && qnode->obstacle3.y != 0) {
    scene->addEllipse(cvt_Print_xy(qnode->obstacle3.x) - 35,
                      cvt_Print_xy(qnode->obstacle3.y) - 35, 70, 70, grayPen,
                      grayBrush);
  }

  // PRINT PARTICLE
  if (true) {
    for (int i = 0; i < PARTICLE_NUM; i++) {
      scene->addLine(cvt_Print_xy(qnode->pt[i].x), cvt_Print_xy(qnode->pt[i].y),
                     cvt_Print_xy(qnode->pt[i].x), cvt_Print_xy(qnode->pt[i].y),
                     grayPen);
    }
  }

  // PRINT TIMESTAMP
  if (qnode->robot0.TIME_STAMP.size() > 0) {
    scene->addLine(
        cvt_Print_xy(qnode->robot0.x), cvt_Print_xy(qnode->robot0.y),
        cvt_Print_xy(
            qnode->robot0.TIME_STAMP[qnode->robot0.TIME_STAMP.size() - 1].X),
        cvt_Print_xy(
            qnode->robot0.TIME_STAMP[qnode->robot0.TIME_STAMP.size() - 1].Y),
        blackPen3);
    for (int i = 1; i < qnode->robot0.TIME_STAMP.size(); i++) {
      scene->addLine(cvt_Print_xy(qnode->robot0.TIME_STAMP[i - 1].X),
                     cvt_Print_xy(qnode->robot0.TIME_STAMP[i - 1].Y),
                     cvt_Print_xy(qnode->robot0.TIME_STAMP[i].X),
                     cvt_Print_xy(qnode->robot0.TIME_STAMP[i].Y), blackPen3);
    }
  }

  // PRINT SIGHT
  // if(qnode->visionMSG.Scan_mode != 3){qnode->robot_sight_flag = 0;
  // qnode->Likelihood.vision_point_vect.clear();}

  // 비전에서 측정된 특징점 표시
  for (int i = 0; i < 27; i++) {
    if (qnode->Likelihood.Local_point_on_off[i] == 1) {
      scene->addEllipse(cvt_Print_xy(qnode->Likelihood.Local_point_x[i]) - 4,
                        cvt_Print_xy(qnode->Likelihood.Local_point_y[i]) - 4, 8,
                        8, blackPen1);
    }
  }
  // cout<<qnode->Likelihood.vision_point_vect.size()<<endl;
  for (int i = 0; i < qnode->Likelihood.vision_point_vect.size(); i++) {
    // 파티클 중 신뢰도가 0.9이상인 파티클의 특징점 위치 출력
    if (qnode->Likelihood.vision_point_vect[i].CONFIDENCE > 0.9) {
      scene->addLine(
          cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_X +
                       qnode->Likelihood.vision_point_vect[i].POINT_VEC_X),
          cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_Y +
                       qnode->Likelihood.vision_point_vect[i].POINT_VEC_Y),
          cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_X +
                       qnode->Likelihood.vision_point_vect[i].POINT_VEC_X),
          cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_Y +
                       qnode->Likelihood.vision_point_vect[i].POINT_VEC_Y),
          LocalPen1);
    }
    // 파티클 중 신뢰도가 0.7이상인 파티클의 특징점 위치 출력
    else if (qnode->Likelihood.vision_point_vect[i].CONFIDENCE > 0.7) {
      scene->addLine(
          cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_X +
                       qnode->Likelihood.vision_point_vect[i].POINT_VEC_X),
          cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_Y +
                       qnode->Likelihood.vision_point_vect[i].POINT_VEC_Y),
          cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_X +
                       qnode->Likelihood.vision_point_vect[i].POINT_VEC_X),
          cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_Y +
                       qnode->Likelihood.vision_point_vect[i].POINT_VEC_Y),
          LocalPen2);
    }
    // 파티클 중 신뢰도가 0.5이상인 파티클의 특징점 위치 출력
    else if (qnode->Likelihood.vision_point_vect[i].CONFIDENCE > 0.5) {
      scene->addLine(
          cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_X +
                       qnode->Likelihood.vision_point_vect[i].POINT_VEC_X),
          cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_Y +
                       qnode->Likelihood.vision_point_vect[i].POINT_VEC_Y),
          cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_X +
                       qnode->Likelihood.vision_point_vect[i].POINT_VEC_X),
          cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_Y +
                       qnode->Likelihood.vision_point_vect[i].POINT_VEC_Y),
          LocalPen3);
    }
    //        else
    //        {
    //            scene->addLine(cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_X
    //            + qnode->Likelihood.vision_point_vect[i].POINT_VEC_X),
    //            cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_Y +
    //            qnode->Likelihood.vision_point_vect[i].POINT_VEC_Y),
    //            cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_X +
    //            qnode->Likelihood.vision_point_vect[i].POINT_VEC_X),
    //            cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_Y +
    //            qnode->Likelihood.vision_point_vect[i].POINT_VEC_Y),
    //            LocalPen4);
    //        }
  }
  if (qnode->Likelihood.vision_point_vect.size() != 0) {
    // cout<<qnode->Likelihood.CIRCLE_R<<endl;
    scene->addEllipse(cvt_Print_xy(qnode->Likelihood.CIRCLE_CENTER.x -
                                   qnode->Likelihood.CIRCLE_R),
                      cvt_Print_xy(qnode->Likelihood.CIRCLE_CENTER.y -
                                   qnode->Likelihood.CIRCLE_R),
                      cvt_Print_xy(qnode->Likelihood.CIRCLE_R * 2),
                      cvt_Print_xy(qnode->Likelihood.CIRCLE_R * 2), blackPen);
    // scene->addLine(cvt_Print_xy(qnode->Likelihood.CIRCLE_CENTER.x),
    // cvt_Print_xy(qnode->Likelihood.CIRCLE_CENTER.y),
    // cvt_Print_xy(qnode->Likelihood.CIRCLE_CENTER.x),
    // cvt_Print_xy(qnode->Likelihood.CIRCLE_CENTER.y), targetPen);
  }

  // PRINT MASTER TARGET
  if (qnode->master_target_x != 0 || qnode->master_target_y != 0) {
    scene->addLine(cvt_Print_xy(qnode->master_target_x) + 4,
                   cvt_Print_xy(qnode->master_target_y),
                   cvt_Print_xy(qnode->master_target_x) - 4,
                   cvt_Print_xy(qnode->master_target_y), targetPen);
    scene->addLine(cvt_Print_xy(qnode->master_target_x),
                   cvt_Print_xy(qnode->master_target_y) + 4,
                   cvt_Print_xy(qnode->master_target_x),
                   cvt_Print_xy(qnode->master_target_y) - 4, targetPen);
    scene->addLine(cvt_Print_xy(qnode->master_target_x) + 2,
                   cvt_Print_xy(qnode->master_target_y) + 2,
                   cvt_Print_xy(qnode->robot0.x), cvt_Print_xy(qnode->robot0.y),
                   targetlinePen);
  }

  // PRINT ROBOT
  QPolygonF robot0_poly = create_Print_robot(qnode->robot0);
  if (qnode->robot1.state != 3 && qnode->robot2.state != 3 &&
      qnode->robot3.state != 3 && qnode->robot4.state != 3 &&
      (qnode->ball.x != 0 || qnode->ball.y != 0)) {
    scene->addPolygon(robot0_poly, redPen3, blueBrush);
  } else {
    scene->addPolygon(robot0_poly, blackPen, blueBrush);
  }
  scene->addLine(
      cvt_Print_xy(qnode->robot0.x), cvt_Print_xy(qnode->robot0.y),
      cvt_Print_xy(qnode->robot0.x) +
          sin((-1) * qnode->robot0.z * DEG2RAD) * 10,
      cvt_Print_xy(qnode->robot0.y) - cos(qnode->robot0.z * DEG2RAD) * 10,
      blackPen3);

  if (qnode->robot1.x != 0 || qnode->robot1.y != 0) {
    QPolygonF robot1_poly = create_Print_robot(qnode->robot1);

    if (qnode->robot1.state == 3) {
      scene->addPolygon(robot1_poly, redPen3, redBrush);
    } else {
      scene->addPolygon(robot1_poly, blackPen, redBrush);
    }
    scene->addLine(
        cvt_Print_xy(qnode->robot1.x), cvt_Print_xy(qnode->robot1.y),
        cvt_Print_xy(qnode->robot1.x) +
            sin((-1) * qnode->robot1.z * DEG2RAD) * 10,
        cvt_Print_xy(qnode->robot1.y) - cos(qnode->robot1.z * DEG2RAD) * 10,
        blackPen3);
  }
  if (qnode->robot2.x != 0 || qnode->robot2.y != 0) {
    QPolygonF robot2_poly = create_Print_robot(qnode->robot2);
    if (qnode->robot2.state == 3) {
      scene->addPolygon(robot2_poly, redPen3, greenBrush);
    } else {
      scene->addPolygon(robot2_poly, blackPen, greenBrush);
    }
    scene->addLine(
        cvt_Print_xy(qnode->robot2.x), cvt_Print_xy(qnode->robot2.y),
        cvt_Print_xy(qnode->robot2.x) +
            sin((-1) * qnode->robot2.z * DEG2RAD) * 10,
        cvt_Print_xy(qnode->robot2.y) - cos(qnode->robot2.z * DEG2RAD) * 10,
        blackPen3);
  }
  if (qnode->robot3.x != 0 || qnode->robot3.y != 0) {
    QPolygonF robot3_poly = create_Print_robot(qnode->robot3);
    if (qnode->robot3.state == 3) {
      scene->addPolygon(robot3_poly, redPen3, yellowBrush);
    } else {
      scene->addPolygon(robot3_poly, blackPen, yellowBrush);
    }
    scene->addLine(
        cvt_Print_xy(qnode->robot3.x), cvt_Print_xy(qnode->robot3.y),
        cvt_Print_xy(qnode->robot3.x) +
            sin((-1) * qnode->robot3.z * DEG2RAD) * 10,
        cvt_Print_xy(qnode->robot3.y) - cos(qnode->robot3.z * DEG2RAD) * 10,
        blackPen3);
  }
  if (qnode->robot4.x != 0 || qnode->robot4.y != 0) {
    QPolygonF robot4_poly = create_Print_robot(qnode->robot4);
    if (qnode->robot4.state == 3) {
      scene->addPolygon(robot4_poly, redPen3, blackBrush);
    } else {
      scene->addPolygon(robot4_poly, blackPen, blackBrush);
    }
    scene->addLine(
        cvt_Print_xy(qnode->robot4.x), cvt_Print_xy(qnode->robot4.y),
        cvt_Print_xy(qnode->robot4.x) +
            sin((-1) * qnode->robot4.z * DEG2RAD) * 10,
        cvt_Print_xy(qnode->robot4.y) - cos(qnode->robot4.z * DEG2RAD) * 10,
        blackPen3);
  }

  // PRINT BALL
  if (qnode->visionMSG.Ball_2d_X != 0 || qnode->visionMSG.Ball_2d_Y != 0) {
    scene->addEllipse(cvt_Print_xy(qnode->ball.x) - 5,
                      cvt_Print_xy(qnode->ball.y) - 5, 11, 11, blackPen,
                      redBrush);
    scene->addLine(
        cvt_Print_xy(qnode->ball.x), cvt_Print_xy(qnode->ball.y),
        cvt_Print_xy(qnode->ball.x +
                     qnode->ball.speed_x * qnode->visionMSG.Ball_speed_level),
        cvt_Print_xy(qnode->ball.y +
                     qnode->ball.speed_y * qnode->visionMSG.Ball_speed_level),
        blackPen);
  } else {
    if (qnode->ball.x != 0 || qnode->ball.y != 0) {
      scene->addEllipse(cvt_Print_xy(qnode->ball.x) - 5,
                        cvt_Print_xy(qnode->ball.y) - 5, 11, 11, blackPen,
                        yellowBrush);
    }
  }

  if (qnode->set_ball_flag) {
    scene->addEllipse(cvt_Print_xy(qnode->ball.x) - 5,
                      cvt_Print_xy(qnode->ball.y) - 5, 11, 11, blackPen,
                      blueBrush);
  }
}

void MainWindow::mouseReleaseEvent(QMouseEvent *e) {
  // PRE CONDITION : mouseEvent, set_robot_flag, qnode->set_ball_flag
  // POST CONDITION : qnode->robot0.n, qnode->ball.n master_target_n,
  // purpose : set_object_flag가 활성화 되있고, 마우스 클릭 이벤트가 적용 시
  // 해당 object를 마우스 위치로 이동 & 이동한 위치에 파티클 생성

  QPointF point = mapToParent(e->pos());
  QPoint position = mapToGlobal(QPoint(21, 60));
  if (set_robot_flag) {
    qnode->robot0.x = (point.x() - position.x()) * 100 / 75;
    qnode->robot0.y = (point.y() - position.y()) * 100 / 75;
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
    set_robot_flag = 0;
    qnode->robot0.TIME_STAMP.clear();
    qnode->master_target_x = 0;
    qnode->master_target_y = 0;
  }

  if (qnode->set_ball_flag) {
    qnode->ball.x = (point.x() - position.x()) * 100 / 75;
    qnode->ball.y = (point.y() - position.y()) * 100 / 75;
    qnode->ball.set_x = qnode->ball.x;
    qnode->ball.set_y = qnode->ball.y;
    qnode->ball.noballcnt = 0;
  }
}

void MainWindow::mouseMoveEvent(QMouseEvent *e) {
  // PRE CONDITION : mouseEvent, set_robot_flag, qnode->set_ball_flag
  // POST CONDITION : qnode->robot0.n, qnode->ball.n master_target_n,
  // purpose : set_object_flag가 활성화 되있고, 마우스 이동 이벤트가 적용 시
  // 해당 object를 마우스 위치로 이동 & 이동한 위치에 파티클 생성
  QPointF point = mapToParent(e->pos());
  QPoint position = mapToGlobal(QPoint(21, 60));
  if (set_robot_flag) {
    qnode->robot0.x = (point.x() - position.x()) * 100 / 75;
    qnode->robot0.y = (point.y() - position.y()) * 100 / 75;
    qnode->robot0.TIME_STAMP.clear();
    qnode->master_target_x = 0;
    qnode->master_target_y = 0;
  }
  if (qnode->set_ball_flag) {
    qnode->ball.x = (point.x() - position.x()) * 100 / 75;
    qnode->ball.y = (point.y() - position.y()) * 100 / 75;
    qnode->ball.set_x = qnode->ball.x;
    qnode->ball.set_y = qnode->ball.y;
    qnode->ball.noballcnt = 0;
  }
}

QPolygonF MainWindow::create_Print_robot(ROBOT robot) {
  // PRE CONDITION : robot.n, zoom(0.75)
  // POST CONDITION : Robot_model_poly
  // purpose : 로봇의 x, y, z좌표에 zoom값을 곱한 후 해당 값에 의한 폴리곤 생성

  float zoom = 0.75;
  float Robot_model_x = robot.x * zoom;
  float Robot_model_y = robot.y * zoom;
  float Robot_model_z = robot.z + 90;

  float robot_shape_1_x = -12 * zoom, robot_shape_1_y = 9 * zoom,
        robot_shape_2_x = 12 * zoom, robot_shape_2_y = 9 * zoom,
        robot_shape_3_x = -12 * zoom, robot_shape_3_y = -9 * zoom,
        robot_shape_4_x = 12 * zoom, robot_shape_4_y = -9 * zoom;

  QPolygonF Robot_model_poly;
  Robot_model_poly
      << QPointF((robot_shape_1_x)*sin(Robot_model_z * M_PI / 180) -
                     (robot_shape_1_y)*cos(Robot_model_z * M_PI / 180) +
                     Robot_model_x,
                 (robot_shape_1_x)*cos(Robot_model_z * M_PI / 180) +
                     (robot_shape_1_y)*sin(Robot_model_z * M_PI / 180) +
                     Robot_model_y)
      << QPointF((robot_shape_2_x)*sin(Robot_model_z * M_PI / 180) -
                     (robot_shape_2_y)*cos(Robot_model_z * M_PI / 180) +
                     Robot_model_x,
                 (robot_shape_2_x)*cos(Robot_model_z * M_PI / 180) +
                     (robot_shape_2_y)*sin(Robot_model_z * M_PI / 180) +
                     Robot_model_y)
      << QPointF((robot_shape_4_x)*sin(Robot_model_z * M_PI / 180) -
                     (robot_shape_4_y)*cos(Robot_model_z * M_PI / 180) +
                     Robot_model_x,
                 (robot_shape_4_x)*cos(Robot_model_z * M_PI / 180) +
                     (robot_shape_4_y)*sin(Robot_model_z * M_PI / 180) +
                     Robot_model_y)
      << QPointF((robot_shape_3_x)*sin(Robot_model_z * M_PI / 180) -
                     (robot_shape_3_y)*cos(Robot_model_z * M_PI / 180) +
                     Robot_model_x,
                 (robot_shape_3_x)*cos(Robot_model_z * M_PI / 180) +
                     (robot_shape_3_y)*sin(Robot_model_z * M_PI / 180) +
                     Robot_model_y);

  return Robot_model_poly;
}

int MainWindow::cvt_Print_xy(float target) {
  // PRE CONDITION : target
  // POST CONDITION : tartet * 0.75
  // purpose : target에 0.75를 곱한 값을 리턴

  float zoom = 0.75;
  target *= zoom;
  return (int)target;
}

void MainWindow::featureCalc() {
  if (qnode->Likelihood.vision_point_vect.size() >
      100)  // 비전에서 충분한 양의 특징점 데이터를 찾을 시 실행
  {
    if (qnode->vision_data_size / qnode->vision_data_cnt <= 1) {
      for (int i = 0; i < PARTICLE_NUM; i++) {
        qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y, 20);
      }
    }  // 비전에서 충분하지 못한 시간동안 데이터를 찾을 경우 파티클 재생성
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->Likelihood.set_circle(
          qnode->robot0.z,
          qnode->Likelihood.vision_point_vect);  // 파티클 위치의 제한 설정
      measurement.NUM =
          i;  // 파티클의 가중치를 저장하는 measurement의 NUM 값에 번호 부여
      measurement.WEIGHT = qnode->Likelihood.sence(
          qnode->pt[i].x, qnode->pt[i].y, qnode->robot0.x, qnode->robot0.y,
          qnode->Likelihood
              .vision_point_vect);  // 파티클의 가중치를 저장하는 measurement의
                                    // WEIGHT에 해당 파티클의 가중치를 계산 한
                                    // 후 가중치 값 저장
      // cout << "i : " << i << "  " << measurement.WEIGHT << endl;
      double dis = sqrt(pow(qnode->pt[i].x - qnode->robot0.x, 2) +
                        pow(qnode->pt[i].y - qnode->robot0.y,
                            2));  // 파티클과 로봇의 위치 사이의 거리 계산
      // cout << "dis 1 : " << dis << endl;
      //  dis /= particle_range;
      // cout << "dis 2 : " << dis << endl; // 해당 거리를 파티클 범위로 나눗셈
      if (dis < 1) {
        dis = 1;
      } // 나눗셈 결과 값이 1 이하일때 예외처리
      measurement.WEIGHT /= dis;              // 가중치에 거리 값 나눗셈
      particle_weight.push_back(measurement); // particle_weight 벡터 컨테이너에 해당 measurement값 저장
      // cout << "i : " << i << "  " << measurement.WEIGHT << endl;
    }
    sort(particle_weight.begin(), particle_weight.end(), sort_return); // particle_weight 벡터 컨테이너 정렬
    if (particle_weight[0].WEIGHT > 30)                                // 가장 가중치가 높은 값이 30 이상일 경우 실행
    {
      if (qnode->vision_data_size / qnode->vision_data_cnt <= 1) {
        cout << "SMALL MATCHING!! : " << particle_weight[0].WEIGHT << endl;
      }  // 비전에 충분한 데이터가 저장되지 않은 경우 SMALL MATCHING문구 와 함께
         // 가중치 값 출력
      else {
        cout << "MATCHING!! : " << particle_weight[0].WEIGHT << endl;
      }  // 비전에 충분한 데이터가 저장된 경우 MATCHING문구 와 함께 가중치 값
         // 출력

      // qnode->robot0의 데이터를 가장 가중치가 높은 값으로 설정
      qnode->robot0.x = qnode->pt[particle_weight[0].NUM]
                            .x;  //(qnode->pt[particle_weight[0].NUM].x +
                                 // qnode->pt[particle_weight[1].NUM].x +
                                 // qnode->pt[particle_weight[2].NUM].x) / 3;
      qnode->robot0.y = qnode->pt[particle_weight[0].NUM]
                            .y;  //(qnode->pt[particle_weight[0].NUM].y +
                                 // qnode->pt[particle_weight[1].NUM].y +
                                 // qnode->pt[particle_weight[2].NUM].y) / 3;
    } else {
      cout << "FAIL!! : " << particle_weight[0].WEIGHT << endl;
    }  // 그렇지 않을 경우 가중치 값 및 실패 메세지 출력
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }  // 파티클 재생성

    qnode->Likelihood.check_local_point(
        50, qnode->robot0.z,
        qnode->Likelihood
            .vision_point_vect);  // 특징점이 로봇 근처에 있는지에 따라 해당
                                  // 특징점 활성화 또는 비활성화

    // qnode->robot_sight_flag = 0;
    particle_weight.clear();  // particle_weight 벡터 컨테이너 초기화
    qnode->Likelihood.vision_point_vect
        .clear();  // qnode->Likelihood.vision_point_vect 벡터 컨테이너 초기화

    // qnode->vision_data_cnt 및 qnode->vision_data_size 변수 초기화
    qnode->vision_data_cnt = 0;
    qnode->vision_data_size = 0;

    for (int i = 0; i < 27; i++)  // 모든 로컬 포인트 체크
    {
      if (qnode->Likelihood.Local_point_check[i] ==
          1)  // 특정 로컬 포인트가 활성화 된 상태일 시 실행
      {
        // qnode->Likelihood에 데이터 저장
        qnode->Likelihood.vision_point.CONFIDENCE = 0.1;
        qnode->Likelihood.vision_point.DISTANCE = 1;
        qnode->Likelihood.vision_point.POINT_VEC_X =
            qnode->Likelihood.Local_point_x[i] - qnode->robot0.x;
        qnode->Likelihood.vision_point.POINT_VEC_Y =
            qnode->Likelihood.Local_point_y[i] - qnode->robot0.y;
        qnode->Likelihood.vision_point.STD_X = qnode->robot0.x;
        qnode->Likelihood.vision_point.STD_Y = qnode->robot0.y;
        qnode->Likelihood.vision_point_vect.push_back(
            qnode->Likelihood.vision_point);
      }
    }
  }
}

void MainWindow::on_btn_free_set_clicked() {
  // PRE CONDITION : UI
  // POST CONDITION : set_robot_flag
  // purpose : set_robot_flag를 1로 설정

  set_robot_flag = 1;
}
void MainWindow::on_btn_objects_save_clicked() {
  // PRE CONDITION : particle_range, robot0.odom_Nn
  // POST CONDITION : param file
  // purpose : 현재 적용되어있는 오도메트리 값을 입출력스트림을 통해 저장

  String param = "";
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_1
  param = "src/robocup_localization25/resources/param/PARAM1.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_2
  param = "src/robocup_localization25/resources/param/PARAM2.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_3
  param = "src/robocup_localization25/resources/param/PARAM3.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_4
  param = "src/robocup_localization25/resources/param/PARAM4.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_5
  param = "src/robocup_localization25/resources/param/PARAM5.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_6
  param = "src/robocup_localization25/resources/param/PARAM6.txt";
#endif
  ofstream Last_Index_Num_OUT(param);
  if (Last_Index_Num_OUT.is_open()) {
    Last_Index_Num_OUT << particle_range << endl;
    Last_Index_Num_OUT << qnode->robot0.odom_fx << endl;
    Last_Index_Num_OUT << qnode->robot0.odom_bx << endl;
    Last_Index_Num_OUT << qnode->robot0.odom_ly << endl;
    Last_Index_Num_OUT << qnode->robot0.odom_ry << endl;
    std::cout << "Odom Param set" << std::endl;
  } else {
    std::cout << "No Odom Prameter" << std::endl;
  }
  Last_Index_Num_OUT.close();
}

void MainWindow::on_btn_set_1_clicked() {
  // PRE CONDITION : UI
  // POST CONDITION : qnode->robot0.N, qnode->master_target_x
  // purpose : get in시 위치 매크로

  if (qnode->gameMSG.mySide) {
    qnode->robot0.x = 630;
    qnode->robot0.y = 100;
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
  } else {
    qnode->robot0.x = 200;
    qnode->robot0.y = 100;
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
  }
  qnode->robot0.TIME_STAMP.clear();
  qnode->master_target_x = 0;
  qnode->master_target_y = 0;
}
void MainWindow::on_btn_set_2_clicked() {
  // PRE CONDITION : UI
  // POST CONDITION : qnode->robot0.N, qnode->master_target_x
  // purpose : get in시 위치 매크로

  if (qnode->gameMSG.mySide) {
    qnode->robot0.x = 800;
    qnode->robot0.y = 100;
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
  } else {
    qnode->robot0.x = 300;
    qnode->robot0.y = 100;
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
  }
  qnode->robot0.TIME_STAMP.clear();
  qnode->master_target_x = 0;
  qnode->master_target_y = 0;
}
void MainWindow::on_btn_set_3_clicked() {
  // PRE CONDITION : UI
  // POST CONDITION : qnode->robot0.N, qnode->master_target_x
  // purpose : get in시 위치 매크로

  if (qnode->gameMSG.mySide) {
    qnode->robot0.x = 900;
    qnode->robot0.y = 100;
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
  } else {
    qnode->robot0.x = 470;
    qnode->robot0.y = 100;
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
  }
  qnode->robot0.TIME_STAMP.clear();
  qnode->master_target_x = 0;
  qnode->master_target_y = 0;
}
void MainWindow::on_btn_set_4_clicked() {
  // PRE CONDITION : UI
  // POST CONDITION : qnode->robot0.N, qnode->master_target_x
  // purpose : get in시 위치 매크로

  if (qnode->gameMSG.mySide) {
    qnode->robot0.x = 630;
    qnode->robot0.y = 700;
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
  } else {
    qnode->robot0.x = 200;
    qnode->robot0.y = 700;
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
  }
  qnode->robot0.TIME_STAMP.clear();
  qnode->master_target_x = 0;
  qnode->master_target_y = 0;
}
void MainWindow::on_btn_set_5_clicked() {
  // PRE CONDITION : UI
  // POST CONDITION : qnode->robot0.N, qnode->master_target_x
  // purpose : get in시 위치 매크로

  if (qnode->gameMSG.mySide) {
    qnode->robot0.x = 800;
    qnode->robot0.y = 700;
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
  } else {
    qnode->robot0.x = 300;
    qnode->robot0.y = 700;
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
  }
  qnode->robot0.TIME_STAMP.clear();
  qnode->master_target_x = 0;
  qnode->master_target_y = 0;
}
void MainWindow::on_btn_set_6_clicked() {
  // PRE CONDITION : UI
  // POST CONDITION : qnode->robot0.N, qnode->master_target_x
  // purpose : get in시 위치 매크로

  if (qnode->gameMSG.mySide) {
    qnode->robot0.x = 900;
    qnode->robot0.y = 700;
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
  } else {
    qnode->robot0.x = 470;
    qnode->robot0.y = 700;
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
  }
  qnode->robot0.TIME_STAMP.clear();
  qnode->master_target_x = 0;
  qnode->master_target_y = 0;
}
void MainWindow::on_btn_set_auto_clicked() {
  // PRE CONDITION : UI
  // POST CONDITION : qnode->robot0.N, qnode->master_target_x
  // purpose : get in시 위치 자동 매크로

  if (qnode->gameMSG.mySide) {
    if (qnode->gameMSG.robotNum == 1) {
      qnode->robot0.x = 630;
      qnode->robot0.y = 100;
    }
    if (qnode->gameMSG.robotNum == 2) {
      qnode->robot0.x = 800;
      qnode->robot0.y = 100;
    }
    if (qnode->gameMSG.robotNum == 3) {
      qnode->robot0.x = 630;
      qnode->robot0.y = 700;
    }
    if (qnode->gameMSG.robotNum == 4) {
      qnode->robot0.x = 900;
      qnode->robot0.y = 700;
    }
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
  } else {
    if (qnode->gameMSG.robotNum == 1) {
      qnode->robot0.x = 470;
      qnode->robot0.y = 700;
    }
    if (qnode->gameMSG.robotNum == 2) {
      qnode->robot0.x = 300;
      qnode->robot0.y = 700;
    }
    if (qnode->gameMSG.robotNum == 3) {
      qnode->robot0.x = 470;
      qnode->robot0.y = 100;
    }
    if (qnode->gameMSG.robotNum == 4) {
      qnode->robot0.x = 200;
      qnode->robot0.y = 100;
    }
    for (int i = 0; i < PARTICLE_NUM; i++) {
      qnode->pt[i].random_point(qnode->robot0.x, qnode->robot0.y,
                                particle_range);
    }
  }
  qnode->robot0.TIME_STAMP.clear();
  qnode->master_target_x = 0;
  qnode->master_target_y = 0;
}
void MainWindow::on_btn_ball_set_clicked() {
  // PRE CONDITION : UI
  // POST CONDITION : qnode->set_ball_flag
  // purpose : 공 설정

  qnode->set_ball_flag = 1;
}
void MainWindow::on_btn_ball_del_clicked() {
  // PRE CONDITION : UI
  // POST CONDITION : qnode->set_ball_flag, qnode->ball.N
  // purpose : 공 삭제 버튼

  qnode->set_ball_flag = 0;
  qnode->ball.x = 0;
  qnode->ball.y = 0;
}
void MainWindow::on_btn_test_clicked() {
  qnode->robot1.x = 0;
  qnode->robot1.y = 0;
  qnode->robot1.z = 0;

  qnode->ball1.x = 0;
  qnode->ball1.y = 0;
  qnode->ball1.d = 999999;

  qnode->robot2.x = 0;
  qnode->robot2.y = 0;
  qnode->robot2.z = 0;

  qnode->ball2.x = 0;
  qnode->ball2.y = 0;
  qnode->ball2.d = 999999;

  qnode->robot3.x = 0;
  qnode->robot3.y = 0;
  qnode->robot3.z = 0;

  qnode->ball3.x = 0;
  qnode->ball3.y = 0;
  qnode->ball3.d = 999999;

  qnode->robot4.x = 0;
  qnode->robot4.y = 0;
  qnode->robot4.z = 0;

  qnode->ball4.x = 0;
  qnode->ball4.y = 0;
  qnode->ball4.d = 999999;

  qnode->Likelihood.vision_point_vect.clear();
}

void MainWindow::closeEvent(QCloseEvent *event) {
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow() { delete ui; }
