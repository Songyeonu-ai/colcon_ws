/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date August 2024
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/robocup_localization25/qnode.hpp"

QNode::QNode()
{
  int argc = 0;
  char **argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("robocup_localization25");
  this->start();

  imu_sub_ = node->create_subscription<humanoid_interfaces::msg::ImuMsg>("Imu", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort(), std::bind(&QNode::imuCallback, this, std::placeholders::_1));
  vision_sub_ = node->create_subscription<humanoid_interfaces::msg::Robocupvision25>("vision", 100, std::bind(&QNode::visionCallback, this, std::placeholders::_1));
  vision_feature_sub_ = node->create_subscription<humanoid_interfaces::msg::Robocupvision25feature>("vision_feature", 100, std::bind(&QNode::visionFeatureCallback, this, std::placeholders::_1));
  ik_sub_ = node->create_subscription<humanoid_interfaces::msg::IkCoordMsg>("ikcoordinate", 10, std::bind(&QNode::ikCallback, this, std::placeholders::_1));
  game_control_sub_ = node->create_subscription<humanoid_interfaces::msg::Gamecontroldata>("gamecontroldata", 10, std::bind(&QNode::gameControlCallback, this, std::placeholders::_1));
  udp_sub_ = node->create_subscription<humanoid_interfaces::msg::Udp2master>("udp", 10, std::bind(&QNode::udpCallback, this, std::placeholders::_1));
  master_sub_ = node->create_subscription<humanoid_interfaces::msg::Master2localization25>("master2local", 10, std::bind(&QNode::masterCallback, this, std::placeholders::_1));

  localization_pub_ = node->create_publisher<humanoid_interfaces::msg::Robocuplocalization25>("localization", 10);
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

void QNode::visionFeatureCallback(const humanoid_interfaces::msg::Robocupvision25feature::SharedPtr msg)
{

  for (int i = 0; i < msg->confidence.size(); i++)
  {
    vision_data_cnt += 1;
    vision_data_size += msg->confidence.size();
    Likelihood.vision_point.CONFIDENCE = msg->confidence[i];
    Likelihood.vision_point.DISTANCE = msg->distance[i];
    Likelihood.vision_point.POINT_VEC_X = (-1) * (msg->point_vec_x[i] / 10 * cos((robot0.z) * M_PI / 180) + msg->point_vec_y[i] / 10 * sin((robot0.z) * M_PI / 180));
    Likelihood.vision_point.POINT_VEC_Y = (msg->point_vec_x[i] / 10 * sin((robot0.z) * M_PI / 180) - msg->point_vec_y[i] / 10 * cos((robot0.z) * M_PI / 180));
    Likelihood.vision_point.STD_X = robot0.x;
    Likelihood.vision_point.STD_Y = robot0.y;
    Likelihood.vision_point_vect.push_back(Likelihood.vision_point);

    Likelihood.set_circle(robot0.z, Likelihood.vision_point_vect);

    Likelihood.on_local_point(robot0.x, robot0.y, robot0.x, robot0.y);

    emit featureReceived();
  }
}

void QNode::imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg)
{
  robot0.z = msg->yaw;
}

void QNode::visionCallback(const humanoid_interfaces::msg::Robocupvision25::SharedPtr msg)
{
  // PRE CONDITION : msg(vision)
  // POST CONDITION : Ball_cam_N, Ball_2d_N, Ball_D, PAN, TILT, Ball_speed_N, ROBOT_VEC_N, Ball_speed_level
  // purpose : 비전에서 받아온 데이터 콜백

  visionMSG.Ball_cam_X = msg->ball_cam_x;
  visionMSG.Ball_cam_Y = msg->ball_cam_y;
  visionMSG.Ball_2d_X = msg->ball_2d_x;
  visionMSG.Ball_2d_Y = msg->ball_2d_y;
  visionMSG.Ball_D = msg->ball_d;
  visionMSG.PAN = msg->pan;
  visionMSG.TILT = msg->tilt;
  visionMSG.Ball_speed_X = msg->ball_speed_x;
  visionMSG.Ball_speed_Y = msg->ball_speed_y;
  visionMSG.ROBOT_VEC_X.clear();
  visionMSG.ROBOT_VEC_Y.clear();
  for (int i = 0; i < msg->robot_vec_x.size(); i++)
  {
    visionMSG.ROBOT_VEC_X.push_back(msg->robot_vec_x[i]);
    visionMSG.ROBOT_VEC_Y.push_back(msg->robot_vec_y[i]);
  }
  visionMSG.Ball_speed_level = msg->ball_speed_level;
  visionMSG.Scan_mode = msg->scan_mode;
  if (visionMSG.Ball_speed_level > 150)
  {
    visionMSG.Ball_speed_level = 150;
  }
  if (visionMSG.Ball_cam_X != 0 || visionMSG.Ball_cam_Y != 0)
  {
    set_ball_flag = 0;
  }
  vision_callback_timer = 0;
}

void QNode::ikCallback(const humanoid_interfaces::msg::IkCoordMsg::SharedPtr msg)
{
  // PRE CONDITION : msg(IK)
  // POST CONDITION : Xmoved, Ymoved
  // purpose : IK에서 받아온 데이터 콜백 및 해당 데이터로 robot0의 위치 업데이트, 업데이트 된 위치에 파티클 가우시안 분포로 생성

  double Xmoved = msg->x;
  double Ymoved = msg->y;
  robot0.move(Xmoved, Ymoved);
  for (int i = 0; i < PARTICLE_NUM; i++)
  {
    pt[i].random_point(robot0.x, robot0.y, particle_range);
  }
}

void QNode::gameControlCallback(const humanoid_interfaces::msg::Gamecontroldata::SharedPtr msg)
{
  // PRE CONDITION : msg(gamecontroller)
  // POST CONDITION : mySide, penalty, position, robotNum
  // purpose : gamecontroller에서 콜백 받아온 경기 데이터를 적용

  gameMSG.mySide = msg->myside;
  gameMSG.penalty = msg->penalty;
  gameMSG.position = msg->position;
  gameMSG.robotNum = msg->robotnum;
  if (gameMSG.penalty != 0)
  {
    if (gameMSG.mySide)
    {
      if (gameMSG.position == 1)
      {
        robot0.x = 900;
        robot0.y = 700;
      }
      else
      {
        robot0.x = 800;
        robot0.y = 700;
      }
      for (int i = 0; i < PARTICLE_NUM; i++)
      {
        pt[i].random_point(robot0.x, robot0.y, particle_range);
      }
    }
    else
    {
      if (gameMSG.position == 1)
      {
        robot0.x = 200;
        robot0.y = 700;
      }
      else
      {
        robot0.x = 300;
        robot0.y = 700;
      }
      for (int i = 0; i < PARTICLE_NUM; i++)
      {
        pt[i].random_point(robot0.x, robot0.y, particle_range);
      }
    }
    robot0.TIME_STAMP.clear();
    master_target_x = 0;
    master_target_y = 0;
  }
}

void QNode::udpCallback(const humanoid_interfaces::msg::Udp2master::SharedPtr msg)
{
  // PRE CONDITION : msg(udpcom)
  // POST CONDITION : robotX.n, ballX.n,
  // purpose : udp통신으로 받아온 다른 로봇 데이터를 콜백하여 내부 변수에 적용
  if (msg->robotnum == 1)
  {
    robot1.x = msg->localx;
    robot1.y = msg->localy;
    robot1.z = msg->localyaw;
    robot1.b = 1;
    robot1.state = msg->robotcase;

    ball1.x = msg->ballx;
    ball1.y = msg->bally;
    ball1.d = sqrt(pow(msg->localx - msg->ballx, 2) + pow(msg->localy - msg->bally, 2));
  }
  if (msg->robotnum == 2)
  {
    robot2.x = msg->localx;
    robot2.y = msg->localy;
    robot2.z = msg->localyaw;
    robot2.b = 1;
    robot2.state = msg->robotcase;

    ball2.x = msg->ballx;
    ball2.y = msg->bally;
    ball2.d = sqrt(pow(msg->localx - msg->ballx, 2) + pow(msg->localy - msg->bally, 2));
  }
  if (msg->robotnum == 3)
  {
    robot3.x = msg->localx;
    robot3.y = msg->localy;
    robot3.z = msg->localyaw;
    robot3.b = 1;
    robot3.state = msg->robotcase;

    ball3.x = msg->ballx;
    ball3.y = msg->bally;
    ball3.d = sqrt(pow(msg->localx - msg->ballx, 2) + pow(msg->localy - msg->bally, 2));
  }
  if (msg->robotnum == 4)
  {
    robot4.x = msg->localx;
    robot4.y = msg->localy;
    robot4.z = msg->localyaw;
    robot4.b = 1;
    robot4.state = msg->robotcase;

    ball4.x = msg->ballx;
    ball4.y = msg->bally;
    ball4.d = sqrt(pow(msg->localx - msg->ballx, 2) + pow(msg->localy - msg->bally, 2));
  }
}

void QNode::masterCallback(const humanoid_interfaces::msg::Master2localization25::SharedPtr msg)
{
  // PRE CONDITION : msg(master)
  // POST CONDITION : master_target_N
  // purpose : 마스터에서 받은 타겟 좌표 데이터 적용

  master_target_x = msg->targetx;
  master_target_y = msg->targety;
}
