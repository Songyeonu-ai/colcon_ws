/**
 * @file /include/robocup_localization25/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robocup_localization25_QNODE_HPP_
#define robocup_localization25_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif
#include <QThread>
#include <QStringListModel>

#include "humanoid_interfaces/msg/imu_msg.hpp"
#include "humanoid_interfaces/msg/robocupvision25.hpp"
#include "humanoid_interfaces/msg/robocupvision25feature.hpp"
#include "humanoid_interfaces/msg/ik_coord_msg.hpp"
#include "humanoid_interfaces/msg/robocuplocalization25.hpp"
#include "humanoid_interfaces/msg/gamecontroldata.hpp"
#include "humanoid_interfaces/msg/udp2master.hpp"
#include "humanoid_interfaces/msg/master2localization25.hpp"

#include "../objects/robot.hpp"
#include "../objects/obstacle.hpp"
#include "../objects/ball.hpp"
#include "../objects/line.hpp"

#define DEG2RAD (M_PI / 180)
#define PARTICLE_NUM 10000

struct VISIONMSG
{
  int Ball_cam_X;
  int Ball_cam_Y;
  float Ball_2d_X;
  float Ball_2d_Y;
  float Ball_D;
  float PAN;
  float TILT;
  float Ball_speed_X;
  float Ball_speed_Y;
  vector<float> ROBOT_VEC_X;
  vector<float> ROBOT_VEC_Y;
  int Ball_speed_level;
  int Scan_mode;
  int flag_pan;
};

struct GAMEMSG
{
  int mySide = 1;
  int penalty = 0;
  int position = 0;
  int robotNum = 0;
};

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();

  rclcpp::Publisher<humanoid_interfaces::msg::Robocuplocalization25>::SharedPtr localization_pub_;

  int vision_data_cnt = 0;
  int vision_data_size = 0;

  // callback
  int vision_callback_timer = 0;

  // For print
  int robot_sight_flag = 0;
  int master_target_x = 0;
  int master_target_y = 0;

  // For UI
  int set_ball_flag = 0;

  // For Objects
  VISIONMSG visionMSG;
  ROBOT robot0 = ROBOT();
  ROBOT robot1 = ROBOT();
  ROBOT robot2 = ROBOT();
  ROBOT robot3 = ROBOT();
  ROBOT robot4 = ROBOT();
  BALL ball = BALL();
  BALL ball1 = BALL();
  BALL ball2 = BALL();
  BALL ball3 = BALL();
  BALL ball4 = BALL();
  LINE Likelihood = LINE();
  OBSTACLE obstacle0 = OBSTACLE();
  OBSTACLE obstacle1 = OBSTACLE();
  OBSTACLE obstacle2 = OBSTACLE();
  OBSTACLE obstacle3 = OBSTACLE();

  // For gamecontrol
  GAMEMSG gameMSG;

  // For particle
  ROBOT pt[PARTICLE_NUM];

  float particle_range = 0.0; // is 100

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;

  void imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
  void visionCallback(const humanoid_interfaces::msg::Robocupvision25::SharedPtr msg);
  void visionFeatureCallback(const humanoid_interfaces::msg::Robocupvision25feature::SharedPtr msg);
  void ikCallback(const humanoid_interfaces::msg::IkCoordMsg::SharedPtr msg);
  void gameControlCallback(const humanoid_interfaces::msg::Gamecontroldata::SharedPtr msg);
  void udpCallback(const humanoid_interfaces::msg::Udp2master::SharedPtr msg);
  void masterCallback(const humanoid_interfaces::msg::Master2localization25::SharedPtr msg);

  rclcpp::Subscription<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Robocupvision25>::SharedPtr vision_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Robocupvision25feature>::SharedPtr vision_feature_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::IkCoordMsg>::SharedPtr ik_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Gamecontroldata>::SharedPtr game_control_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Udp2master>::SharedPtr udp_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Master2localization25>::SharedPtr master_sub_;

Q_SIGNALS:
  void rosShutDown();
signals:
  void featureReceived();
};

#endif /* robocup_localization25_QNODE_HPP_ */
