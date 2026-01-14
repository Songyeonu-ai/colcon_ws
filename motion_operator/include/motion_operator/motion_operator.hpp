#ifndef MOTION_OPERATOR_HPP
#define MOTION_OPERATOR_HPP

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

// #include "dynamixel_rdk_msgs/msg/dynamixel_bulk_read_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_control_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_status_msgs.hpp"

#endif  // MOTION_OPERATOR_HPP

struct Dynamixel_postition {
  int id;
  double position;
};

struct Step {
  int time;
  int delay;
  int repeat;
  std::string name;
  std::vector<Dynamixel_postition> dxl;
  bool selected;
};

namespace motion_operator {

class MOTION {
 public:  // Variables
  dynamixel_rdk_msgs::msg::DynamixelControlMsgs DXL;
  dynamixel_rdk_msgs::msg::DynamixelMsgs Motor_Info;
  dynamixel_rdk_msgs::msg::DynamixelStatusMsgs Motor_Status;

  std::vector<Step> steps;
  Step init;

 public:  // Functions
  MOTION(std::string KickRAddr, std::string KickLAddr,
         std::string FrontRiseAddr, std::string BackRiseAddr,
         std::string RightRiseAddr, std::string LeftRiseAddr);
  ~MOTION();

  bool read_motion(std::string motion_path);

  bool read_step(std::string motion_path);

  std::vector<double> set_velocity(std::vector<double> goal_position,
                                   std::vector<double> current_position,
                                   int time);
  std::vector<double> get_goal_position(int step_num);

  std::vector<double> get_before_position(int step_num);

 private:  // Variables
  std::string KickRAddr_;
  std::string KickLAddr_;
  std::string FrontRiseAddr_;
  std::string BackRiseAddr_;
  std::string RightRiseAddr_;
  std::string LeftRiseAddr_;

 private:  // Functions
  double calc_velocity(double goal_position, double current_position, int time);
};

}  // namespace motion_operator
