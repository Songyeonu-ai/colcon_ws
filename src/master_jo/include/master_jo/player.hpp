#ifndef PLAYER_HPP
#define PLAYER_HPP

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <iostream>
#include <map>
#include <vector>

#include "define.hpp"
#include "master_jo.hpp"
using namespace std;

namespace master_jo {
class Player {
 public:
  Player(std::shared_ptr<master_jo::MasterRcko> master) {
    this->master = master;
  }

  std::shared_ptr<master_jo::MasterRcko> master;
  int robot_state;

  bool falldownExeption();
  void selectGoalPost();
  void penaltyControl(bool &isPenalty);
  void gameStateControl(bool control_on, int state);
  virtual void selectRobotState(bool isPenalty);
  void publishMsg();

  void move(bool flag);

  int publish_motion_num;
  bool publish_motion_flag = false;
  bool publish_motion_complete = true;
  int x=0;
  int y=0;
  int yaw=0;

  bool walkStop();

  virtual void stateInitial() = 0;
  virtual void stateReady() = 0;
  virtual void stateSet() = 0;
  virtual void statePlay() = 0;
  virtual void stateFinished() = 0;
  bool walkStart(int x, int y, int yaw);

 protected:
  const double PI = 3.14159265;

  Point opponent_goal;


  bool walkPublish();
  bool visionPublish(int scan_mode, int pan = 0, int tilt = 0);
  bool udpPublish();
  bool localPublish();

};

}
#endif