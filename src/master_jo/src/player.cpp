#include "../include/master_jo/player.hpp"

namespace master_jo
{

  using namespace std;

  bool Player::falldownExeption()
  {
    return false;
  }

  void Player::selectGoalPost()
  {
    if (master->gameControlData.myside == RIGHT)
    {
      opponent_goal = Point(0, 400);
    }
    else
    {
      opponent_goal = Point(1100, 400);
    }
  }

  void Player::penaltyControl(bool &isPenalty)
  {
    isPenalty = false;
  }

  void Player::gameStateControl(bool control_on, int state)
  {
    if (!control_on)
    {
      return;
    }
    state = STATE_PLAYING;
    switch (state)
    {
    case STATE_INITIAL:
      stateInitial();
      break;
    case STATE_READY:
      stateReady();
      break;
    case STATE_SET:
      stateSet();
      break;
    case STATE_PLAYING:
      statePlay();
      break;
    case STATE_FINISHED:
      stateFinished();
      break;
    default:
      break;
    }
  }

  void Player::selectRobotState(bool isPenalty)
  {
  }

  void Player::publishMsg()
  {
    walkPublish();
    udpPublish();
    localPublish();
  }

  void Player::move(bool flag)
  {
    if (!flag)
    {
      walkStop();
    }
  }

  bool Player::walkStart(int x, int y, int yaw)
  {
    master->ik.flag = true;

    this->x = x;
    this->y = y;
    this->yaw = yaw;

    cout<<x<<endl;
    cout<<y<<endl;
    cout<<yaw<<endl; //got dameit master_jo_node에서는 반영 1나도 안됨
    return master->ikEnd.ikend;
  }

  bool Player::walkStop()
  {
    master->ik.flag = false;
    this->x = 0;
    this->y = 0;
    this->yaw = 0;

    return master->ikEnd.ikend;
  }

  bool Player::walkPublish()
  {
    bool error = false;
    if (master->ik.x_length > master->FRONT_MAX)
    {
      master->ik.x_length = master->FRONT_MAX;
      error = true;
    }
    else if (master->ik.x_length < master->REAR_MAX)
    {
      master->ik.x_length = master->REAR_MAX;
      error = true;
    }

    if (master->ik.y_length > master->RIGHT_MAX)
    {
      master->ik.y_length = master->RIGHT_MAX;
      error = true;
    }
    else if (master->ik.y_length < master->LEFT_MAX)
    {
      master->ik.y_length = master->LEFT_MAX;
      error = true;
    }

    if (master->ik.yaw > master->R_YAW_MAX)
    {
      master->ik.yaw = master->R_YAW_MAX;
      error = true;
    }
    else if (master->ik.yaw < master->L_YAW_MAX)
    {
      master->ik.yaw = master->L_YAW_MAX;
      error = true;
    }

    return error;
  }

  bool Player::visionPublish(int scan_mode, int pan, int tilt)
  {
    master->master2vision.scanmode = scan_mode;
    master->master2vision.tilt = -45;
    master->master2vision.pan = 0;

    return true;
  }

  bool Player::udpPublish()
  {
    master->master2udp.robotnum = master->gameControlData.robotnum;
    master->master2udp.robotstate = robot_state;
    master->master2udp.robotcoorx = static_cast<int>(master->local.robot_x);
    master->master2udp.robotcoory = static_cast<int>(master->local.robot_y);
    master->master2udp.robotimuyaw = static_cast<int>(master->imu.yaw);
    master->master2udp.balldist = 0;
    master->master2udp.ballcoorx = static_cast<int>(master->local.ball_x);
    master->master2udp.ballcoory = static_cast<int>(master->local.ball_y);
    master->master2udp.myteam = static_cast<int>(master->gameControlData.myteam);

    return true;
  }

  bool Player::localPublish()
  {
    if (static_cast<int>(master->ik.flag) == false)
    {
      master->master2local.targetx = 0;
      master->master2local.targety = 0;
    }
    else
      cout << "pub" << endl;

    return true;
  }

}