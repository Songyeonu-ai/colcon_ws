#ifndef MASTER_JO_HPP
#define MASTER_JO_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include "point.hpp" 
#include "humanoid_interfaces/msg/udp2master.hpp"
#include "humanoid_interfaces/msg/motion_operator.hpp"
#include "humanoid_interfaces/msg/imu_msg.hpp"
#include "humanoid_interfaces/msg/robocupvision25.hpp"
#include "humanoid_interfaces/msg/human_pj_vision.hpp"
#include "humanoid_interfaces/msg/ik_end_msg.hpp"
#include "humanoid_interfaces/msg/gamecontroldata.hpp"
#include "humanoid_interfaces/msg/robocuplocalization25.hpp"
#include "humanoid_interfaces/msg/pidtuning.hpp"
#include "humanoid_interfaces/msg/master2_ik_msg.hpp"
#include "humanoid_interfaces/msg/master2vision25.hpp"
#include "humanoid_interfaces/msg/gamecontrolreturndata.hpp"
#include "humanoid_interfaces/msg/master2localization25.hpp"
#include "humanoid_interfaces/msg/master2udp.hpp"

enum GameState {
    STATE_INITIAL,
    STATE_READY,
    STATE_SET,
    STATE_PLAYING,
    STATE_FINISHED
};


namespace master_jo {
class MasterRcko {
 public:
  // IK 관련 메시지
    humanoid_interfaces::msg::Master2IkMsg ik; //ㅇㅋ
    humanoid_interfaces::msg::IkEndMsg ikEnd; //ㅇㅋ

    // 게임 컨트롤
    humanoid_interfaces::msg::Gamecontroldata gameControlData; //ㅇㅋ
    humanoid_interfaces::msg::Gamecontrolreturndata gameControlReturnData; //ㅇㅋ

    // 센서 / 비전 / 로컬라이제이션
    humanoid_interfaces::msg::ImuMsg imu; //ㅇㅋ
    humanoid_interfaces::msg::HumanPjVision vision; //ㅇㅋ
    humanoid_interfaces::msg::Robocuplocalization25 local; //ㅇㅋ

    // 마스터 → 서브시스템 메시지
    humanoid_interfaces::msg::Master2vision25 master2vision; //ㅇㅋ
    humanoid_interfaces::msg::Master2localization25 master2local; //ㅇㅋ
    humanoid_interfaces::msg::Master2udp master2udp; //ㅇㅋ

    // UDP → 마스터 메시지
    humanoid_interfaces::msg::Udp2master udp[256];  //ㅇ?ㅋ

    // 모션
    humanoid_interfaces::msg::MotionOperator motion; //ㅇㅋ
    humanoid_interfaces::msg::MotionOperator motionEnd; //ㅇㅋ

    // 기타
    //humanoid_interfaces::msg::MotorMsg motor;
    //humanoid_interfaces::msg::Pidtuning pid; 

    // 제어 상수
    double kp = 0.0;
    double kd = 0.0;
    double ki = 0.0;
    double FRONT_MAX = 0.0;
    double REAR_MAX = 0.0;
    double LEFT_MAX = 0.0;
    double RIGHT_MAX = 0.0;
    double X_MIN = 0.0;
    double Y_MIN = 0.0;
    double ROUND_Y = 0.0;
    double ROUND_YAW_MIN = 0.0;
    double R_YAW_MAX = 0.0;
    double L_YAW_MAX = 0.0;
    // 상태 플래그
    bool In = false;
    bool Out = false;
    bool Back = false;
    bool Front = false;
};
}
#endif