// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from humanoid_interfaces:msg/Tune2IkMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/tune2_ik_msg__rosidl_typesupport_fastrtps_cpp.hpp"
#include "humanoid_interfaces/msg/detail/tune2_ik_msg__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace humanoid_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
cdr_serialize(
  const humanoid_interfaces::msg::Tune2IkMsg & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: ik_flag
  cdr << (ros_message.ik_flag ? true : false);
  // Member: entire_time
  cdr << ros_message.entire_time;
  // Member: frequency
  cdr << ros_message.frequency;
  // Member: default_x_left
  cdr << ros_message.default_x_left;
  // Member: default_y_left
  cdr << ros_message.default_y_left;
  // Member: default_z_left
  cdr << ros_message.default_z_left;
  // Member: default_x_right
  cdr << ros_message.default_x_right;
  // Member: default_y_right
  cdr << ros_message.default_y_right;
  // Member: default_z_right
  cdr << ros_message.default_z_right;
  // Member: offset_motor
  {
    cdr << ros_message.offset_motor;
  }
  // Member: swing_shoulder_left
  cdr << ros_message.swing_shoulder_left;
  // Member: swing_shoulder_right
  cdr << ros_message.swing_shoulder_right;
  // Member: swing_leg_left
  cdr << ros_message.swing_leg_left;
  // Member: swing_leg_right
  cdr << ros_message.swing_leg_right;
  // Member: rise_leg_left
  cdr << ros_message.rise_leg_left;
  // Member: rise_leg_right
  cdr << ros_message.rise_leg_right;
  // Member: start_entire_time
  cdr << ros_message.start_entire_time;
  // Member: start_swing
  cdr << ros_message.start_swing;
  // Member: start_rise
  cdr << ros_message.start_rise;
  // Member: end_entire_time
  cdr << ros_message.end_entire_time;
  // Member: end_swing
  cdr << ros_message.end_swing;
  // Member: end_rise
  cdr << ros_message.end_rise;
  // Member: test_x
  cdr << ros_message.test_x;
  // Member: test_side
  cdr << ros_message.test_side;
  // Member: test_yaw
  cdr << ros_message.test_yaw;
  // Member: tuning_x
  cdr << ros_message.tuning_x;
  // Member: tuning_side
  cdr << ros_message.tuning_side;
  // Member: tuning_yaw
  cdr << ros_message.tuning_yaw;
  // Member: balance_value_0
  cdr << ros_message.balance_value_0;
  // Member: balance_value_1
  cdr << ros_message.balance_value_1;
  // Member: balance_value_2
  cdr << ros_message.balance_value_2;
  // Member: balance_value_3
  cdr << ros_message.balance_value_3;
  // Member: balance_pitch_gp
  cdr << ros_message.balance_pitch_gp;
  // Member: balance_pitch_gi
  cdr << ros_message.balance_pitch_gi;
  // Member: balance_pitch_gd
  cdr << ros_message.balance_pitch_gd;
  // Member: balance_pitch_elimit
  cdr << ros_message.balance_pitch_elimit;
  // Member: balance_pitch_olimit
  cdr << ros_message.balance_pitch_olimit;
  // Member: balance_pitch_neg_target
  cdr << ros_message.balance_pitch_neg_target;
  // Member: balance_pitch_pos_target
  cdr << ros_message.balance_pitch_pos_target;
  // Member: balance_roll_gp
  cdr << ros_message.balance_roll_gp;
  // Member: balance_roll_gi
  cdr << ros_message.balance_roll_gi;
  // Member: balance_roll_gd
  cdr << ros_message.balance_roll_gd;
  // Member: balance_roll_elimit
  cdr << ros_message.balance_roll_elimit;
  // Member: balance_roll_olimit
  cdr << ros_message.balance_roll_olimit;
  // Member: balance_roll_neg_target
  cdr << ros_message.balance_roll_neg_target;
  // Member: balance_roll_pos_target
  cdr << ros_message.balance_roll_pos_target;
  // Member: balance_angle_pitch_gp
  cdr << ros_message.balance_angle_pitch_gp;
  // Member: balance_angle_pitch_gi
  cdr << ros_message.balance_angle_pitch_gi;
  // Member: balance_angle_pitch_gd
  cdr << ros_message.balance_angle_pitch_gd;
  // Member: balance_angle_pitch_elimit
  cdr << ros_message.balance_angle_pitch_elimit;
  // Member: balance_angle_pitch_olimit
  cdr << ros_message.balance_angle_pitch_olimit;
  // Member: balance_angle_pitch_neg_target
  cdr << ros_message.balance_angle_pitch_neg_target;
  // Member: balance_angle_pitch_pos_target
  cdr << ros_message.balance_angle_pitch_pos_target;
  // Member: balance_value_4
  cdr << ros_message.balance_value_4;
  // Member: balance_value_5
  cdr << ros_message.balance_value_5;
  // Member: balance_pitch_flag_imu
  cdr << ros_message.balance_pitch_flag_imu;
  // Member: balance_pitch_gp_imu
  cdr << ros_message.balance_pitch_gp_imu;
  // Member: balance_pitch_gi_imu
  cdr << ros_message.balance_pitch_gi_imu;
  // Member: balance_pitch_gd_imu
  cdr << ros_message.balance_pitch_gd_imu;
  // Member: balance_pitch_neg_target_imu
  cdr << ros_message.balance_pitch_neg_target_imu;
  // Member: balance_pitch_pos_target_imu
  cdr << ros_message.balance_pitch_pos_target_imu;
  // Member: balance_pitch_elimit_imu
  cdr << ros_message.balance_pitch_elimit_imu;
  // Member: balance_pitch_olimit_imu
  cdr << ros_message.balance_pitch_olimit_imu;
  // Member: balance_roll_flag_imu
  cdr << ros_message.balance_roll_flag_imu;
  // Member: balance_roll_gp_imu
  cdr << ros_message.balance_roll_gp_imu;
  // Member: balance_roll_gi_imu
  cdr << ros_message.balance_roll_gi_imu;
  // Member: balance_roll_gd_imu
  cdr << ros_message.balance_roll_gd_imu;
  // Member: balance_roll_neg_target_imu
  cdr << ros_message.balance_roll_neg_target_imu;
  // Member: balance_roll_pos_target_imu
  cdr << ros_message.balance_roll_pos_target_imu;
  // Member: balance_roll_elimit_imu
  cdr << ros_message.balance_roll_elimit_imu;
  // Member: balance_roll_olimit_imu
  cdr << ros_message.balance_roll_olimit_imu;
  // Member: balance_pitch_flag
  cdr << ros_message.balance_pitch_flag;
  // Member: balance_ankle_pitch_flag
  cdr << ros_message.balance_ankle_pitch_flag;
  // Member: balance_roll_flag
  cdr << ros_message.balance_roll_flag;
  // Member: ratio_check_flag
  cdr << ros_message.ratio_check_flag;
  // Member: center2leg
  cdr << ros_message.center2leg;
  // Member: link2link
  cdr << ros_message.link2link;
  // Member: init_z_up
  cdr << ros_message.init_z_up;
  // Member: percentage_of_ik_motor
  {
    cdr << ros_message.percentage_of_ik_motor;
  }
  // Member: first_pos_xr
  cdr << ros_message.first_pos_xr;
  // Member: first_neg_xr
  cdr << ros_message.first_neg_xr;
  // Member: first_pos_side_r
  cdr << ros_message.first_pos_side_r;
  // Member: first_neg_side_r
  cdr << ros_message.first_neg_side_r;
  // Member: first_pos_yaw_r
  cdr << ros_message.first_pos_yaw_r;
  // Member: first_neg_yaw_r
  cdr << ros_message.first_neg_yaw_r;
  // Member: first_pos_xl
  cdr << ros_message.first_pos_xl;
  // Member: first_neg_xl
  cdr << ros_message.first_neg_xl;
  // Member: first_pos_side_l
  cdr << ros_message.first_pos_side_l;
  // Member: first_neg_side_l
  cdr << ros_message.first_neg_side_l;
  // Member: first_pos_yaw_l
  cdr << ros_message.first_pos_yaw_l;
  // Member: first_neg_yaw_l
  cdr << ros_message.first_neg_yaw_l;
  // Member: first_pos_side_r_swing_minus
  cdr << ros_message.first_pos_side_r_swing_minus;
  // Member: first_neg_side_r_swing_minus
  cdr << ros_message.first_neg_side_r_swing_minus;
  // Member: first_pos_side_l_swing_minus
  cdr << ros_message.first_pos_side_l_swing_minus;
  // Member: first_neg_side_l_swing_minus
  cdr << ros_message.first_neg_side_l_swing_minus;
  // Member: first_min
  cdr << ros_message.first_min;
  // Member: first_max
  cdr << ros_message.first_max;
  // Member: second_pos_xr
  cdr << ros_message.second_pos_xr;
  // Member: second_neg_xr
  cdr << ros_message.second_neg_xr;
  // Member: second_pos_side_r
  cdr << ros_message.second_pos_side_r;
  // Member: second_neg_side_r
  cdr << ros_message.second_neg_side_r;
  // Member: second_pos_yaw_r
  cdr << ros_message.second_pos_yaw_r;
  // Member: second_neg_yaw_r
  cdr << ros_message.second_neg_yaw_r;
  // Member: second_pos_xl
  cdr << ros_message.second_pos_xl;
  // Member: second_neg_xl
  cdr << ros_message.second_neg_xl;
  // Member: second_pos_side_l
  cdr << ros_message.second_pos_side_l;
  // Member: second_neg_side_l
  cdr << ros_message.second_neg_side_l;
  // Member: second_pos_yaw_l
  cdr << ros_message.second_pos_yaw_l;
  // Member: second_neg_yaw_l
  cdr << ros_message.second_neg_yaw_l;
  // Member: second_pos_side_r_swing_minus
  cdr << ros_message.second_pos_side_r_swing_minus;
  // Member: second_neg_side_r_swing_minus
  cdr << ros_message.second_neg_side_r_swing_minus;
  // Member: second_pos_side_l_swing_minus
  cdr << ros_message.second_pos_side_l_swing_minus;
  // Member: second_neg_side_l_swing_minus
  cdr << ros_message.second_neg_side_l_swing_minus;
  // Member: second_min
  cdr << ros_message.second_min;
  // Member: second_max
  cdr << ros_message.second_max;
  // Member: third_pos_xr
  cdr << ros_message.third_pos_xr;
  // Member: third_neg_xr
  cdr << ros_message.third_neg_xr;
  // Member: third_pos_side_r
  cdr << ros_message.third_pos_side_r;
  // Member: third_neg_side_r
  cdr << ros_message.third_neg_side_r;
  // Member: third_pos_yaw_r
  cdr << ros_message.third_pos_yaw_r;
  // Member: third_neg_yaw_r
  cdr << ros_message.third_neg_yaw_r;
  // Member: third_pos_xl
  cdr << ros_message.third_pos_xl;
  // Member: third_neg_xl
  cdr << ros_message.third_neg_xl;
  // Member: third_pos_side_l
  cdr << ros_message.third_pos_side_l;
  // Member: third_neg_side_l
  cdr << ros_message.third_neg_side_l;
  // Member: third_pos_yaw_l
  cdr << ros_message.third_pos_yaw_l;
  // Member: third_neg_yaw_l
  cdr << ros_message.third_neg_yaw_l;
  // Member: third_pos_side_r_swing_minus
  cdr << ros_message.third_pos_side_r_swing_minus;
  // Member: third_neg_side_r_swing_minus
  cdr << ros_message.third_neg_side_r_swing_minus;
  // Member: third_pos_side_l_swing_minus
  cdr << ros_message.third_pos_side_l_swing_minus;
  // Member: third_neg_side_l_swing_minus
  cdr << ros_message.third_neg_side_l_swing_minus;
  // Member: third_min
  cdr << ros_message.third_min;
  // Member: third_max
  cdr << ros_message.third_max;
  // Member: fourth_pos_xr
  cdr << ros_message.fourth_pos_xr;
  // Member: fourth_neg_xr
  cdr << ros_message.fourth_neg_xr;
  // Member: fourth_pos_side_r
  cdr << ros_message.fourth_pos_side_r;
  // Member: fourth_neg_side_r
  cdr << ros_message.fourth_neg_side_r;
  // Member: fourth_pos_yaw_r
  cdr << ros_message.fourth_pos_yaw_r;
  // Member: fourth_neg_yaw_r
  cdr << ros_message.fourth_neg_yaw_r;
  // Member: fourth_pos_xl
  cdr << ros_message.fourth_pos_xl;
  // Member: fourth_neg_xl
  cdr << ros_message.fourth_neg_xl;
  // Member: fourth_pos_side_l
  cdr << ros_message.fourth_pos_side_l;
  // Member: fourth_neg_side_l
  cdr << ros_message.fourth_neg_side_l;
  // Member: fourth_pos_yaw_l
  cdr << ros_message.fourth_pos_yaw_l;
  // Member: fourth_neg_yaw_l
  cdr << ros_message.fourth_neg_yaw_l;
  // Member: fourth_pos_side_r_swing_minus
  cdr << ros_message.fourth_pos_side_r_swing_minus;
  // Member: fourth_neg_side_r_swing_minus
  cdr << ros_message.fourth_neg_side_r_swing_minus;
  // Member: fourth_pos_side_l_swing_minus
  cdr << ros_message.fourth_pos_side_l_swing_minus;
  // Member: fourth_neg_side_l_swing_minus
  cdr << ros_message.fourth_neg_side_l_swing_minus;
  // Member: fourth_min
  cdr << ros_message.fourth_min;
  // Member: fourth_max
  cdr << ros_message.fourth_max;
  // Member: landing_time_control_flag
  cdr << ros_message.landing_time_control_flag;
  // Member: control_system_flag
  cdr << ros_message.control_system_flag;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  humanoid_interfaces::msg::Tune2IkMsg & ros_message)
{
  // Member: ik_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.ik_flag = tmp ? true : false;
  }

  // Member: entire_time
  cdr >> ros_message.entire_time;

  // Member: frequency
  cdr >> ros_message.frequency;

  // Member: default_x_left
  cdr >> ros_message.default_x_left;

  // Member: default_y_left
  cdr >> ros_message.default_y_left;

  // Member: default_z_left
  cdr >> ros_message.default_z_left;

  // Member: default_x_right
  cdr >> ros_message.default_x_right;

  // Member: default_y_right
  cdr >> ros_message.default_y_right;

  // Member: default_z_right
  cdr >> ros_message.default_z_right;

  // Member: offset_motor
  {
    cdr >> ros_message.offset_motor;
  }

  // Member: swing_shoulder_left
  cdr >> ros_message.swing_shoulder_left;

  // Member: swing_shoulder_right
  cdr >> ros_message.swing_shoulder_right;

  // Member: swing_leg_left
  cdr >> ros_message.swing_leg_left;

  // Member: swing_leg_right
  cdr >> ros_message.swing_leg_right;

  // Member: rise_leg_left
  cdr >> ros_message.rise_leg_left;

  // Member: rise_leg_right
  cdr >> ros_message.rise_leg_right;

  // Member: start_entire_time
  cdr >> ros_message.start_entire_time;

  // Member: start_swing
  cdr >> ros_message.start_swing;

  // Member: start_rise
  cdr >> ros_message.start_rise;

  // Member: end_entire_time
  cdr >> ros_message.end_entire_time;

  // Member: end_swing
  cdr >> ros_message.end_swing;

  // Member: end_rise
  cdr >> ros_message.end_rise;

  // Member: test_x
  cdr >> ros_message.test_x;

  // Member: test_side
  cdr >> ros_message.test_side;

  // Member: test_yaw
  cdr >> ros_message.test_yaw;

  // Member: tuning_x
  cdr >> ros_message.tuning_x;

  // Member: tuning_side
  cdr >> ros_message.tuning_side;

  // Member: tuning_yaw
  cdr >> ros_message.tuning_yaw;

  // Member: balance_value_0
  cdr >> ros_message.balance_value_0;

  // Member: balance_value_1
  cdr >> ros_message.balance_value_1;

  // Member: balance_value_2
  cdr >> ros_message.balance_value_2;

  // Member: balance_value_3
  cdr >> ros_message.balance_value_3;

  // Member: balance_pitch_gp
  cdr >> ros_message.balance_pitch_gp;

  // Member: balance_pitch_gi
  cdr >> ros_message.balance_pitch_gi;

  // Member: balance_pitch_gd
  cdr >> ros_message.balance_pitch_gd;

  // Member: balance_pitch_elimit
  cdr >> ros_message.balance_pitch_elimit;

  // Member: balance_pitch_olimit
  cdr >> ros_message.balance_pitch_olimit;

  // Member: balance_pitch_neg_target
  cdr >> ros_message.balance_pitch_neg_target;

  // Member: balance_pitch_pos_target
  cdr >> ros_message.balance_pitch_pos_target;

  // Member: balance_roll_gp
  cdr >> ros_message.balance_roll_gp;

  // Member: balance_roll_gi
  cdr >> ros_message.balance_roll_gi;

  // Member: balance_roll_gd
  cdr >> ros_message.balance_roll_gd;

  // Member: balance_roll_elimit
  cdr >> ros_message.balance_roll_elimit;

  // Member: balance_roll_olimit
  cdr >> ros_message.balance_roll_olimit;

  // Member: balance_roll_neg_target
  cdr >> ros_message.balance_roll_neg_target;

  // Member: balance_roll_pos_target
  cdr >> ros_message.balance_roll_pos_target;

  // Member: balance_angle_pitch_gp
  cdr >> ros_message.balance_angle_pitch_gp;

  // Member: balance_angle_pitch_gi
  cdr >> ros_message.balance_angle_pitch_gi;

  // Member: balance_angle_pitch_gd
  cdr >> ros_message.balance_angle_pitch_gd;

  // Member: balance_angle_pitch_elimit
  cdr >> ros_message.balance_angle_pitch_elimit;

  // Member: balance_angle_pitch_olimit
  cdr >> ros_message.balance_angle_pitch_olimit;

  // Member: balance_angle_pitch_neg_target
  cdr >> ros_message.balance_angle_pitch_neg_target;

  // Member: balance_angle_pitch_pos_target
  cdr >> ros_message.balance_angle_pitch_pos_target;

  // Member: balance_value_4
  cdr >> ros_message.balance_value_4;

  // Member: balance_value_5
  cdr >> ros_message.balance_value_5;

  // Member: balance_pitch_flag_imu
  cdr >> ros_message.balance_pitch_flag_imu;

  // Member: balance_pitch_gp_imu
  cdr >> ros_message.balance_pitch_gp_imu;

  // Member: balance_pitch_gi_imu
  cdr >> ros_message.balance_pitch_gi_imu;

  // Member: balance_pitch_gd_imu
  cdr >> ros_message.balance_pitch_gd_imu;

  // Member: balance_pitch_neg_target_imu
  cdr >> ros_message.balance_pitch_neg_target_imu;

  // Member: balance_pitch_pos_target_imu
  cdr >> ros_message.balance_pitch_pos_target_imu;

  // Member: balance_pitch_elimit_imu
  cdr >> ros_message.balance_pitch_elimit_imu;

  // Member: balance_pitch_olimit_imu
  cdr >> ros_message.balance_pitch_olimit_imu;

  // Member: balance_roll_flag_imu
  cdr >> ros_message.balance_roll_flag_imu;

  // Member: balance_roll_gp_imu
  cdr >> ros_message.balance_roll_gp_imu;

  // Member: balance_roll_gi_imu
  cdr >> ros_message.balance_roll_gi_imu;

  // Member: balance_roll_gd_imu
  cdr >> ros_message.balance_roll_gd_imu;

  // Member: balance_roll_neg_target_imu
  cdr >> ros_message.balance_roll_neg_target_imu;

  // Member: balance_roll_pos_target_imu
  cdr >> ros_message.balance_roll_pos_target_imu;

  // Member: balance_roll_elimit_imu
  cdr >> ros_message.balance_roll_elimit_imu;

  // Member: balance_roll_olimit_imu
  cdr >> ros_message.balance_roll_olimit_imu;

  // Member: balance_pitch_flag
  cdr >> ros_message.balance_pitch_flag;

  // Member: balance_ankle_pitch_flag
  cdr >> ros_message.balance_ankle_pitch_flag;

  // Member: balance_roll_flag
  cdr >> ros_message.balance_roll_flag;

  // Member: ratio_check_flag
  cdr >> ros_message.ratio_check_flag;

  // Member: center2leg
  cdr >> ros_message.center2leg;

  // Member: link2link
  cdr >> ros_message.link2link;

  // Member: init_z_up
  cdr >> ros_message.init_z_up;

  // Member: percentage_of_ik_motor
  {
    cdr >> ros_message.percentage_of_ik_motor;
  }

  // Member: first_pos_xr
  cdr >> ros_message.first_pos_xr;

  // Member: first_neg_xr
  cdr >> ros_message.first_neg_xr;

  // Member: first_pos_side_r
  cdr >> ros_message.first_pos_side_r;

  // Member: first_neg_side_r
  cdr >> ros_message.first_neg_side_r;

  // Member: first_pos_yaw_r
  cdr >> ros_message.first_pos_yaw_r;

  // Member: first_neg_yaw_r
  cdr >> ros_message.first_neg_yaw_r;

  // Member: first_pos_xl
  cdr >> ros_message.first_pos_xl;

  // Member: first_neg_xl
  cdr >> ros_message.first_neg_xl;

  // Member: first_pos_side_l
  cdr >> ros_message.first_pos_side_l;

  // Member: first_neg_side_l
  cdr >> ros_message.first_neg_side_l;

  // Member: first_pos_yaw_l
  cdr >> ros_message.first_pos_yaw_l;

  // Member: first_neg_yaw_l
  cdr >> ros_message.first_neg_yaw_l;

  // Member: first_pos_side_r_swing_minus
  cdr >> ros_message.first_pos_side_r_swing_minus;

  // Member: first_neg_side_r_swing_minus
  cdr >> ros_message.first_neg_side_r_swing_minus;

  // Member: first_pos_side_l_swing_minus
  cdr >> ros_message.first_pos_side_l_swing_minus;

  // Member: first_neg_side_l_swing_minus
  cdr >> ros_message.first_neg_side_l_swing_minus;

  // Member: first_min
  cdr >> ros_message.first_min;

  // Member: first_max
  cdr >> ros_message.first_max;

  // Member: second_pos_xr
  cdr >> ros_message.second_pos_xr;

  // Member: second_neg_xr
  cdr >> ros_message.second_neg_xr;

  // Member: second_pos_side_r
  cdr >> ros_message.second_pos_side_r;

  // Member: second_neg_side_r
  cdr >> ros_message.second_neg_side_r;

  // Member: second_pos_yaw_r
  cdr >> ros_message.second_pos_yaw_r;

  // Member: second_neg_yaw_r
  cdr >> ros_message.second_neg_yaw_r;

  // Member: second_pos_xl
  cdr >> ros_message.second_pos_xl;

  // Member: second_neg_xl
  cdr >> ros_message.second_neg_xl;

  // Member: second_pos_side_l
  cdr >> ros_message.second_pos_side_l;

  // Member: second_neg_side_l
  cdr >> ros_message.second_neg_side_l;

  // Member: second_pos_yaw_l
  cdr >> ros_message.second_pos_yaw_l;

  // Member: second_neg_yaw_l
  cdr >> ros_message.second_neg_yaw_l;

  // Member: second_pos_side_r_swing_minus
  cdr >> ros_message.second_pos_side_r_swing_minus;

  // Member: second_neg_side_r_swing_minus
  cdr >> ros_message.second_neg_side_r_swing_minus;

  // Member: second_pos_side_l_swing_minus
  cdr >> ros_message.second_pos_side_l_swing_minus;

  // Member: second_neg_side_l_swing_minus
  cdr >> ros_message.second_neg_side_l_swing_minus;

  // Member: second_min
  cdr >> ros_message.second_min;

  // Member: second_max
  cdr >> ros_message.second_max;

  // Member: third_pos_xr
  cdr >> ros_message.third_pos_xr;

  // Member: third_neg_xr
  cdr >> ros_message.third_neg_xr;

  // Member: third_pos_side_r
  cdr >> ros_message.third_pos_side_r;

  // Member: third_neg_side_r
  cdr >> ros_message.third_neg_side_r;

  // Member: third_pos_yaw_r
  cdr >> ros_message.third_pos_yaw_r;

  // Member: third_neg_yaw_r
  cdr >> ros_message.third_neg_yaw_r;

  // Member: third_pos_xl
  cdr >> ros_message.third_pos_xl;

  // Member: third_neg_xl
  cdr >> ros_message.third_neg_xl;

  // Member: third_pos_side_l
  cdr >> ros_message.third_pos_side_l;

  // Member: third_neg_side_l
  cdr >> ros_message.third_neg_side_l;

  // Member: third_pos_yaw_l
  cdr >> ros_message.third_pos_yaw_l;

  // Member: third_neg_yaw_l
  cdr >> ros_message.third_neg_yaw_l;

  // Member: third_pos_side_r_swing_minus
  cdr >> ros_message.third_pos_side_r_swing_minus;

  // Member: third_neg_side_r_swing_minus
  cdr >> ros_message.third_neg_side_r_swing_minus;

  // Member: third_pos_side_l_swing_minus
  cdr >> ros_message.third_pos_side_l_swing_minus;

  // Member: third_neg_side_l_swing_minus
  cdr >> ros_message.third_neg_side_l_swing_minus;

  // Member: third_min
  cdr >> ros_message.third_min;

  // Member: third_max
  cdr >> ros_message.third_max;

  // Member: fourth_pos_xr
  cdr >> ros_message.fourth_pos_xr;

  // Member: fourth_neg_xr
  cdr >> ros_message.fourth_neg_xr;

  // Member: fourth_pos_side_r
  cdr >> ros_message.fourth_pos_side_r;

  // Member: fourth_neg_side_r
  cdr >> ros_message.fourth_neg_side_r;

  // Member: fourth_pos_yaw_r
  cdr >> ros_message.fourth_pos_yaw_r;

  // Member: fourth_neg_yaw_r
  cdr >> ros_message.fourth_neg_yaw_r;

  // Member: fourth_pos_xl
  cdr >> ros_message.fourth_pos_xl;

  // Member: fourth_neg_xl
  cdr >> ros_message.fourth_neg_xl;

  // Member: fourth_pos_side_l
  cdr >> ros_message.fourth_pos_side_l;

  // Member: fourth_neg_side_l
  cdr >> ros_message.fourth_neg_side_l;

  // Member: fourth_pos_yaw_l
  cdr >> ros_message.fourth_pos_yaw_l;

  // Member: fourth_neg_yaw_l
  cdr >> ros_message.fourth_neg_yaw_l;

  // Member: fourth_pos_side_r_swing_minus
  cdr >> ros_message.fourth_pos_side_r_swing_minus;

  // Member: fourth_neg_side_r_swing_minus
  cdr >> ros_message.fourth_neg_side_r_swing_minus;

  // Member: fourth_pos_side_l_swing_minus
  cdr >> ros_message.fourth_pos_side_l_swing_minus;

  // Member: fourth_neg_side_l_swing_minus
  cdr >> ros_message.fourth_neg_side_l_swing_minus;

  // Member: fourth_min
  cdr >> ros_message.fourth_min;

  // Member: fourth_max
  cdr >> ros_message.fourth_max;

  // Member: landing_time_control_flag
  cdr >> ros_message.landing_time_control_flag;

  // Member: control_system_flag
  cdr >> ros_message.control_system_flag;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
get_serialized_size(
  const humanoid_interfaces::msg::Tune2IkMsg & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: ik_flag
  {
    size_t item_size = sizeof(ros_message.ik_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: entire_time
  {
    size_t item_size = sizeof(ros_message.entire_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: frequency
  {
    size_t item_size = sizeof(ros_message.frequency);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: default_x_left
  {
    size_t item_size = sizeof(ros_message.default_x_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: default_y_left
  {
    size_t item_size = sizeof(ros_message.default_y_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: default_z_left
  {
    size_t item_size = sizeof(ros_message.default_z_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: default_x_right
  {
    size_t item_size = sizeof(ros_message.default_x_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: default_y_right
  {
    size_t item_size = sizeof(ros_message.default_y_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: default_z_right
  {
    size_t item_size = sizeof(ros_message.default_z_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: offset_motor
  {
    size_t array_size = ros_message.offset_motor.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.offset_motor[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: swing_shoulder_left
  {
    size_t item_size = sizeof(ros_message.swing_shoulder_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: swing_shoulder_right
  {
    size_t item_size = sizeof(ros_message.swing_shoulder_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: swing_leg_left
  {
    size_t item_size = sizeof(ros_message.swing_leg_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: swing_leg_right
  {
    size_t item_size = sizeof(ros_message.swing_leg_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rise_leg_left
  {
    size_t item_size = sizeof(ros_message.rise_leg_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rise_leg_right
  {
    size_t item_size = sizeof(ros_message.rise_leg_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: start_entire_time
  {
    size_t item_size = sizeof(ros_message.start_entire_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: start_swing
  {
    size_t item_size = sizeof(ros_message.start_swing);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: start_rise
  {
    size_t item_size = sizeof(ros_message.start_rise);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: end_entire_time
  {
    size_t item_size = sizeof(ros_message.end_entire_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: end_swing
  {
    size_t item_size = sizeof(ros_message.end_swing);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: end_rise
  {
    size_t item_size = sizeof(ros_message.end_rise);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: test_x
  {
    size_t item_size = sizeof(ros_message.test_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: test_side
  {
    size_t item_size = sizeof(ros_message.test_side);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: test_yaw
  {
    size_t item_size = sizeof(ros_message.test_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tuning_x
  {
    size_t item_size = sizeof(ros_message.tuning_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tuning_side
  {
    size_t item_size = sizeof(ros_message.tuning_side);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tuning_yaw
  {
    size_t item_size = sizeof(ros_message.tuning_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_value_0
  {
    size_t item_size = sizeof(ros_message.balance_value_0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_value_1
  {
    size_t item_size = sizeof(ros_message.balance_value_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_value_2
  {
    size_t item_size = sizeof(ros_message.balance_value_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_value_3
  {
    size_t item_size = sizeof(ros_message.balance_value_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_gp
  {
    size_t item_size = sizeof(ros_message.balance_pitch_gp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_gi
  {
    size_t item_size = sizeof(ros_message.balance_pitch_gi);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_gd
  {
    size_t item_size = sizeof(ros_message.balance_pitch_gd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_elimit
  {
    size_t item_size = sizeof(ros_message.balance_pitch_elimit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_olimit
  {
    size_t item_size = sizeof(ros_message.balance_pitch_olimit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_neg_target
  {
    size_t item_size = sizeof(ros_message.balance_pitch_neg_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_pos_target
  {
    size_t item_size = sizeof(ros_message.balance_pitch_pos_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_gp
  {
    size_t item_size = sizeof(ros_message.balance_roll_gp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_gi
  {
    size_t item_size = sizeof(ros_message.balance_roll_gi);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_gd
  {
    size_t item_size = sizeof(ros_message.balance_roll_gd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_elimit
  {
    size_t item_size = sizeof(ros_message.balance_roll_elimit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_olimit
  {
    size_t item_size = sizeof(ros_message.balance_roll_olimit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_neg_target
  {
    size_t item_size = sizeof(ros_message.balance_roll_neg_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_pos_target
  {
    size_t item_size = sizeof(ros_message.balance_roll_pos_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_angle_pitch_gp
  {
    size_t item_size = sizeof(ros_message.balance_angle_pitch_gp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_angle_pitch_gi
  {
    size_t item_size = sizeof(ros_message.balance_angle_pitch_gi);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_angle_pitch_gd
  {
    size_t item_size = sizeof(ros_message.balance_angle_pitch_gd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_angle_pitch_elimit
  {
    size_t item_size = sizeof(ros_message.balance_angle_pitch_elimit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_angle_pitch_olimit
  {
    size_t item_size = sizeof(ros_message.balance_angle_pitch_olimit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_angle_pitch_neg_target
  {
    size_t item_size = sizeof(ros_message.balance_angle_pitch_neg_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_angle_pitch_pos_target
  {
    size_t item_size = sizeof(ros_message.balance_angle_pitch_pos_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_value_4
  {
    size_t item_size = sizeof(ros_message.balance_value_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_value_5
  {
    size_t item_size = sizeof(ros_message.balance_value_5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_flag_imu
  {
    size_t item_size = sizeof(ros_message.balance_pitch_flag_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_gp_imu
  {
    size_t item_size = sizeof(ros_message.balance_pitch_gp_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_gi_imu
  {
    size_t item_size = sizeof(ros_message.balance_pitch_gi_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_gd_imu
  {
    size_t item_size = sizeof(ros_message.balance_pitch_gd_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_neg_target_imu
  {
    size_t item_size = sizeof(ros_message.balance_pitch_neg_target_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_pos_target_imu
  {
    size_t item_size = sizeof(ros_message.balance_pitch_pos_target_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_elimit_imu
  {
    size_t item_size = sizeof(ros_message.balance_pitch_elimit_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_olimit_imu
  {
    size_t item_size = sizeof(ros_message.balance_pitch_olimit_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_flag_imu
  {
    size_t item_size = sizeof(ros_message.balance_roll_flag_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_gp_imu
  {
    size_t item_size = sizeof(ros_message.balance_roll_gp_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_gi_imu
  {
    size_t item_size = sizeof(ros_message.balance_roll_gi_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_gd_imu
  {
    size_t item_size = sizeof(ros_message.balance_roll_gd_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_neg_target_imu
  {
    size_t item_size = sizeof(ros_message.balance_roll_neg_target_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_pos_target_imu
  {
    size_t item_size = sizeof(ros_message.balance_roll_pos_target_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_elimit_imu
  {
    size_t item_size = sizeof(ros_message.balance_roll_elimit_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_olimit_imu
  {
    size_t item_size = sizeof(ros_message.balance_roll_olimit_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_pitch_flag
  {
    size_t item_size = sizeof(ros_message.balance_pitch_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_ankle_pitch_flag
  {
    size_t item_size = sizeof(ros_message.balance_ankle_pitch_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balance_roll_flag
  {
    size_t item_size = sizeof(ros_message.balance_roll_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ratio_check_flag
  {
    size_t item_size = sizeof(ros_message.ratio_check_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: center2leg
  {
    size_t item_size = sizeof(ros_message.center2leg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: link2link
  {
    size_t item_size = sizeof(ros_message.link2link);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: init_z_up
  {
    size_t item_size = sizeof(ros_message.init_z_up);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: percentage_of_ik_motor
  {
    size_t array_size = ros_message.percentage_of_ik_motor.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.percentage_of_ik_motor[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_pos_xr
  {
    size_t item_size = sizeof(ros_message.first_pos_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_neg_xr
  {
    size_t item_size = sizeof(ros_message.first_neg_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_pos_side_r
  {
    size_t item_size = sizeof(ros_message.first_pos_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_neg_side_r
  {
    size_t item_size = sizeof(ros_message.first_neg_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_pos_yaw_r
  {
    size_t item_size = sizeof(ros_message.first_pos_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_neg_yaw_r
  {
    size_t item_size = sizeof(ros_message.first_neg_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_pos_xl
  {
    size_t item_size = sizeof(ros_message.first_pos_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_neg_xl
  {
    size_t item_size = sizeof(ros_message.first_neg_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_pos_side_l
  {
    size_t item_size = sizeof(ros_message.first_pos_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_neg_side_l
  {
    size_t item_size = sizeof(ros_message.first_neg_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_pos_yaw_l
  {
    size_t item_size = sizeof(ros_message.first_pos_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_neg_yaw_l
  {
    size_t item_size = sizeof(ros_message.first_neg_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_pos_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message.first_pos_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_neg_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message.first_neg_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_pos_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message.first_pos_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_neg_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message.first_neg_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_min
  {
    size_t item_size = sizeof(ros_message.first_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_max
  {
    size_t item_size = sizeof(ros_message.first_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_pos_xr
  {
    size_t item_size = sizeof(ros_message.second_pos_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_neg_xr
  {
    size_t item_size = sizeof(ros_message.second_neg_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_pos_side_r
  {
    size_t item_size = sizeof(ros_message.second_pos_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_neg_side_r
  {
    size_t item_size = sizeof(ros_message.second_neg_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_pos_yaw_r
  {
    size_t item_size = sizeof(ros_message.second_pos_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_neg_yaw_r
  {
    size_t item_size = sizeof(ros_message.second_neg_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_pos_xl
  {
    size_t item_size = sizeof(ros_message.second_pos_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_neg_xl
  {
    size_t item_size = sizeof(ros_message.second_neg_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_pos_side_l
  {
    size_t item_size = sizeof(ros_message.second_pos_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_neg_side_l
  {
    size_t item_size = sizeof(ros_message.second_neg_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_pos_yaw_l
  {
    size_t item_size = sizeof(ros_message.second_pos_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_neg_yaw_l
  {
    size_t item_size = sizeof(ros_message.second_neg_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_pos_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message.second_pos_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_neg_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message.second_neg_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_pos_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message.second_pos_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_neg_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message.second_neg_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_min
  {
    size_t item_size = sizeof(ros_message.second_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: second_max
  {
    size_t item_size = sizeof(ros_message.second_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_pos_xr
  {
    size_t item_size = sizeof(ros_message.third_pos_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_neg_xr
  {
    size_t item_size = sizeof(ros_message.third_neg_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_pos_side_r
  {
    size_t item_size = sizeof(ros_message.third_pos_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_neg_side_r
  {
    size_t item_size = sizeof(ros_message.third_neg_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_pos_yaw_r
  {
    size_t item_size = sizeof(ros_message.third_pos_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_neg_yaw_r
  {
    size_t item_size = sizeof(ros_message.third_neg_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_pos_xl
  {
    size_t item_size = sizeof(ros_message.third_pos_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_neg_xl
  {
    size_t item_size = sizeof(ros_message.third_neg_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_pos_side_l
  {
    size_t item_size = sizeof(ros_message.third_pos_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_neg_side_l
  {
    size_t item_size = sizeof(ros_message.third_neg_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_pos_yaw_l
  {
    size_t item_size = sizeof(ros_message.third_pos_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_neg_yaw_l
  {
    size_t item_size = sizeof(ros_message.third_neg_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_pos_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message.third_pos_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_neg_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message.third_neg_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_pos_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message.third_pos_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_neg_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message.third_neg_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_min
  {
    size_t item_size = sizeof(ros_message.third_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: third_max
  {
    size_t item_size = sizeof(ros_message.third_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_pos_xr
  {
    size_t item_size = sizeof(ros_message.fourth_pos_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_neg_xr
  {
    size_t item_size = sizeof(ros_message.fourth_neg_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_pos_side_r
  {
    size_t item_size = sizeof(ros_message.fourth_pos_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_neg_side_r
  {
    size_t item_size = sizeof(ros_message.fourth_neg_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_pos_yaw_r
  {
    size_t item_size = sizeof(ros_message.fourth_pos_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_neg_yaw_r
  {
    size_t item_size = sizeof(ros_message.fourth_neg_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_pos_xl
  {
    size_t item_size = sizeof(ros_message.fourth_pos_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_neg_xl
  {
    size_t item_size = sizeof(ros_message.fourth_neg_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_pos_side_l
  {
    size_t item_size = sizeof(ros_message.fourth_pos_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_neg_side_l
  {
    size_t item_size = sizeof(ros_message.fourth_neg_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_pos_yaw_l
  {
    size_t item_size = sizeof(ros_message.fourth_pos_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_neg_yaw_l
  {
    size_t item_size = sizeof(ros_message.fourth_neg_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_pos_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message.fourth_pos_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_neg_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message.fourth_neg_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_pos_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message.fourth_pos_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_neg_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message.fourth_neg_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_min
  {
    size_t item_size = sizeof(ros_message.fourth_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fourth_max
  {
    size_t item_size = sizeof(ros_message.fourth_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: landing_time_control_flag
  {
    size_t item_size = sizeof(ros_message.landing_time_control_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: control_system_flag
  {
    size_t item_size = sizeof(ros_message.control_system_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
max_serialized_size_Tune2IkMsg(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: ik_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: entire_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: frequency
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: default_x_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: default_y_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: default_z_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: default_x_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: default_y_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: default_z_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: offset_motor
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: swing_shoulder_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: swing_shoulder_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: swing_leg_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: swing_leg_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: rise_leg_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: rise_leg_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: start_entire_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: start_swing
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: start_rise
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: end_entire_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: end_swing
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: end_rise
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: test_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: test_side
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: test_yaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: tuning_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: tuning_side
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: tuning_yaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_value_0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_value_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_value_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_value_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_gp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_gi
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_gd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_elimit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_olimit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_neg_target
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_pos_target
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_gp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_gi
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_gd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_elimit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_olimit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_neg_target
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_pos_target
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_angle_pitch_gp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_angle_pitch_gi
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_angle_pitch_gd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_angle_pitch_elimit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_angle_pitch_olimit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_angle_pitch_neg_target
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_angle_pitch_pos_target
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_value_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_value_5
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_flag_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_gp_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_gi_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_gd_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_neg_target_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_pos_target_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_elimit_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_olimit_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_flag_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_gp_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_gi_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_gd_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_neg_target_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_pos_target_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_elimit_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_olimit_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_pitch_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_ankle_pitch_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balance_roll_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ratio_check_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: center2leg
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: link2link
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: init_z_up
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: percentage_of_ik_motor
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_pos_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_neg_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_pos_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_neg_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_pos_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_neg_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_pos_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_neg_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_pos_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_neg_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_pos_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_neg_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_pos_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_neg_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_pos_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_neg_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_min
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: first_max
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_pos_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_neg_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_pos_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_neg_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_pos_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_neg_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_pos_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_neg_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_pos_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_neg_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_pos_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_neg_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_pos_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_neg_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_pos_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_neg_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_min
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: second_max
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_pos_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_neg_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_pos_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_neg_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_pos_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_neg_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_pos_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_neg_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_pos_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_neg_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_pos_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_neg_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_pos_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_neg_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_pos_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_neg_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_min
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: third_max
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_pos_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_neg_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_pos_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_neg_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_pos_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_neg_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_pos_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_neg_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_pos_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_neg_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_pos_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_neg_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_pos_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_neg_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_pos_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_neg_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_min
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: fourth_max
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: landing_time_control_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: control_system_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = humanoid_interfaces::msg::Tune2IkMsg;
    is_plain =
      (
      offsetof(DataType, control_system_flag) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Tune2IkMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const humanoid_interfaces::msg::Tune2IkMsg *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Tune2IkMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<humanoid_interfaces::msg::Tune2IkMsg *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Tune2IkMsg__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const humanoid_interfaces::msg::Tune2IkMsg *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Tune2IkMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Tune2IkMsg(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Tune2IkMsg__callbacks = {
  "humanoid_interfaces::msg",
  "Tune2IkMsg",
  _Tune2IkMsg__cdr_serialize,
  _Tune2IkMsg__cdr_deserialize,
  _Tune2IkMsg__get_serialized_size,
  _Tune2IkMsg__max_serialized_size
};

static rosidl_message_type_support_t _Tune2IkMsg__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Tune2IkMsg__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace humanoid_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_humanoid_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<humanoid_interfaces::msg::Tune2IkMsg>()
{
  return &humanoid_interfaces::msg::typesupport_fastrtps_cpp::_Tune2IkMsg__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, humanoid_interfaces, msg, Tune2IkMsg)() {
  return &humanoid_interfaces::msg::typesupport_fastrtps_cpp::_Tune2IkMsg__handle;
}

#ifdef __cplusplus
}
#endif
