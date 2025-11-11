// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/Tune2IkMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__TUNE2_IK_MSG__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__TUNE2_IK_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/tune2_ik_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Tune2IkMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: ik_flag
  {
    out << "ik_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.ik_flag, out);
    out << ", ";
  }

  // member: entire_time
  {
    out << "entire_time: ";
    rosidl_generator_traits::value_to_yaml(msg.entire_time, out);
    out << ", ";
  }

  // member: frequency
  {
    out << "frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.frequency, out);
    out << ", ";
  }

  // member: default_x_left
  {
    out << "default_x_left: ";
    rosidl_generator_traits::value_to_yaml(msg.default_x_left, out);
    out << ", ";
  }

  // member: default_y_left
  {
    out << "default_y_left: ";
    rosidl_generator_traits::value_to_yaml(msg.default_y_left, out);
    out << ", ";
  }

  // member: default_z_left
  {
    out << "default_z_left: ";
    rosidl_generator_traits::value_to_yaml(msg.default_z_left, out);
    out << ", ";
  }

  // member: default_x_right
  {
    out << "default_x_right: ";
    rosidl_generator_traits::value_to_yaml(msg.default_x_right, out);
    out << ", ";
  }

  // member: default_y_right
  {
    out << "default_y_right: ";
    rosidl_generator_traits::value_to_yaml(msg.default_y_right, out);
    out << ", ";
  }

  // member: default_z_right
  {
    out << "default_z_right: ";
    rosidl_generator_traits::value_to_yaml(msg.default_z_right, out);
    out << ", ";
  }

  // member: offset_motor
  {
    if (msg.offset_motor.size() == 0) {
      out << "offset_motor: []";
    } else {
      out << "offset_motor: [";
      size_t pending_items = msg.offset_motor.size();
      for (auto item : msg.offset_motor) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: swing_shoulder_left
  {
    out << "swing_shoulder_left: ";
    rosidl_generator_traits::value_to_yaml(msg.swing_shoulder_left, out);
    out << ", ";
  }

  // member: swing_shoulder_right
  {
    out << "swing_shoulder_right: ";
    rosidl_generator_traits::value_to_yaml(msg.swing_shoulder_right, out);
    out << ", ";
  }

  // member: swing_leg_left
  {
    out << "swing_leg_left: ";
    rosidl_generator_traits::value_to_yaml(msg.swing_leg_left, out);
    out << ", ";
  }

  // member: swing_leg_right
  {
    out << "swing_leg_right: ";
    rosidl_generator_traits::value_to_yaml(msg.swing_leg_right, out);
    out << ", ";
  }

  // member: rise_leg_left
  {
    out << "rise_leg_left: ";
    rosidl_generator_traits::value_to_yaml(msg.rise_leg_left, out);
    out << ", ";
  }

  // member: rise_leg_right
  {
    out << "rise_leg_right: ";
    rosidl_generator_traits::value_to_yaml(msg.rise_leg_right, out);
    out << ", ";
  }

  // member: start_entire_time
  {
    out << "start_entire_time: ";
    rosidl_generator_traits::value_to_yaml(msg.start_entire_time, out);
    out << ", ";
  }

  // member: start_swing
  {
    out << "start_swing: ";
    rosidl_generator_traits::value_to_yaml(msg.start_swing, out);
    out << ", ";
  }

  // member: start_rise
  {
    out << "start_rise: ";
    rosidl_generator_traits::value_to_yaml(msg.start_rise, out);
    out << ", ";
  }

  // member: end_entire_time
  {
    out << "end_entire_time: ";
    rosidl_generator_traits::value_to_yaml(msg.end_entire_time, out);
    out << ", ";
  }

  // member: end_swing
  {
    out << "end_swing: ";
    rosidl_generator_traits::value_to_yaml(msg.end_swing, out);
    out << ", ";
  }

  // member: end_rise
  {
    out << "end_rise: ";
    rosidl_generator_traits::value_to_yaml(msg.end_rise, out);
    out << ", ";
  }

  // member: test_x
  {
    out << "test_x: ";
    rosidl_generator_traits::value_to_yaml(msg.test_x, out);
    out << ", ";
  }

  // member: test_side
  {
    out << "test_side: ";
    rosidl_generator_traits::value_to_yaml(msg.test_side, out);
    out << ", ";
  }

  // member: test_yaw
  {
    out << "test_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.test_yaw, out);
    out << ", ";
  }

  // member: tuning_x
  {
    out << "tuning_x: ";
    rosidl_generator_traits::value_to_yaml(msg.tuning_x, out);
    out << ", ";
  }

  // member: tuning_side
  {
    out << "tuning_side: ";
    rosidl_generator_traits::value_to_yaml(msg.tuning_side, out);
    out << ", ";
  }

  // member: tuning_yaw
  {
    out << "tuning_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.tuning_yaw, out);
    out << ", ";
  }

  // member: balance_value_0
  {
    out << "balance_value_0: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_value_0, out);
    out << ", ";
  }

  // member: balance_value_1
  {
    out << "balance_value_1: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_value_1, out);
    out << ", ";
  }

  // member: balance_value_2
  {
    out << "balance_value_2: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_value_2, out);
    out << ", ";
  }

  // member: balance_value_3
  {
    out << "balance_value_3: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_value_3, out);
    out << ", ";
  }

  // member: balance_pitch_gp
  {
    out << "balance_pitch_gp: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_gp, out);
    out << ", ";
  }

  // member: balance_pitch_gi
  {
    out << "balance_pitch_gi: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_gi, out);
    out << ", ";
  }

  // member: balance_pitch_gd
  {
    out << "balance_pitch_gd: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_gd, out);
    out << ", ";
  }

  // member: balance_pitch_elimit
  {
    out << "balance_pitch_elimit: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_elimit, out);
    out << ", ";
  }

  // member: balance_pitch_olimit
  {
    out << "balance_pitch_olimit: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_olimit, out);
    out << ", ";
  }

  // member: balance_pitch_neg_target
  {
    out << "balance_pitch_neg_target: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_neg_target, out);
    out << ", ";
  }

  // member: balance_pitch_pos_target
  {
    out << "balance_pitch_pos_target: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_pos_target, out);
    out << ", ";
  }

  // member: balance_roll_gp
  {
    out << "balance_roll_gp: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_gp, out);
    out << ", ";
  }

  // member: balance_roll_gi
  {
    out << "balance_roll_gi: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_gi, out);
    out << ", ";
  }

  // member: balance_roll_gd
  {
    out << "balance_roll_gd: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_gd, out);
    out << ", ";
  }

  // member: balance_roll_elimit
  {
    out << "balance_roll_elimit: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_elimit, out);
    out << ", ";
  }

  // member: balance_roll_olimit
  {
    out << "balance_roll_olimit: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_olimit, out);
    out << ", ";
  }

  // member: balance_roll_neg_target
  {
    out << "balance_roll_neg_target: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_neg_target, out);
    out << ", ";
  }

  // member: balance_roll_pos_target
  {
    out << "balance_roll_pos_target: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_pos_target, out);
    out << ", ";
  }

  // member: balance_angle_pitch_gp
  {
    out << "balance_angle_pitch_gp: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_angle_pitch_gp, out);
    out << ", ";
  }

  // member: balance_angle_pitch_gi
  {
    out << "balance_angle_pitch_gi: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_angle_pitch_gi, out);
    out << ", ";
  }

  // member: balance_angle_pitch_gd
  {
    out << "balance_angle_pitch_gd: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_angle_pitch_gd, out);
    out << ", ";
  }

  // member: balance_angle_pitch_elimit
  {
    out << "balance_angle_pitch_elimit: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_angle_pitch_elimit, out);
    out << ", ";
  }

  // member: balance_angle_pitch_olimit
  {
    out << "balance_angle_pitch_olimit: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_angle_pitch_olimit, out);
    out << ", ";
  }

  // member: balance_angle_pitch_neg_target
  {
    out << "balance_angle_pitch_neg_target: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_angle_pitch_neg_target, out);
    out << ", ";
  }

  // member: balance_angle_pitch_pos_target
  {
    out << "balance_angle_pitch_pos_target: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_angle_pitch_pos_target, out);
    out << ", ";
  }

  // member: balance_value_4
  {
    out << "balance_value_4: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_value_4, out);
    out << ", ";
  }

  // member: balance_value_5
  {
    out << "balance_value_5: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_value_5, out);
    out << ", ";
  }

  // member: balance_pitch_flag_imu
  {
    out << "balance_pitch_flag_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_flag_imu, out);
    out << ", ";
  }

  // member: balance_pitch_gp_imu
  {
    out << "balance_pitch_gp_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_gp_imu, out);
    out << ", ";
  }

  // member: balance_pitch_gi_imu
  {
    out << "balance_pitch_gi_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_gi_imu, out);
    out << ", ";
  }

  // member: balance_pitch_gd_imu
  {
    out << "balance_pitch_gd_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_gd_imu, out);
    out << ", ";
  }

  // member: balance_pitch_neg_target_imu
  {
    out << "balance_pitch_neg_target_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_neg_target_imu, out);
    out << ", ";
  }

  // member: balance_pitch_pos_target_imu
  {
    out << "balance_pitch_pos_target_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_pos_target_imu, out);
    out << ", ";
  }

  // member: balance_pitch_elimit_imu
  {
    out << "balance_pitch_elimit_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_elimit_imu, out);
    out << ", ";
  }

  // member: balance_pitch_olimit_imu
  {
    out << "balance_pitch_olimit_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_olimit_imu, out);
    out << ", ";
  }

  // member: balance_roll_flag_imu
  {
    out << "balance_roll_flag_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_flag_imu, out);
    out << ", ";
  }

  // member: balance_roll_gp_imu
  {
    out << "balance_roll_gp_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_gp_imu, out);
    out << ", ";
  }

  // member: balance_roll_gi_imu
  {
    out << "balance_roll_gi_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_gi_imu, out);
    out << ", ";
  }

  // member: balance_roll_gd_imu
  {
    out << "balance_roll_gd_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_gd_imu, out);
    out << ", ";
  }

  // member: balance_roll_neg_target_imu
  {
    out << "balance_roll_neg_target_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_neg_target_imu, out);
    out << ", ";
  }

  // member: balance_roll_pos_target_imu
  {
    out << "balance_roll_pos_target_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_pos_target_imu, out);
    out << ", ";
  }

  // member: balance_roll_elimit_imu
  {
    out << "balance_roll_elimit_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_elimit_imu, out);
    out << ", ";
  }

  // member: balance_roll_olimit_imu
  {
    out << "balance_roll_olimit_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_olimit_imu, out);
    out << ", ";
  }

  // member: balance_pitch_flag
  {
    out << "balance_pitch_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_flag, out);
    out << ", ";
  }

  // member: balance_ankle_pitch_flag
  {
    out << "balance_ankle_pitch_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_ankle_pitch_flag, out);
    out << ", ";
  }

  // member: balance_roll_flag
  {
    out << "balance_roll_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_flag, out);
    out << ", ";
  }

  // member: ratio_check_flag
  {
    out << "ratio_check_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.ratio_check_flag, out);
    out << ", ";
  }

  // member: center2leg
  {
    out << "center2leg: ";
    rosidl_generator_traits::value_to_yaml(msg.center2leg, out);
    out << ", ";
  }

  // member: link2link
  {
    out << "link2link: ";
    rosidl_generator_traits::value_to_yaml(msg.link2link, out);
    out << ", ";
  }

  // member: init_z_up
  {
    out << "init_z_up: ";
    rosidl_generator_traits::value_to_yaml(msg.init_z_up, out);
    out << ", ";
  }

  // member: percentage_of_ik_motor
  {
    if (msg.percentage_of_ik_motor.size() == 0) {
      out << "percentage_of_ik_motor: []";
    } else {
      out << "percentage_of_ik_motor: [";
      size_t pending_items = msg.percentage_of_ik_motor.size();
      for (auto item : msg.percentage_of_ik_motor) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: first_pos_xr
  {
    out << "first_pos_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_xr, out);
    out << ", ";
  }

  // member: first_neg_xr
  {
    out << "first_neg_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_xr, out);
    out << ", ";
  }

  // member: first_pos_side_r
  {
    out << "first_pos_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_side_r, out);
    out << ", ";
  }

  // member: first_neg_side_r
  {
    out << "first_neg_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_side_r, out);
    out << ", ";
  }

  // member: first_pos_yaw_r
  {
    out << "first_pos_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_yaw_r, out);
    out << ", ";
  }

  // member: first_neg_yaw_r
  {
    out << "first_neg_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_yaw_r, out);
    out << ", ";
  }

  // member: first_pos_xl
  {
    out << "first_pos_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_xl, out);
    out << ", ";
  }

  // member: first_neg_xl
  {
    out << "first_neg_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_xl, out);
    out << ", ";
  }

  // member: first_pos_side_l
  {
    out << "first_pos_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_side_l, out);
    out << ", ";
  }

  // member: first_neg_side_l
  {
    out << "first_neg_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_side_l, out);
    out << ", ";
  }

  // member: first_pos_yaw_l
  {
    out << "first_pos_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_yaw_l, out);
    out << ", ";
  }

  // member: first_neg_yaw_l
  {
    out << "first_neg_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_yaw_l, out);
    out << ", ";
  }

  // member: first_pos_side_r_swing_minus
  {
    out << "first_pos_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_side_r_swing_minus, out);
    out << ", ";
  }

  // member: first_neg_side_r_swing_minus
  {
    out << "first_neg_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_side_r_swing_minus, out);
    out << ", ";
  }

  // member: first_pos_side_l_swing_minus
  {
    out << "first_pos_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_side_l_swing_minus, out);
    out << ", ";
  }

  // member: first_neg_side_l_swing_minus
  {
    out << "first_neg_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_side_l_swing_minus, out);
    out << ", ";
  }

  // member: first_min
  {
    out << "first_min: ";
    rosidl_generator_traits::value_to_yaml(msg.first_min, out);
    out << ", ";
  }

  // member: first_max
  {
    out << "first_max: ";
    rosidl_generator_traits::value_to_yaml(msg.first_max, out);
    out << ", ";
  }

  // member: second_pos_xr
  {
    out << "second_pos_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_xr, out);
    out << ", ";
  }

  // member: second_neg_xr
  {
    out << "second_neg_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_xr, out);
    out << ", ";
  }

  // member: second_pos_side_r
  {
    out << "second_pos_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_side_r, out);
    out << ", ";
  }

  // member: second_neg_side_r
  {
    out << "second_neg_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_side_r, out);
    out << ", ";
  }

  // member: second_pos_yaw_r
  {
    out << "second_pos_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_yaw_r, out);
    out << ", ";
  }

  // member: second_neg_yaw_r
  {
    out << "second_neg_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_yaw_r, out);
    out << ", ";
  }

  // member: second_pos_xl
  {
    out << "second_pos_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_xl, out);
    out << ", ";
  }

  // member: second_neg_xl
  {
    out << "second_neg_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_xl, out);
    out << ", ";
  }

  // member: second_pos_side_l
  {
    out << "second_pos_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_side_l, out);
    out << ", ";
  }

  // member: second_neg_side_l
  {
    out << "second_neg_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_side_l, out);
    out << ", ";
  }

  // member: second_pos_yaw_l
  {
    out << "second_pos_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_yaw_l, out);
    out << ", ";
  }

  // member: second_neg_yaw_l
  {
    out << "second_neg_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_yaw_l, out);
    out << ", ";
  }

  // member: second_pos_side_r_swing_minus
  {
    out << "second_pos_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_side_r_swing_minus, out);
    out << ", ";
  }

  // member: second_neg_side_r_swing_minus
  {
    out << "second_neg_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_side_r_swing_minus, out);
    out << ", ";
  }

  // member: second_pos_side_l_swing_minus
  {
    out << "second_pos_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_side_l_swing_minus, out);
    out << ", ";
  }

  // member: second_neg_side_l_swing_minus
  {
    out << "second_neg_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_side_l_swing_minus, out);
    out << ", ";
  }

  // member: second_min
  {
    out << "second_min: ";
    rosidl_generator_traits::value_to_yaml(msg.second_min, out);
    out << ", ";
  }

  // member: second_max
  {
    out << "second_max: ";
    rosidl_generator_traits::value_to_yaml(msg.second_max, out);
    out << ", ";
  }

  // member: third_pos_xr
  {
    out << "third_pos_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_xr, out);
    out << ", ";
  }

  // member: third_neg_xr
  {
    out << "third_neg_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_xr, out);
    out << ", ";
  }

  // member: third_pos_side_r
  {
    out << "third_pos_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_side_r, out);
    out << ", ";
  }

  // member: third_neg_side_r
  {
    out << "third_neg_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_side_r, out);
    out << ", ";
  }

  // member: third_pos_yaw_r
  {
    out << "third_pos_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_yaw_r, out);
    out << ", ";
  }

  // member: third_neg_yaw_r
  {
    out << "third_neg_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_yaw_r, out);
    out << ", ";
  }

  // member: third_pos_xl
  {
    out << "third_pos_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_xl, out);
    out << ", ";
  }

  // member: third_neg_xl
  {
    out << "third_neg_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_xl, out);
    out << ", ";
  }

  // member: third_pos_side_l
  {
    out << "third_pos_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_side_l, out);
    out << ", ";
  }

  // member: third_neg_side_l
  {
    out << "third_neg_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_side_l, out);
    out << ", ";
  }

  // member: third_pos_yaw_l
  {
    out << "third_pos_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_yaw_l, out);
    out << ", ";
  }

  // member: third_neg_yaw_l
  {
    out << "third_neg_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_yaw_l, out);
    out << ", ";
  }

  // member: third_pos_side_r_swing_minus
  {
    out << "third_pos_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_side_r_swing_minus, out);
    out << ", ";
  }

  // member: third_neg_side_r_swing_minus
  {
    out << "third_neg_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_side_r_swing_minus, out);
    out << ", ";
  }

  // member: third_pos_side_l_swing_minus
  {
    out << "third_pos_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_side_l_swing_minus, out);
    out << ", ";
  }

  // member: third_neg_side_l_swing_minus
  {
    out << "third_neg_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_side_l_swing_minus, out);
    out << ", ";
  }

  // member: third_min
  {
    out << "third_min: ";
    rosidl_generator_traits::value_to_yaml(msg.third_min, out);
    out << ", ";
  }

  // member: third_max
  {
    out << "third_max: ";
    rosidl_generator_traits::value_to_yaml(msg.third_max, out);
    out << ", ";
  }

  // member: fourth_pos_xr
  {
    out << "fourth_pos_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_xr, out);
    out << ", ";
  }

  // member: fourth_neg_xr
  {
    out << "fourth_neg_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_xr, out);
    out << ", ";
  }

  // member: fourth_pos_side_r
  {
    out << "fourth_pos_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_side_r, out);
    out << ", ";
  }

  // member: fourth_neg_side_r
  {
    out << "fourth_neg_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_side_r, out);
    out << ", ";
  }

  // member: fourth_pos_yaw_r
  {
    out << "fourth_pos_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_yaw_r, out);
    out << ", ";
  }

  // member: fourth_neg_yaw_r
  {
    out << "fourth_neg_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_yaw_r, out);
    out << ", ";
  }

  // member: fourth_pos_xl
  {
    out << "fourth_pos_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_xl, out);
    out << ", ";
  }

  // member: fourth_neg_xl
  {
    out << "fourth_neg_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_xl, out);
    out << ", ";
  }

  // member: fourth_pos_side_l
  {
    out << "fourth_pos_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_side_l, out);
    out << ", ";
  }

  // member: fourth_neg_side_l
  {
    out << "fourth_neg_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_side_l, out);
    out << ", ";
  }

  // member: fourth_pos_yaw_l
  {
    out << "fourth_pos_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_yaw_l, out);
    out << ", ";
  }

  // member: fourth_neg_yaw_l
  {
    out << "fourth_neg_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_yaw_l, out);
    out << ", ";
  }

  // member: fourth_pos_side_r_swing_minus
  {
    out << "fourth_pos_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_side_r_swing_minus, out);
    out << ", ";
  }

  // member: fourth_neg_side_r_swing_minus
  {
    out << "fourth_neg_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_side_r_swing_minus, out);
    out << ", ";
  }

  // member: fourth_pos_side_l_swing_minus
  {
    out << "fourth_pos_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_side_l_swing_minus, out);
    out << ", ";
  }

  // member: fourth_neg_side_l_swing_minus
  {
    out << "fourth_neg_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_side_l_swing_minus, out);
    out << ", ";
  }

  // member: fourth_min
  {
    out << "fourth_min: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_min, out);
    out << ", ";
  }

  // member: fourth_max
  {
    out << "fourth_max: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_max, out);
    out << ", ";
  }

  // member: landing_time_control_flag
  {
    out << "landing_time_control_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.landing_time_control_flag, out);
    out << ", ";
  }

  // member: control_system_flag
  {
    out << "control_system_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.control_system_flag, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Tune2IkMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ik_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ik_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.ik_flag, out);
    out << "\n";
  }

  // member: entire_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "entire_time: ";
    rosidl_generator_traits::value_to_yaml(msg.entire_time, out);
    out << "\n";
  }

  // member: frequency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.frequency, out);
    out << "\n";
  }

  // member: default_x_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "default_x_left: ";
    rosidl_generator_traits::value_to_yaml(msg.default_x_left, out);
    out << "\n";
  }

  // member: default_y_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "default_y_left: ";
    rosidl_generator_traits::value_to_yaml(msg.default_y_left, out);
    out << "\n";
  }

  // member: default_z_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "default_z_left: ";
    rosidl_generator_traits::value_to_yaml(msg.default_z_left, out);
    out << "\n";
  }

  // member: default_x_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "default_x_right: ";
    rosidl_generator_traits::value_to_yaml(msg.default_x_right, out);
    out << "\n";
  }

  // member: default_y_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "default_y_right: ";
    rosidl_generator_traits::value_to_yaml(msg.default_y_right, out);
    out << "\n";
  }

  // member: default_z_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "default_z_right: ";
    rosidl_generator_traits::value_to_yaml(msg.default_z_right, out);
    out << "\n";
  }

  // member: offset_motor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.offset_motor.size() == 0) {
      out << "offset_motor: []\n";
    } else {
      out << "offset_motor:\n";
      for (auto item : msg.offset_motor) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: swing_shoulder_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "swing_shoulder_left: ";
    rosidl_generator_traits::value_to_yaml(msg.swing_shoulder_left, out);
    out << "\n";
  }

  // member: swing_shoulder_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "swing_shoulder_right: ";
    rosidl_generator_traits::value_to_yaml(msg.swing_shoulder_right, out);
    out << "\n";
  }

  // member: swing_leg_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "swing_leg_left: ";
    rosidl_generator_traits::value_to_yaml(msg.swing_leg_left, out);
    out << "\n";
  }

  // member: swing_leg_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "swing_leg_right: ";
    rosidl_generator_traits::value_to_yaml(msg.swing_leg_right, out);
    out << "\n";
  }

  // member: rise_leg_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rise_leg_left: ";
    rosidl_generator_traits::value_to_yaml(msg.rise_leg_left, out);
    out << "\n";
  }

  // member: rise_leg_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rise_leg_right: ";
    rosidl_generator_traits::value_to_yaml(msg.rise_leg_right, out);
    out << "\n";
  }

  // member: start_entire_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_entire_time: ";
    rosidl_generator_traits::value_to_yaml(msg.start_entire_time, out);
    out << "\n";
  }

  // member: start_swing
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_swing: ";
    rosidl_generator_traits::value_to_yaml(msg.start_swing, out);
    out << "\n";
  }

  // member: start_rise
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_rise: ";
    rosidl_generator_traits::value_to_yaml(msg.start_rise, out);
    out << "\n";
  }

  // member: end_entire_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_entire_time: ";
    rosidl_generator_traits::value_to_yaml(msg.end_entire_time, out);
    out << "\n";
  }

  // member: end_swing
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_swing: ";
    rosidl_generator_traits::value_to_yaml(msg.end_swing, out);
    out << "\n";
  }

  // member: end_rise
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_rise: ";
    rosidl_generator_traits::value_to_yaml(msg.end_rise, out);
    out << "\n";
  }

  // member: test_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "test_x: ";
    rosidl_generator_traits::value_to_yaml(msg.test_x, out);
    out << "\n";
  }

  // member: test_side
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "test_side: ";
    rosidl_generator_traits::value_to_yaml(msg.test_side, out);
    out << "\n";
  }

  // member: test_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "test_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.test_yaw, out);
    out << "\n";
  }

  // member: tuning_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tuning_x: ";
    rosidl_generator_traits::value_to_yaml(msg.tuning_x, out);
    out << "\n";
  }

  // member: tuning_side
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tuning_side: ";
    rosidl_generator_traits::value_to_yaml(msg.tuning_side, out);
    out << "\n";
  }

  // member: tuning_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tuning_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.tuning_yaw, out);
    out << "\n";
  }

  // member: balance_value_0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_value_0: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_value_0, out);
    out << "\n";
  }

  // member: balance_value_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_value_1: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_value_1, out);
    out << "\n";
  }

  // member: balance_value_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_value_2: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_value_2, out);
    out << "\n";
  }

  // member: balance_value_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_value_3: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_value_3, out);
    out << "\n";
  }

  // member: balance_pitch_gp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_gp: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_gp, out);
    out << "\n";
  }

  // member: balance_pitch_gi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_gi: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_gi, out);
    out << "\n";
  }

  // member: balance_pitch_gd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_gd: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_gd, out);
    out << "\n";
  }

  // member: balance_pitch_elimit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_elimit: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_elimit, out);
    out << "\n";
  }

  // member: balance_pitch_olimit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_olimit: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_olimit, out);
    out << "\n";
  }

  // member: balance_pitch_neg_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_neg_target: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_neg_target, out);
    out << "\n";
  }

  // member: balance_pitch_pos_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_pos_target: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_pos_target, out);
    out << "\n";
  }

  // member: balance_roll_gp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_gp: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_gp, out);
    out << "\n";
  }

  // member: balance_roll_gi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_gi: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_gi, out);
    out << "\n";
  }

  // member: balance_roll_gd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_gd: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_gd, out);
    out << "\n";
  }

  // member: balance_roll_elimit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_elimit: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_elimit, out);
    out << "\n";
  }

  // member: balance_roll_olimit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_olimit: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_olimit, out);
    out << "\n";
  }

  // member: balance_roll_neg_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_neg_target: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_neg_target, out);
    out << "\n";
  }

  // member: balance_roll_pos_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_pos_target: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_pos_target, out);
    out << "\n";
  }

  // member: balance_angle_pitch_gp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_angle_pitch_gp: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_angle_pitch_gp, out);
    out << "\n";
  }

  // member: balance_angle_pitch_gi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_angle_pitch_gi: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_angle_pitch_gi, out);
    out << "\n";
  }

  // member: balance_angle_pitch_gd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_angle_pitch_gd: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_angle_pitch_gd, out);
    out << "\n";
  }

  // member: balance_angle_pitch_elimit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_angle_pitch_elimit: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_angle_pitch_elimit, out);
    out << "\n";
  }

  // member: balance_angle_pitch_olimit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_angle_pitch_olimit: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_angle_pitch_olimit, out);
    out << "\n";
  }

  // member: balance_angle_pitch_neg_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_angle_pitch_neg_target: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_angle_pitch_neg_target, out);
    out << "\n";
  }

  // member: balance_angle_pitch_pos_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_angle_pitch_pos_target: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_angle_pitch_pos_target, out);
    out << "\n";
  }

  // member: balance_value_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_value_4: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_value_4, out);
    out << "\n";
  }

  // member: balance_value_5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_value_5: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_value_5, out);
    out << "\n";
  }

  // member: balance_pitch_flag_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_flag_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_flag_imu, out);
    out << "\n";
  }

  // member: balance_pitch_gp_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_gp_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_gp_imu, out);
    out << "\n";
  }

  // member: balance_pitch_gi_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_gi_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_gi_imu, out);
    out << "\n";
  }

  // member: balance_pitch_gd_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_gd_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_gd_imu, out);
    out << "\n";
  }

  // member: balance_pitch_neg_target_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_neg_target_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_neg_target_imu, out);
    out << "\n";
  }

  // member: balance_pitch_pos_target_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_pos_target_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_pos_target_imu, out);
    out << "\n";
  }

  // member: balance_pitch_elimit_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_elimit_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_elimit_imu, out);
    out << "\n";
  }

  // member: balance_pitch_olimit_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_olimit_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_olimit_imu, out);
    out << "\n";
  }

  // member: balance_roll_flag_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_flag_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_flag_imu, out);
    out << "\n";
  }

  // member: balance_roll_gp_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_gp_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_gp_imu, out);
    out << "\n";
  }

  // member: balance_roll_gi_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_gi_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_gi_imu, out);
    out << "\n";
  }

  // member: balance_roll_gd_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_gd_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_gd_imu, out);
    out << "\n";
  }

  // member: balance_roll_neg_target_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_neg_target_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_neg_target_imu, out);
    out << "\n";
  }

  // member: balance_roll_pos_target_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_pos_target_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_pos_target_imu, out);
    out << "\n";
  }

  // member: balance_roll_elimit_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_elimit_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_elimit_imu, out);
    out << "\n";
  }

  // member: balance_roll_olimit_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_olimit_imu: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_olimit_imu, out);
    out << "\n";
  }

  // member: balance_pitch_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_pitch_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_pitch_flag, out);
    out << "\n";
  }

  // member: balance_ankle_pitch_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_ankle_pitch_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_ankle_pitch_flag, out);
    out << "\n";
  }

  // member: balance_roll_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balance_roll_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.balance_roll_flag, out);
    out << "\n";
  }

  // member: ratio_check_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ratio_check_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.ratio_check_flag, out);
    out << "\n";
  }

  // member: center2leg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center2leg: ";
    rosidl_generator_traits::value_to_yaml(msg.center2leg, out);
    out << "\n";
  }

  // member: link2link
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "link2link: ";
    rosidl_generator_traits::value_to_yaml(msg.link2link, out);
    out << "\n";
  }

  // member: init_z_up
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "init_z_up: ";
    rosidl_generator_traits::value_to_yaml(msg.init_z_up, out);
    out << "\n";
  }

  // member: percentage_of_ik_motor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.percentage_of_ik_motor.size() == 0) {
      out << "percentage_of_ik_motor: []\n";
    } else {
      out << "percentage_of_ik_motor:\n";
      for (auto item : msg.percentage_of_ik_motor) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: first_pos_xr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_pos_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_xr, out);
    out << "\n";
  }

  // member: first_neg_xr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_neg_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_xr, out);
    out << "\n";
  }

  // member: first_pos_side_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_pos_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_side_r, out);
    out << "\n";
  }

  // member: first_neg_side_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_neg_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_side_r, out);
    out << "\n";
  }

  // member: first_pos_yaw_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_pos_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_yaw_r, out);
    out << "\n";
  }

  // member: first_neg_yaw_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_neg_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_yaw_r, out);
    out << "\n";
  }

  // member: first_pos_xl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_pos_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_xl, out);
    out << "\n";
  }

  // member: first_neg_xl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_neg_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_xl, out);
    out << "\n";
  }

  // member: first_pos_side_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_pos_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_side_l, out);
    out << "\n";
  }

  // member: first_neg_side_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_neg_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_side_l, out);
    out << "\n";
  }

  // member: first_pos_yaw_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_pos_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_yaw_l, out);
    out << "\n";
  }

  // member: first_neg_yaw_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_neg_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_yaw_l, out);
    out << "\n";
  }

  // member: first_pos_side_r_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_pos_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_side_r_swing_minus, out);
    out << "\n";
  }

  // member: first_neg_side_r_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_neg_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_side_r_swing_minus, out);
    out << "\n";
  }

  // member: first_pos_side_l_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_pos_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.first_pos_side_l_swing_minus, out);
    out << "\n";
  }

  // member: first_neg_side_l_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_neg_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.first_neg_side_l_swing_minus, out);
    out << "\n";
  }

  // member: first_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_min: ";
    rosidl_generator_traits::value_to_yaml(msg.first_min, out);
    out << "\n";
  }

  // member: first_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_max: ";
    rosidl_generator_traits::value_to_yaml(msg.first_max, out);
    out << "\n";
  }

  // member: second_pos_xr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_pos_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_xr, out);
    out << "\n";
  }

  // member: second_neg_xr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_neg_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_xr, out);
    out << "\n";
  }

  // member: second_pos_side_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_pos_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_side_r, out);
    out << "\n";
  }

  // member: second_neg_side_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_neg_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_side_r, out);
    out << "\n";
  }

  // member: second_pos_yaw_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_pos_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_yaw_r, out);
    out << "\n";
  }

  // member: second_neg_yaw_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_neg_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_yaw_r, out);
    out << "\n";
  }

  // member: second_pos_xl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_pos_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_xl, out);
    out << "\n";
  }

  // member: second_neg_xl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_neg_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_xl, out);
    out << "\n";
  }

  // member: second_pos_side_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_pos_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_side_l, out);
    out << "\n";
  }

  // member: second_neg_side_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_neg_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_side_l, out);
    out << "\n";
  }

  // member: second_pos_yaw_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_pos_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_yaw_l, out);
    out << "\n";
  }

  // member: second_neg_yaw_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_neg_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_yaw_l, out);
    out << "\n";
  }

  // member: second_pos_side_r_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_pos_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_side_r_swing_minus, out);
    out << "\n";
  }

  // member: second_neg_side_r_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_neg_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_side_r_swing_minus, out);
    out << "\n";
  }

  // member: second_pos_side_l_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_pos_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.second_pos_side_l_swing_minus, out);
    out << "\n";
  }

  // member: second_neg_side_l_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_neg_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.second_neg_side_l_swing_minus, out);
    out << "\n";
  }

  // member: second_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_min: ";
    rosidl_generator_traits::value_to_yaml(msg.second_min, out);
    out << "\n";
  }

  // member: second_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second_max: ";
    rosidl_generator_traits::value_to_yaml(msg.second_max, out);
    out << "\n";
  }

  // member: third_pos_xr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_pos_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_xr, out);
    out << "\n";
  }

  // member: third_neg_xr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_neg_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_xr, out);
    out << "\n";
  }

  // member: third_pos_side_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_pos_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_side_r, out);
    out << "\n";
  }

  // member: third_neg_side_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_neg_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_side_r, out);
    out << "\n";
  }

  // member: third_pos_yaw_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_pos_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_yaw_r, out);
    out << "\n";
  }

  // member: third_neg_yaw_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_neg_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_yaw_r, out);
    out << "\n";
  }

  // member: third_pos_xl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_pos_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_xl, out);
    out << "\n";
  }

  // member: third_neg_xl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_neg_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_xl, out);
    out << "\n";
  }

  // member: third_pos_side_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_pos_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_side_l, out);
    out << "\n";
  }

  // member: third_neg_side_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_neg_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_side_l, out);
    out << "\n";
  }

  // member: third_pos_yaw_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_pos_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_yaw_l, out);
    out << "\n";
  }

  // member: third_neg_yaw_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_neg_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_yaw_l, out);
    out << "\n";
  }

  // member: third_pos_side_r_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_pos_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_side_r_swing_minus, out);
    out << "\n";
  }

  // member: third_neg_side_r_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_neg_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_side_r_swing_minus, out);
    out << "\n";
  }

  // member: third_pos_side_l_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_pos_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.third_pos_side_l_swing_minus, out);
    out << "\n";
  }

  // member: third_neg_side_l_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_neg_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.third_neg_side_l_swing_minus, out);
    out << "\n";
  }

  // member: third_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_min: ";
    rosidl_generator_traits::value_to_yaml(msg.third_min, out);
    out << "\n";
  }

  // member: third_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "third_max: ";
    rosidl_generator_traits::value_to_yaml(msg.third_max, out);
    out << "\n";
  }

  // member: fourth_pos_xr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_pos_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_xr, out);
    out << "\n";
  }

  // member: fourth_neg_xr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_neg_xr: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_xr, out);
    out << "\n";
  }

  // member: fourth_pos_side_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_pos_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_side_r, out);
    out << "\n";
  }

  // member: fourth_neg_side_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_neg_side_r: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_side_r, out);
    out << "\n";
  }

  // member: fourth_pos_yaw_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_pos_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_yaw_r, out);
    out << "\n";
  }

  // member: fourth_neg_yaw_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_neg_yaw_r: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_yaw_r, out);
    out << "\n";
  }

  // member: fourth_pos_xl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_pos_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_xl, out);
    out << "\n";
  }

  // member: fourth_neg_xl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_neg_xl: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_xl, out);
    out << "\n";
  }

  // member: fourth_pos_side_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_pos_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_side_l, out);
    out << "\n";
  }

  // member: fourth_neg_side_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_neg_side_l: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_side_l, out);
    out << "\n";
  }

  // member: fourth_pos_yaw_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_pos_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_yaw_l, out);
    out << "\n";
  }

  // member: fourth_neg_yaw_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_neg_yaw_l: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_yaw_l, out);
    out << "\n";
  }

  // member: fourth_pos_side_r_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_pos_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_side_r_swing_minus, out);
    out << "\n";
  }

  // member: fourth_neg_side_r_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_neg_side_r_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_side_r_swing_minus, out);
    out << "\n";
  }

  // member: fourth_pos_side_l_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_pos_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_pos_side_l_swing_minus, out);
    out << "\n";
  }

  // member: fourth_neg_side_l_swing_minus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_neg_side_l_swing_minus: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_neg_side_l_swing_minus, out);
    out << "\n";
  }

  // member: fourth_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_min: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_min, out);
    out << "\n";
  }

  // member: fourth_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fourth_max: ";
    rosidl_generator_traits::value_to_yaml(msg.fourth_max, out);
    out << "\n";
  }

  // member: landing_time_control_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "landing_time_control_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.landing_time_control_flag, out);
    out << "\n";
  }

  // member: control_system_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_system_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.control_system_flag, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Tune2IkMsg & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace humanoid_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use humanoid_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const humanoid_interfaces::msg::Tune2IkMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::Tune2IkMsg & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::Tune2IkMsg>()
{
  return "humanoid_interfaces::msg::Tune2IkMsg";
}

template<>
inline const char * name<humanoid_interfaces::msg::Tune2IkMsg>()
{
  return "humanoid_interfaces/msg/Tune2IkMsg";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::Tune2IkMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::Tune2IkMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<humanoid_interfaces::msg::Tune2IkMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__TUNE2_IK_MSG__TRAITS_HPP_
