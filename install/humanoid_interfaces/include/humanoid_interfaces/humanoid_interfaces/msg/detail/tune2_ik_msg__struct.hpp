// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/Tune2IkMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__TUNE2_IK_MSG__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__TUNE2_IK_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__Tune2IkMsg __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__Tune2IkMsg __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Tune2IkMsg_
{
  using Type = Tune2IkMsg_<ContainerAllocator>;

  explicit Tune2IkMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ik_flag = false;
      this->entire_time = 0.0;
      this->frequency = 0.0;
      this->default_x_left = 0.0;
      this->default_y_left = 0.0;
      this->default_z_left = 0.0;
      this->default_x_right = 0.0;
      this->default_y_right = 0.0;
      this->default_z_right = 0.0;
      this->swing_shoulder_left = 0.0;
      this->swing_shoulder_right = 0.0;
      this->swing_leg_left = 0.0;
      this->swing_leg_right = 0.0;
      this->rise_leg_left = 0.0;
      this->rise_leg_right = 0.0;
      this->start_entire_time = 0.0;
      this->start_swing = 0.0;
      this->start_rise = 0.0;
      this->end_entire_time = 0.0;
      this->end_swing = 0.0;
      this->end_rise = 0.0;
      this->test_x = 0.0;
      this->test_side = 0.0;
      this->test_yaw = 0.0;
      this->tuning_x = 0.0;
      this->tuning_side = 0.0;
      this->tuning_yaw = 0.0;
      this->balance_value_0 = 0.0;
      this->balance_value_1 = 0.0;
      this->balance_value_2 = 0.0;
      this->balance_value_3 = 0.0;
      this->balance_pitch_gp = 0.0;
      this->balance_pitch_gi = 0.0;
      this->balance_pitch_gd = 0.0;
      this->balance_pitch_elimit = 0.0;
      this->balance_pitch_olimit = 0.0;
      this->balance_pitch_neg_target = 0.0;
      this->balance_pitch_pos_target = 0.0;
      this->balance_roll_gp = 0.0;
      this->balance_roll_gi = 0.0;
      this->balance_roll_gd = 0.0;
      this->balance_roll_elimit = 0.0;
      this->balance_roll_olimit = 0.0;
      this->balance_roll_neg_target = 0.0;
      this->balance_roll_pos_target = 0.0;
      this->balance_angle_pitch_gp = 0.0;
      this->balance_angle_pitch_gi = 0.0;
      this->balance_angle_pitch_gd = 0.0;
      this->balance_angle_pitch_elimit = 0.0;
      this->balance_angle_pitch_olimit = 0.0;
      this->balance_angle_pitch_neg_target = 0.0;
      this->balance_angle_pitch_pos_target = 0.0;
      this->balance_value_4 = 0.0;
      this->balance_value_5 = 0.0;
      this->balance_pitch_flag_imu = 0ll;
      this->balance_pitch_gp_imu = 0.0;
      this->balance_pitch_gi_imu = 0.0;
      this->balance_pitch_gd_imu = 0.0;
      this->balance_pitch_neg_target_imu = 0.0;
      this->balance_pitch_pos_target_imu = 0.0;
      this->balance_pitch_elimit_imu = 0.0;
      this->balance_pitch_olimit_imu = 0.0;
      this->balance_roll_flag_imu = 0ll;
      this->balance_roll_gp_imu = 0.0;
      this->balance_roll_gi_imu = 0.0;
      this->balance_roll_gd_imu = 0.0;
      this->balance_roll_neg_target_imu = 0.0;
      this->balance_roll_pos_target_imu = 0.0;
      this->balance_roll_elimit_imu = 0.0;
      this->balance_roll_olimit_imu = 0.0;
      this->balance_pitch_flag = 0ll;
      this->balance_ankle_pitch_flag = 0ll;
      this->balance_roll_flag = 0ll;
      this->ratio_check_flag = 0ll;
      this->center2leg = 0.0;
      this->link2link = 0.0;
      this->init_z_up = 0.0;
      this->first_pos_xr = 0.0;
      this->first_neg_xr = 0.0;
      this->first_pos_side_r = 0.0;
      this->first_neg_side_r = 0.0;
      this->first_pos_yaw_r = 0.0;
      this->first_neg_yaw_r = 0.0;
      this->first_pos_xl = 0.0;
      this->first_neg_xl = 0.0;
      this->first_pos_side_l = 0.0;
      this->first_neg_side_l = 0.0;
      this->first_pos_yaw_l = 0.0;
      this->first_neg_yaw_l = 0.0;
      this->first_pos_side_r_swing_minus = 0.0;
      this->first_neg_side_r_swing_minus = 0.0;
      this->first_pos_side_l_swing_minus = 0.0;
      this->first_neg_side_l_swing_minus = 0.0;
      this->first_min = 0ll;
      this->first_max = 0ll;
      this->second_pos_xr = 0.0;
      this->second_neg_xr = 0.0;
      this->second_pos_side_r = 0.0;
      this->second_neg_side_r = 0.0;
      this->second_pos_yaw_r = 0.0;
      this->second_neg_yaw_r = 0.0;
      this->second_pos_xl = 0.0;
      this->second_neg_xl = 0.0;
      this->second_pos_side_l = 0.0;
      this->second_neg_side_l = 0.0;
      this->second_pos_yaw_l = 0.0;
      this->second_neg_yaw_l = 0.0;
      this->second_pos_side_r_swing_minus = 0.0;
      this->second_neg_side_r_swing_minus = 0.0;
      this->second_pos_side_l_swing_minus = 0.0;
      this->second_neg_side_l_swing_minus = 0.0;
      this->second_min = 0ll;
      this->second_max = 0ll;
      this->third_pos_xr = 0.0;
      this->third_neg_xr = 0.0;
      this->third_pos_side_r = 0.0;
      this->third_neg_side_r = 0.0;
      this->third_pos_yaw_r = 0.0;
      this->third_neg_yaw_r = 0.0;
      this->third_pos_xl = 0.0;
      this->third_neg_xl = 0.0;
      this->third_pos_side_l = 0.0;
      this->third_neg_side_l = 0.0;
      this->third_pos_yaw_l = 0.0;
      this->third_neg_yaw_l = 0.0;
      this->third_pos_side_r_swing_minus = 0.0;
      this->third_neg_side_r_swing_minus = 0.0;
      this->third_pos_side_l_swing_minus = 0.0;
      this->third_neg_side_l_swing_minus = 0.0;
      this->third_min = 0ll;
      this->third_max = 0ll;
      this->fourth_pos_xr = 0.0;
      this->fourth_neg_xr = 0.0;
      this->fourth_pos_side_r = 0.0;
      this->fourth_neg_side_r = 0.0;
      this->fourth_pos_yaw_r = 0.0;
      this->fourth_neg_yaw_r = 0.0;
      this->fourth_pos_xl = 0.0;
      this->fourth_neg_xl = 0.0;
      this->fourth_pos_side_l = 0.0;
      this->fourth_neg_side_l = 0.0;
      this->fourth_pos_yaw_l = 0.0;
      this->fourth_neg_yaw_l = 0.0;
      this->fourth_pos_side_r_swing_minus = 0.0;
      this->fourth_neg_side_r_swing_minus = 0.0;
      this->fourth_pos_side_l_swing_minus = 0.0;
      this->fourth_neg_side_l_swing_minus = 0.0;
      this->fourth_min = 0ll;
      this->fourth_max = 0ll;
      this->landing_time_control_flag = 0ll;
      this->control_system_flag = 0ll;
    }
  }

  explicit Tune2IkMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ik_flag = false;
      this->entire_time = 0.0;
      this->frequency = 0.0;
      this->default_x_left = 0.0;
      this->default_y_left = 0.0;
      this->default_z_left = 0.0;
      this->default_x_right = 0.0;
      this->default_y_right = 0.0;
      this->default_z_right = 0.0;
      this->swing_shoulder_left = 0.0;
      this->swing_shoulder_right = 0.0;
      this->swing_leg_left = 0.0;
      this->swing_leg_right = 0.0;
      this->rise_leg_left = 0.0;
      this->rise_leg_right = 0.0;
      this->start_entire_time = 0.0;
      this->start_swing = 0.0;
      this->start_rise = 0.0;
      this->end_entire_time = 0.0;
      this->end_swing = 0.0;
      this->end_rise = 0.0;
      this->test_x = 0.0;
      this->test_side = 0.0;
      this->test_yaw = 0.0;
      this->tuning_x = 0.0;
      this->tuning_side = 0.0;
      this->tuning_yaw = 0.0;
      this->balance_value_0 = 0.0;
      this->balance_value_1 = 0.0;
      this->balance_value_2 = 0.0;
      this->balance_value_3 = 0.0;
      this->balance_pitch_gp = 0.0;
      this->balance_pitch_gi = 0.0;
      this->balance_pitch_gd = 0.0;
      this->balance_pitch_elimit = 0.0;
      this->balance_pitch_olimit = 0.0;
      this->balance_pitch_neg_target = 0.0;
      this->balance_pitch_pos_target = 0.0;
      this->balance_roll_gp = 0.0;
      this->balance_roll_gi = 0.0;
      this->balance_roll_gd = 0.0;
      this->balance_roll_elimit = 0.0;
      this->balance_roll_olimit = 0.0;
      this->balance_roll_neg_target = 0.0;
      this->balance_roll_pos_target = 0.0;
      this->balance_angle_pitch_gp = 0.0;
      this->balance_angle_pitch_gi = 0.0;
      this->balance_angle_pitch_gd = 0.0;
      this->balance_angle_pitch_elimit = 0.0;
      this->balance_angle_pitch_olimit = 0.0;
      this->balance_angle_pitch_neg_target = 0.0;
      this->balance_angle_pitch_pos_target = 0.0;
      this->balance_value_4 = 0.0;
      this->balance_value_5 = 0.0;
      this->balance_pitch_flag_imu = 0ll;
      this->balance_pitch_gp_imu = 0.0;
      this->balance_pitch_gi_imu = 0.0;
      this->balance_pitch_gd_imu = 0.0;
      this->balance_pitch_neg_target_imu = 0.0;
      this->balance_pitch_pos_target_imu = 0.0;
      this->balance_pitch_elimit_imu = 0.0;
      this->balance_pitch_olimit_imu = 0.0;
      this->balance_roll_flag_imu = 0ll;
      this->balance_roll_gp_imu = 0.0;
      this->balance_roll_gi_imu = 0.0;
      this->balance_roll_gd_imu = 0.0;
      this->balance_roll_neg_target_imu = 0.0;
      this->balance_roll_pos_target_imu = 0.0;
      this->balance_roll_elimit_imu = 0.0;
      this->balance_roll_olimit_imu = 0.0;
      this->balance_pitch_flag = 0ll;
      this->balance_ankle_pitch_flag = 0ll;
      this->balance_roll_flag = 0ll;
      this->ratio_check_flag = 0ll;
      this->center2leg = 0.0;
      this->link2link = 0.0;
      this->init_z_up = 0.0;
      this->first_pos_xr = 0.0;
      this->first_neg_xr = 0.0;
      this->first_pos_side_r = 0.0;
      this->first_neg_side_r = 0.0;
      this->first_pos_yaw_r = 0.0;
      this->first_neg_yaw_r = 0.0;
      this->first_pos_xl = 0.0;
      this->first_neg_xl = 0.0;
      this->first_pos_side_l = 0.0;
      this->first_neg_side_l = 0.0;
      this->first_pos_yaw_l = 0.0;
      this->first_neg_yaw_l = 0.0;
      this->first_pos_side_r_swing_minus = 0.0;
      this->first_neg_side_r_swing_minus = 0.0;
      this->first_pos_side_l_swing_minus = 0.0;
      this->first_neg_side_l_swing_minus = 0.0;
      this->first_min = 0ll;
      this->first_max = 0ll;
      this->second_pos_xr = 0.0;
      this->second_neg_xr = 0.0;
      this->second_pos_side_r = 0.0;
      this->second_neg_side_r = 0.0;
      this->second_pos_yaw_r = 0.0;
      this->second_neg_yaw_r = 0.0;
      this->second_pos_xl = 0.0;
      this->second_neg_xl = 0.0;
      this->second_pos_side_l = 0.0;
      this->second_neg_side_l = 0.0;
      this->second_pos_yaw_l = 0.0;
      this->second_neg_yaw_l = 0.0;
      this->second_pos_side_r_swing_minus = 0.0;
      this->second_neg_side_r_swing_minus = 0.0;
      this->second_pos_side_l_swing_minus = 0.0;
      this->second_neg_side_l_swing_minus = 0.0;
      this->second_min = 0ll;
      this->second_max = 0ll;
      this->third_pos_xr = 0.0;
      this->third_neg_xr = 0.0;
      this->third_pos_side_r = 0.0;
      this->third_neg_side_r = 0.0;
      this->third_pos_yaw_r = 0.0;
      this->third_neg_yaw_r = 0.0;
      this->third_pos_xl = 0.0;
      this->third_neg_xl = 0.0;
      this->third_pos_side_l = 0.0;
      this->third_neg_side_l = 0.0;
      this->third_pos_yaw_l = 0.0;
      this->third_neg_yaw_l = 0.0;
      this->third_pos_side_r_swing_minus = 0.0;
      this->third_neg_side_r_swing_minus = 0.0;
      this->third_pos_side_l_swing_minus = 0.0;
      this->third_neg_side_l_swing_minus = 0.0;
      this->third_min = 0ll;
      this->third_max = 0ll;
      this->fourth_pos_xr = 0.0;
      this->fourth_neg_xr = 0.0;
      this->fourth_pos_side_r = 0.0;
      this->fourth_neg_side_r = 0.0;
      this->fourth_pos_yaw_r = 0.0;
      this->fourth_neg_yaw_r = 0.0;
      this->fourth_pos_xl = 0.0;
      this->fourth_neg_xl = 0.0;
      this->fourth_pos_side_l = 0.0;
      this->fourth_neg_side_l = 0.0;
      this->fourth_pos_yaw_l = 0.0;
      this->fourth_neg_yaw_l = 0.0;
      this->fourth_pos_side_r_swing_minus = 0.0;
      this->fourth_neg_side_r_swing_minus = 0.0;
      this->fourth_pos_side_l_swing_minus = 0.0;
      this->fourth_neg_side_l_swing_minus = 0.0;
      this->fourth_min = 0ll;
      this->fourth_max = 0ll;
      this->landing_time_control_flag = 0ll;
      this->control_system_flag = 0ll;
    }
  }

  // field types and members
  using _ik_flag_type =
    bool;
  _ik_flag_type ik_flag;
  using _entire_time_type =
    double;
  _entire_time_type entire_time;
  using _frequency_type =
    double;
  _frequency_type frequency;
  using _default_x_left_type =
    double;
  _default_x_left_type default_x_left;
  using _default_y_left_type =
    double;
  _default_y_left_type default_y_left;
  using _default_z_left_type =
    double;
  _default_z_left_type default_z_left;
  using _default_x_right_type =
    double;
  _default_x_right_type default_x_right;
  using _default_y_right_type =
    double;
  _default_y_right_type default_y_right;
  using _default_z_right_type =
    double;
  _default_z_right_type default_z_right;
  using _offset_motor_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _offset_motor_type offset_motor;
  using _swing_shoulder_left_type =
    double;
  _swing_shoulder_left_type swing_shoulder_left;
  using _swing_shoulder_right_type =
    double;
  _swing_shoulder_right_type swing_shoulder_right;
  using _swing_leg_left_type =
    double;
  _swing_leg_left_type swing_leg_left;
  using _swing_leg_right_type =
    double;
  _swing_leg_right_type swing_leg_right;
  using _rise_leg_left_type =
    double;
  _rise_leg_left_type rise_leg_left;
  using _rise_leg_right_type =
    double;
  _rise_leg_right_type rise_leg_right;
  using _start_entire_time_type =
    double;
  _start_entire_time_type start_entire_time;
  using _start_swing_type =
    double;
  _start_swing_type start_swing;
  using _start_rise_type =
    double;
  _start_rise_type start_rise;
  using _end_entire_time_type =
    double;
  _end_entire_time_type end_entire_time;
  using _end_swing_type =
    double;
  _end_swing_type end_swing;
  using _end_rise_type =
    double;
  _end_rise_type end_rise;
  using _test_x_type =
    double;
  _test_x_type test_x;
  using _test_side_type =
    double;
  _test_side_type test_side;
  using _test_yaw_type =
    double;
  _test_yaw_type test_yaw;
  using _tuning_x_type =
    double;
  _tuning_x_type tuning_x;
  using _tuning_side_type =
    double;
  _tuning_side_type tuning_side;
  using _tuning_yaw_type =
    double;
  _tuning_yaw_type tuning_yaw;
  using _balance_value_0_type =
    double;
  _balance_value_0_type balance_value_0;
  using _balance_value_1_type =
    double;
  _balance_value_1_type balance_value_1;
  using _balance_value_2_type =
    double;
  _balance_value_2_type balance_value_2;
  using _balance_value_3_type =
    double;
  _balance_value_3_type balance_value_3;
  using _balance_pitch_gp_type =
    double;
  _balance_pitch_gp_type balance_pitch_gp;
  using _balance_pitch_gi_type =
    double;
  _balance_pitch_gi_type balance_pitch_gi;
  using _balance_pitch_gd_type =
    double;
  _balance_pitch_gd_type balance_pitch_gd;
  using _balance_pitch_elimit_type =
    double;
  _balance_pitch_elimit_type balance_pitch_elimit;
  using _balance_pitch_olimit_type =
    double;
  _balance_pitch_olimit_type balance_pitch_olimit;
  using _balance_pitch_neg_target_type =
    double;
  _balance_pitch_neg_target_type balance_pitch_neg_target;
  using _balance_pitch_pos_target_type =
    double;
  _balance_pitch_pos_target_type balance_pitch_pos_target;
  using _balance_roll_gp_type =
    double;
  _balance_roll_gp_type balance_roll_gp;
  using _balance_roll_gi_type =
    double;
  _balance_roll_gi_type balance_roll_gi;
  using _balance_roll_gd_type =
    double;
  _balance_roll_gd_type balance_roll_gd;
  using _balance_roll_elimit_type =
    double;
  _balance_roll_elimit_type balance_roll_elimit;
  using _balance_roll_olimit_type =
    double;
  _balance_roll_olimit_type balance_roll_olimit;
  using _balance_roll_neg_target_type =
    double;
  _balance_roll_neg_target_type balance_roll_neg_target;
  using _balance_roll_pos_target_type =
    double;
  _balance_roll_pos_target_type balance_roll_pos_target;
  using _balance_angle_pitch_gp_type =
    double;
  _balance_angle_pitch_gp_type balance_angle_pitch_gp;
  using _balance_angle_pitch_gi_type =
    double;
  _balance_angle_pitch_gi_type balance_angle_pitch_gi;
  using _balance_angle_pitch_gd_type =
    double;
  _balance_angle_pitch_gd_type balance_angle_pitch_gd;
  using _balance_angle_pitch_elimit_type =
    double;
  _balance_angle_pitch_elimit_type balance_angle_pitch_elimit;
  using _balance_angle_pitch_olimit_type =
    double;
  _balance_angle_pitch_olimit_type balance_angle_pitch_olimit;
  using _balance_angle_pitch_neg_target_type =
    double;
  _balance_angle_pitch_neg_target_type balance_angle_pitch_neg_target;
  using _balance_angle_pitch_pos_target_type =
    double;
  _balance_angle_pitch_pos_target_type balance_angle_pitch_pos_target;
  using _balance_value_4_type =
    double;
  _balance_value_4_type balance_value_4;
  using _balance_value_5_type =
    double;
  _balance_value_5_type balance_value_5;
  using _balance_pitch_flag_imu_type =
    int64_t;
  _balance_pitch_flag_imu_type balance_pitch_flag_imu;
  using _balance_pitch_gp_imu_type =
    double;
  _balance_pitch_gp_imu_type balance_pitch_gp_imu;
  using _balance_pitch_gi_imu_type =
    double;
  _balance_pitch_gi_imu_type balance_pitch_gi_imu;
  using _balance_pitch_gd_imu_type =
    double;
  _balance_pitch_gd_imu_type balance_pitch_gd_imu;
  using _balance_pitch_neg_target_imu_type =
    double;
  _balance_pitch_neg_target_imu_type balance_pitch_neg_target_imu;
  using _balance_pitch_pos_target_imu_type =
    double;
  _balance_pitch_pos_target_imu_type balance_pitch_pos_target_imu;
  using _balance_pitch_elimit_imu_type =
    double;
  _balance_pitch_elimit_imu_type balance_pitch_elimit_imu;
  using _balance_pitch_olimit_imu_type =
    double;
  _balance_pitch_olimit_imu_type balance_pitch_olimit_imu;
  using _balance_roll_flag_imu_type =
    int64_t;
  _balance_roll_flag_imu_type balance_roll_flag_imu;
  using _balance_roll_gp_imu_type =
    double;
  _balance_roll_gp_imu_type balance_roll_gp_imu;
  using _balance_roll_gi_imu_type =
    double;
  _balance_roll_gi_imu_type balance_roll_gi_imu;
  using _balance_roll_gd_imu_type =
    double;
  _balance_roll_gd_imu_type balance_roll_gd_imu;
  using _balance_roll_neg_target_imu_type =
    double;
  _balance_roll_neg_target_imu_type balance_roll_neg_target_imu;
  using _balance_roll_pos_target_imu_type =
    double;
  _balance_roll_pos_target_imu_type balance_roll_pos_target_imu;
  using _balance_roll_elimit_imu_type =
    double;
  _balance_roll_elimit_imu_type balance_roll_elimit_imu;
  using _balance_roll_olimit_imu_type =
    double;
  _balance_roll_olimit_imu_type balance_roll_olimit_imu;
  using _balance_pitch_flag_type =
    int64_t;
  _balance_pitch_flag_type balance_pitch_flag;
  using _balance_ankle_pitch_flag_type =
    int64_t;
  _balance_ankle_pitch_flag_type balance_ankle_pitch_flag;
  using _balance_roll_flag_type =
    int64_t;
  _balance_roll_flag_type balance_roll_flag;
  using _ratio_check_flag_type =
    int64_t;
  _ratio_check_flag_type ratio_check_flag;
  using _center2leg_type =
    double;
  _center2leg_type center2leg;
  using _link2link_type =
    double;
  _link2link_type link2link;
  using _init_z_up_type =
    double;
  _init_z_up_type init_z_up;
  using _percentage_of_ik_motor_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _percentage_of_ik_motor_type percentage_of_ik_motor;
  using _first_pos_xr_type =
    double;
  _first_pos_xr_type first_pos_xr;
  using _first_neg_xr_type =
    double;
  _first_neg_xr_type first_neg_xr;
  using _first_pos_side_r_type =
    double;
  _first_pos_side_r_type first_pos_side_r;
  using _first_neg_side_r_type =
    double;
  _first_neg_side_r_type first_neg_side_r;
  using _first_pos_yaw_r_type =
    double;
  _first_pos_yaw_r_type first_pos_yaw_r;
  using _first_neg_yaw_r_type =
    double;
  _first_neg_yaw_r_type first_neg_yaw_r;
  using _first_pos_xl_type =
    double;
  _first_pos_xl_type first_pos_xl;
  using _first_neg_xl_type =
    double;
  _first_neg_xl_type first_neg_xl;
  using _first_pos_side_l_type =
    double;
  _first_pos_side_l_type first_pos_side_l;
  using _first_neg_side_l_type =
    double;
  _first_neg_side_l_type first_neg_side_l;
  using _first_pos_yaw_l_type =
    double;
  _first_pos_yaw_l_type first_pos_yaw_l;
  using _first_neg_yaw_l_type =
    double;
  _first_neg_yaw_l_type first_neg_yaw_l;
  using _first_pos_side_r_swing_minus_type =
    double;
  _first_pos_side_r_swing_minus_type first_pos_side_r_swing_minus;
  using _first_neg_side_r_swing_minus_type =
    double;
  _first_neg_side_r_swing_minus_type first_neg_side_r_swing_minus;
  using _first_pos_side_l_swing_minus_type =
    double;
  _first_pos_side_l_swing_minus_type first_pos_side_l_swing_minus;
  using _first_neg_side_l_swing_minus_type =
    double;
  _first_neg_side_l_swing_minus_type first_neg_side_l_swing_minus;
  using _first_min_type =
    int64_t;
  _first_min_type first_min;
  using _first_max_type =
    int64_t;
  _first_max_type first_max;
  using _second_pos_xr_type =
    double;
  _second_pos_xr_type second_pos_xr;
  using _second_neg_xr_type =
    double;
  _second_neg_xr_type second_neg_xr;
  using _second_pos_side_r_type =
    double;
  _second_pos_side_r_type second_pos_side_r;
  using _second_neg_side_r_type =
    double;
  _second_neg_side_r_type second_neg_side_r;
  using _second_pos_yaw_r_type =
    double;
  _second_pos_yaw_r_type second_pos_yaw_r;
  using _second_neg_yaw_r_type =
    double;
  _second_neg_yaw_r_type second_neg_yaw_r;
  using _second_pos_xl_type =
    double;
  _second_pos_xl_type second_pos_xl;
  using _second_neg_xl_type =
    double;
  _second_neg_xl_type second_neg_xl;
  using _second_pos_side_l_type =
    double;
  _second_pos_side_l_type second_pos_side_l;
  using _second_neg_side_l_type =
    double;
  _second_neg_side_l_type second_neg_side_l;
  using _second_pos_yaw_l_type =
    double;
  _second_pos_yaw_l_type second_pos_yaw_l;
  using _second_neg_yaw_l_type =
    double;
  _second_neg_yaw_l_type second_neg_yaw_l;
  using _second_pos_side_r_swing_minus_type =
    double;
  _second_pos_side_r_swing_minus_type second_pos_side_r_swing_minus;
  using _second_neg_side_r_swing_minus_type =
    double;
  _second_neg_side_r_swing_minus_type second_neg_side_r_swing_minus;
  using _second_pos_side_l_swing_minus_type =
    double;
  _second_pos_side_l_swing_minus_type second_pos_side_l_swing_minus;
  using _second_neg_side_l_swing_minus_type =
    double;
  _second_neg_side_l_swing_minus_type second_neg_side_l_swing_minus;
  using _second_min_type =
    int64_t;
  _second_min_type second_min;
  using _second_max_type =
    int64_t;
  _second_max_type second_max;
  using _third_pos_xr_type =
    double;
  _third_pos_xr_type third_pos_xr;
  using _third_neg_xr_type =
    double;
  _third_neg_xr_type third_neg_xr;
  using _third_pos_side_r_type =
    double;
  _third_pos_side_r_type third_pos_side_r;
  using _third_neg_side_r_type =
    double;
  _third_neg_side_r_type third_neg_side_r;
  using _third_pos_yaw_r_type =
    double;
  _third_pos_yaw_r_type third_pos_yaw_r;
  using _third_neg_yaw_r_type =
    double;
  _third_neg_yaw_r_type third_neg_yaw_r;
  using _third_pos_xl_type =
    double;
  _third_pos_xl_type third_pos_xl;
  using _third_neg_xl_type =
    double;
  _third_neg_xl_type third_neg_xl;
  using _third_pos_side_l_type =
    double;
  _third_pos_side_l_type third_pos_side_l;
  using _third_neg_side_l_type =
    double;
  _third_neg_side_l_type third_neg_side_l;
  using _third_pos_yaw_l_type =
    double;
  _third_pos_yaw_l_type third_pos_yaw_l;
  using _third_neg_yaw_l_type =
    double;
  _third_neg_yaw_l_type third_neg_yaw_l;
  using _third_pos_side_r_swing_minus_type =
    double;
  _third_pos_side_r_swing_minus_type third_pos_side_r_swing_minus;
  using _third_neg_side_r_swing_minus_type =
    double;
  _third_neg_side_r_swing_minus_type third_neg_side_r_swing_minus;
  using _third_pos_side_l_swing_minus_type =
    double;
  _third_pos_side_l_swing_minus_type third_pos_side_l_swing_minus;
  using _third_neg_side_l_swing_minus_type =
    double;
  _third_neg_side_l_swing_minus_type third_neg_side_l_swing_minus;
  using _third_min_type =
    int64_t;
  _third_min_type third_min;
  using _third_max_type =
    int64_t;
  _third_max_type third_max;
  using _fourth_pos_xr_type =
    double;
  _fourth_pos_xr_type fourth_pos_xr;
  using _fourth_neg_xr_type =
    double;
  _fourth_neg_xr_type fourth_neg_xr;
  using _fourth_pos_side_r_type =
    double;
  _fourth_pos_side_r_type fourth_pos_side_r;
  using _fourth_neg_side_r_type =
    double;
  _fourth_neg_side_r_type fourth_neg_side_r;
  using _fourth_pos_yaw_r_type =
    double;
  _fourth_pos_yaw_r_type fourth_pos_yaw_r;
  using _fourth_neg_yaw_r_type =
    double;
  _fourth_neg_yaw_r_type fourth_neg_yaw_r;
  using _fourth_pos_xl_type =
    double;
  _fourth_pos_xl_type fourth_pos_xl;
  using _fourth_neg_xl_type =
    double;
  _fourth_neg_xl_type fourth_neg_xl;
  using _fourth_pos_side_l_type =
    double;
  _fourth_pos_side_l_type fourth_pos_side_l;
  using _fourth_neg_side_l_type =
    double;
  _fourth_neg_side_l_type fourth_neg_side_l;
  using _fourth_pos_yaw_l_type =
    double;
  _fourth_pos_yaw_l_type fourth_pos_yaw_l;
  using _fourth_neg_yaw_l_type =
    double;
  _fourth_neg_yaw_l_type fourth_neg_yaw_l;
  using _fourth_pos_side_r_swing_minus_type =
    double;
  _fourth_pos_side_r_swing_minus_type fourth_pos_side_r_swing_minus;
  using _fourth_neg_side_r_swing_minus_type =
    double;
  _fourth_neg_side_r_swing_minus_type fourth_neg_side_r_swing_minus;
  using _fourth_pos_side_l_swing_minus_type =
    double;
  _fourth_pos_side_l_swing_minus_type fourth_pos_side_l_swing_minus;
  using _fourth_neg_side_l_swing_minus_type =
    double;
  _fourth_neg_side_l_swing_minus_type fourth_neg_side_l_swing_minus;
  using _fourth_min_type =
    int64_t;
  _fourth_min_type fourth_min;
  using _fourth_max_type =
    int64_t;
  _fourth_max_type fourth_max;
  using _landing_time_control_flag_type =
    int64_t;
  _landing_time_control_flag_type landing_time_control_flag;
  using _control_system_flag_type =
    int64_t;
  _control_system_flag_type control_system_flag;

  // setters for named parameter idiom
  Type & set__ik_flag(
    const bool & _arg)
  {
    this->ik_flag = _arg;
    return *this;
  }
  Type & set__entire_time(
    const double & _arg)
  {
    this->entire_time = _arg;
    return *this;
  }
  Type & set__frequency(
    const double & _arg)
  {
    this->frequency = _arg;
    return *this;
  }
  Type & set__default_x_left(
    const double & _arg)
  {
    this->default_x_left = _arg;
    return *this;
  }
  Type & set__default_y_left(
    const double & _arg)
  {
    this->default_y_left = _arg;
    return *this;
  }
  Type & set__default_z_left(
    const double & _arg)
  {
    this->default_z_left = _arg;
    return *this;
  }
  Type & set__default_x_right(
    const double & _arg)
  {
    this->default_x_right = _arg;
    return *this;
  }
  Type & set__default_y_right(
    const double & _arg)
  {
    this->default_y_right = _arg;
    return *this;
  }
  Type & set__default_z_right(
    const double & _arg)
  {
    this->default_z_right = _arg;
    return *this;
  }
  Type & set__offset_motor(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->offset_motor = _arg;
    return *this;
  }
  Type & set__swing_shoulder_left(
    const double & _arg)
  {
    this->swing_shoulder_left = _arg;
    return *this;
  }
  Type & set__swing_shoulder_right(
    const double & _arg)
  {
    this->swing_shoulder_right = _arg;
    return *this;
  }
  Type & set__swing_leg_left(
    const double & _arg)
  {
    this->swing_leg_left = _arg;
    return *this;
  }
  Type & set__swing_leg_right(
    const double & _arg)
  {
    this->swing_leg_right = _arg;
    return *this;
  }
  Type & set__rise_leg_left(
    const double & _arg)
  {
    this->rise_leg_left = _arg;
    return *this;
  }
  Type & set__rise_leg_right(
    const double & _arg)
  {
    this->rise_leg_right = _arg;
    return *this;
  }
  Type & set__start_entire_time(
    const double & _arg)
  {
    this->start_entire_time = _arg;
    return *this;
  }
  Type & set__start_swing(
    const double & _arg)
  {
    this->start_swing = _arg;
    return *this;
  }
  Type & set__start_rise(
    const double & _arg)
  {
    this->start_rise = _arg;
    return *this;
  }
  Type & set__end_entire_time(
    const double & _arg)
  {
    this->end_entire_time = _arg;
    return *this;
  }
  Type & set__end_swing(
    const double & _arg)
  {
    this->end_swing = _arg;
    return *this;
  }
  Type & set__end_rise(
    const double & _arg)
  {
    this->end_rise = _arg;
    return *this;
  }
  Type & set__test_x(
    const double & _arg)
  {
    this->test_x = _arg;
    return *this;
  }
  Type & set__test_side(
    const double & _arg)
  {
    this->test_side = _arg;
    return *this;
  }
  Type & set__test_yaw(
    const double & _arg)
  {
    this->test_yaw = _arg;
    return *this;
  }
  Type & set__tuning_x(
    const double & _arg)
  {
    this->tuning_x = _arg;
    return *this;
  }
  Type & set__tuning_side(
    const double & _arg)
  {
    this->tuning_side = _arg;
    return *this;
  }
  Type & set__tuning_yaw(
    const double & _arg)
  {
    this->tuning_yaw = _arg;
    return *this;
  }
  Type & set__balance_value_0(
    const double & _arg)
  {
    this->balance_value_0 = _arg;
    return *this;
  }
  Type & set__balance_value_1(
    const double & _arg)
  {
    this->balance_value_1 = _arg;
    return *this;
  }
  Type & set__balance_value_2(
    const double & _arg)
  {
    this->balance_value_2 = _arg;
    return *this;
  }
  Type & set__balance_value_3(
    const double & _arg)
  {
    this->balance_value_3 = _arg;
    return *this;
  }
  Type & set__balance_pitch_gp(
    const double & _arg)
  {
    this->balance_pitch_gp = _arg;
    return *this;
  }
  Type & set__balance_pitch_gi(
    const double & _arg)
  {
    this->balance_pitch_gi = _arg;
    return *this;
  }
  Type & set__balance_pitch_gd(
    const double & _arg)
  {
    this->balance_pitch_gd = _arg;
    return *this;
  }
  Type & set__balance_pitch_elimit(
    const double & _arg)
  {
    this->balance_pitch_elimit = _arg;
    return *this;
  }
  Type & set__balance_pitch_olimit(
    const double & _arg)
  {
    this->balance_pitch_olimit = _arg;
    return *this;
  }
  Type & set__balance_pitch_neg_target(
    const double & _arg)
  {
    this->balance_pitch_neg_target = _arg;
    return *this;
  }
  Type & set__balance_pitch_pos_target(
    const double & _arg)
  {
    this->balance_pitch_pos_target = _arg;
    return *this;
  }
  Type & set__balance_roll_gp(
    const double & _arg)
  {
    this->balance_roll_gp = _arg;
    return *this;
  }
  Type & set__balance_roll_gi(
    const double & _arg)
  {
    this->balance_roll_gi = _arg;
    return *this;
  }
  Type & set__balance_roll_gd(
    const double & _arg)
  {
    this->balance_roll_gd = _arg;
    return *this;
  }
  Type & set__balance_roll_elimit(
    const double & _arg)
  {
    this->balance_roll_elimit = _arg;
    return *this;
  }
  Type & set__balance_roll_olimit(
    const double & _arg)
  {
    this->balance_roll_olimit = _arg;
    return *this;
  }
  Type & set__balance_roll_neg_target(
    const double & _arg)
  {
    this->balance_roll_neg_target = _arg;
    return *this;
  }
  Type & set__balance_roll_pos_target(
    const double & _arg)
  {
    this->balance_roll_pos_target = _arg;
    return *this;
  }
  Type & set__balance_angle_pitch_gp(
    const double & _arg)
  {
    this->balance_angle_pitch_gp = _arg;
    return *this;
  }
  Type & set__balance_angle_pitch_gi(
    const double & _arg)
  {
    this->balance_angle_pitch_gi = _arg;
    return *this;
  }
  Type & set__balance_angle_pitch_gd(
    const double & _arg)
  {
    this->balance_angle_pitch_gd = _arg;
    return *this;
  }
  Type & set__balance_angle_pitch_elimit(
    const double & _arg)
  {
    this->balance_angle_pitch_elimit = _arg;
    return *this;
  }
  Type & set__balance_angle_pitch_olimit(
    const double & _arg)
  {
    this->balance_angle_pitch_olimit = _arg;
    return *this;
  }
  Type & set__balance_angle_pitch_neg_target(
    const double & _arg)
  {
    this->balance_angle_pitch_neg_target = _arg;
    return *this;
  }
  Type & set__balance_angle_pitch_pos_target(
    const double & _arg)
  {
    this->balance_angle_pitch_pos_target = _arg;
    return *this;
  }
  Type & set__balance_value_4(
    const double & _arg)
  {
    this->balance_value_4 = _arg;
    return *this;
  }
  Type & set__balance_value_5(
    const double & _arg)
  {
    this->balance_value_5 = _arg;
    return *this;
  }
  Type & set__balance_pitch_flag_imu(
    const int64_t & _arg)
  {
    this->balance_pitch_flag_imu = _arg;
    return *this;
  }
  Type & set__balance_pitch_gp_imu(
    const double & _arg)
  {
    this->balance_pitch_gp_imu = _arg;
    return *this;
  }
  Type & set__balance_pitch_gi_imu(
    const double & _arg)
  {
    this->balance_pitch_gi_imu = _arg;
    return *this;
  }
  Type & set__balance_pitch_gd_imu(
    const double & _arg)
  {
    this->balance_pitch_gd_imu = _arg;
    return *this;
  }
  Type & set__balance_pitch_neg_target_imu(
    const double & _arg)
  {
    this->balance_pitch_neg_target_imu = _arg;
    return *this;
  }
  Type & set__balance_pitch_pos_target_imu(
    const double & _arg)
  {
    this->balance_pitch_pos_target_imu = _arg;
    return *this;
  }
  Type & set__balance_pitch_elimit_imu(
    const double & _arg)
  {
    this->balance_pitch_elimit_imu = _arg;
    return *this;
  }
  Type & set__balance_pitch_olimit_imu(
    const double & _arg)
  {
    this->balance_pitch_olimit_imu = _arg;
    return *this;
  }
  Type & set__balance_roll_flag_imu(
    const int64_t & _arg)
  {
    this->balance_roll_flag_imu = _arg;
    return *this;
  }
  Type & set__balance_roll_gp_imu(
    const double & _arg)
  {
    this->balance_roll_gp_imu = _arg;
    return *this;
  }
  Type & set__balance_roll_gi_imu(
    const double & _arg)
  {
    this->balance_roll_gi_imu = _arg;
    return *this;
  }
  Type & set__balance_roll_gd_imu(
    const double & _arg)
  {
    this->balance_roll_gd_imu = _arg;
    return *this;
  }
  Type & set__balance_roll_neg_target_imu(
    const double & _arg)
  {
    this->balance_roll_neg_target_imu = _arg;
    return *this;
  }
  Type & set__balance_roll_pos_target_imu(
    const double & _arg)
  {
    this->balance_roll_pos_target_imu = _arg;
    return *this;
  }
  Type & set__balance_roll_elimit_imu(
    const double & _arg)
  {
    this->balance_roll_elimit_imu = _arg;
    return *this;
  }
  Type & set__balance_roll_olimit_imu(
    const double & _arg)
  {
    this->balance_roll_olimit_imu = _arg;
    return *this;
  }
  Type & set__balance_pitch_flag(
    const int64_t & _arg)
  {
    this->balance_pitch_flag = _arg;
    return *this;
  }
  Type & set__balance_ankle_pitch_flag(
    const int64_t & _arg)
  {
    this->balance_ankle_pitch_flag = _arg;
    return *this;
  }
  Type & set__balance_roll_flag(
    const int64_t & _arg)
  {
    this->balance_roll_flag = _arg;
    return *this;
  }
  Type & set__ratio_check_flag(
    const int64_t & _arg)
  {
    this->ratio_check_flag = _arg;
    return *this;
  }
  Type & set__center2leg(
    const double & _arg)
  {
    this->center2leg = _arg;
    return *this;
  }
  Type & set__link2link(
    const double & _arg)
  {
    this->link2link = _arg;
    return *this;
  }
  Type & set__init_z_up(
    const double & _arg)
  {
    this->init_z_up = _arg;
    return *this;
  }
  Type & set__percentage_of_ik_motor(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->percentage_of_ik_motor = _arg;
    return *this;
  }
  Type & set__first_pos_xr(
    const double & _arg)
  {
    this->first_pos_xr = _arg;
    return *this;
  }
  Type & set__first_neg_xr(
    const double & _arg)
  {
    this->first_neg_xr = _arg;
    return *this;
  }
  Type & set__first_pos_side_r(
    const double & _arg)
  {
    this->first_pos_side_r = _arg;
    return *this;
  }
  Type & set__first_neg_side_r(
    const double & _arg)
  {
    this->first_neg_side_r = _arg;
    return *this;
  }
  Type & set__first_pos_yaw_r(
    const double & _arg)
  {
    this->first_pos_yaw_r = _arg;
    return *this;
  }
  Type & set__first_neg_yaw_r(
    const double & _arg)
  {
    this->first_neg_yaw_r = _arg;
    return *this;
  }
  Type & set__first_pos_xl(
    const double & _arg)
  {
    this->first_pos_xl = _arg;
    return *this;
  }
  Type & set__first_neg_xl(
    const double & _arg)
  {
    this->first_neg_xl = _arg;
    return *this;
  }
  Type & set__first_pos_side_l(
    const double & _arg)
  {
    this->first_pos_side_l = _arg;
    return *this;
  }
  Type & set__first_neg_side_l(
    const double & _arg)
  {
    this->first_neg_side_l = _arg;
    return *this;
  }
  Type & set__first_pos_yaw_l(
    const double & _arg)
  {
    this->first_pos_yaw_l = _arg;
    return *this;
  }
  Type & set__first_neg_yaw_l(
    const double & _arg)
  {
    this->first_neg_yaw_l = _arg;
    return *this;
  }
  Type & set__first_pos_side_r_swing_minus(
    const double & _arg)
  {
    this->first_pos_side_r_swing_minus = _arg;
    return *this;
  }
  Type & set__first_neg_side_r_swing_minus(
    const double & _arg)
  {
    this->first_neg_side_r_swing_minus = _arg;
    return *this;
  }
  Type & set__first_pos_side_l_swing_minus(
    const double & _arg)
  {
    this->first_pos_side_l_swing_minus = _arg;
    return *this;
  }
  Type & set__first_neg_side_l_swing_minus(
    const double & _arg)
  {
    this->first_neg_side_l_swing_minus = _arg;
    return *this;
  }
  Type & set__first_min(
    const int64_t & _arg)
  {
    this->first_min = _arg;
    return *this;
  }
  Type & set__first_max(
    const int64_t & _arg)
  {
    this->first_max = _arg;
    return *this;
  }
  Type & set__second_pos_xr(
    const double & _arg)
  {
    this->second_pos_xr = _arg;
    return *this;
  }
  Type & set__second_neg_xr(
    const double & _arg)
  {
    this->second_neg_xr = _arg;
    return *this;
  }
  Type & set__second_pos_side_r(
    const double & _arg)
  {
    this->second_pos_side_r = _arg;
    return *this;
  }
  Type & set__second_neg_side_r(
    const double & _arg)
  {
    this->second_neg_side_r = _arg;
    return *this;
  }
  Type & set__second_pos_yaw_r(
    const double & _arg)
  {
    this->second_pos_yaw_r = _arg;
    return *this;
  }
  Type & set__second_neg_yaw_r(
    const double & _arg)
  {
    this->second_neg_yaw_r = _arg;
    return *this;
  }
  Type & set__second_pos_xl(
    const double & _arg)
  {
    this->second_pos_xl = _arg;
    return *this;
  }
  Type & set__second_neg_xl(
    const double & _arg)
  {
    this->second_neg_xl = _arg;
    return *this;
  }
  Type & set__second_pos_side_l(
    const double & _arg)
  {
    this->second_pos_side_l = _arg;
    return *this;
  }
  Type & set__second_neg_side_l(
    const double & _arg)
  {
    this->second_neg_side_l = _arg;
    return *this;
  }
  Type & set__second_pos_yaw_l(
    const double & _arg)
  {
    this->second_pos_yaw_l = _arg;
    return *this;
  }
  Type & set__second_neg_yaw_l(
    const double & _arg)
  {
    this->second_neg_yaw_l = _arg;
    return *this;
  }
  Type & set__second_pos_side_r_swing_minus(
    const double & _arg)
  {
    this->second_pos_side_r_swing_minus = _arg;
    return *this;
  }
  Type & set__second_neg_side_r_swing_minus(
    const double & _arg)
  {
    this->second_neg_side_r_swing_minus = _arg;
    return *this;
  }
  Type & set__second_pos_side_l_swing_minus(
    const double & _arg)
  {
    this->second_pos_side_l_swing_minus = _arg;
    return *this;
  }
  Type & set__second_neg_side_l_swing_minus(
    const double & _arg)
  {
    this->second_neg_side_l_swing_minus = _arg;
    return *this;
  }
  Type & set__second_min(
    const int64_t & _arg)
  {
    this->second_min = _arg;
    return *this;
  }
  Type & set__second_max(
    const int64_t & _arg)
  {
    this->second_max = _arg;
    return *this;
  }
  Type & set__third_pos_xr(
    const double & _arg)
  {
    this->third_pos_xr = _arg;
    return *this;
  }
  Type & set__third_neg_xr(
    const double & _arg)
  {
    this->third_neg_xr = _arg;
    return *this;
  }
  Type & set__third_pos_side_r(
    const double & _arg)
  {
    this->third_pos_side_r = _arg;
    return *this;
  }
  Type & set__third_neg_side_r(
    const double & _arg)
  {
    this->third_neg_side_r = _arg;
    return *this;
  }
  Type & set__third_pos_yaw_r(
    const double & _arg)
  {
    this->third_pos_yaw_r = _arg;
    return *this;
  }
  Type & set__third_neg_yaw_r(
    const double & _arg)
  {
    this->third_neg_yaw_r = _arg;
    return *this;
  }
  Type & set__third_pos_xl(
    const double & _arg)
  {
    this->third_pos_xl = _arg;
    return *this;
  }
  Type & set__third_neg_xl(
    const double & _arg)
  {
    this->third_neg_xl = _arg;
    return *this;
  }
  Type & set__third_pos_side_l(
    const double & _arg)
  {
    this->third_pos_side_l = _arg;
    return *this;
  }
  Type & set__third_neg_side_l(
    const double & _arg)
  {
    this->third_neg_side_l = _arg;
    return *this;
  }
  Type & set__third_pos_yaw_l(
    const double & _arg)
  {
    this->third_pos_yaw_l = _arg;
    return *this;
  }
  Type & set__third_neg_yaw_l(
    const double & _arg)
  {
    this->third_neg_yaw_l = _arg;
    return *this;
  }
  Type & set__third_pos_side_r_swing_minus(
    const double & _arg)
  {
    this->third_pos_side_r_swing_minus = _arg;
    return *this;
  }
  Type & set__third_neg_side_r_swing_minus(
    const double & _arg)
  {
    this->third_neg_side_r_swing_minus = _arg;
    return *this;
  }
  Type & set__third_pos_side_l_swing_minus(
    const double & _arg)
  {
    this->third_pos_side_l_swing_minus = _arg;
    return *this;
  }
  Type & set__third_neg_side_l_swing_minus(
    const double & _arg)
  {
    this->third_neg_side_l_swing_minus = _arg;
    return *this;
  }
  Type & set__third_min(
    const int64_t & _arg)
  {
    this->third_min = _arg;
    return *this;
  }
  Type & set__third_max(
    const int64_t & _arg)
  {
    this->third_max = _arg;
    return *this;
  }
  Type & set__fourth_pos_xr(
    const double & _arg)
  {
    this->fourth_pos_xr = _arg;
    return *this;
  }
  Type & set__fourth_neg_xr(
    const double & _arg)
  {
    this->fourth_neg_xr = _arg;
    return *this;
  }
  Type & set__fourth_pos_side_r(
    const double & _arg)
  {
    this->fourth_pos_side_r = _arg;
    return *this;
  }
  Type & set__fourth_neg_side_r(
    const double & _arg)
  {
    this->fourth_neg_side_r = _arg;
    return *this;
  }
  Type & set__fourth_pos_yaw_r(
    const double & _arg)
  {
    this->fourth_pos_yaw_r = _arg;
    return *this;
  }
  Type & set__fourth_neg_yaw_r(
    const double & _arg)
  {
    this->fourth_neg_yaw_r = _arg;
    return *this;
  }
  Type & set__fourth_pos_xl(
    const double & _arg)
  {
    this->fourth_pos_xl = _arg;
    return *this;
  }
  Type & set__fourth_neg_xl(
    const double & _arg)
  {
    this->fourth_neg_xl = _arg;
    return *this;
  }
  Type & set__fourth_pos_side_l(
    const double & _arg)
  {
    this->fourth_pos_side_l = _arg;
    return *this;
  }
  Type & set__fourth_neg_side_l(
    const double & _arg)
  {
    this->fourth_neg_side_l = _arg;
    return *this;
  }
  Type & set__fourth_pos_yaw_l(
    const double & _arg)
  {
    this->fourth_pos_yaw_l = _arg;
    return *this;
  }
  Type & set__fourth_neg_yaw_l(
    const double & _arg)
  {
    this->fourth_neg_yaw_l = _arg;
    return *this;
  }
  Type & set__fourth_pos_side_r_swing_minus(
    const double & _arg)
  {
    this->fourth_pos_side_r_swing_minus = _arg;
    return *this;
  }
  Type & set__fourth_neg_side_r_swing_minus(
    const double & _arg)
  {
    this->fourth_neg_side_r_swing_minus = _arg;
    return *this;
  }
  Type & set__fourth_pos_side_l_swing_minus(
    const double & _arg)
  {
    this->fourth_pos_side_l_swing_minus = _arg;
    return *this;
  }
  Type & set__fourth_neg_side_l_swing_minus(
    const double & _arg)
  {
    this->fourth_neg_side_l_swing_minus = _arg;
    return *this;
  }
  Type & set__fourth_min(
    const int64_t & _arg)
  {
    this->fourth_min = _arg;
    return *this;
  }
  Type & set__fourth_max(
    const int64_t & _arg)
  {
    this->fourth_max = _arg;
    return *this;
  }
  Type & set__landing_time_control_flag(
    const int64_t & _arg)
  {
    this->landing_time_control_flag = _arg;
    return *this;
  }
  Type & set__control_system_flag(
    const int64_t & _arg)
  {
    this->control_system_flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::Tune2IkMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::Tune2IkMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Tune2IkMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Tune2IkMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Tune2IkMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Tune2IkMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Tune2IkMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Tune2IkMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Tune2IkMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Tune2IkMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__Tune2IkMsg
    std::shared_ptr<humanoid_interfaces::msg::Tune2IkMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__Tune2IkMsg
    std::shared_ptr<humanoid_interfaces::msg::Tune2IkMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Tune2IkMsg_ & other) const
  {
    if (this->ik_flag != other.ik_flag) {
      return false;
    }
    if (this->entire_time != other.entire_time) {
      return false;
    }
    if (this->frequency != other.frequency) {
      return false;
    }
    if (this->default_x_left != other.default_x_left) {
      return false;
    }
    if (this->default_y_left != other.default_y_left) {
      return false;
    }
    if (this->default_z_left != other.default_z_left) {
      return false;
    }
    if (this->default_x_right != other.default_x_right) {
      return false;
    }
    if (this->default_y_right != other.default_y_right) {
      return false;
    }
    if (this->default_z_right != other.default_z_right) {
      return false;
    }
    if (this->offset_motor != other.offset_motor) {
      return false;
    }
    if (this->swing_shoulder_left != other.swing_shoulder_left) {
      return false;
    }
    if (this->swing_shoulder_right != other.swing_shoulder_right) {
      return false;
    }
    if (this->swing_leg_left != other.swing_leg_left) {
      return false;
    }
    if (this->swing_leg_right != other.swing_leg_right) {
      return false;
    }
    if (this->rise_leg_left != other.rise_leg_left) {
      return false;
    }
    if (this->rise_leg_right != other.rise_leg_right) {
      return false;
    }
    if (this->start_entire_time != other.start_entire_time) {
      return false;
    }
    if (this->start_swing != other.start_swing) {
      return false;
    }
    if (this->start_rise != other.start_rise) {
      return false;
    }
    if (this->end_entire_time != other.end_entire_time) {
      return false;
    }
    if (this->end_swing != other.end_swing) {
      return false;
    }
    if (this->end_rise != other.end_rise) {
      return false;
    }
    if (this->test_x != other.test_x) {
      return false;
    }
    if (this->test_side != other.test_side) {
      return false;
    }
    if (this->test_yaw != other.test_yaw) {
      return false;
    }
    if (this->tuning_x != other.tuning_x) {
      return false;
    }
    if (this->tuning_side != other.tuning_side) {
      return false;
    }
    if (this->tuning_yaw != other.tuning_yaw) {
      return false;
    }
    if (this->balance_value_0 != other.balance_value_0) {
      return false;
    }
    if (this->balance_value_1 != other.balance_value_1) {
      return false;
    }
    if (this->balance_value_2 != other.balance_value_2) {
      return false;
    }
    if (this->balance_value_3 != other.balance_value_3) {
      return false;
    }
    if (this->balance_pitch_gp != other.balance_pitch_gp) {
      return false;
    }
    if (this->balance_pitch_gi != other.balance_pitch_gi) {
      return false;
    }
    if (this->balance_pitch_gd != other.balance_pitch_gd) {
      return false;
    }
    if (this->balance_pitch_elimit != other.balance_pitch_elimit) {
      return false;
    }
    if (this->balance_pitch_olimit != other.balance_pitch_olimit) {
      return false;
    }
    if (this->balance_pitch_neg_target != other.balance_pitch_neg_target) {
      return false;
    }
    if (this->balance_pitch_pos_target != other.balance_pitch_pos_target) {
      return false;
    }
    if (this->balance_roll_gp != other.balance_roll_gp) {
      return false;
    }
    if (this->balance_roll_gi != other.balance_roll_gi) {
      return false;
    }
    if (this->balance_roll_gd != other.balance_roll_gd) {
      return false;
    }
    if (this->balance_roll_elimit != other.balance_roll_elimit) {
      return false;
    }
    if (this->balance_roll_olimit != other.balance_roll_olimit) {
      return false;
    }
    if (this->balance_roll_neg_target != other.balance_roll_neg_target) {
      return false;
    }
    if (this->balance_roll_pos_target != other.balance_roll_pos_target) {
      return false;
    }
    if (this->balance_angle_pitch_gp != other.balance_angle_pitch_gp) {
      return false;
    }
    if (this->balance_angle_pitch_gi != other.balance_angle_pitch_gi) {
      return false;
    }
    if (this->balance_angle_pitch_gd != other.balance_angle_pitch_gd) {
      return false;
    }
    if (this->balance_angle_pitch_elimit != other.balance_angle_pitch_elimit) {
      return false;
    }
    if (this->balance_angle_pitch_olimit != other.balance_angle_pitch_olimit) {
      return false;
    }
    if (this->balance_angle_pitch_neg_target != other.balance_angle_pitch_neg_target) {
      return false;
    }
    if (this->balance_angle_pitch_pos_target != other.balance_angle_pitch_pos_target) {
      return false;
    }
    if (this->balance_value_4 != other.balance_value_4) {
      return false;
    }
    if (this->balance_value_5 != other.balance_value_5) {
      return false;
    }
    if (this->balance_pitch_flag_imu != other.balance_pitch_flag_imu) {
      return false;
    }
    if (this->balance_pitch_gp_imu != other.balance_pitch_gp_imu) {
      return false;
    }
    if (this->balance_pitch_gi_imu != other.balance_pitch_gi_imu) {
      return false;
    }
    if (this->balance_pitch_gd_imu != other.balance_pitch_gd_imu) {
      return false;
    }
    if (this->balance_pitch_neg_target_imu != other.balance_pitch_neg_target_imu) {
      return false;
    }
    if (this->balance_pitch_pos_target_imu != other.balance_pitch_pos_target_imu) {
      return false;
    }
    if (this->balance_pitch_elimit_imu != other.balance_pitch_elimit_imu) {
      return false;
    }
    if (this->balance_pitch_olimit_imu != other.balance_pitch_olimit_imu) {
      return false;
    }
    if (this->balance_roll_flag_imu != other.balance_roll_flag_imu) {
      return false;
    }
    if (this->balance_roll_gp_imu != other.balance_roll_gp_imu) {
      return false;
    }
    if (this->balance_roll_gi_imu != other.balance_roll_gi_imu) {
      return false;
    }
    if (this->balance_roll_gd_imu != other.balance_roll_gd_imu) {
      return false;
    }
    if (this->balance_roll_neg_target_imu != other.balance_roll_neg_target_imu) {
      return false;
    }
    if (this->balance_roll_pos_target_imu != other.balance_roll_pos_target_imu) {
      return false;
    }
    if (this->balance_roll_elimit_imu != other.balance_roll_elimit_imu) {
      return false;
    }
    if (this->balance_roll_olimit_imu != other.balance_roll_olimit_imu) {
      return false;
    }
    if (this->balance_pitch_flag != other.balance_pitch_flag) {
      return false;
    }
    if (this->balance_ankle_pitch_flag != other.balance_ankle_pitch_flag) {
      return false;
    }
    if (this->balance_roll_flag != other.balance_roll_flag) {
      return false;
    }
    if (this->ratio_check_flag != other.ratio_check_flag) {
      return false;
    }
    if (this->center2leg != other.center2leg) {
      return false;
    }
    if (this->link2link != other.link2link) {
      return false;
    }
    if (this->init_z_up != other.init_z_up) {
      return false;
    }
    if (this->percentage_of_ik_motor != other.percentage_of_ik_motor) {
      return false;
    }
    if (this->first_pos_xr != other.first_pos_xr) {
      return false;
    }
    if (this->first_neg_xr != other.first_neg_xr) {
      return false;
    }
    if (this->first_pos_side_r != other.first_pos_side_r) {
      return false;
    }
    if (this->first_neg_side_r != other.first_neg_side_r) {
      return false;
    }
    if (this->first_pos_yaw_r != other.first_pos_yaw_r) {
      return false;
    }
    if (this->first_neg_yaw_r != other.first_neg_yaw_r) {
      return false;
    }
    if (this->first_pos_xl != other.first_pos_xl) {
      return false;
    }
    if (this->first_neg_xl != other.first_neg_xl) {
      return false;
    }
    if (this->first_pos_side_l != other.first_pos_side_l) {
      return false;
    }
    if (this->first_neg_side_l != other.first_neg_side_l) {
      return false;
    }
    if (this->first_pos_yaw_l != other.first_pos_yaw_l) {
      return false;
    }
    if (this->first_neg_yaw_l != other.first_neg_yaw_l) {
      return false;
    }
    if (this->first_pos_side_r_swing_minus != other.first_pos_side_r_swing_minus) {
      return false;
    }
    if (this->first_neg_side_r_swing_minus != other.first_neg_side_r_swing_minus) {
      return false;
    }
    if (this->first_pos_side_l_swing_minus != other.first_pos_side_l_swing_minus) {
      return false;
    }
    if (this->first_neg_side_l_swing_minus != other.first_neg_side_l_swing_minus) {
      return false;
    }
    if (this->first_min != other.first_min) {
      return false;
    }
    if (this->first_max != other.first_max) {
      return false;
    }
    if (this->second_pos_xr != other.second_pos_xr) {
      return false;
    }
    if (this->second_neg_xr != other.second_neg_xr) {
      return false;
    }
    if (this->second_pos_side_r != other.second_pos_side_r) {
      return false;
    }
    if (this->second_neg_side_r != other.second_neg_side_r) {
      return false;
    }
    if (this->second_pos_yaw_r != other.second_pos_yaw_r) {
      return false;
    }
    if (this->second_neg_yaw_r != other.second_neg_yaw_r) {
      return false;
    }
    if (this->second_pos_xl != other.second_pos_xl) {
      return false;
    }
    if (this->second_neg_xl != other.second_neg_xl) {
      return false;
    }
    if (this->second_pos_side_l != other.second_pos_side_l) {
      return false;
    }
    if (this->second_neg_side_l != other.second_neg_side_l) {
      return false;
    }
    if (this->second_pos_yaw_l != other.second_pos_yaw_l) {
      return false;
    }
    if (this->second_neg_yaw_l != other.second_neg_yaw_l) {
      return false;
    }
    if (this->second_pos_side_r_swing_minus != other.second_pos_side_r_swing_minus) {
      return false;
    }
    if (this->second_neg_side_r_swing_minus != other.second_neg_side_r_swing_minus) {
      return false;
    }
    if (this->second_pos_side_l_swing_minus != other.second_pos_side_l_swing_minus) {
      return false;
    }
    if (this->second_neg_side_l_swing_minus != other.second_neg_side_l_swing_minus) {
      return false;
    }
    if (this->second_min != other.second_min) {
      return false;
    }
    if (this->second_max != other.second_max) {
      return false;
    }
    if (this->third_pos_xr != other.third_pos_xr) {
      return false;
    }
    if (this->third_neg_xr != other.third_neg_xr) {
      return false;
    }
    if (this->third_pos_side_r != other.third_pos_side_r) {
      return false;
    }
    if (this->third_neg_side_r != other.third_neg_side_r) {
      return false;
    }
    if (this->third_pos_yaw_r != other.third_pos_yaw_r) {
      return false;
    }
    if (this->third_neg_yaw_r != other.third_neg_yaw_r) {
      return false;
    }
    if (this->third_pos_xl != other.third_pos_xl) {
      return false;
    }
    if (this->third_neg_xl != other.third_neg_xl) {
      return false;
    }
    if (this->third_pos_side_l != other.third_pos_side_l) {
      return false;
    }
    if (this->third_neg_side_l != other.third_neg_side_l) {
      return false;
    }
    if (this->third_pos_yaw_l != other.third_pos_yaw_l) {
      return false;
    }
    if (this->third_neg_yaw_l != other.third_neg_yaw_l) {
      return false;
    }
    if (this->third_pos_side_r_swing_minus != other.third_pos_side_r_swing_minus) {
      return false;
    }
    if (this->third_neg_side_r_swing_minus != other.third_neg_side_r_swing_minus) {
      return false;
    }
    if (this->third_pos_side_l_swing_minus != other.third_pos_side_l_swing_minus) {
      return false;
    }
    if (this->third_neg_side_l_swing_minus != other.third_neg_side_l_swing_minus) {
      return false;
    }
    if (this->third_min != other.third_min) {
      return false;
    }
    if (this->third_max != other.third_max) {
      return false;
    }
    if (this->fourth_pos_xr != other.fourth_pos_xr) {
      return false;
    }
    if (this->fourth_neg_xr != other.fourth_neg_xr) {
      return false;
    }
    if (this->fourth_pos_side_r != other.fourth_pos_side_r) {
      return false;
    }
    if (this->fourth_neg_side_r != other.fourth_neg_side_r) {
      return false;
    }
    if (this->fourth_pos_yaw_r != other.fourth_pos_yaw_r) {
      return false;
    }
    if (this->fourth_neg_yaw_r != other.fourth_neg_yaw_r) {
      return false;
    }
    if (this->fourth_pos_xl != other.fourth_pos_xl) {
      return false;
    }
    if (this->fourth_neg_xl != other.fourth_neg_xl) {
      return false;
    }
    if (this->fourth_pos_side_l != other.fourth_pos_side_l) {
      return false;
    }
    if (this->fourth_neg_side_l != other.fourth_neg_side_l) {
      return false;
    }
    if (this->fourth_pos_yaw_l != other.fourth_pos_yaw_l) {
      return false;
    }
    if (this->fourth_neg_yaw_l != other.fourth_neg_yaw_l) {
      return false;
    }
    if (this->fourth_pos_side_r_swing_minus != other.fourth_pos_side_r_swing_minus) {
      return false;
    }
    if (this->fourth_neg_side_r_swing_minus != other.fourth_neg_side_r_swing_minus) {
      return false;
    }
    if (this->fourth_pos_side_l_swing_minus != other.fourth_pos_side_l_swing_minus) {
      return false;
    }
    if (this->fourth_neg_side_l_swing_minus != other.fourth_neg_side_l_swing_minus) {
      return false;
    }
    if (this->fourth_min != other.fourth_min) {
      return false;
    }
    if (this->fourth_max != other.fourth_max) {
      return false;
    }
    if (this->landing_time_control_flag != other.landing_time_control_flag) {
      return false;
    }
    if (this->control_system_flag != other.control_system_flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const Tune2IkMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Tune2IkMsg_

// alias to use template instance with default allocator
using Tune2IkMsg =
  humanoid_interfaces::msg::Tune2IkMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__TUNE2_IK_MSG__STRUCT_HPP_
