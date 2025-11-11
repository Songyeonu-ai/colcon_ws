// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/Tune2IkMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__TUNE2_IK_MSG__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__TUNE2_IK_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'offset_motor'
// Member 'percentage_of_ik_motor'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Tune2IkMsg in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__Tune2IkMsg
{
  bool ik_flag;
  double entire_time;
  double frequency;
  double default_x_left;
  double default_y_left;
  double default_z_left;
  double default_x_right;
  double default_y_right;
  double default_z_right;
  rosidl_runtime_c__double__Sequence offset_motor;
  double swing_shoulder_left;
  double swing_shoulder_right;
  double swing_leg_left;
  double swing_leg_right;
  double rise_leg_left;
  double rise_leg_right;
  double start_entire_time;
  double start_swing;
  double start_rise;
  double end_entire_time;
  double end_swing;
  double end_rise;
  double test_x;
  double test_side;
  double test_yaw;
  double tuning_x;
  double tuning_side;
  double tuning_yaw;
  double balance_value_0;
  double balance_value_1;
  double balance_value_2;
  double balance_value_3;
  double balance_pitch_gp;
  double balance_pitch_gi;
  double balance_pitch_gd;
  double balance_pitch_elimit;
  double balance_pitch_olimit;
  double balance_pitch_neg_target;
  double balance_pitch_pos_target;
  double balance_roll_gp;
  double balance_roll_gi;
  double balance_roll_gd;
  double balance_roll_elimit;
  double balance_roll_olimit;
  double balance_roll_neg_target;
  double balance_roll_pos_target;
  double balance_angle_pitch_gp;
  double balance_angle_pitch_gi;
  double balance_angle_pitch_gd;
  double balance_angle_pitch_elimit;
  double balance_angle_pitch_olimit;
  double balance_angle_pitch_neg_target;
  double balance_angle_pitch_pos_target;
  double balance_value_4;
  double balance_value_5;
  /// bool
  int64_t balance_pitch_flag_imu;
  double balance_pitch_gp_imu;
  double balance_pitch_gi_imu;
  double balance_pitch_gd_imu;
  double balance_pitch_neg_target_imu;
  double balance_pitch_pos_target_imu;
  double balance_pitch_elimit_imu;
  double balance_pitch_olimit_imu;
  /// bool
  int64_t balance_roll_flag_imu;
  double balance_roll_gp_imu;
  double balance_roll_gi_imu;
  double balance_roll_gd_imu;
  double balance_roll_neg_target_imu;
  double balance_roll_pos_target_imu;
  double balance_roll_elimit_imu;
  double balance_roll_olimit_imu;
  /// bool
  int64_t balance_pitch_flag;
  /// bool
  int64_t balance_ankle_pitch_flag;
  /// bool
  int64_t balance_roll_flag;
  int64_t ratio_check_flag;
  double center2leg;
  double link2link;
  double init_z_up;
  rosidl_runtime_c__double__Sequence percentage_of_ik_motor;
  double first_pos_xr;
  double first_neg_xr;
  double first_pos_side_r;
  double first_neg_side_r;
  double first_pos_yaw_r;
  double first_neg_yaw_r;
  double first_pos_xl;
  double first_neg_xl;
  double first_pos_side_l;
  double first_neg_side_l;
  double first_pos_yaw_l;
  double first_neg_yaw_l;
  double first_pos_side_r_swing_minus;
  double first_neg_side_r_swing_minus;
  double first_pos_side_l_swing_minus;
  double first_neg_side_l_swing_minus;
  int64_t first_min;
  int64_t first_max;
  double second_pos_xr;
  double second_neg_xr;
  double second_pos_side_r;
  double second_neg_side_r;
  double second_pos_yaw_r;
  double second_neg_yaw_r;
  double second_pos_xl;
  double second_neg_xl;
  double second_pos_side_l;
  double second_neg_side_l;
  double second_pos_yaw_l;
  double second_neg_yaw_l;
  double second_pos_side_r_swing_minus;
  double second_neg_side_r_swing_minus;
  double second_pos_side_l_swing_minus;
  double second_neg_side_l_swing_minus;
  int64_t second_min;
  int64_t second_max;
  double third_pos_xr;
  double third_neg_xr;
  double third_pos_side_r;
  double third_neg_side_r;
  double third_pos_yaw_r;
  double third_neg_yaw_r;
  double third_pos_xl;
  double third_neg_xl;
  double third_pos_side_l;
  double third_neg_side_l;
  double third_pos_yaw_l;
  double third_neg_yaw_l;
  double third_pos_side_r_swing_minus;
  double third_neg_side_r_swing_minus;
  double third_pos_side_l_swing_minus;
  double third_neg_side_l_swing_minus;
  int64_t third_min;
  int64_t third_max;
  double fourth_pos_xr;
  double fourth_neg_xr;
  double fourth_pos_side_r;
  double fourth_neg_side_r;
  double fourth_pos_yaw_r;
  double fourth_neg_yaw_r;
  double fourth_pos_xl;
  double fourth_neg_xl;
  double fourth_pos_side_l;
  double fourth_neg_side_l;
  double fourth_pos_yaw_l;
  double fourth_neg_yaw_l;
  double fourth_pos_side_r_swing_minus;
  double fourth_neg_side_r_swing_minus;
  double fourth_pos_side_l_swing_minus;
  double fourth_neg_side_l_swing_minus;
  int64_t fourth_min;
  int64_t fourth_max;
  int64_t landing_time_control_flag;
  int64_t control_system_flag;
} humanoid_interfaces__msg__Tune2IkMsg;

// Struct for a sequence of humanoid_interfaces__msg__Tune2IkMsg.
typedef struct humanoid_interfaces__msg__Tune2IkMsg__Sequence
{
  humanoid_interfaces__msg__Tune2IkMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__Tune2IkMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__TUNE2_IK_MSG__STRUCT_H_
