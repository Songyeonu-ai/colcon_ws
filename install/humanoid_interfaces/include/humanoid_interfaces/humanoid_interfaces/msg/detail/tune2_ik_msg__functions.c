// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/Tune2IkMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/tune2_ik_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `offset_motor`
// Member `percentage_of_ik_motor`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
humanoid_interfaces__msg__Tune2IkMsg__init(humanoid_interfaces__msg__Tune2IkMsg * msg)
{
  if (!msg) {
    return false;
  }
  // ik_flag
  // entire_time
  // frequency
  // default_x_left
  // default_y_left
  // default_z_left
  // default_x_right
  // default_y_right
  // default_z_right
  // offset_motor
  if (!rosidl_runtime_c__double__Sequence__init(&msg->offset_motor, 0)) {
    humanoid_interfaces__msg__Tune2IkMsg__fini(msg);
    return false;
  }
  // swing_shoulder_left
  // swing_shoulder_right
  // swing_leg_left
  // swing_leg_right
  // rise_leg_left
  // rise_leg_right
  // start_entire_time
  // start_swing
  // start_rise
  // end_entire_time
  // end_swing
  // end_rise
  // test_x
  // test_side
  // test_yaw
  // tuning_x
  // tuning_side
  // tuning_yaw
  // balance_value_0
  // balance_value_1
  // balance_value_2
  // balance_value_3
  // balance_pitch_gp
  // balance_pitch_gi
  // balance_pitch_gd
  // balance_pitch_elimit
  // balance_pitch_olimit
  // balance_pitch_neg_target
  // balance_pitch_pos_target
  // balance_roll_gp
  // balance_roll_gi
  // balance_roll_gd
  // balance_roll_elimit
  // balance_roll_olimit
  // balance_roll_neg_target
  // balance_roll_pos_target
  // balance_angle_pitch_gp
  // balance_angle_pitch_gi
  // balance_angle_pitch_gd
  // balance_angle_pitch_elimit
  // balance_angle_pitch_olimit
  // balance_angle_pitch_neg_target
  // balance_angle_pitch_pos_target
  // balance_value_4
  // balance_value_5
  // balance_pitch_flag_imu
  // balance_pitch_gp_imu
  // balance_pitch_gi_imu
  // balance_pitch_gd_imu
  // balance_pitch_neg_target_imu
  // balance_pitch_pos_target_imu
  // balance_pitch_elimit_imu
  // balance_pitch_olimit_imu
  // balance_roll_flag_imu
  // balance_roll_gp_imu
  // balance_roll_gi_imu
  // balance_roll_gd_imu
  // balance_roll_neg_target_imu
  // balance_roll_pos_target_imu
  // balance_roll_elimit_imu
  // balance_roll_olimit_imu
  // balance_pitch_flag
  // balance_ankle_pitch_flag
  // balance_roll_flag
  // ratio_check_flag
  // center2leg
  // link2link
  // init_z_up
  // percentage_of_ik_motor
  if (!rosidl_runtime_c__double__Sequence__init(&msg->percentage_of_ik_motor, 0)) {
    humanoid_interfaces__msg__Tune2IkMsg__fini(msg);
    return false;
  }
  // first_pos_xr
  // first_neg_xr
  // first_pos_side_r
  // first_neg_side_r
  // first_pos_yaw_r
  // first_neg_yaw_r
  // first_pos_xl
  // first_neg_xl
  // first_pos_side_l
  // first_neg_side_l
  // first_pos_yaw_l
  // first_neg_yaw_l
  // first_pos_side_r_swing_minus
  // first_neg_side_r_swing_minus
  // first_pos_side_l_swing_minus
  // first_neg_side_l_swing_minus
  // first_min
  // first_max
  // second_pos_xr
  // second_neg_xr
  // second_pos_side_r
  // second_neg_side_r
  // second_pos_yaw_r
  // second_neg_yaw_r
  // second_pos_xl
  // second_neg_xl
  // second_pos_side_l
  // second_neg_side_l
  // second_pos_yaw_l
  // second_neg_yaw_l
  // second_pos_side_r_swing_minus
  // second_neg_side_r_swing_minus
  // second_pos_side_l_swing_minus
  // second_neg_side_l_swing_minus
  // second_min
  // second_max
  // third_pos_xr
  // third_neg_xr
  // third_pos_side_r
  // third_neg_side_r
  // third_pos_yaw_r
  // third_neg_yaw_r
  // third_pos_xl
  // third_neg_xl
  // third_pos_side_l
  // third_neg_side_l
  // third_pos_yaw_l
  // third_neg_yaw_l
  // third_pos_side_r_swing_minus
  // third_neg_side_r_swing_minus
  // third_pos_side_l_swing_minus
  // third_neg_side_l_swing_minus
  // third_min
  // third_max
  // fourth_pos_xr
  // fourth_neg_xr
  // fourth_pos_side_r
  // fourth_neg_side_r
  // fourth_pos_yaw_r
  // fourth_neg_yaw_r
  // fourth_pos_xl
  // fourth_neg_xl
  // fourth_pos_side_l
  // fourth_neg_side_l
  // fourth_pos_yaw_l
  // fourth_neg_yaw_l
  // fourth_pos_side_r_swing_minus
  // fourth_neg_side_r_swing_minus
  // fourth_pos_side_l_swing_minus
  // fourth_neg_side_l_swing_minus
  // fourth_min
  // fourth_max
  // landing_time_control_flag
  // control_system_flag
  return true;
}

void
humanoid_interfaces__msg__Tune2IkMsg__fini(humanoid_interfaces__msg__Tune2IkMsg * msg)
{
  if (!msg) {
    return;
  }
  // ik_flag
  // entire_time
  // frequency
  // default_x_left
  // default_y_left
  // default_z_left
  // default_x_right
  // default_y_right
  // default_z_right
  // offset_motor
  rosidl_runtime_c__double__Sequence__fini(&msg->offset_motor);
  // swing_shoulder_left
  // swing_shoulder_right
  // swing_leg_left
  // swing_leg_right
  // rise_leg_left
  // rise_leg_right
  // start_entire_time
  // start_swing
  // start_rise
  // end_entire_time
  // end_swing
  // end_rise
  // test_x
  // test_side
  // test_yaw
  // tuning_x
  // tuning_side
  // tuning_yaw
  // balance_value_0
  // balance_value_1
  // balance_value_2
  // balance_value_3
  // balance_pitch_gp
  // balance_pitch_gi
  // balance_pitch_gd
  // balance_pitch_elimit
  // balance_pitch_olimit
  // balance_pitch_neg_target
  // balance_pitch_pos_target
  // balance_roll_gp
  // balance_roll_gi
  // balance_roll_gd
  // balance_roll_elimit
  // balance_roll_olimit
  // balance_roll_neg_target
  // balance_roll_pos_target
  // balance_angle_pitch_gp
  // balance_angle_pitch_gi
  // balance_angle_pitch_gd
  // balance_angle_pitch_elimit
  // balance_angle_pitch_olimit
  // balance_angle_pitch_neg_target
  // balance_angle_pitch_pos_target
  // balance_value_4
  // balance_value_5
  // balance_pitch_flag_imu
  // balance_pitch_gp_imu
  // balance_pitch_gi_imu
  // balance_pitch_gd_imu
  // balance_pitch_neg_target_imu
  // balance_pitch_pos_target_imu
  // balance_pitch_elimit_imu
  // balance_pitch_olimit_imu
  // balance_roll_flag_imu
  // balance_roll_gp_imu
  // balance_roll_gi_imu
  // balance_roll_gd_imu
  // balance_roll_neg_target_imu
  // balance_roll_pos_target_imu
  // balance_roll_elimit_imu
  // balance_roll_olimit_imu
  // balance_pitch_flag
  // balance_ankle_pitch_flag
  // balance_roll_flag
  // ratio_check_flag
  // center2leg
  // link2link
  // init_z_up
  // percentage_of_ik_motor
  rosidl_runtime_c__double__Sequence__fini(&msg->percentage_of_ik_motor);
  // first_pos_xr
  // first_neg_xr
  // first_pos_side_r
  // first_neg_side_r
  // first_pos_yaw_r
  // first_neg_yaw_r
  // first_pos_xl
  // first_neg_xl
  // first_pos_side_l
  // first_neg_side_l
  // first_pos_yaw_l
  // first_neg_yaw_l
  // first_pos_side_r_swing_minus
  // first_neg_side_r_swing_minus
  // first_pos_side_l_swing_minus
  // first_neg_side_l_swing_minus
  // first_min
  // first_max
  // second_pos_xr
  // second_neg_xr
  // second_pos_side_r
  // second_neg_side_r
  // second_pos_yaw_r
  // second_neg_yaw_r
  // second_pos_xl
  // second_neg_xl
  // second_pos_side_l
  // second_neg_side_l
  // second_pos_yaw_l
  // second_neg_yaw_l
  // second_pos_side_r_swing_minus
  // second_neg_side_r_swing_minus
  // second_pos_side_l_swing_minus
  // second_neg_side_l_swing_minus
  // second_min
  // second_max
  // third_pos_xr
  // third_neg_xr
  // third_pos_side_r
  // third_neg_side_r
  // third_pos_yaw_r
  // third_neg_yaw_r
  // third_pos_xl
  // third_neg_xl
  // third_pos_side_l
  // third_neg_side_l
  // third_pos_yaw_l
  // third_neg_yaw_l
  // third_pos_side_r_swing_minus
  // third_neg_side_r_swing_minus
  // third_pos_side_l_swing_minus
  // third_neg_side_l_swing_minus
  // third_min
  // third_max
  // fourth_pos_xr
  // fourth_neg_xr
  // fourth_pos_side_r
  // fourth_neg_side_r
  // fourth_pos_yaw_r
  // fourth_neg_yaw_r
  // fourth_pos_xl
  // fourth_neg_xl
  // fourth_pos_side_l
  // fourth_neg_side_l
  // fourth_pos_yaw_l
  // fourth_neg_yaw_l
  // fourth_pos_side_r_swing_minus
  // fourth_neg_side_r_swing_minus
  // fourth_pos_side_l_swing_minus
  // fourth_neg_side_l_swing_minus
  // fourth_min
  // fourth_max
  // landing_time_control_flag
  // control_system_flag
}

bool
humanoid_interfaces__msg__Tune2IkMsg__are_equal(const humanoid_interfaces__msg__Tune2IkMsg * lhs, const humanoid_interfaces__msg__Tune2IkMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ik_flag
  if (lhs->ik_flag != rhs->ik_flag) {
    return false;
  }
  // entire_time
  if (lhs->entire_time != rhs->entire_time) {
    return false;
  }
  // frequency
  if (lhs->frequency != rhs->frequency) {
    return false;
  }
  // default_x_left
  if (lhs->default_x_left != rhs->default_x_left) {
    return false;
  }
  // default_y_left
  if (lhs->default_y_left != rhs->default_y_left) {
    return false;
  }
  // default_z_left
  if (lhs->default_z_left != rhs->default_z_left) {
    return false;
  }
  // default_x_right
  if (lhs->default_x_right != rhs->default_x_right) {
    return false;
  }
  // default_y_right
  if (lhs->default_y_right != rhs->default_y_right) {
    return false;
  }
  // default_z_right
  if (lhs->default_z_right != rhs->default_z_right) {
    return false;
  }
  // offset_motor
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->offset_motor), &(rhs->offset_motor)))
  {
    return false;
  }
  // swing_shoulder_left
  if (lhs->swing_shoulder_left != rhs->swing_shoulder_left) {
    return false;
  }
  // swing_shoulder_right
  if (lhs->swing_shoulder_right != rhs->swing_shoulder_right) {
    return false;
  }
  // swing_leg_left
  if (lhs->swing_leg_left != rhs->swing_leg_left) {
    return false;
  }
  // swing_leg_right
  if (lhs->swing_leg_right != rhs->swing_leg_right) {
    return false;
  }
  // rise_leg_left
  if (lhs->rise_leg_left != rhs->rise_leg_left) {
    return false;
  }
  // rise_leg_right
  if (lhs->rise_leg_right != rhs->rise_leg_right) {
    return false;
  }
  // start_entire_time
  if (lhs->start_entire_time != rhs->start_entire_time) {
    return false;
  }
  // start_swing
  if (lhs->start_swing != rhs->start_swing) {
    return false;
  }
  // start_rise
  if (lhs->start_rise != rhs->start_rise) {
    return false;
  }
  // end_entire_time
  if (lhs->end_entire_time != rhs->end_entire_time) {
    return false;
  }
  // end_swing
  if (lhs->end_swing != rhs->end_swing) {
    return false;
  }
  // end_rise
  if (lhs->end_rise != rhs->end_rise) {
    return false;
  }
  // test_x
  if (lhs->test_x != rhs->test_x) {
    return false;
  }
  // test_side
  if (lhs->test_side != rhs->test_side) {
    return false;
  }
  // test_yaw
  if (lhs->test_yaw != rhs->test_yaw) {
    return false;
  }
  // tuning_x
  if (lhs->tuning_x != rhs->tuning_x) {
    return false;
  }
  // tuning_side
  if (lhs->tuning_side != rhs->tuning_side) {
    return false;
  }
  // tuning_yaw
  if (lhs->tuning_yaw != rhs->tuning_yaw) {
    return false;
  }
  // balance_value_0
  if (lhs->balance_value_0 != rhs->balance_value_0) {
    return false;
  }
  // balance_value_1
  if (lhs->balance_value_1 != rhs->balance_value_1) {
    return false;
  }
  // balance_value_2
  if (lhs->balance_value_2 != rhs->balance_value_2) {
    return false;
  }
  // balance_value_3
  if (lhs->balance_value_3 != rhs->balance_value_3) {
    return false;
  }
  // balance_pitch_gp
  if (lhs->balance_pitch_gp != rhs->balance_pitch_gp) {
    return false;
  }
  // balance_pitch_gi
  if (lhs->balance_pitch_gi != rhs->balance_pitch_gi) {
    return false;
  }
  // balance_pitch_gd
  if (lhs->balance_pitch_gd != rhs->balance_pitch_gd) {
    return false;
  }
  // balance_pitch_elimit
  if (lhs->balance_pitch_elimit != rhs->balance_pitch_elimit) {
    return false;
  }
  // balance_pitch_olimit
  if (lhs->balance_pitch_olimit != rhs->balance_pitch_olimit) {
    return false;
  }
  // balance_pitch_neg_target
  if (lhs->balance_pitch_neg_target != rhs->balance_pitch_neg_target) {
    return false;
  }
  // balance_pitch_pos_target
  if (lhs->balance_pitch_pos_target != rhs->balance_pitch_pos_target) {
    return false;
  }
  // balance_roll_gp
  if (lhs->balance_roll_gp != rhs->balance_roll_gp) {
    return false;
  }
  // balance_roll_gi
  if (lhs->balance_roll_gi != rhs->balance_roll_gi) {
    return false;
  }
  // balance_roll_gd
  if (lhs->balance_roll_gd != rhs->balance_roll_gd) {
    return false;
  }
  // balance_roll_elimit
  if (lhs->balance_roll_elimit != rhs->balance_roll_elimit) {
    return false;
  }
  // balance_roll_olimit
  if (lhs->balance_roll_olimit != rhs->balance_roll_olimit) {
    return false;
  }
  // balance_roll_neg_target
  if (lhs->balance_roll_neg_target != rhs->balance_roll_neg_target) {
    return false;
  }
  // balance_roll_pos_target
  if (lhs->balance_roll_pos_target != rhs->balance_roll_pos_target) {
    return false;
  }
  // balance_angle_pitch_gp
  if (lhs->balance_angle_pitch_gp != rhs->balance_angle_pitch_gp) {
    return false;
  }
  // balance_angle_pitch_gi
  if (lhs->balance_angle_pitch_gi != rhs->balance_angle_pitch_gi) {
    return false;
  }
  // balance_angle_pitch_gd
  if (lhs->balance_angle_pitch_gd != rhs->balance_angle_pitch_gd) {
    return false;
  }
  // balance_angle_pitch_elimit
  if (lhs->balance_angle_pitch_elimit != rhs->balance_angle_pitch_elimit) {
    return false;
  }
  // balance_angle_pitch_olimit
  if (lhs->balance_angle_pitch_olimit != rhs->balance_angle_pitch_olimit) {
    return false;
  }
  // balance_angle_pitch_neg_target
  if (lhs->balance_angle_pitch_neg_target != rhs->balance_angle_pitch_neg_target) {
    return false;
  }
  // balance_angle_pitch_pos_target
  if (lhs->balance_angle_pitch_pos_target != rhs->balance_angle_pitch_pos_target) {
    return false;
  }
  // balance_value_4
  if (lhs->balance_value_4 != rhs->balance_value_4) {
    return false;
  }
  // balance_value_5
  if (lhs->balance_value_5 != rhs->balance_value_5) {
    return false;
  }
  // balance_pitch_flag_imu
  if (lhs->balance_pitch_flag_imu != rhs->balance_pitch_flag_imu) {
    return false;
  }
  // balance_pitch_gp_imu
  if (lhs->balance_pitch_gp_imu != rhs->balance_pitch_gp_imu) {
    return false;
  }
  // balance_pitch_gi_imu
  if (lhs->balance_pitch_gi_imu != rhs->balance_pitch_gi_imu) {
    return false;
  }
  // balance_pitch_gd_imu
  if (lhs->balance_pitch_gd_imu != rhs->balance_pitch_gd_imu) {
    return false;
  }
  // balance_pitch_neg_target_imu
  if (lhs->balance_pitch_neg_target_imu != rhs->balance_pitch_neg_target_imu) {
    return false;
  }
  // balance_pitch_pos_target_imu
  if (lhs->balance_pitch_pos_target_imu != rhs->balance_pitch_pos_target_imu) {
    return false;
  }
  // balance_pitch_elimit_imu
  if (lhs->balance_pitch_elimit_imu != rhs->balance_pitch_elimit_imu) {
    return false;
  }
  // balance_pitch_olimit_imu
  if (lhs->balance_pitch_olimit_imu != rhs->balance_pitch_olimit_imu) {
    return false;
  }
  // balance_roll_flag_imu
  if (lhs->balance_roll_flag_imu != rhs->balance_roll_flag_imu) {
    return false;
  }
  // balance_roll_gp_imu
  if (lhs->balance_roll_gp_imu != rhs->balance_roll_gp_imu) {
    return false;
  }
  // balance_roll_gi_imu
  if (lhs->balance_roll_gi_imu != rhs->balance_roll_gi_imu) {
    return false;
  }
  // balance_roll_gd_imu
  if (lhs->balance_roll_gd_imu != rhs->balance_roll_gd_imu) {
    return false;
  }
  // balance_roll_neg_target_imu
  if (lhs->balance_roll_neg_target_imu != rhs->balance_roll_neg_target_imu) {
    return false;
  }
  // balance_roll_pos_target_imu
  if (lhs->balance_roll_pos_target_imu != rhs->balance_roll_pos_target_imu) {
    return false;
  }
  // balance_roll_elimit_imu
  if (lhs->balance_roll_elimit_imu != rhs->balance_roll_elimit_imu) {
    return false;
  }
  // balance_roll_olimit_imu
  if (lhs->balance_roll_olimit_imu != rhs->balance_roll_olimit_imu) {
    return false;
  }
  // balance_pitch_flag
  if (lhs->balance_pitch_flag != rhs->balance_pitch_flag) {
    return false;
  }
  // balance_ankle_pitch_flag
  if (lhs->balance_ankle_pitch_flag != rhs->balance_ankle_pitch_flag) {
    return false;
  }
  // balance_roll_flag
  if (lhs->balance_roll_flag != rhs->balance_roll_flag) {
    return false;
  }
  // ratio_check_flag
  if (lhs->ratio_check_flag != rhs->ratio_check_flag) {
    return false;
  }
  // center2leg
  if (lhs->center2leg != rhs->center2leg) {
    return false;
  }
  // link2link
  if (lhs->link2link != rhs->link2link) {
    return false;
  }
  // init_z_up
  if (lhs->init_z_up != rhs->init_z_up) {
    return false;
  }
  // percentage_of_ik_motor
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->percentage_of_ik_motor), &(rhs->percentage_of_ik_motor)))
  {
    return false;
  }
  // first_pos_xr
  if (lhs->first_pos_xr != rhs->first_pos_xr) {
    return false;
  }
  // first_neg_xr
  if (lhs->first_neg_xr != rhs->first_neg_xr) {
    return false;
  }
  // first_pos_side_r
  if (lhs->first_pos_side_r != rhs->first_pos_side_r) {
    return false;
  }
  // first_neg_side_r
  if (lhs->first_neg_side_r != rhs->first_neg_side_r) {
    return false;
  }
  // first_pos_yaw_r
  if (lhs->first_pos_yaw_r != rhs->first_pos_yaw_r) {
    return false;
  }
  // first_neg_yaw_r
  if (lhs->first_neg_yaw_r != rhs->first_neg_yaw_r) {
    return false;
  }
  // first_pos_xl
  if (lhs->first_pos_xl != rhs->first_pos_xl) {
    return false;
  }
  // first_neg_xl
  if (lhs->first_neg_xl != rhs->first_neg_xl) {
    return false;
  }
  // first_pos_side_l
  if (lhs->first_pos_side_l != rhs->first_pos_side_l) {
    return false;
  }
  // first_neg_side_l
  if (lhs->first_neg_side_l != rhs->first_neg_side_l) {
    return false;
  }
  // first_pos_yaw_l
  if (lhs->first_pos_yaw_l != rhs->first_pos_yaw_l) {
    return false;
  }
  // first_neg_yaw_l
  if (lhs->first_neg_yaw_l != rhs->first_neg_yaw_l) {
    return false;
  }
  // first_pos_side_r_swing_minus
  if (lhs->first_pos_side_r_swing_minus != rhs->first_pos_side_r_swing_minus) {
    return false;
  }
  // first_neg_side_r_swing_minus
  if (lhs->first_neg_side_r_swing_minus != rhs->first_neg_side_r_swing_minus) {
    return false;
  }
  // first_pos_side_l_swing_minus
  if (lhs->first_pos_side_l_swing_minus != rhs->first_pos_side_l_swing_minus) {
    return false;
  }
  // first_neg_side_l_swing_minus
  if (lhs->first_neg_side_l_swing_minus != rhs->first_neg_side_l_swing_minus) {
    return false;
  }
  // first_min
  if (lhs->first_min != rhs->first_min) {
    return false;
  }
  // first_max
  if (lhs->first_max != rhs->first_max) {
    return false;
  }
  // second_pos_xr
  if (lhs->second_pos_xr != rhs->second_pos_xr) {
    return false;
  }
  // second_neg_xr
  if (lhs->second_neg_xr != rhs->second_neg_xr) {
    return false;
  }
  // second_pos_side_r
  if (lhs->second_pos_side_r != rhs->second_pos_side_r) {
    return false;
  }
  // second_neg_side_r
  if (lhs->second_neg_side_r != rhs->second_neg_side_r) {
    return false;
  }
  // second_pos_yaw_r
  if (lhs->second_pos_yaw_r != rhs->second_pos_yaw_r) {
    return false;
  }
  // second_neg_yaw_r
  if (lhs->second_neg_yaw_r != rhs->second_neg_yaw_r) {
    return false;
  }
  // second_pos_xl
  if (lhs->second_pos_xl != rhs->second_pos_xl) {
    return false;
  }
  // second_neg_xl
  if (lhs->second_neg_xl != rhs->second_neg_xl) {
    return false;
  }
  // second_pos_side_l
  if (lhs->second_pos_side_l != rhs->second_pos_side_l) {
    return false;
  }
  // second_neg_side_l
  if (lhs->second_neg_side_l != rhs->second_neg_side_l) {
    return false;
  }
  // second_pos_yaw_l
  if (lhs->second_pos_yaw_l != rhs->second_pos_yaw_l) {
    return false;
  }
  // second_neg_yaw_l
  if (lhs->second_neg_yaw_l != rhs->second_neg_yaw_l) {
    return false;
  }
  // second_pos_side_r_swing_minus
  if (lhs->second_pos_side_r_swing_minus != rhs->second_pos_side_r_swing_minus) {
    return false;
  }
  // second_neg_side_r_swing_minus
  if (lhs->second_neg_side_r_swing_minus != rhs->second_neg_side_r_swing_minus) {
    return false;
  }
  // second_pos_side_l_swing_minus
  if (lhs->second_pos_side_l_swing_minus != rhs->second_pos_side_l_swing_minus) {
    return false;
  }
  // second_neg_side_l_swing_minus
  if (lhs->second_neg_side_l_swing_minus != rhs->second_neg_side_l_swing_minus) {
    return false;
  }
  // second_min
  if (lhs->second_min != rhs->second_min) {
    return false;
  }
  // second_max
  if (lhs->second_max != rhs->second_max) {
    return false;
  }
  // third_pos_xr
  if (lhs->third_pos_xr != rhs->third_pos_xr) {
    return false;
  }
  // third_neg_xr
  if (lhs->third_neg_xr != rhs->third_neg_xr) {
    return false;
  }
  // third_pos_side_r
  if (lhs->third_pos_side_r != rhs->third_pos_side_r) {
    return false;
  }
  // third_neg_side_r
  if (lhs->third_neg_side_r != rhs->third_neg_side_r) {
    return false;
  }
  // third_pos_yaw_r
  if (lhs->third_pos_yaw_r != rhs->third_pos_yaw_r) {
    return false;
  }
  // third_neg_yaw_r
  if (lhs->third_neg_yaw_r != rhs->third_neg_yaw_r) {
    return false;
  }
  // third_pos_xl
  if (lhs->third_pos_xl != rhs->third_pos_xl) {
    return false;
  }
  // third_neg_xl
  if (lhs->third_neg_xl != rhs->third_neg_xl) {
    return false;
  }
  // third_pos_side_l
  if (lhs->third_pos_side_l != rhs->third_pos_side_l) {
    return false;
  }
  // third_neg_side_l
  if (lhs->third_neg_side_l != rhs->third_neg_side_l) {
    return false;
  }
  // third_pos_yaw_l
  if (lhs->third_pos_yaw_l != rhs->third_pos_yaw_l) {
    return false;
  }
  // third_neg_yaw_l
  if (lhs->third_neg_yaw_l != rhs->third_neg_yaw_l) {
    return false;
  }
  // third_pos_side_r_swing_minus
  if (lhs->third_pos_side_r_swing_minus != rhs->third_pos_side_r_swing_minus) {
    return false;
  }
  // third_neg_side_r_swing_minus
  if (lhs->third_neg_side_r_swing_minus != rhs->third_neg_side_r_swing_minus) {
    return false;
  }
  // third_pos_side_l_swing_minus
  if (lhs->third_pos_side_l_swing_minus != rhs->third_pos_side_l_swing_minus) {
    return false;
  }
  // third_neg_side_l_swing_minus
  if (lhs->third_neg_side_l_swing_minus != rhs->third_neg_side_l_swing_minus) {
    return false;
  }
  // third_min
  if (lhs->third_min != rhs->third_min) {
    return false;
  }
  // third_max
  if (lhs->third_max != rhs->third_max) {
    return false;
  }
  // fourth_pos_xr
  if (lhs->fourth_pos_xr != rhs->fourth_pos_xr) {
    return false;
  }
  // fourth_neg_xr
  if (lhs->fourth_neg_xr != rhs->fourth_neg_xr) {
    return false;
  }
  // fourth_pos_side_r
  if (lhs->fourth_pos_side_r != rhs->fourth_pos_side_r) {
    return false;
  }
  // fourth_neg_side_r
  if (lhs->fourth_neg_side_r != rhs->fourth_neg_side_r) {
    return false;
  }
  // fourth_pos_yaw_r
  if (lhs->fourth_pos_yaw_r != rhs->fourth_pos_yaw_r) {
    return false;
  }
  // fourth_neg_yaw_r
  if (lhs->fourth_neg_yaw_r != rhs->fourth_neg_yaw_r) {
    return false;
  }
  // fourth_pos_xl
  if (lhs->fourth_pos_xl != rhs->fourth_pos_xl) {
    return false;
  }
  // fourth_neg_xl
  if (lhs->fourth_neg_xl != rhs->fourth_neg_xl) {
    return false;
  }
  // fourth_pos_side_l
  if (lhs->fourth_pos_side_l != rhs->fourth_pos_side_l) {
    return false;
  }
  // fourth_neg_side_l
  if (lhs->fourth_neg_side_l != rhs->fourth_neg_side_l) {
    return false;
  }
  // fourth_pos_yaw_l
  if (lhs->fourth_pos_yaw_l != rhs->fourth_pos_yaw_l) {
    return false;
  }
  // fourth_neg_yaw_l
  if (lhs->fourth_neg_yaw_l != rhs->fourth_neg_yaw_l) {
    return false;
  }
  // fourth_pos_side_r_swing_minus
  if (lhs->fourth_pos_side_r_swing_minus != rhs->fourth_pos_side_r_swing_minus) {
    return false;
  }
  // fourth_neg_side_r_swing_minus
  if (lhs->fourth_neg_side_r_swing_minus != rhs->fourth_neg_side_r_swing_minus) {
    return false;
  }
  // fourth_pos_side_l_swing_minus
  if (lhs->fourth_pos_side_l_swing_minus != rhs->fourth_pos_side_l_swing_minus) {
    return false;
  }
  // fourth_neg_side_l_swing_minus
  if (lhs->fourth_neg_side_l_swing_minus != rhs->fourth_neg_side_l_swing_minus) {
    return false;
  }
  // fourth_min
  if (lhs->fourth_min != rhs->fourth_min) {
    return false;
  }
  // fourth_max
  if (lhs->fourth_max != rhs->fourth_max) {
    return false;
  }
  // landing_time_control_flag
  if (lhs->landing_time_control_flag != rhs->landing_time_control_flag) {
    return false;
  }
  // control_system_flag
  if (lhs->control_system_flag != rhs->control_system_flag) {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__Tune2IkMsg__copy(
  const humanoid_interfaces__msg__Tune2IkMsg * input,
  humanoid_interfaces__msg__Tune2IkMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // ik_flag
  output->ik_flag = input->ik_flag;
  // entire_time
  output->entire_time = input->entire_time;
  // frequency
  output->frequency = input->frequency;
  // default_x_left
  output->default_x_left = input->default_x_left;
  // default_y_left
  output->default_y_left = input->default_y_left;
  // default_z_left
  output->default_z_left = input->default_z_left;
  // default_x_right
  output->default_x_right = input->default_x_right;
  // default_y_right
  output->default_y_right = input->default_y_right;
  // default_z_right
  output->default_z_right = input->default_z_right;
  // offset_motor
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->offset_motor), &(output->offset_motor)))
  {
    return false;
  }
  // swing_shoulder_left
  output->swing_shoulder_left = input->swing_shoulder_left;
  // swing_shoulder_right
  output->swing_shoulder_right = input->swing_shoulder_right;
  // swing_leg_left
  output->swing_leg_left = input->swing_leg_left;
  // swing_leg_right
  output->swing_leg_right = input->swing_leg_right;
  // rise_leg_left
  output->rise_leg_left = input->rise_leg_left;
  // rise_leg_right
  output->rise_leg_right = input->rise_leg_right;
  // start_entire_time
  output->start_entire_time = input->start_entire_time;
  // start_swing
  output->start_swing = input->start_swing;
  // start_rise
  output->start_rise = input->start_rise;
  // end_entire_time
  output->end_entire_time = input->end_entire_time;
  // end_swing
  output->end_swing = input->end_swing;
  // end_rise
  output->end_rise = input->end_rise;
  // test_x
  output->test_x = input->test_x;
  // test_side
  output->test_side = input->test_side;
  // test_yaw
  output->test_yaw = input->test_yaw;
  // tuning_x
  output->tuning_x = input->tuning_x;
  // tuning_side
  output->tuning_side = input->tuning_side;
  // tuning_yaw
  output->tuning_yaw = input->tuning_yaw;
  // balance_value_0
  output->balance_value_0 = input->balance_value_0;
  // balance_value_1
  output->balance_value_1 = input->balance_value_1;
  // balance_value_2
  output->balance_value_2 = input->balance_value_2;
  // balance_value_3
  output->balance_value_3 = input->balance_value_3;
  // balance_pitch_gp
  output->balance_pitch_gp = input->balance_pitch_gp;
  // balance_pitch_gi
  output->balance_pitch_gi = input->balance_pitch_gi;
  // balance_pitch_gd
  output->balance_pitch_gd = input->balance_pitch_gd;
  // balance_pitch_elimit
  output->balance_pitch_elimit = input->balance_pitch_elimit;
  // balance_pitch_olimit
  output->balance_pitch_olimit = input->balance_pitch_olimit;
  // balance_pitch_neg_target
  output->balance_pitch_neg_target = input->balance_pitch_neg_target;
  // balance_pitch_pos_target
  output->balance_pitch_pos_target = input->balance_pitch_pos_target;
  // balance_roll_gp
  output->balance_roll_gp = input->balance_roll_gp;
  // balance_roll_gi
  output->balance_roll_gi = input->balance_roll_gi;
  // balance_roll_gd
  output->balance_roll_gd = input->balance_roll_gd;
  // balance_roll_elimit
  output->balance_roll_elimit = input->balance_roll_elimit;
  // balance_roll_olimit
  output->balance_roll_olimit = input->balance_roll_olimit;
  // balance_roll_neg_target
  output->balance_roll_neg_target = input->balance_roll_neg_target;
  // balance_roll_pos_target
  output->balance_roll_pos_target = input->balance_roll_pos_target;
  // balance_angle_pitch_gp
  output->balance_angle_pitch_gp = input->balance_angle_pitch_gp;
  // balance_angle_pitch_gi
  output->balance_angle_pitch_gi = input->balance_angle_pitch_gi;
  // balance_angle_pitch_gd
  output->balance_angle_pitch_gd = input->balance_angle_pitch_gd;
  // balance_angle_pitch_elimit
  output->balance_angle_pitch_elimit = input->balance_angle_pitch_elimit;
  // balance_angle_pitch_olimit
  output->balance_angle_pitch_olimit = input->balance_angle_pitch_olimit;
  // balance_angle_pitch_neg_target
  output->balance_angle_pitch_neg_target = input->balance_angle_pitch_neg_target;
  // balance_angle_pitch_pos_target
  output->balance_angle_pitch_pos_target = input->balance_angle_pitch_pos_target;
  // balance_value_4
  output->balance_value_4 = input->balance_value_4;
  // balance_value_5
  output->balance_value_5 = input->balance_value_5;
  // balance_pitch_flag_imu
  output->balance_pitch_flag_imu = input->balance_pitch_flag_imu;
  // balance_pitch_gp_imu
  output->balance_pitch_gp_imu = input->balance_pitch_gp_imu;
  // balance_pitch_gi_imu
  output->balance_pitch_gi_imu = input->balance_pitch_gi_imu;
  // balance_pitch_gd_imu
  output->balance_pitch_gd_imu = input->balance_pitch_gd_imu;
  // balance_pitch_neg_target_imu
  output->balance_pitch_neg_target_imu = input->balance_pitch_neg_target_imu;
  // balance_pitch_pos_target_imu
  output->balance_pitch_pos_target_imu = input->balance_pitch_pos_target_imu;
  // balance_pitch_elimit_imu
  output->balance_pitch_elimit_imu = input->balance_pitch_elimit_imu;
  // balance_pitch_olimit_imu
  output->balance_pitch_olimit_imu = input->balance_pitch_olimit_imu;
  // balance_roll_flag_imu
  output->balance_roll_flag_imu = input->balance_roll_flag_imu;
  // balance_roll_gp_imu
  output->balance_roll_gp_imu = input->balance_roll_gp_imu;
  // balance_roll_gi_imu
  output->balance_roll_gi_imu = input->balance_roll_gi_imu;
  // balance_roll_gd_imu
  output->balance_roll_gd_imu = input->balance_roll_gd_imu;
  // balance_roll_neg_target_imu
  output->balance_roll_neg_target_imu = input->balance_roll_neg_target_imu;
  // balance_roll_pos_target_imu
  output->balance_roll_pos_target_imu = input->balance_roll_pos_target_imu;
  // balance_roll_elimit_imu
  output->balance_roll_elimit_imu = input->balance_roll_elimit_imu;
  // balance_roll_olimit_imu
  output->balance_roll_olimit_imu = input->balance_roll_olimit_imu;
  // balance_pitch_flag
  output->balance_pitch_flag = input->balance_pitch_flag;
  // balance_ankle_pitch_flag
  output->balance_ankle_pitch_flag = input->balance_ankle_pitch_flag;
  // balance_roll_flag
  output->balance_roll_flag = input->balance_roll_flag;
  // ratio_check_flag
  output->ratio_check_flag = input->ratio_check_flag;
  // center2leg
  output->center2leg = input->center2leg;
  // link2link
  output->link2link = input->link2link;
  // init_z_up
  output->init_z_up = input->init_z_up;
  // percentage_of_ik_motor
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->percentage_of_ik_motor), &(output->percentage_of_ik_motor)))
  {
    return false;
  }
  // first_pos_xr
  output->first_pos_xr = input->first_pos_xr;
  // first_neg_xr
  output->first_neg_xr = input->first_neg_xr;
  // first_pos_side_r
  output->first_pos_side_r = input->first_pos_side_r;
  // first_neg_side_r
  output->first_neg_side_r = input->first_neg_side_r;
  // first_pos_yaw_r
  output->first_pos_yaw_r = input->first_pos_yaw_r;
  // first_neg_yaw_r
  output->first_neg_yaw_r = input->first_neg_yaw_r;
  // first_pos_xl
  output->first_pos_xl = input->first_pos_xl;
  // first_neg_xl
  output->first_neg_xl = input->first_neg_xl;
  // first_pos_side_l
  output->first_pos_side_l = input->first_pos_side_l;
  // first_neg_side_l
  output->first_neg_side_l = input->first_neg_side_l;
  // first_pos_yaw_l
  output->first_pos_yaw_l = input->first_pos_yaw_l;
  // first_neg_yaw_l
  output->first_neg_yaw_l = input->first_neg_yaw_l;
  // first_pos_side_r_swing_minus
  output->first_pos_side_r_swing_minus = input->first_pos_side_r_swing_minus;
  // first_neg_side_r_swing_minus
  output->first_neg_side_r_swing_minus = input->first_neg_side_r_swing_minus;
  // first_pos_side_l_swing_minus
  output->first_pos_side_l_swing_minus = input->first_pos_side_l_swing_minus;
  // first_neg_side_l_swing_minus
  output->first_neg_side_l_swing_minus = input->first_neg_side_l_swing_minus;
  // first_min
  output->first_min = input->first_min;
  // first_max
  output->first_max = input->first_max;
  // second_pos_xr
  output->second_pos_xr = input->second_pos_xr;
  // second_neg_xr
  output->second_neg_xr = input->second_neg_xr;
  // second_pos_side_r
  output->second_pos_side_r = input->second_pos_side_r;
  // second_neg_side_r
  output->second_neg_side_r = input->second_neg_side_r;
  // second_pos_yaw_r
  output->second_pos_yaw_r = input->second_pos_yaw_r;
  // second_neg_yaw_r
  output->second_neg_yaw_r = input->second_neg_yaw_r;
  // second_pos_xl
  output->second_pos_xl = input->second_pos_xl;
  // second_neg_xl
  output->second_neg_xl = input->second_neg_xl;
  // second_pos_side_l
  output->second_pos_side_l = input->second_pos_side_l;
  // second_neg_side_l
  output->second_neg_side_l = input->second_neg_side_l;
  // second_pos_yaw_l
  output->second_pos_yaw_l = input->second_pos_yaw_l;
  // second_neg_yaw_l
  output->second_neg_yaw_l = input->second_neg_yaw_l;
  // second_pos_side_r_swing_minus
  output->second_pos_side_r_swing_minus = input->second_pos_side_r_swing_minus;
  // second_neg_side_r_swing_minus
  output->second_neg_side_r_swing_minus = input->second_neg_side_r_swing_minus;
  // second_pos_side_l_swing_minus
  output->second_pos_side_l_swing_minus = input->second_pos_side_l_swing_minus;
  // second_neg_side_l_swing_minus
  output->second_neg_side_l_swing_minus = input->second_neg_side_l_swing_minus;
  // second_min
  output->second_min = input->second_min;
  // second_max
  output->second_max = input->second_max;
  // third_pos_xr
  output->third_pos_xr = input->third_pos_xr;
  // third_neg_xr
  output->third_neg_xr = input->third_neg_xr;
  // third_pos_side_r
  output->third_pos_side_r = input->third_pos_side_r;
  // third_neg_side_r
  output->third_neg_side_r = input->third_neg_side_r;
  // third_pos_yaw_r
  output->third_pos_yaw_r = input->third_pos_yaw_r;
  // third_neg_yaw_r
  output->third_neg_yaw_r = input->third_neg_yaw_r;
  // third_pos_xl
  output->third_pos_xl = input->third_pos_xl;
  // third_neg_xl
  output->third_neg_xl = input->third_neg_xl;
  // third_pos_side_l
  output->third_pos_side_l = input->third_pos_side_l;
  // third_neg_side_l
  output->third_neg_side_l = input->third_neg_side_l;
  // third_pos_yaw_l
  output->third_pos_yaw_l = input->third_pos_yaw_l;
  // third_neg_yaw_l
  output->third_neg_yaw_l = input->third_neg_yaw_l;
  // third_pos_side_r_swing_minus
  output->third_pos_side_r_swing_minus = input->third_pos_side_r_swing_minus;
  // third_neg_side_r_swing_minus
  output->third_neg_side_r_swing_minus = input->third_neg_side_r_swing_minus;
  // third_pos_side_l_swing_minus
  output->third_pos_side_l_swing_minus = input->third_pos_side_l_swing_minus;
  // third_neg_side_l_swing_minus
  output->third_neg_side_l_swing_minus = input->third_neg_side_l_swing_minus;
  // third_min
  output->third_min = input->third_min;
  // third_max
  output->third_max = input->third_max;
  // fourth_pos_xr
  output->fourth_pos_xr = input->fourth_pos_xr;
  // fourth_neg_xr
  output->fourth_neg_xr = input->fourth_neg_xr;
  // fourth_pos_side_r
  output->fourth_pos_side_r = input->fourth_pos_side_r;
  // fourth_neg_side_r
  output->fourth_neg_side_r = input->fourth_neg_side_r;
  // fourth_pos_yaw_r
  output->fourth_pos_yaw_r = input->fourth_pos_yaw_r;
  // fourth_neg_yaw_r
  output->fourth_neg_yaw_r = input->fourth_neg_yaw_r;
  // fourth_pos_xl
  output->fourth_pos_xl = input->fourth_pos_xl;
  // fourth_neg_xl
  output->fourth_neg_xl = input->fourth_neg_xl;
  // fourth_pos_side_l
  output->fourth_pos_side_l = input->fourth_pos_side_l;
  // fourth_neg_side_l
  output->fourth_neg_side_l = input->fourth_neg_side_l;
  // fourth_pos_yaw_l
  output->fourth_pos_yaw_l = input->fourth_pos_yaw_l;
  // fourth_neg_yaw_l
  output->fourth_neg_yaw_l = input->fourth_neg_yaw_l;
  // fourth_pos_side_r_swing_minus
  output->fourth_pos_side_r_swing_minus = input->fourth_pos_side_r_swing_minus;
  // fourth_neg_side_r_swing_minus
  output->fourth_neg_side_r_swing_minus = input->fourth_neg_side_r_swing_minus;
  // fourth_pos_side_l_swing_minus
  output->fourth_pos_side_l_swing_minus = input->fourth_pos_side_l_swing_minus;
  // fourth_neg_side_l_swing_minus
  output->fourth_neg_side_l_swing_minus = input->fourth_neg_side_l_swing_minus;
  // fourth_min
  output->fourth_min = input->fourth_min;
  // fourth_max
  output->fourth_max = input->fourth_max;
  // landing_time_control_flag
  output->landing_time_control_flag = input->landing_time_control_flag;
  // control_system_flag
  output->control_system_flag = input->control_system_flag;
  return true;
}

humanoid_interfaces__msg__Tune2IkMsg *
humanoid_interfaces__msg__Tune2IkMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Tune2IkMsg * msg = (humanoid_interfaces__msg__Tune2IkMsg *)allocator.allocate(sizeof(humanoid_interfaces__msg__Tune2IkMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__Tune2IkMsg));
  bool success = humanoid_interfaces__msg__Tune2IkMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__Tune2IkMsg__destroy(humanoid_interfaces__msg__Tune2IkMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__Tune2IkMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__Tune2IkMsg__Sequence__init(humanoid_interfaces__msg__Tune2IkMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Tune2IkMsg * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__Tune2IkMsg *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__Tune2IkMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__Tune2IkMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__Tune2IkMsg__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
humanoid_interfaces__msg__Tune2IkMsg__Sequence__fini(humanoid_interfaces__msg__Tune2IkMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      humanoid_interfaces__msg__Tune2IkMsg__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

humanoid_interfaces__msg__Tune2IkMsg__Sequence *
humanoid_interfaces__msg__Tune2IkMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Tune2IkMsg__Sequence * array = (humanoid_interfaces__msg__Tune2IkMsg__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__Tune2IkMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__Tune2IkMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__Tune2IkMsg__Sequence__destroy(humanoid_interfaces__msg__Tune2IkMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__Tune2IkMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__Tune2IkMsg__Sequence__are_equal(const humanoid_interfaces__msg__Tune2IkMsg__Sequence * lhs, const humanoid_interfaces__msg__Tune2IkMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__Tune2IkMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__Tune2IkMsg__Sequence__copy(
  const humanoid_interfaces__msg__Tune2IkMsg__Sequence * input,
  humanoid_interfaces__msg__Tune2IkMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__Tune2IkMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__Tune2IkMsg * data =
      (humanoid_interfaces__msg__Tune2IkMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__Tune2IkMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__Tune2IkMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__Tune2IkMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
