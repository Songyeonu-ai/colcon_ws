// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from humanoid_interfaces:msg/Tune2IkMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/tune2_ik_msg__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "humanoid_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "humanoid_interfaces/msg/detail/tune2_ik_msg__struct.h"
#include "humanoid_interfaces/msg/detail/tune2_ik_msg__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // offset_motor, percentage_of_ik_motor
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // offset_motor, percentage_of_ik_motor

// forward declare type support functions


using _Tune2IkMsg__ros_msg_type = humanoid_interfaces__msg__Tune2IkMsg;

static bool _Tune2IkMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Tune2IkMsg__ros_msg_type * ros_message = static_cast<const _Tune2IkMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: ik_flag
  {
    cdr << (ros_message->ik_flag ? true : false);
  }

  // Field name: entire_time
  {
    cdr << ros_message->entire_time;
  }

  // Field name: frequency
  {
    cdr << ros_message->frequency;
  }

  // Field name: default_x_left
  {
    cdr << ros_message->default_x_left;
  }

  // Field name: default_y_left
  {
    cdr << ros_message->default_y_left;
  }

  // Field name: default_z_left
  {
    cdr << ros_message->default_z_left;
  }

  // Field name: default_x_right
  {
    cdr << ros_message->default_x_right;
  }

  // Field name: default_y_right
  {
    cdr << ros_message->default_y_right;
  }

  // Field name: default_z_right
  {
    cdr << ros_message->default_z_right;
  }

  // Field name: offset_motor
  {
    size_t size = ros_message->offset_motor.size;
    auto array_ptr = ros_message->offset_motor.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: swing_shoulder_left
  {
    cdr << ros_message->swing_shoulder_left;
  }

  // Field name: swing_shoulder_right
  {
    cdr << ros_message->swing_shoulder_right;
  }

  // Field name: swing_leg_left
  {
    cdr << ros_message->swing_leg_left;
  }

  // Field name: swing_leg_right
  {
    cdr << ros_message->swing_leg_right;
  }

  // Field name: rise_leg_left
  {
    cdr << ros_message->rise_leg_left;
  }

  // Field name: rise_leg_right
  {
    cdr << ros_message->rise_leg_right;
  }

  // Field name: start_entire_time
  {
    cdr << ros_message->start_entire_time;
  }

  // Field name: start_swing
  {
    cdr << ros_message->start_swing;
  }

  // Field name: start_rise
  {
    cdr << ros_message->start_rise;
  }

  // Field name: end_entire_time
  {
    cdr << ros_message->end_entire_time;
  }

  // Field name: end_swing
  {
    cdr << ros_message->end_swing;
  }

  // Field name: end_rise
  {
    cdr << ros_message->end_rise;
  }

  // Field name: test_x
  {
    cdr << ros_message->test_x;
  }

  // Field name: test_side
  {
    cdr << ros_message->test_side;
  }

  // Field name: test_yaw
  {
    cdr << ros_message->test_yaw;
  }

  // Field name: tuning_x
  {
    cdr << ros_message->tuning_x;
  }

  // Field name: tuning_side
  {
    cdr << ros_message->tuning_side;
  }

  // Field name: tuning_yaw
  {
    cdr << ros_message->tuning_yaw;
  }

  // Field name: balance_value_0
  {
    cdr << ros_message->balance_value_0;
  }

  // Field name: balance_value_1
  {
    cdr << ros_message->balance_value_1;
  }

  // Field name: balance_value_2
  {
    cdr << ros_message->balance_value_2;
  }

  // Field name: balance_value_3
  {
    cdr << ros_message->balance_value_3;
  }

  // Field name: balance_pitch_gp
  {
    cdr << ros_message->balance_pitch_gp;
  }

  // Field name: balance_pitch_gi
  {
    cdr << ros_message->balance_pitch_gi;
  }

  // Field name: balance_pitch_gd
  {
    cdr << ros_message->balance_pitch_gd;
  }

  // Field name: balance_pitch_elimit
  {
    cdr << ros_message->balance_pitch_elimit;
  }

  // Field name: balance_pitch_olimit
  {
    cdr << ros_message->balance_pitch_olimit;
  }

  // Field name: balance_pitch_neg_target
  {
    cdr << ros_message->balance_pitch_neg_target;
  }

  // Field name: balance_pitch_pos_target
  {
    cdr << ros_message->balance_pitch_pos_target;
  }

  // Field name: balance_roll_gp
  {
    cdr << ros_message->balance_roll_gp;
  }

  // Field name: balance_roll_gi
  {
    cdr << ros_message->balance_roll_gi;
  }

  // Field name: balance_roll_gd
  {
    cdr << ros_message->balance_roll_gd;
  }

  // Field name: balance_roll_elimit
  {
    cdr << ros_message->balance_roll_elimit;
  }

  // Field name: balance_roll_olimit
  {
    cdr << ros_message->balance_roll_olimit;
  }

  // Field name: balance_roll_neg_target
  {
    cdr << ros_message->balance_roll_neg_target;
  }

  // Field name: balance_roll_pos_target
  {
    cdr << ros_message->balance_roll_pos_target;
  }

  // Field name: balance_angle_pitch_gp
  {
    cdr << ros_message->balance_angle_pitch_gp;
  }

  // Field name: balance_angle_pitch_gi
  {
    cdr << ros_message->balance_angle_pitch_gi;
  }

  // Field name: balance_angle_pitch_gd
  {
    cdr << ros_message->balance_angle_pitch_gd;
  }

  // Field name: balance_angle_pitch_elimit
  {
    cdr << ros_message->balance_angle_pitch_elimit;
  }

  // Field name: balance_angle_pitch_olimit
  {
    cdr << ros_message->balance_angle_pitch_olimit;
  }

  // Field name: balance_angle_pitch_neg_target
  {
    cdr << ros_message->balance_angle_pitch_neg_target;
  }

  // Field name: balance_angle_pitch_pos_target
  {
    cdr << ros_message->balance_angle_pitch_pos_target;
  }

  // Field name: balance_value_4
  {
    cdr << ros_message->balance_value_4;
  }

  // Field name: balance_value_5
  {
    cdr << ros_message->balance_value_5;
  }

  // Field name: balance_pitch_flag_imu
  {
    cdr << ros_message->balance_pitch_flag_imu;
  }

  // Field name: balance_pitch_gp_imu
  {
    cdr << ros_message->balance_pitch_gp_imu;
  }

  // Field name: balance_pitch_gi_imu
  {
    cdr << ros_message->balance_pitch_gi_imu;
  }

  // Field name: balance_pitch_gd_imu
  {
    cdr << ros_message->balance_pitch_gd_imu;
  }

  // Field name: balance_pitch_neg_target_imu
  {
    cdr << ros_message->balance_pitch_neg_target_imu;
  }

  // Field name: balance_pitch_pos_target_imu
  {
    cdr << ros_message->balance_pitch_pos_target_imu;
  }

  // Field name: balance_pitch_elimit_imu
  {
    cdr << ros_message->balance_pitch_elimit_imu;
  }

  // Field name: balance_pitch_olimit_imu
  {
    cdr << ros_message->balance_pitch_olimit_imu;
  }

  // Field name: balance_roll_flag_imu
  {
    cdr << ros_message->balance_roll_flag_imu;
  }

  // Field name: balance_roll_gp_imu
  {
    cdr << ros_message->balance_roll_gp_imu;
  }

  // Field name: balance_roll_gi_imu
  {
    cdr << ros_message->balance_roll_gi_imu;
  }

  // Field name: balance_roll_gd_imu
  {
    cdr << ros_message->balance_roll_gd_imu;
  }

  // Field name: balance_roll_neg_target_imu
  {
    cdr << ros_message->balance_roll_neg_target_imu;
  }

  // Field name: balance_roll_pos_target_imu
  {
    cdr << ros_message->balance_roll_pos_target_imu;
  }

  // Field name: balance_roll_elimit_imu
  {
    cdr << ros_message->balance_roll_elimit_imu;
  }

  // Field name: balance_roll_olimit_imu
  {
    cdr << ros_message->balance_roll_olimit_imu;
  }

  // Field name: balance_pitch_flag
  {
    cdr << ros_message->balance_pitch_flag;
  }

  // Field name: balance_ankle_pitch_flag
  {
    cdr << ros_message->balance_ankle_pitch_flag;
  }

  // Field name: balance_roll_flag
  {
    cdr << ros_message->balance_roll_flag;
  }

  // Field name: ratio_check_flag
  {
    cdr << ros_message->ratio_check_flag;
  }

  // Field name: center2leg
  {
    cdr << ros_message->center2leg;
  }

  // Field name: link2link
  {
    cdr << ros_message->link2link;
  }

  // Field name: init_z_up
  {
    cdr << ros_message->init_z_up;
  }

  // Field name: percentage_of_ik_motor
  {
    size_t size = ros_message->percentage_of_ik_motor.size;
    auto array_ptr = ros_message->percentage_of_ik_motor.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: first_pos_xr
  {
    cdr << ros_message->first_pos_xr;
  }

  // Field name: first_neg_xr
  {
    cdr << ros_message->first_neg_xr;
  }

  // Field name: first_pos_side_r
  {
    cdr << ros_message->first_pos_side_r;
  }

  // Field name: first_neg_side_r
  {
    cdr << ros_message->first_neg_side_r;
  }

  // Field name: first_pos_yaw_r
  {
    cdr << ros_message->first_pos_yaw_r;
  }

  // Field name: first_neg_yaw_r
  {
    cdr << ros_message->first_neg_yaw_r;
  }

  // Field name: first_pos_xl
  {
    cdr << ros_message->first_pos_xl;
  }

  // Field name: first_neg_xl
  {
    cdr << ros_message->first_neg_xl;
  }

  // Field name: first_pos_side_l
  {
    cdr << ros_message->first_pos_side_l;
  }

  // Field name: first_neg_side_l
  {
    cdr << ros_message->first_neg_side_l;
  }

  // Field name: first_pos_yaw_l
  {
    cdr << ros_message->first_pos_yaw_l;
  }

  // Field name: first_neg_yaw_l
  {
    cdr << ros_message->first_neg_yaw_l;
  }

  // Field name: first_pos_side_r_swing_minus
  {
    cdr << ros_message->first_pos_side_r_swing_minus;
  }

  // Field name: first_neg_side_r_swing_minus
  {
    cdr << ros_message->first_neg_side_r_swing_minus;
  }

  // Field name: first_pos_side_l_swing_minus
  {
    cdr << ros_message->first_pos_side_l_swing_minus;
  }

  // Field name: first_neg_side_l_swing_minus
  {
    cdr << ros_message->first_neg_side_l_swing_minus;
  }

  // Field name: first_min
  {
    cdr << ros_message->first_min;
  }

  // Field name: first_max
  {
    cdr << ros_message->first_max;
  }

  // Field name: second_pos_xr
  {
    cdr << ros_message->second_pos_xr;
  }

  // Field name: second_neg_xr
  {
    cdr << ros_message->second_neg_xr;
  }

  // Field name: second_pos_side_r
  {
    cdr << ros_message->second_pos_side_r;
  }

  // Field name: second_neg_side_r
  {
    cdr << ros_message->second_neg_side_r;
  }

  // Field name: second_pos_yaw_r
  {
    cdr << ros_message->second_pos_yaw_r;
  }

  // Field name: second_neg_yaw_r
  {
    cdr << ros_message->second_neg_yaw_r;
  }

  // Field name: second_pos_xl
  {
    cdr << ros_message->second_pos_xl;
  }

  // Field name: second_neg_xl
  {
    cdr << ros_message->second_neg_xl;
  }

  // Field name: second_pos_side_l
  {
    cdr << ros_message->second_pos_side_l;
  }

  // Field name: second_neg_side_l
  {
    cdr << ros_message->second_neg_side_l;
  }

  // Field name: second_pos_yaw_l
  {
    cdr << ros_message->second_pos_yaw_l;
  }

  // Field name: second_neg_yaw_l
  {
    cdr << ros_message->second_neg_yaw_l;
  }

  // Field name: second_pos_side_r_swing_minus
  {
    cdr << ros_message->second_pos_side_r_swing_minus;
  }

  // Field name: second_neg_side_r_swing_minus
  {
    cdr << ros_message->second_neg_side_r_swing_minus;
  }

  // Field name: second_pos_side_l_swing_minus
  {
    cdr << ros_message->second_pos_side_l_swing_minus;
  }

  // Field name: second_neg_side_l_swing_minus
  {
    cdr << ros_message->second_neg_side_l_swing_minus;
  }

  // Field name: second_min
  {
    cdr << ros_message->second_min;
  }

  // Field name: second_max
  {
    cdr << ros_message->second_max;
  }

  // Field name: third_pos_xr
  {
    cdr << ros_message->third_pos_xr;
  }

  // Field name: third_neg_xr
  {
    cdr << ros_message->third_neg_xr;
  }

  // Field name: third_pos_side_r
  {
    cdr << ros_message->third_pos_side_r;
  }

  // Field name: third_neg_side_r
  {
    cdr << ros_message->third_neg_side_r;
  }

  // Field name: third_pos_yaw_r
  {
    cdr << ros_message->third_pos_yaw_r;
  }

  // Field name: third_neg_yaw_r
  {
    cdr << ros_message->third_neg_yaw_r;
  }

  // Field name: third_pos_xl
  {
    cdr << ros_message->third_pos_xl;
  }

  // Field name: third_neg_xl
  {
    cdr << ros_message->third_neg_xl;
  }

  // Field name: third_pos_side_l
  {
    cdr << ros_message->third_pos_side_l;
  }

  // Field name: third_neg_side_l
  {
    cdr << ros_message->third_neg_side_l;
  }

  // Field name: third_pos_yaw_l
  {
    cdr << ros_message->third_pos_yaw_l;
  }

  // Field name: third_neg_yaw_l
  {
    cdr << ros_message->third_neg_yaw_l;
  }

  // Field name: third_pos_side_r_swing_minus
  {
    cdr << ros_message->third_pos_side_r_swing_minus;
  }

  // Field name: third_neg_side_r_swing_minus
  {
    cdr << ros_message->third_neg_side_r_swing_minus;
  }

  // Field name: third_pos_side_l_swing_minus
  {
    cdr << ros_message->third_pos_side_l_swing_minus;
  }

  // Field name: third_neg_side_l_swing_minus
  {
    cdr << ros_message->third_neg_side_l_swing_minus;
  }

  // Field name: third_min
  {
    cdr << ros_message->third_min;
  }

  // Field name: third_max
  {
    cdr << ros_message->third_max;
  }

  // Field name: fourth_pos_xr
  {
    cdr << ros_message->fourth_pos_xr;
  }

  // Field name: fourth_neg_xr
  {
    cdr << ros_message->fourth_neg_xr;
  }

  // Field name: fourth_pos_side_r
  {
    cdr << ros_message->fourth_pos_side_r;
  }

  // Field name: fourth_neg_side_r
  {
    cdr << ros_message->fourth_neg_side_r;
  }

  // Field name: fourth_pos_yaw_r
  {
    cdr << ros_message->fourth_pos_yaw_r;
  }

  // Field name: fourth_neg_yaw_r
  {
    cdr << ros_message->fourth_neg_yaw_r;
  }

  // Field name: fourth_pos_xl
  {
    cdr << ros_message->fourth_pos_xl;
  }

  // Field name: fourth_neg_xl
  {
    cdr << ros_message->fourth_neg_xl;
  }

  // Field name: fourth_pos_side_l
  {
    cdr << ros_message->fourth_pos_side_l;
  }

  // Field name: fourth_neg_side_l
  {
    cdr << ros_message->fourth_neg_side_l;
  }

  // Field name: fourth_pos_yaw_l
  {
    cdr << ros_message->fourth_pos_yaw_l;
  }

  // Field name: fourth_neg_yaw_l
  {
    cdr << ros_message->fourth_neg_yaw_l;
  }

  // Field name: fourth_pos_side_r_swing_minus
  {
    cdr << ros_message->fourth_pos_side_r_swing_minus;
  }

  // Field name: fourth_neg_side_r_swing_minus
  {
    cdr << ros_message->fourth_neg_side_r_swing_minus;
  }

  // Field name: fourth_pos_side_l_swing_minus
  {
    cdr << ros_message->fourth_pos_side_l_swing_minus;
  }

  // Field name: fourth_neg_side_l_swing_minus
  {
    cdr << ros_message->fourth_neg_side_l_swing_minus;
  }

  // Field name: fourth_min
  {
    cdr << ros_message->fourth_min;
  }

  // Field name: fourth_max
  {
    cdr << ros_message->fourth_max;
  }

  // Field name: landing_time_control_flag
  {
    cdr << ros_message->landing_time_control_flag;
  }

  // Field name: control_system_flag
  {
    cdr << ros_message->control_system_flag;
  }

  return true;
}

static bool _Tune2IkMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Tune2IkMsg__ros_msg_type * ros_message = static_cast<_Tune2IkMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: ik_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->ik_flag = tmp ? true : false;
  }

  // Field name: entire_time
  {
    cdr >> ros_message->entire_time;
  }

  // Field name: frequency
  {
    cdr >> ros_message->frequency;
  }

  // Field name: default_x_left
  {
    cdr >> ros_message->default_x_left;
  }

  // Field name: default_y_left
  {
    cdr >> ros_message->default_y_left;
  }

  // Field name: default_z_left
  {
    cdr >> ros_message->default_z_left;
  }

  // Field name: default_x_right
  {
    cdr >> ros_message->default_x_right;
  }

  // Field name: default_y_right
  {
    cdr >> ros_message->default_y_right;
  }

  // Field name: default_z_right
  {
    cdr >> ros_message->default_z_right;
  }

  // Field name: offset_motor
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->offset_motor.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->offset_motor);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->offset_motor, size)) {
      fprintf(stderr, "failed to create array for field 'offset_motor'");
      return false;
    }
    auto array_ptr = ros_message->offset_motor.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: swing_shoulder_left
  {
    cdr >> ros_message->swing_shoulder_left;
  }

  // Field name: swing_shoulder_right
  {
    cdr >> ros_message->swing_shoulder_right;
  }

  // Field name: swing_leg_left
  {
    cdr >> ros_message->swing_leg_left;
  }

  // Field name: swing_leg_right
  {
    cdr >> ros_message->swing_leg_right;
  }

  // Field name: rise_leg_left
  {
    cdr >> ros_message->rise_leg_left;
  }

  // Field name: rise_leg_right
  {
    cdr >> ros_message->rise_leg_right;
  }

  // Field name: start_entire_time
  {
    cdr >> ros_message->start_entire_time;
  }

  // Field name: start_swing
  {
    cdr >> ros_message->start_swing;
  }

  // Field name: start_rise
  {
    cdr >> ros_message->start_rise;
  }

  // Field name: end_entire_time
  {
    cdr >> ros_message->end_entire_time;
  }

  // Field name: end_swing
  {
    cdr >> ros_message->end_swing;
  }

  // Field name: end_rise
  {
    cdr >> ros_message->end_rise;
  }

  // Field name: test_x
  {
    cdr >> ros_message->test_x;
  }

  // Field name: test_side
  {
    cdr >> ros_message->test_side;
  }

  // Field name: test_yaw
  {
    cdr >> ros_message->test_yaw;
  }

  // Field name: tuning_x
  {
    cdr >> ros_message->tuning_x;
  }

  // Field name: tuning_side
  {
    cdr >> ros_message->tuning_side;
  }

  // Field name: tuning_yaw
  {
    cdr >> ros_message->tuning_yaw;
  }

  // Field name: balance_value_0
  {
    cdr >> ros_message->balance_value_0;
  }

  // Field name: balance_value_1
  {
    cdr >> ros_message->balance_value_1;
  }

  // Field name: balance_value_2
  {
    cdr >> ros_message->balance_value_2;
  }

  // Field name: balance_value_3
  {
    cdr >> ros_message->balance_value_3;
  }

  // Field name: balance_pitch_gp
  {
    cdr >> ros_message->balance_pitch_gp;
  }

  // Field name: balance_pitch_gi
  {
    cdr >> ros_message->balance_pitch_gi;
  }

  // Field name: balance_pitch_gd
  {
    cdr >> ros_message->balance_pitch_gd;
  }

  // Field name: balance_pitch_elimit
  {
    cdr >> ros_message->balance_pitch_elimit;
  }

  // Field name: balance_pitch_olimit
  {
    cdr >> ros_message->balance_pitch_olimit;
  }

  // Field name: balance_pitch_neg_target
  {
    cdr >> ros_message->balance_pitch_neg_target;
  }

  // Field name: balance_pitch_pos_target
  {
    cdr >> ros_message->balance_pitch_pos_target;
  }

  // Field name: balance_roll_gp
  {
    cdr >> ros_message->balance_roll_gp;
  }

  // Field name: balance_roll_gi
  {
    cdr >> ros_message->balance_roll_gi;
  }

  // Field name: balance_roll_gd
  {
    cdr >> ros_message->balance_roll_gd;
  }

  // Field name: balance_roll_elimit
  {
    cdr >> ros_message->balance_roll_elimit;
  }

  // Field name: balance_roll_olimit
  {
    cdr >> ros_message->balance_roll_olimit;
  }

  // Field name: balance_roll_neg_target
  {
    cdr >> ros_message->balance_roll_neg_target;
  }

  // Field name: balance_roll_pos_target
  {
    cdr >> ros_message->balance_roll_pos_target;
  }

  // Field name: balance_angle_pitch_gp
  {
    cdr >> ros_message->balance_angle_pitch_gp;
  }

  // Field name: balance_angle_pitch_gi
  {
    cdr >> ros_message->balance_angle_pitch_gi;
  }

  // Field name: balance_angle_pitch_gd
  {
    cdr >> ros_message->balance_angle_pitch_gd;
  }

  // Field name: balance_angle_pitch_elimit
  {
    cdr >> ros_message->balance_angle_pitch_elimit;
  }

  // Field name: balance_angle_pitch_olimit
  {
    cdr >> ros_message->balance_angle_pitch_olimit;
  }

  // Field name: balance_angle_pitch_neg_target
  {
    cdr >> ros_message->balance_angle_pitch_neg_target;
  }

  // Field name: balance_angle_pitch_pos_target
  {
    cdr >> ros_message->balance_angle_pitch_pos_target;
  }

  // Field name: balance_value_4
  {
    cdr >> ros_message->balance_value_4;
  }

  // Field name: balance_value_5
  {
    cdr >> ros_message->balance_value_5;
  }

  // Field name: balance_pitch_flag_imu
  {
    cdr >> ros_message->balance_pitch_flag_imu;
  }

  // Field name: balance_pitch_gp_imu
  {
    cdr >> ros_message->balance_pitch_gp_imu;
  }

  // Field name: balance_pitch_gi_imu
  {
    cdr >> ros_message->balance_pitch_gi_imu;
  }

  // Field name: balance_pitch_gd_imu
  {
    cdr >> ros_message->balance_pitch_gd_imu;
  }

  // Field name: balance_pitch_neg_target_imu
  {
    cdr >> ros_message->balance_pitch_neg_target_imu;
  }

  // Field name: balance_pitch_pos_target_imu
  {
    cdr >> ros_message->balance_pitch_pos_target_imu;
  }

  // Field name: balance_pitch_elimit_imu
  {
    cdr >> ros_message->balance_pitch_elimit_imu;
  }

  // Field name: balance_pitch_olimit_imu
  {
    cdr >> ros_message->balance_pitch_olimit_imu;
  }

  // Field name: balance_roll_flag_imu
  {
    cdr >> ros_message->balance_roll_flag_imu;
  }

  // Field name: balance_roll_gp_imu
  {
    cdr >> ros_message->balance_roll_gp_imu;
  }

  // Field name: balance_roll_gi_imu
  {
    cdr >> ros_message->balance_roll_gi_imu;
  }

  // Field name: balance_roll_gd_imu
  {
    cdr >> ros_message->balance_roll_gd_imu;
  }

  // Field name: balance_roll_neg_target_imu
  {
    cdr >> ros_message->balance_roll_neg_target_imu;
  }

  // Field name: balance_roll_pos_target_imu
  {
    cdr >> ros_message->balance_roll_pos_target_imu;
  }

  // Field name: balance_roll_elimit_imu
  {
    cdr >> ros_message->balance_roll_elimit_imu;
  }

  // Field name: balance_roll_olimit_imu
  {
    cdr >> ros_message->balance_roll_olimit_imu;
  }

  // Field name: balance_pitch_flag
  {
    cdr >> ros_message->balance_pitch_flag;
  }

  // Field name: balance_ankle_pitch_flag
  {
    cdr >> ros_message->balance_ankle_pitch_flag;
  }

  // Field name: balance_roll_flag
  {
    cdr >> ros_message->balance_roll_flag;
  }

  // Field name: ratio_check_flag
  {
    cdr >> ros_message->ratio_check_flag;
  }

  // Field name: center2leg
  {
    cdr >> ros_message->center2leg;
  }

  // Field name: link2link
  {
    cdr >> ros_message->link2link;
  }

  // Field name: init_z_up
  {
    cdr >> ros_message->init_z_up;
  }

  // Field name: percentage_of_ik_motor
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->percentage_of_ik_motor.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->percentage_of_ik_motor);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->percentage_of_ik_motor, size)) {
      fprintf(stderr, "failed to create array for field 'percentage_of_ik_motor'");
      return false;
    }
    auto array_ptr = ros_message->percentage_of_ik_motor.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: first_pos_xr
  {
    cdr >> ros_message->first_pos_xr;
  }

  // Field name: first_neg_xr
  {
    cdr >> ros_message->first_neg_xr;
  }

  // Field name: first_pos_side_r
  {
    cdr >> ros_message->first_pos_side_r;
  }

  // Field name: first_neg_side_r
  {
    cdr >> ros_message->first_neg_side_r;
  }

  // Field name: first_pos_yaw_r
  {
    cdr >> ros_message->first_pos_yaw_r;
  }

  // Field name: first_neg_yaw_r
  {
    cdr >> ros_message->first_neg_yaw_r;
  }

  // Field name: first_pos_xl
  {
    cdr >> ros_message->first_pos_xl;
  }

  // Field name: first_neg_xl
  {
    cdr >> ros_message->first_neg_xl;
  }

  // Field name: first_pos_side_l
  {
    cdr >> ros_message->first_pos_side_l;
  }

  // Field name: first_neg_side_l
  {
    cdr >> ros_message->first_neg_side_l;
  }

  // Field name: first_pos_yaw_l
  {
    cdr >> ros_message->first_pos_yaw_l;
  }

  // Field name: first_neg_yaw_l
  {
    cdr >> ros_message->first_neg_yaw_l;
  }

  // Field name: first_pos_side_r_swing_minus
  {
    cdr >> ros_message->first_pos_side_r_swing_minus;
  }

  // Field name: first_neg_side_r_swing_minus
  {
    cdr >> ros_message->first_neg_side_r_swing_minus;
  }

  // Field name: first_pos_side_l_swing_minus
  {
    cdr >> ros_message->first_pos_side_l_swing_minus;
  }

  // Field name: first_neg_side_l_swing_minus
  {
    cdr >> ros_message->first_neg_side_l_swing_minus;
  }

  // Field name: first_min
  {
    cdr >> ros_message->first_min;
  }

  // Field name: first_max
  {
    cdr >> ros_message->first_max;
  }

  // Field name: second_pos_xr
  {
    cdr >> ros_message->second_pos_xr;
  }

  // Field name: second_neg_xr
  {
    cdr >> ros_message->second_neg_xr;
  }

  // Field name: second_pos_side_r
  {
    cdr >> ros_message->second_pos_side_r;
  }

  // Field name: second_neg_side_r
  {
    cdr >> ros_message->second_neg_side_r;
  }

  // Field name: second_pos_yaw_r
  {
    cdr >> ros_message->second_pos_yaw_r;
  }

  // Field name: second_neg_yaw_r
  {
    cdr >> ros_message->second_neg_yaw_r;
  }

  // Field name: second_pos_xl
  {
    cdr >> ros_message->second_pos_xl;
  }

  // Field name: second_neg_xl
  {
    cdr >> ros_message->second_neg_xl;
  }

  // Field name: second_pos_side_l
  {
    cdr >> ros_message->second_pos_side_l;
  }

  // Field name: second_neg_side_l
  {
    cdr >> ros_message->second_neg_side_l;
  }

  // Field name: second_pos_yaw_l
  {
    cdr >> ros_message->second_pos_yaw_l;
  }

  // Field name: second_neg_yaw_l
  {
    cdr >> ros_message->second_neg_yaw_l;
  }

  // Field name: second_pos_side_r_swing_minus
  {
    cdr >> ros_message->second_pos_side_r_swing_minus;
  }

  // Field name: second_neg_side_r_swing_minus
  {
    cdr >> ros_message->second_neg_side_r_swing_minus;
  }

  // Field name: second_pos_side_l_swing_minus
  {
    cdr >> ros_message->second_pos_side_l_swing_minus;
  }

  // Field name: second_neg_side_l_swing_minus
  {
    cdr >> ros_message->second_neg_side_l_swing_minus;
  }

  // Field name: second_min
  {
    cdr >> ros_message->second_min;
  }

  // Field name: second_max
  {
    cdr >> ros_message->second_max;
  }

  // Field name: third_pos_xr
  {
    cdr >> ros_message->third_pos_xr;
  }

  // Field name: third_neg_xr
  {
    cdr >> ros_message->third_neg_xr;
  }

  // Field name: third_pos_side_r
  {
    cdr >> ros_message->third_pos_side_r;
  }

  // Field name: third_neg_side_r
  {
    cdr >> ros_message->third_neg_side_r;
  }

  // Field name: third_pos_yaw_r
  {
    cdr >> ros_message->third_pos_yaw_r;
  }

  // Field name: third_neg_yaw_r
  {
    cdr >> ros_message->third_neg_yaw_r;
  }

  // Field name: third_pos_xl
  {
    cdr >> ros_message->third_pos_xl;
  }

  // Field name: third_neg_xl
  {
    cdr >> ros_message->third_neg_xl;
  }

  // Field name: third_pos_side_l
  {
    cdr >> ros_message->third_pos_side_l;
  }

  // Field name: third_neg_side_l
  {
    cdr >> ros_message->third_neg_side_l;
  }

  // Field name: third_pos_yaw_l
  {
    cdr >> ros_message->third_pos_yaw_l;
  }

  // Field name: third_neg_yaw_l
  {
    cdr >> ros_message->third_neg_yaw_l;
  }

  // Field name: third_pos_side_r_swing_minus
  {
    cdr >> ros_message->third_pos_side_r_swing_minus;
  }

  // Field name: third_neg_side_r_swing_minus
  {
    cdr >> ros_message->third_neg_side_r_swing_minus;
  }

  // Field name: third_pos_side_l_swing_minus
  {
    cdr >> ros_message->third_pos_side_l_swing_minus;
  }

  // Field name: third_neg_side_l_swing_minus
  {
    cdr >> ros_message->third_neg_side_l_swing_minus;
  }

  // Field name: third_min
  {
    cdr >> ros_message->third_min;
  }

  // Field name: third_max
  {
    cdr >> ros_message->third_max;
  }

  // Field name: fourth_pos_xr
  {
    cdr >> ros_message->fourth_pos_xr;
  }

  // Field name: fourth_neg_xr
  {
    cdr >> ros_message->fourth_neg_xr;
  }

  // Field name: fourth_pos_side_r
  {
    cdr >> ros_message->fourth_pos_side_r;
  }

  // Field name: fourth_neg_side_r
  {
    cdr >> ros_message->fourth_neg_side_r;
  }

  // Field name: fourth_pos_yaw_r
  {
    cdr >> ros_message->fourth_pos_yaw_r;
  }

  // Field name: fourth_neg_yaw_r
  {
    cdr >> ros_message->fourth_neg_yaw_r;
  }

  // Field name: fourth_pos_xl
  {
    cdr >> ros_message->fourth_pos_xl;
  }

  // Field name: fourth_neg_xl
  {
    cdr >> ros_message->fourth_neg_xl;
  }

  // Field name: fourth_pos_side_l
  {
    cdr >> ros_message->fourth_pos_side_l;
  }

  // Field name: fourth_neg_side_l
  {
    cdr >> ros_message->fourth_neg_side_l;
  }

  // Field name: fourth_pos_yaw_l
  {
    cdr >> ros_message->fourth_pos_yaw_l;
  }

  // Field name: fourth_neg_yaw_l
  {
    cdr >> ros_message->fourth_neg_yaw_l;
  }

  // Field name: fourth_pos_side_r_swing_minus
  {
    cdr >> ros_message->fourth_pos_side_r_swing_minus;
  }

  // Field name: fourth_neg_side_r_swing_minus
  {
    cdr >> ros_message->fourth_neg_side_r_swing_minus;
  }

  // Field name: fourth_pos_side_l_swing_minus
  {
    cdr >> ros_message->fourth_pos_side_l_swing_minus;
  }

  // Field name: fourth_neg_side_l_swing_minus
  {
    cdr >> ros_message->fourth_neg_side_l_swing_minus;
  }

  // Field name: fourth_min
  {
    cdr >> ros_message->fourth_min;
  }

  // Field name: fourth_max
  {
    cdr >> ros_message->fourth_max;
  }

  // Field name: landing_time_control_flag
  {
    cdr >> ros_message->landing_time_control_flag;
  }

  // Field name: control_system_flag
  {
    cdr >> ros_message->control_system_flag;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t get_serialized_size_humanoid_interfaces__msg__Tune2IkMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Tune2IkMsg__ros_msg_type * ros_message = static_cast<const _Tune2IkMsg__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name ik_flag
  {
    size_t item_size = sizeof(ros_message->ik_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name entire_time
  {
    size_t item_size = sizeof(ros_message->entire_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name frequency
  {
    size_t item_size = sizeof(ros_message->frequency);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name default_x_left
  {
    size_t item_size = sizeof(ros_message->default_x_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name default_y_left
  {
    size_t item_size = sizeof(ros_message->default_y_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name default_z_left
  {
    size_t item_size = sizeof(ros_message->default_z_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name default_x_right
  {
    size_t item_size = sizeof(ros_message->default_x_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name default_y_right
  {
    size_t item_size = sizeof(ros_message->default_y_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name default_z_right
  {
    size_t item_size = sizeof(ros_message->default_z_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name offset_motor
  {
    size_t array_size = ros_message->offset_motor.size;
    auto array_ptr = ros_message->offset_motor.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name swing_shoulder_left
  {
    size_t item_size = sizeof(ros_message->swing_shoulder_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name swing_shoulder_right
  {
    size_t item_size = sizeof(ros_message->swing_shoulder_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name swing_leg_left
  {
    size_t item_size = sizeof(ros_message->swing_leg_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name swing_leg_right
  {
    size_t item_size = sizeof(ros_message->swing_leg_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rise_leg_left
  {
    size_t item_size = sizeof(ros_message->rise_leg_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rise_leg_right
  {
    size_t item_size = sizeof(ros_message->rise_leg_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name start_entire_time
  {
    size_t item_size = sizeof(ros_message->start_entire_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name start_swing
  {
    size_t item_size = sizeof(ros_message->start_swing);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name start_rise
  {
    size_t item_size = sizeof(ros_message->start_rise);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name end_entire_time
  {
    size_t item_size = sizeof(ros_message->end_entire_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name end_swing
  {
    size_t item_size = sizeof(ros_message->end_swing);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name end_rise
  {
    size_t item_size = sizeof(ros_message->end_rise);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name test_x
  {
    size_t item_size = sizeof(ros_message->test_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name test_side
  {
    size_t item_size = sizeof(ros_message->test_side);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name test_yaw
  {
    size_t item_size = sizeof(ros_message->test_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tuning_x
  {
    size_t item_size = sizeof(ros_message->tuning_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tuning_side
  {
    size_t item_size = sizeof(ros_message->tuning_side);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tuning_yaw
  {
    size_t item_size = sizeof(ros_message->tuning_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_value_0
  {
    size_t item_size = sizeof(ros_message->balance_value_0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_value_1
  {
    size_t item_size = sizeof(ros_message->balance_value_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_value_2
  {
    size_t item_size = sizeof(ros_message->balance_value_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_value_3
  {
    size_t item_size = sizeof(ros_message->balance_value_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_gp
  {
    size_t item_size = sizeof(ros_message->balance_pitch_gp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_gi
  {
    size_t item_size = sizeof(ros_message->balance_pitch_gi);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_gd
  {
    size_t item_size = sizeof(ros_message->balance_pitch_gd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_elimit
  {
    size_t item_size = sizeof(ros_message->balance_pitch_elimit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_olimit
  {
    size_t item_size = sizeof(ros_message->balance_pitch_olimit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_neg_target
  {
    size_t item_size = sizeof(ros_message->balance_pitch_neg_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_pos_target
  {
    size_t item_size = sizeof(ros_message->balance_pitch_pos_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_gp
  {
    size_t item_size = sizeof(ros_message->balance_roll_gp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_gi
  {
    size_t item_size = sizeof(ros_message->balance_roll_gi);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_gd
  {
    size_t item_size = sizeof(ros_message->balance_roll_gd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_elimit
  {
    size_t item_size = sizeof(ros_message->balance_roll_elimit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_olimit
  {
    size_t item_size = sizeof(ros_message->balance_roll_olimit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_neg_target
  {
    size_t item_size = sizeof(ros_message->balance_roll_neg_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_pos_target
  {
    size_t item_size = sizeof(ros_message->balance_roll_pos_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_angle_pitch_gp
  {
    size_t item_size = sizeof(ros_message->balance_angle_pitch_gp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_angle_pitch_gi
  {
    size_t item_size = sizeof(ros_message->balance_angle_pitch_gi);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_angle_pitch_gd
  {
    size_t item_size = sizeof(ros_message->balance_angle_pitch_gd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_angle_pitch_elimit
  {
    size_t item_size = sizeof(ros_message->balance_angle_pitch_elimit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_angle_pitch_olimit
  {
    size_t item_size = sizeof(ros_message->balance_angle_pitch_olimit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_angle_pitch_neg_target
  {
    size_t item_size = sizeof(ros_message->balance_angle_pitch_neg_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_angle_pitch_pos_target
  {
    size_t item_size = sizeof(ros_message->balance_angle_pitch_pos_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_value_4
  {
    size_t item_size = sizeof(ros_message->balance_value_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_value_5
  {
    size_t item_size = sizeof(ros_message->balance_value_5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_flag_imu
  {
    size_t item_size = sizeof(ros_message->balance_pitch_flag_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_gp_imu
  {
    size_t item_size = sizeof(ros_message->balance_pitch_gp_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_gi_imu
  {
    size_t item_size = sizeof(ros_message->balance_pitch_gi_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_gd_imu
  {
    size_t item_size = sizeof(ros_message->balance_pitch_gd_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_neg_target_imu
  {
    size_t item_size = sizeof(ros_message->balance_pitch_neg_target_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_pos_target_imu
  {
    size_t item_size = sizeof(ros_message->balance_pitch_pos_target_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_elimit_imu
  {
    size_t item_size = sizeof(ros_message->balance_pitch_elimit_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_olimit_imu
  {
    size_t item_size = sizeof(ros_message->balance_pitch_olimit_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_flag_imu
  {
    size_t item_size = sizeof(ros_message->balance_roll_flag_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_gp_imu
  {
    size_t item_size = sizeof(ros_message->balance_roll_gp_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_gi_imu
  {
    size_t item_size = sizeof(ros_message->balance_roll_gi_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_gd_imu
  {
    size_t item_size = sizeof(ros_message->balance_roll_gd_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_neg_target_imu
  {
    size_t item_size = sizeof(ros_message->balance_roll_neg_target_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_pos_target_imu
  {
    size_t item_size = sizeof(ros_message->balance_roll_pos_target_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_elimit_imu
  {
    size_t item_size = sizeof(ros_message->balance_roll_elimit_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_olimit_imu
  {
    size_t item_size = sizeof(ros_message->balance_roll_olimit_imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_pitch_flag
  {
    size_t item_size = sizeof(ros_message->balance_pitch_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_ankle_pitch_flag
  {
    size_t item_size = sizeof(ros_message->balance_ankle_pitch_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balance_roll_flag
  {
    size_t item_size = sizeof(ros_message->balance_roll_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ratio_check_flag
  {
    size_t item_size = sizeof(ros_message->ratio_check_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name center2leg
  {
    size_t item_size = sizeof(ros_message->center2leg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name link2link
  {
    size_t item_size = sizeof(ros_message->link2link);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name init_z_up
  {
    size_t item_size = sizeof(ros_message->init_z_up);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name percentage_of_ik_motor
  {
    size_t array_size = ros_message->percentage_of_ik_motor.size;
    auto array_ptr = ros_message->percentage_of_ik_motor.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_pos_xr
  {
    size_t item_size = sizeof(ros_message->first_pos_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_neg_xr
  {
    size_t item_size = sizeof(ros_message->first_neg_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_pos_side_r
  {
    size_t item_size = sizeof(ros_message->first_pos_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_neg_side_r
  {
    size_t item_size = sizeof(ros_message->first_neg_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_pos_yaw_r
  {
    size_t item_size = sizeof(ros_message->first_pos_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_neg_yaw_r
  {
    size_t item_size = sizeof(ros_message->first_neg_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_pos_xl
  {
    size_t item_size = sizeof(ros_message->first_pos_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_neg_xl
  {
    size_t item_size = sizeof(ros_message->first_neg_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_pos_side_l
  {
    size_t item_size = sizeof(ros_message->first_pos_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_neg_side_l
  {
    size_t item_size = sizeof(ros_message->first_neg_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_pos_yaw_l
  {
    size_t item_size = sizeof(ros_message->first_pos_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_neg_yaw_l
  {
    size_t item_size = sizeof(ros_message->first_neg_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_pos_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message->first_pos_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_neg_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message->first_neg_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_pos_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message->first_pos_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_neg_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message->first_neg_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_min
  {
    size_t item_size = sizeof(ros_message->first_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_max
  {
    size_t item_size = sizeof(ros_message->first_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_pos_xr
  {
    size_t item_size = sizeof(ros_message->second_pos_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_neg_xr
  {
    size_t item_size = sizeof(ros_message->second_neg_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_pos_side_r
  {
    size_t item_size = sizeof(ros_message->second_pos_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_neg_side_r
  {
    size_t item_size = sizeof(ros_message->second_neg_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_pos_yaw_r
  {
    size_t item_size = sizeof(ros_message->second_pos_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_neg_yaw_r
  {
    size_t item_size = sizeof(ros_message->second_neg_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_pos_xl
  {
    size_t item_size = sizeof(ros_message->second_pos_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_neg_xl
  {
    size_t item_size = sizeof(ros_message->second_neg_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_pos_side_l
  {
    size_t item_size = sizeof(ros_message->second_pos_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_neg_side_l
  {
    size_t item_size = sizeof(ros_message->second_neg_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_pos_yaw_l
  {
    size_t item_size = sizeof(ros_message->second_pos_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_neg_yaw_l
  {
    size_t item_size = sizeof(ros_message->second_neg_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_pos_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message->second_pos_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_neg_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message->second_neg_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_pos_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message->second_pos_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_neg_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message->second_neg_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_min
  {
    size_t item_size = sizeof(ros_message->second_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name second_max
  {
    size_t item_size = sizeof(ros_message->second_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_pos_xr
  {
    size_t item_size = sizeof(ros_message->third_pos_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_neg_xr
  {
    size_t item_size = sizeof(ros_message->third_neg_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_pos_side_r
  {
    size_t item_size = sizeof(ros_message->third_pos_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_neg_side_r
  {
    size_t item_size = sizeof(ros_message->third_neg_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_pos_yaw_r
  {
    size_t item_size = sizeof(ros_message->third_pos_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_neg_yaw_r
  {
    size_t item_size = sizeof(ros_message->third_neg_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_pos_xl
  {
    size_t item_size = sizeof(ros_message->third_pos_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_neg_xl
  {
    size_t item_size = sizeof(ros_message->third_neg_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_pos_side_l
  {
    size_t item_size = sizeof(ros_message->third_pos_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_neg_side_l
  {
    size_t item_size = sizeof(ros_message->third_neg_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_pos_yaw_l
  {
    size_t item_size = sizeof(ros_message->third_pos_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_neg_yaw_l
  {
    size_t item_size = sizeof(ros_message->third_neg_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_pos_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message->third_pos_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_neg_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message->third_neg_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_pos_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message->third_pos_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_neg_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message->third_neg_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_min
  {
    size_t item_size = sizeof(ros_message->third_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name third_max
  {
    size_t item_size = sizeof(ros_message->third_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_pos_xr
  {
    size_t item_size = sizeof(ros_message->fourth_pos_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_neg_xr
  {
    size_t item_size = sizeof(ros_message->fourth_neg_xr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_pos_side_r
  {
    size_t item_size = sizeof(ros_message->fourth_pos_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_neg_side_r
  {
    size_t item_size = sizeof(ros_message->fourth_neg_side_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_pos_yaw_r
  {
    size_t item_size = sizeof(ros_message->fourth_pos_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_neg_yaw_r
  {
    size_t item_size = sizeof(ros_message->fourth_neg_yaw_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_pos_xl
  {
    size_t item_size = sizeof(ros_message->fourth_pos_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_neg_xl
  {
    size_t item_size = sizeof(ros_message->fourth_neg_xl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_pos_side_l
  {
    size_t item_size = sizeof(ros_message->fourth_pos_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_neg_side_l
  {
    size_t item_size = sizeof(ros_message->fourth_neg_side_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_pos_yaw_l
  {
    size_t item_size = sizeof(ros_message->fourth_pos_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_neg_yaw_l
  {
    size_t item_size = sizeof(ros_message->fourth_neg_yaw_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_pos_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message->fourth_pos_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_neg_side_r_swing_minus
  {
    size_t item_size = sizeof(ros_message->fourth_neg_side_r_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_pos_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message->fourth_pos_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_neg_side_l_swing_minus
  {
    size_t item_size = sizeof(ros_message->fourth_neg_side_l_swing_minus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_min
  {
    size_t item_size = sizeof(ros_message->fourth_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fourth_max
  {
    size_t item_size = sizeof(ros_message->fourth_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name landing_time_control_flag
  {
    size_t item_size = sizeof(ros_message->landing_time_control_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name control_system_flag
  {
    size_t item_size = sizeof(ros_message->control_system_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Tune2IkMsg__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_humanoid_interfaces__msg__Tune2IkMsg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t max_serialized_size_humanoid_interfaces__msg__Tune2IkMsg(
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

  // member: ik_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: entire_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: frequency
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: default_x_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: default_y_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: default_z_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: default_x_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: default_y_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: default_z_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: offset_motor
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
  // member: swing_shoulder_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: swing_shoulder_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: swing_leg_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: swing_leg_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: rise_leg_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: rise_leg_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: start_entire_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: start_swing
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: start_rise
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: end_entire_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: end_swing
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: end_rise
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: test_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: test_side
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: test_yaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: tuning_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: tuning_side
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: tuning_yaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_value_0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_value_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_value_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_value_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_gp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_gi
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_gd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_elimit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_olimit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_neg_target
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_pos_target
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_gp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_gi
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_gd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_elimit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_olimit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_neg_target
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_pos_target
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_angle_pitch_gp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_angle_pitch_gi
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_angle_pitch_gd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_angle_pitch_elimit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_angle_pitch_olimit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_angle_pitch_neg_target
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_angle_pitch_pos_target
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_value_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_value_5
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_flag_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_gp_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_gi_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_gd_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_neg_target_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_pos_target_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_elimit_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_olimit_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_flag_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_gp_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_gi_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_gd_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_neg_target_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_pos_target_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_elimit_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_olimit_imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_pitch_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_ankle_pitch_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balance_roll_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ratio_check_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: center2leg
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: link2link
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: init_z_up
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: percentage_of_ik_motor
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
  // member: first_pos_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_neg_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_pos_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_neg_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_pos_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_neg_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_pos_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_neg_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_pos_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_neg_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_pos_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_neg_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_pos_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_neg_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_pos_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_neg_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_min
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: first_max
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_pos_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_neg_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_pos_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_neg_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_pos_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_neg_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_pos_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_neg_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_pos_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_neg_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_pos_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_neg_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_pos_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_neg_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_pos_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_neg_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_min
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: second_max
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_pos_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_neg_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_pos_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_neg_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_pos_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_neg_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_pos_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_neg_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_pos_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_neg_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_pos_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_neg_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_pos_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_neg_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_pos_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_neg_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_min
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: third_max
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_pos_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_neg_xr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_pos_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_neg_side_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_pos_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_neg_yaw_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_pos_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_neg_xl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_pos_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_neg_side_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_pos_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_neg_yaw_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_pos_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_neg_side_r_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_pos_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_neg_side_l_swing_minus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_min
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fourth_max
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: landing_time_control_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: control_system_flag
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
    using DataType = humanoid_interfaces__msg__Tune2IkMsg;
    is_plain =
      (
      offsetof(DataType, control_system_flag) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Tune2IkMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_humanoid_interfaces__msg__Tune2IkMsg(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Tune2IkMsg = {
  "humanoid_interfaces::msg",
  "Tune2IkMsg",
  _Tune2IkMsg__cdr_serialize,
  _Tune2IkMsg__cdr_deserialize,
  _Tune2IkMsg__get_serialized_size,
  _Tune2IkMsg__max_serialized_size
};

static rosidl_message_type_support_t _Tune2IkMsg__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Tune2IkMsg,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, humanoid_interfaces, msg, Tune2IkMsg)() {
  return &_Tune2IkMsg__type_support;
}

#if defined(__cplusplus)
}
#endif
