// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from humanoid_interfaces:msg/Tune2IkMsg.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "humanoid_interfaces/msg/detail/tune2_ik_msg__struct.h"
#include "humanoid_interfaces/msg/detail/tune2_ik_msg__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool humanoid_interfaces__msg__tune2_ik_msg__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("humanoid_interfaces.msg._tune2_ik_msg.Tune2IkMsg", full_classname_dest, 48) == 0);
  }
  humanoid_interfaces__msg__Tune2IkMsg * ros_message = _ros_message;
  {  // ik_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "ik_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->ik_flag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // entire_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "entire_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->entire_time = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // frequency
    PyObject * field = PyObject_GetAttrString(_pymsg, "frequency");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->frequency = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // default_x_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "default_x_left");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->default_x_left = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // default_y_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "default_y_left");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->default_y_left = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // default_z_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "default_z_left");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->default_z_left = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // default_x_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "default_x_right");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->default_x_right = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // default_y_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "default_y_right");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->default_y_right = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // default_z_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "default_z_right");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->default_z_right = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // offset_motor
    PyObject * field = PyObject_GetAttrString(_pymsg, "offset_motor");
    if (!field) {
      return false;
    }
    if (PyObject_CheckBuffer(field)) {
      // Optimization for converting arrays of primitives
      Py_buffer view;
      int rc = PyObject_GetBuffer(field, &view, PyBUF_SIMPLE);
      if (rc < 0) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = view.len / sizeof(double);
      if (!rosidl_runtime_c__double__Sequence__init(&(ros_message->offset_motor), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create double__Sequence ros_message");
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      double * dest = ros_message->offset_motor.data;
      rc = PyBuffer_ToContiguous(dest, &view, view.len, 'C');
      if (rc < 0) {
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      PyBuffer_Release(&view);
    } else {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'offset_motor'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = PySequence_Size(field);
      if (-1 == size) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      if (!rosidl_runtime_c__double__Sequence__init(&(ros_message->offset_motor), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create double__Sequence ros_message");
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      double * dest = ros_message->offset_motor.data;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyFloat_Check(item));
        double tmp = PyFloat_AS_DOUBLE(item);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // swing_shoulder_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "swing_shoulder_left");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->swing_shoulder_left = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // swing_shoulder_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "swing_shoulder_right");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->swing_shoulder_right = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // swing_leg_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "swing_leg_left");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->swing_leg_left = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // swing_leg_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "swing_leg_right");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->swing_leg_right = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rise_leg_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "rise_leg_left");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rise_leg_left = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rise_leg_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "rise_leg_right");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rise_leg_right = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // start_entire_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "start_entire_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->start_entire_time = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // start_swing
    PyObject * field = PyObject_GetAttrString(_pymsg, "start_swing");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->start_swing = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // start_rise
    PyObject * field = PyObject_GetAttrString(_pymsg, "start_rise");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->start_rise = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // end_entire_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "end_entire_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->end_entire_time = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // end_swing
    PyObject * field = PyObject_GetAttrString(_pymsg, "end_swing");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->end_swing = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // end_rise
    PyObject * field = PyObject_GetAttrString(_pymsg, "end_rise");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->end_rise = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // test_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "test_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->test_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // test_side
    PyObject * field = PyObject_GetAttrString(_pymsg, "test_side");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->test_side = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // test_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "test_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->test_yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tuning_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "tuning_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tuning_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tuning_side
    PyObject * field = PyObject_GetAttrString(_pymsg, "tuning_side");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tuning_side = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tuning_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "tuning_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tuning_yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_value_0
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_value_0");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_value_0 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_value_1
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_value_1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_value_1 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_value_2
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_value_2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_value_2 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_value_3
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_value_3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_value_3 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_gp
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_gp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_pitch_gp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_gi
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_gi");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_pitch_gi = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_gd
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_gd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_pitch_gd = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_elimit
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_elimit");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_pitch_elimit = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_olimit
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_olimit");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_pitch_olimit = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_neg_target
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_neg_target");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_pitch_neg_target = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_pos_target
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_pos_target");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_pitch_pos_target = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_roll_gp
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_gp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_roll_gp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_roll_gi
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_gi");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_roll_gi = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_roll_gd
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_gd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_roll_gd = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_roll_elimit
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_elimit");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_roll_elimit = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_roll_olimit
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_olimit");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_roll_olimit = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_roll_neg_target
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_neg_target");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_roll_neg_target = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_roll_pos_target
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_pos_target");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_roll_pos_target = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_angle_pitch_gp
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_angle_pitch_gp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_angle_pitch_gp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_angle_pitch_gi
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_angle_pitch_gi");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_angle_pitch_gi = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_angle_pitch_gd
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_angle_pitch_gd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_angle_pitch_gd = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_angle_pitch_elimit
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_angle_pitch_elimit");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_angle_pitch_elimit = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_angle_pitch_olimit
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_angle_pitch_olimit");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_angle_pitch_olimit = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_angle_pitch_neg_target
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_angle_pitch_neg_target");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_angle_pitch_neg_target = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_angle_pitch_pos_target
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_angle_pitch_pos_target");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_angle_pitch_pos_target = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_value_4
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_value_4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_value_4 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_value_5
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_value_5");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_value_5 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_flag_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_flag_imu");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->balance_pitch_flag_imu = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_gp_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_gp_imu");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_pitch_gp_imu = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_gi_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_gi_imu");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_pitch_gi_imu = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_gd_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_gd_imu");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_pitch_gd_imu = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_neg_target_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_neg_target_imu");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_pitch_neg_target_imu = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_pos_target_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_pos_target_imu");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_pitch_pos_target_imu = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_elimit_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_elimit_imu");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_pitch_elimit_imu = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_olimit_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_olimit_imu");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_pitch_olimit_imu = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_roll_flag_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_flag_imu");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->balance_roll_flag_imu = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // balance_roll_gp_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_gp_imu");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_roll_gp_imu = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_roll_gi_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_gi_imu");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_roll_gi_imu = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_roll_gd_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_gd_imu");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_roll_gd_imu = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_roll_neg_target_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_neg_target_imu");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_roll_neg_target_imu = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_roll_pos_target_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_pos_target_imu");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_roll_pos_target_imu = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_roll_elimit_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_elimit_imu");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_roll_elimit_imu = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_roll_olimit_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_olimit_imu");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->balance_roll_olimit_imu = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // balance_pitch_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_pitch_flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->balance_pitch_flag = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // balance_ankle_pitch_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_ankle_pitch_flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->balance_ankle_pitch_flag = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // balance_roll_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "balance_roll_flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->balance_roll_flag = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // ratio_check_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "ratio_check_flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ratio_check_flag = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // center2leg
    PyObject * field = PyObject_GetAttrString(_pymsg, "center2leg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->center2leg = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // link2link
    PyObject * field = PyObject_GetAttrString(_pymsg, "link2link");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->link2link = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // init_z_up
    PyObject * field = PyObject_GetAttrString(_pymsg, "init_z_up");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->init_z_up = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // percentage_of_ik_motor
    PyObject * field = PyObject_GetAttrString(_pymsg, "percentage_of_ik_motor");
    if (!field) {
      return false;
    }
    if (PyObject_CheckBuffer(field)) {
      // Optimization for converting arrays of primitives
      Py_buffer view;
      int rc = PyObject_GetBuffer(field, &view, PyBUF_SIMPLE);
      if (rc < 0) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = view.len / sizeof(double);
      if (!rosidl_runtime_c__double__Sequence__init(&(ros_message->percentage_of_ik_motor), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create double__Sequence ros_message");
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      double * dest = ros_message->percentage_of_ik_motor.data;
      rc = PyBuffer_ToContiguous(dest, &view, view.len, 'C');
      if (rc < 0) {
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      PyBuffer_Release(&view);
    } else {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'percentage_of_ik_motor'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = PySequence_Size(field);
      if (-1 == size) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      if (!rosidl_runtime_c__double__Sequence__init(&(ros_message->percentage_of_ik_motor), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create double__Sequence ros_message");
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      double * dest = ros_message->percentage_of_ik_motor.data;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyFloat_Check(item));
        double tmp = PyFloat_AS_DOUBLE(item);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // first_pos_xr
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_pos_xr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_pos_xr = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_neg_xr
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_neg_xr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_neg_xr = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_pos_side_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_pos_side_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_pos_side_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_neg_side_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_neg_side_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_neg_side_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_pos_yaw_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_pos_yaw_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_pos_yaw_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_neg_yaw_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_neg_yaw_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_neg_yaw_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_pos_xl
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_pos_xl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_pos_xl = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_neg_xl
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_neg_xl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_neg_xl = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_pos_side_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_pos_side_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_pos_side_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_neg_side_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_neg_side_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_neg_side_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_pos_yaw_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_pos_yaw_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_pos_yaw_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_neg_yaw_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_neg_yaw_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_neg_yaw_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_pos_side_r_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_pos_side_r_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_pos_side_r_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_neg_side_r_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_neg_side_r_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_neg_side_r_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_pos_side_l_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_pos_side_l_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_pos_side_l_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_neg_side_l_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_neg_side_l_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->first_neg_side_l_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // first_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_min");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->first_min = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // first_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_max");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->first_max = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // second_pos_xr
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_pos_xr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_pos_xr = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_neg_xr
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_neg_xr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_neg_xr = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_pos_side_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_pos_side_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_pos_side_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_neg_side_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_neg_side_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_neg_side_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_pos_yaw_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_pos_yaw_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_pos_yaw_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_neg_yaw_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_neg_yaw_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_neg_yaw_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_pos_xl
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_pos_xl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_pos_xl = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_neg_xl
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_neg_xl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_neg_xl = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_pos_side_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_pos_side_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_pos_side_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_neg_side_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_neg_side_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_neg_side_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_pos_yaw_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_pos_yaw_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_pos_yaw_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_neg_yaw_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_neg_yaw_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_neg_yaw_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_pos_side_r_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_pos_side_r_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_pos_side_r_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_neg_side_r_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_neg_side_r_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_neg_side_r_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_pos_side_l_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_pos_side_l_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_pos_side_l_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_neg_side_l_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_neg_side_l_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->second_neg_side_l_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // second_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_min");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->second_min = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // second_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "second_max");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->second_max = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // third_pos_xr
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_pos_xr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_pos_xr = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_neg_xr
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_neg_xr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_neg_xr = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_pos_side_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_pos_side_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_pos_side_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_neg_side_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_neg_side_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_neg_side_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_pos_yaw_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_pos_yaw_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_pos_yaw_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_neg_yaw_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_neg_yaw_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_neg_yaw_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_pos_xl
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_pos_xl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_pos_xl = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_neg_xl
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_neg_xl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_neg_xl = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_pos_side_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_pos_side_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_pos_side_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_neg_side_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_neg_side_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_neg_side_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_pos_yaw_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_pos_yaw_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_pos_yaw_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_neg_yaw_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_neg_yaw_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_neg_yaw_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_pos_side_r_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_pos_side_r_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_pos_side_r_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_neg_side_r_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_neg_side_r_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_neg_side_r_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_pos_side_l_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_pos_side_l_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_pos_side_l_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_neg_side_l_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_neg_side_l_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->third_neg_side_l_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // third_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_min");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->third_min = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // third_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "third_max");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->third_max = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // fourth_pos_xr
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_pos_xr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_pos_xr = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_neg_xr
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_neg_xr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_neg_xr = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_pos_side_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_pos_side_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_pos_side_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_neg_side_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_neg_side_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_neg_side_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_pos_yaw_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_pos_yaw_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_pos_yaw_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_neg_yaw_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_neg_yaw_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_neg_yaw_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_pos_xl
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_pos_xl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_pos_xl = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_neg_xl
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_neg_xl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_neg_xl = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_pos_side_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_pos_side_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_pos_side_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_neg_side_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_neg_side_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_neg_side_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_pos_yaw_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_pos_yaw_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_pos_yaw_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_neg_yaw_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_neg_yaw_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_neg_yaw_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_pos_side_r_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_pos_side_r_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_pos_side_r_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_neg_side_r_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_neg_side_r_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_neg_side_r_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_pos_side_l_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_pos_side_l_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_pos_side_l_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_neg_side_l_swing_minus
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_neg_side_l_swing_minus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fourth_neg_side_l_swing_minus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fourth_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_min");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->fourth_min = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // fourth_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "fourth_max");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->fourth_max = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // landing_time_control_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "landing_time_control_flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->landing_time_control_flag = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // control_system_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "control_system_flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->control_system_flag = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * humanoid_interfaces__msg__tune2_ik_msg__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Tune2IkMsg */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("humanoid_interfaces.msg._tune2_ik_msg");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Tune2IkMsg");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  humanoid_interfaces__msg__Tune2IkMsg * ros_message = (humanoid_interfaces__msg__Tune2IkMsg *)raw_ros_message;
  {  // ik_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->ik_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ik_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // entire_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->entire_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "entire_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // frequency
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->frequency);
    {
      int rc = PyObject_SetAttrString(_pymessage, "frequency", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // default_x_left
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->default_x_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "default_x_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // default_y_left
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->default_y_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "default_y_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // default_z_left
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->default_z_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "default_z_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // default_x_right
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->default_x_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "default_x_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // default_y_right
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->default_y_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "default_y_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // default_z_right
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->default_z_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "default_z_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // offset_motor
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "offset_motor");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "array.array") == 0);
    // ensure that itemsize matches the sizeof of the ROS message field
    PyObject * itemsize_attr = PyObject_GetAttrString(field, "itemsize");
    assert(itemsize_attr != NULL);
    size_t itemsize = PyLong_AsSize_t(itemsize_attr);
    Py_DECREF(itemsize_attr);
    if (itemsize != sizeof(double)) {
      PyErr_SetString(PyExc_RuntimeError, "itemsize doesn't match expectation");
      Py_DECREF(field);
      return NULL;
    }
    // clear the array, poor approach to remove potential default values
    Py_ssize_t length = PyObject_Length(field);
    if (-1 == length) {
      Py_DECREF(field);
      return NULL;
    }
    if (length > 0) {
      PyObject * pop = PyObject_GetAttrString(field, "pop");
      assert(pop != NULL);
      for (Py_ssize_t i = 0; i < length; ++i) {
        PyObject * ret = PyObject_CallFunctionObjArgs(pop, NULL);
        if (!ret) {
          Py_DECREF(pop);
          Py_DECREF(field);
          return NULL;
        }
        Py_DECREF(ret);
      }
      Py_DECREF(pop);
    }
    if (ros_message->offset_motor.size > 0) {
      // populating the array.array using the frombytes method
      PyObject * frombytes = PyObject_GetAttrString(field, "frombytes");
      assert(frombytes != NULL);
      double * src = &(ros_message->offset_motor.data[0]);
      PyObject * data = PyBytes_FromStringAndSize((const char *)src, ros_message->offset_motor.size * sizeof(double));
      assert(data != NULL);
      PyObject * ret = PyObject_CallFunctionObjArgs(frombytes, data, NULL);
      Py_DECREF(data);
      Py_DECREF(frombytes);
      if (!ret) {
        Py_DECREF(field);
        return NULL;
      }
      Py_DECREF(ret);
    }
    Py_DECREF(field);
  }
  {  // swing_shoulder_left
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->swing_shoulder_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "swing_shoulder_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // swing_shoulder_right
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->swing_shoulder_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "swing_shoulder_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // swing_leg_left
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->swing_leg_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "swing_leg_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // swing_leg_right
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->swing_leg_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "swing_leg_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rise_leg_left
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rise_leg_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rise_leg_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rise_leg_right
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rise_leg_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rise_leg_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // start_entire_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->start_entire_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "start_entire_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // start_swing
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->start_swing);
    {
      int rc = PyObject_SetAttrString(_pymessage, "start_swing", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // start_rise
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->start_rise);
    {
      int rc = PyObject_SetAttrString(_pymessage, "start_rise", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // end_entire_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->end_entire_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "end_entire_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // end_swing
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->end_swing);
    {
      int rc = PyObject_SetAttrString(_pymessage, "end_swing", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // end_rise
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->end_rise);
    {
      int rc = PyObject_SetAttrString(_pymessage, "end_rise", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // test_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->test_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "test_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // test_side
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->test_side);
    {
      int rc = PyObject_SetAttrString(_pymessage, "test_side", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // test_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->test_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "test_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tuning_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tuning_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tuning_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tuning_side
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tuning_side);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tuning_side", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tuning_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tuning_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tuning_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_value_0
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_value_0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_value_0", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_value_1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_value_1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_value_1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_value_2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_value_2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_value_2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_value_3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_value_3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_value_3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_gp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_pitch_gp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_gp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_gi
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_pitch_gi);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_gi", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_gd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_pitch_gd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_gd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_elimit
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_pitch_elimit);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_elimit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_olimit
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_pitch_olimit);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_olimit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_neg_target
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_pitch_neg_target);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_neg_target", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_pos_target
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_pitch_pos_target);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_pos_target", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_gp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_roll_gp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_gp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_gi
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_roll_gi);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_gi", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_gd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_roll_gd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_gd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_elimit
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_roll_elimit);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_elimit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_olimit
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_roll_olimit);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_olimit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_neg_target
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_roll_neg_target);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_neg_target", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_pos_target
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_roll_pos_target);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_pos_target", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_angle_pitch_gp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_angle_pitch_gp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_angle_pitch_gp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_angle_pitch_gi
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_angle_pitch_gi);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_angle_pitch_gi", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_angle_pitch_gd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_angle_pitch_gd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_angle_pitch_gd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_angle_pitch_elimit
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_angle_pitch_elimit);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_angle_pitch_elimit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_angle_pitch_olimit
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_angle_pitch_olimit);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_angle_pitch_olimit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_angle_pitch_neg_target
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_angle_pitch_neg_target);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_angle_pitch_neg_target", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_angle_pitch_pos_target
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_angle_pitch_pos_target);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_angle_pitch_pos_target", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_value_4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_value_4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_value_4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_value_5
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_value_5);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_value_5", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_flag_imu
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->balance_pitch_flag_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_flag_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_gp_imu
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_pitch_gp_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_gp_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_gi_imu
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_pitch_gi_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_gi_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_gd_imu
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_pitch_gd_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_gd_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_neg_target_imu
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_pitch_neg_target_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_neg_target_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_pos_target_imu
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_pitch_pos_target_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_pos_target_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_elimit_imu
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_pitch_elimit_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_elimit_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_olimit_imu
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_pitch_olimit_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_olimit_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_flag_imu
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->balance_roll_flag_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_flag_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_gp_imu
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_roll_gp_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_gp_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_gi_imu
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_roll_gi_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_gi_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_gd_imu
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_roll_gd_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_gd_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_neg_target_imu
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_roll_neg_target_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_neg_target_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_pos_target_imu
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_roll_pos_target_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_pos_target_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_elimit_imu
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_roll_elimit_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_elimit_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_olimit_imu
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->balance_roll_olimit_imu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_olimit_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_pitch_flag
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->balance_pitch_flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_pitch_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_ankle_pitch_flag
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->balance_ankle_pitch_flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_ankle_pitch_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balance_roll_flag
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->balance_roll_flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balance_roll_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ratio_check_flag
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->ratio_check_flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ratio_check_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // center2leg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->center2leg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "center2leg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // link2link
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->link2link);
    {
      int rc = PyObject_SetAttrString(_pymessage, "link2link", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // init_z_up
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->init_z_up);
    {
      int rc = PyObject_SetAttrString(_pymessage, "init_z_up", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // percentage_of_ik_motor
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "percentage_of_ik_motor");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "array.array") == 0);
    // ensure that itemsize matches the sizeof of the ROS message field
    PyObject * itemsize_attr = PyObject_GetAttrString(field, "itemsize");
    assert(itemsize_attr != NULL);
    size_t itemsize = PyLong_AsSize_t(itemsize_attr);
    Py_DECREF(itemsize_attr);
    if (itemsize != sizeof(double)) {
      PyErr_SetString(PyExc_RuntimeError, "itemsize doesn't match expectation");
      Py_DECREF(field);
      return NULL;
    }
    // clear the array, poor approach to remove potential default values
    Py_ssize_t length = PyObject_Length(field);
    if (-1 == length) {
      Py_DECREF(field);
      return NULL;
    }
    if (length > 0) {
      PyObject * pop = PyObject_GetAttrString(field, "pop");
      assert(pop != NULL);
      for (Py_ssize_t i = 0; i < length; ++i) {
        PyObject * ret = PyObject_CallFunctionObjArgs(pop, NULL);
        if (!ret) {
          Py_DECREF(pop);
          Py_DECREF(field);
          return NULL;
        }
        Py_DECREF(ret);
      }
      Py_DECREF(pop);
    }
    if (ros_message->percentage_of_ik_motor.size > 0) {
      // populating the array.array using the frombytes method
      PyObject * frombytes = PyObject_GetAttrString(field, "frombytes");
      assert(frombytes != NULL);
      double * src = &(ros_message->percentage_of_ik_motor.data[0]);
      PyObject * data = PyBytes_FromStringAndSize((const char *)src, ros_message->percentage_of_ik_motor.size * sizeof(double));
      assert(data != NULL);
      PyObject * ret = PyObject_CallFunctionObjArgs(frombytes, data, NULL);
      Py_DECREF(data);
      Py_DECREF(frombytes);
      if (!ret) {
        Py_DECREF(field);
        return NULL;
      }
      Py_DECREF(ret);
    }
    Py_DECREF(field);
  }
  {  // first_pos_xr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_pos_xr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_pos_xr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_neg_xr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_neg_xr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_neg_xr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_pos_side_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_pos_side_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_pos_side_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_neg_side_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_neg_side_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_neg_side_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_pos_yaw_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_pos_yaw_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_pos_yaw_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_neg_yaw_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_neg_yaw_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_neg_yaw_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_pos_xl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_pos_xl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_pos_xl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_neg_xl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_neg_xl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_neg_xl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_pos_side_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_pos_side_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_pos_side_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_neg_side_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_neg_side_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_neg_side_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_pos_yaw_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_pos_yaw_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_pos_yaw_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_neg_yaw_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_neg_yaw_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_neg_yaw_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_pos_side_r_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_pos_side_r_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_pos_side_r_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_neg_side_r_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_neg_side_r_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_neg_side_r_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_pos_side_l_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_pos_side_l_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_pos_side_l_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_neg_side_l_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->first_neg_side_l_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_neg_side_l_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_min
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->first_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_max
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->first_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_pos_xr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_pos_xr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_pos_xr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_neg_xr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_neg_xr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_neg_xr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_pos_side_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_pos_side_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_pos_side_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_neg_side_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_neg_side_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_neg_side_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_pos_yaw_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_pos_yaw_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_pos_yaw_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_neg_yaw_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_neg_yaw_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_neg_yaw_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_pos_xl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_pos_xl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_pos_xl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_neg_xl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_neg_xl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_neg_xl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_pos_side_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_pos_side_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_pos_side_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_neg_side_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_neg_side_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_neg_side_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_pos_yaw_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_pos_yaw_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_pos_yaw_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_neg_yaw_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_neg_yaw_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_neg_yaw_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_pos_side_r_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_pos_side_r_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_pos_side_r_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_neg_side_r_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_neg_side_r_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_neg_side_r_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_pos_side_l_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_pos_side_l_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_pos_side_l_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_neg_side_l_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->second_neg_side_l_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_neg_side_l_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_min
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->second_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // second_max
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->second_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "second_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_pos_xr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_pos_xr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_pos_xr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_neg_xr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_neg_xr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_neg_xr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_pos_side_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_pos_side_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_pos_side_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_neg_side_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_neg_side_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_neg_side_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_pos_yaw_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_pos_yaw_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_pos_yaw_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_neg_yaw_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_neg_yaw_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_neg_yaw_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_pos_xl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_pos_xl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_pos_xl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_neg_xl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_neg_xl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_neg_xl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_pos_side_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_pos_side_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_pos_side_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_neg_side_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_neg_side_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_neg_side_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_pos_yaw_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_pos_yaw_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_pos_yaw_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_neg_yaw_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_neg_yaw_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_neg_yaw_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_pos_side_r_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_pos_side_r_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_pos_side_r_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_neg_side_r_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_neg_side_r_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_neg_side_r_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_pos_side_l_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_pos_side_l_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_pos_side_l_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_neg_side_l_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->third_neg_side_l_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_neg_side_l_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_min
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->third_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // third_max
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->third_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "third_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_pos_xr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_pos_xr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_pos_xr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_neg_xr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_neg_xr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_neg_xr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_pos_side_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_pos_side_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_pos_side_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_neg_side_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_neg_side_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_neg_side_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_pos_yaw_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_pos_yaw_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_pos_yaw_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_neg_yaw_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_neg_yaw_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_neg_yaw_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_pos_xl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_pos_xl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_pos_xl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_neg_xl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_neg_xl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_neg_xl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_pos_side_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_pos_side_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_pos_side_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_neg_side_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_neg_side_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_neg_side_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_pos_yaw_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_pos_yaw_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_pos_yaw_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_neg_yaw_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_neg_yaw_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_neg_yaw_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_pos_side_r_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_pos_side_r_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_pos_side_r_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_neg_side_r_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_neg_side_r_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_neg_side_r_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_pos_side_l_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_pos_side_l_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_pos_side_l_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_neg_side_l_swing_minus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fourth_neg_side_l_swing_minus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_neg_side_l_swing_minus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_min
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->fourth_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fourth_max
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->fourth_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fourth_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // landing_time_control_flag
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->landing_time_control_flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "landing_time_control_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // control_system_flag
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->control_system_flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "control_system_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
