// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from dynamixel_rdk_msgs:msg/DynamixelStatusMsgs.idl
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
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__struct.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool dynamixel_rdk_msgs__msg__dynamixel_status_msgs__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[66];
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
    assert(strncmp("dynamixel_rdk_msgs.msg._dynamixel_status_msgs.DynamixelStatusMsgs", full_classname_dest, 65) == 0);
  }
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // torque_enabled
    PyObject * field = PyObject_GetAttrString(_pymsg, "torque_enabled");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->torque_enabled = (Py_True == field);
    Py_DECREF(field);
  }
  {  // error_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->error_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // present_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "present_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->present_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // present_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "present_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->present_velocity = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // present_acceleration
    PyObject * field = PyObject_GetAttrString(_pymsg, "present_acceleration");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->present_acceleration = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // present_current
    PyObject * field = PyObject_GetAttrString(_pymsg, "present_current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->present_current = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // present_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "present_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->present_voltage = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // present_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "present_temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->present_temperature = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // min_max_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "min_max_position");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 2;
      double * dest = ros_message->min_max_position;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * dynamixel_rdk_msgs__msg__dynamixel_status_msgs__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DynamixelStatusMsgs */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("dynamixel_rdk_msgs.msg._dynamixel_status_msgs");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DynamixelStatusMsgs");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * ros_message = (dynamixel_rdk_msgs__msg__DynamixelStatusMsgs *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // torque_enabled
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->torque_enabled ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "torque_enabled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->error_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // present_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->present_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "present_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // present_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->present_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "present_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // present_acceleration
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->present_acceleration);
    {
      int rc = PyObject_SetAttrString(_pymessage, "present_acceleration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // present_current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->present_current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "present_current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // present_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->present_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "present_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // present_temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->present_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "present_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min_max_position
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "min_max_position");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->min_max_position[0]);
    memcpy(dst, src, 2 * sizeof(double));
    Py_DECREF(field);
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
