// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from humanoid_interfaces:msg/IkLTCMsg.idl
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
#include "humanoid_interfaces/msg/detail/ik_ltc_msg__struct.h"
#include "humanoid_interfaces/msg/detail/ik_ltc_msg__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool humanoid_interfaces__msg__ik_ltc_msg__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[45];
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
    assert(strncmp("humanoid_interfaces.msg._ik_ltc_msg.IkLTCMsg", full_classname_dest, 44) == 0);
  }
  humanoid_interfaces__msg__IkLTCMsg * ros_message = _ros_message;
  {  // entire_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "entire_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->entire_time = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // swing_gain_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "swing_gain_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->swing_gain_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // swing_gain_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "swing_gain_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->swing_gain_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // warning
    PyObject * field = PyObject_GetAttrString(_pymsg, "warning");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->warning = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // safe
    PyObject * field = PyObject_GetAttrString(_pymsg, "safe");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->safe = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // landing_time_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "landing_time_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->landing_time_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // landing_time_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "landing_time_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->landing_time_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // landing_error_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "landing_error_l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->landing_error_l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // landing_error_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "landing_error_r");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->landing_error_r = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * humanoid_interfaces__msg__ik_ltc_msg__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of IkLTCMsg */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("humanoid_interfaces.msg._ik_ltc_msg");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "IkLTCMsg");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  humanoid_interfaces__msg__IkLTCMsg * ros_message = (humanoid_interfaces__msg__IkLTCMsg *)raw_ros_message;
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
  {  // swing_gain_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->swing_gain_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "swing_gain_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // swing_gain_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->swing_gain_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "swing_gain_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // warning
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->warning);
    {
      int rc = PyObject_SetAttrString(_pymessage, "warning", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // safe
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->safe);
    {
      int rc = PyObject_SetAttrString(_pymessage, "safe", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // landing_time_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->landing_time_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "landing_time_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // landing_time_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->landing_time_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "landing_time_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // landing_error_l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->landing_error_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "landing_error_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // landing_error_r
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->landing_error_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "landing_error_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
