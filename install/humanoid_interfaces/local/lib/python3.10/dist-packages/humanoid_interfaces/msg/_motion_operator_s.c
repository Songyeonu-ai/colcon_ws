// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from humanoid_interfaces:msg/MotionOperator.idl
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
#include "humanoid_interfaces/msg/detail/motion_operator__struct.h"
#include "humanoid_interfaces/msg/detail/motion_operator__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool humanoid_interfaces__msg__motion_operator__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[56];
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
    assert(strncmp("humanoid_interfaces.msg._motion_operator.MotionOperator", full_classname_dest, 55) == 0);
  }
  humanoid_interfaces__msg__MotionOperator * ros_message = _ros_message;
  {  // motion_num
    PyObject * field = PyObject_GetAttrString(_pymsg, "motion_num");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motion_num = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // motion_end
    PyObject * field = PyObject_GetAttrString(_pymsg, "motion_end");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motion_end = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * humanoid_interfaces__msg__motion_operator__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MotionOperator */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("humanoid_interfaces.msg._motion_operator");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MotionOperator");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  humanoid_interfaces__msg__MotionOperator * ros_message = (humanoid_interfaces__msg__MotionOperator *)raw_ros_message;
  {  // motion_num
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motion_num);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motion_num", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motion_end
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motion_end);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motion_end", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
