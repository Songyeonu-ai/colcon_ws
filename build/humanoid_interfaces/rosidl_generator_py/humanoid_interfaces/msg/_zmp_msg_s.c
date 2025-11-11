// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from humanoid_interfaces:msg/ZmpMsg.idl
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
#include "humanoid_interfaces/msg/detail/zmp_msg__struct.h"
#include "humanoid_interfaces/msg/detail/zmp_msg__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool humanoid_interfaces__msg__zmp_msg__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[40];
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
    assert(strncmp("humanoid_interfaces.msg._zmp_msg.ZmpMsg", full_classname_dest, 39) == 0);
  }
  humanoid_interfaces__msg__ZmpMsg * ros_message = _ros_message;
  {  // left_x_zmp
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_x_zmp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_x_zmp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // left_y_zmp
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_y_zmp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_y_zmp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_x_zmp
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_x_zmp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_x_zmp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_y_zmp
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_y_zmp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_y_zmp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // total_x_zmp
    PyObject * field = PyObject_GetAttrString(_pymsg, "total_x_zmp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->total_x_zmp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // total_y_zmp
    PyObject * field = PyObject_GetAttrString(_pymsg, "total_y_zmp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->total_y_zmp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // left_foot
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_foot");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->left_foot = (Py_True == field);
    Py_DECREF(field);
  }
  {  // right_foot
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_foot");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->right_foot = (Py_True == field);
    Py_DECREF(field);
  }
  {  // both_feet
    PyObject * field = PyObject_GetAttrString(_pymsg, "both_feet");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->both_feet = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * humanoid_interfaces__msg__zmp_msg__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ZmpMsg */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("humanoid_interfaces.msg._zmp_msg");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ZmpMsg");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  humanoid_interfaces__msg__ZmpMsg * ros_message = (humanoid_interfaces__msg__ZmpMsg *)raw_ros_message;
  {  // left_x_zmp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_x_zmp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_x_zmp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_y_zmp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_y_zmp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_y_zmp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_x_zmp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_x_zmp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_x_zmp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_y_zmp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_y_zmp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_y_zmp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // total_x_zmp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->total_x_zmp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "total_x_zmp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // total_y_zmp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->total_y_zmp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "total_y_zmp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_foot
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->left_foot ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_foot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_foot
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->right_foot ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_foot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // both_feet
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->both_feet ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "both_feet", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
