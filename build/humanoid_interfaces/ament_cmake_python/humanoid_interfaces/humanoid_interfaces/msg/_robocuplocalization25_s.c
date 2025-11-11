// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from humanoid_interfaces:msg/Robocuplocalization25.idl
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
#include "humanoid_interfaces/msg/detail/robocuplocalization25__struct.h"
#include "humanoid_interfaces/msg/detail/robocuplocalization25__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool humanoid_interfaces__msg__robocuplocalization25__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[69];
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
    assert(strncmp("humanoid_interfaces.msg._robocuplocalization25.Robocuplocalization25", full_classname_dest, 68) == 0);
  }
  humanoid_interfaces__msg__Robocuplocalization25 * ros_message = _ros_message;
  {  // ball_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "ball_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ball_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ball_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "ball_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ball_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ball_speed_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "ball_speed_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ball_speed_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ball_speed_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "ball_speed_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ball_speed_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // robot_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "robot_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->robot_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // robot_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "robot_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->robot_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // obstacle_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "obstacle_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->obstacle_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // obstacle_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "obstacle_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->obstacle_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // obstacle0_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "obstacle0_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->obstacle0_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // obstacle0_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "obstacle0_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->obstacle0_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // obstacle1_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "obstacle1_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->obstacle1_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // obstacle1_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "obstacle1_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->obstacle1_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // obstacle2_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "obstacle2_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->obstacle2_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // obstacle2_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "obstacle2_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->obstacle2_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // obstacle3_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "obstacle3_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->obstacle3_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // obstacle3_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "obstacle3_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->obstacle3_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * humanoid_interfaces__msg__robocuplocalization25__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Robocuplocalization25 */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("humanoid_interfaces.msg._robocuplocalization25");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Robocuplocalization25");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  humanoid_interfaces__msg__Robocuplocalization25 * ros_message = (humanoid_interfaces__msg__Robocuplocalization25 *)raw_ros_message;
  {  // ball_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ball_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ball_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ball_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ball_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ball_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ball_speed_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ball_speed_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ball_speed_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ball_speed_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ball_speed_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ball_speed_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robot_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->robot_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robot_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robot_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->robot_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robot_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // obstacle_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->obstacle_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "obstacle_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // obstacle_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->obstacle_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "obstacle_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // obstacle0_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->obstacle0_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "obstacle0_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // obstacle0_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->obstacle0_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "obstacle0_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // obstacle1_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->obstacle1_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "obstacle1_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // obstacle1_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->obstacle1_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "obstacle1_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // obstacle2_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->obstacle2_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "obstacle2_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // obstacle2_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->obstacle2_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "obstacle2_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // obstacle3_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->obstacle3_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "obstacle3_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // obstacle3_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->obstacle3_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "obstacle3_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
