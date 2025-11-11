// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from humanoid_interfaces:msg/Master2IkMsg.idl
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
#include "humanoid_interfaces/msg/detail/master2_ik_msg__struct.h"
#include "humanoid_interfaces/msg/detail/master2_ik_msg__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool humanoid_interfaces__msg__master2_ik_msg__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[53];
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
    assert(strncmp("humanoid_interfaces.msg._master2_ik_msg.Master2IkMsg", full_classname_dest, 52) == 0);
  }
  humanoid_interfaces__msg__Master2IkMsg * ros_message = _ros_message;
  {  // x_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_length");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_length = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_length");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_length = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "flag");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->flag = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // one_x_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "one_x_length");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->one_x_length = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // one_y_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "one_y_length");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->one_y_length = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // one_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "one_yaw");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->one_yaw = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // one_step_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "one_step_flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->one_step_flag = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * humanoid_interfaces__msg__master2_ik_msg__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Master2IkMsg */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("humanoid_interfaces.msg._master2_ik_msg");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Master2IkMsg");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  humanoid_interfaces__msg__Master2IkMsg * ros_message = (humanoid_interfaces__msg__Master2IkMsg *)raw_ros_message;
  {  // x_length
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_length
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // flag
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // one_x_length
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->one_x_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "one_x_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // one_y_length
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->one_y_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "one_y_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // one_yaw
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->one_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "one_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // one_step_flag
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->one_step_flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "one_step_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
