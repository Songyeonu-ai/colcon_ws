// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from humanoid_interfaces:msg/Master2udp.idl
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
#include "humanoid_interfaces/msg/detail/master2udp__struct.h"
#include "humanoid_interfaces/msg/detail/master2udp__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool humanoid_interfaces__msg__master2udp__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[47];
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
    assert(strncmp("humanoid_interfaces.msg._master2udp.Master2udp", full_classname_dest, 46) == 0);
  }
  humanoid_interfaces__msg__Master2udp * ros_message = _ros_message;
  {  // robotnum
    PyObject * field = PyObject_GetAttrString(_pymsg, "robotnum");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robotnum = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // robotstate
    PyObject * field = PyObject_GetAttrString(_pymsg, "robotstate");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robotstate = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // robotcoorx
    PyObject * field = PyObject_GetAttrString(_pymsg, "robotcoorx");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robotcoorx = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // robotcoory
    PyObject * field = PyObject_GetAttrString(_pymsg, "robotcoory");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robotcoory = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // robotimuyaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "robotimuyaw");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robotimuyaw = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // balldist
    PyObject * field = PyObject_GetAttrString(_pymsg, "balldist");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->balldist = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // ballcoorx
    PyObject * field = PyObject_GetAttrString(_pymsg, "ballcoorx");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ballcoorx = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // ballcoory
    PyObject * field = PyObject_GetAttrString(_pymsg, "ballcoory");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ballcoory = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // myteam
    PyObject * field = PyObject_GetAttrString(_pymsg, "myteam");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->myteam = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * humanoid_interfaces__msg__master2udp__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Master2udp */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("humanoid_interfaces.msg._master2udp");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Master2udp");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  humanoid_interfaces__msg__Master2udp * ros_message = (humanoid_interfaces__msg__Master2udp *)raw_ros_message;
  {  // robotnum
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->robotnum);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robotnum", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robotstate
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->robotstate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robotstate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robotcoorx
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->robotcoorx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robotcoorx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robotcoory
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->robotcoory);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robotcoory", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robotimuyaw
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->robotimuyaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robotimuyaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balldist
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->balldist);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balldist", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ballcoorx
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->ballcoorx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ballcoorx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ballcoory
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->ballcoory);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ballcoory", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // myteam
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->myteam);
    {
      int rc = PyObject_SetAttrString(_pymessage, "myteam", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
