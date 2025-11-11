// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from humanoid_interfaces:msg/Udp2master.idl
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
#include "humanoid_interfaces/msg/detail/udp2master__struct.h"
#include "humanoid_interfaces/msg/detail/udp2master__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool humanoid_interfaces__msg__udp2master__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("humanoid_interfaces.msg._udp2master.Udp2master", full_classname_dest, 46) == 0);
  }
  humanoid_interfaces__msg__Udp2master * ros_message = _ros_message;
  {  // robotnum
    PyObject * field = PyObject_GetAttrString(_pymsg, "robotnum");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robotnum = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // robotcase
    PyObject * field = PyObject_GetAttrString(_pymsg, "robotcase");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robotcase = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // localx
    PyObject * field = PyObject_GetAttrString(_pymsg, "localx");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->localx = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // localy
    PyObject * field = PyObject_GetAttrString(_pymsg, "localy");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->localy = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // localyaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "localyaw");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->localyaw = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // ballx
    PyObject * field = PyObject_GetAttrString(_pymsg, "ballx");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ballx = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // bally
    PyObject * field = PyObject_GetAttrString(_pymsg, "bally");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->bally = PyLong_AsLongLong(field);
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
  {  // balltheta
    PyObject * field = PyObject_GetAttrString(_pymsg, "balltheta");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->balltheta = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * humanoid_interfaces__msg__udp2master__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Udp2master */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("humanoid_interfaces.msg._udp2master");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Udp2master");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  humanoid_interfaces__msg__Udp2master * ros_message = (humanoid_interfaces__msg__Udp2master *)raw_ros_message;
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
  {  // robotcase
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->robotcase);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robotcase", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // localx
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->localx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "localx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // localy
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->localy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "localy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // localyaw
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->localyaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "localyaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ballx
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->ballx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ballx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bally
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->bally);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bally", field);
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
  {  // balltheta
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->balltheta);
    {
      int rc = PyObject_SetAttrString(_pymessage, "balltheta", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
