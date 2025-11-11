// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from humanoid_interfaces:msg/Master2localization25.idl
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
#include "humanoid_interfaces/msg/detail/master2localization25__struct.h"
#include "humanoid_interfaces/msg/detail/master2localization25__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool humanoid_interfaces__msg__master2localization25__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("humanoid_interfaces.msg._master2localization25.Master2localization25", full_classname_dest, 68) == 0);
  }
  humanoid_interfaces__msg__Master2localization25 * ros_message = _ros_message;
  {  // goalx
    PyObject * field = PyObject_GetAttrString(_pymsg, "goalx");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->goalx = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // goaly
    PyObject * field = PyObject_GetAttrString(_pymsg, "goaly");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->goaly = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // targetx
    PyObject * field = PyObject_GetAttrString(_pymsg, "targetx");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->targetx = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // targety
    PyObject * field = PyObject_GetAttrString(_pymsg, "targety");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->targety = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // robotallowanceerror
    PyObject * field = PyObject_GetAttrString(_pymsg, "robotallowanceerror");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robotallowanceerror = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // robotcaliradius
    PyObject * field = PyObject_GetAttrString(_pymsg, "robotcaliradius");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robotcaliradius = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * humanoid_interfaces__msg__master2localization25__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Master2localization25 */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("humanoid_interfaces.msg._master2localization25");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Master2localization25");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  humanoid_interfaces__msg__Master2localization25 * ros_message = (humanoid_interfaces__msg__Master2localization25 *)raw_ros_message;
  {  // goalx
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->goalx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "goalx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // goaly
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->goaly);
    {
      int rc = PyObject_SetAttrString(_pymessage, "goaly", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // targetx
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->targetx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "targetx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // targety
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->targety);
    {
      int rc = PyObject_SetAttrString(_pymessage, "targety", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robotallowanceerror
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->robotallowanceerror);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robotallowanceerror", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robotcaliradius
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->robotcaliradius);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robotcaliradius", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
