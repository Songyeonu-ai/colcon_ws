// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from humanoid_interfaces:msg/Motionend.idl
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
#include "humanoid_interfaces/msg/detail/motionend__struct.h"
#include "humanoid_interfaces/msg/detail/motionend__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool humanoid_interfaces__msg__motionend__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("humanoid_interfaces.msg._motionend.Motionend", full_classname_dest, 44) == 0);
  }
  humanoid_interfaces__msg__Motionend * ros_message = _ros_message;
  {  // motionend
    PyObject * field = PyObject_GetAttrString(_pymsg, "motionend");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motionend = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // motioning
    PyObject * field = PyObject_GetAttrString(_pymsg, "motioning");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motioning = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // motionstarting
    PyObject * field = PyObject_GetAttrString(_pymsg, "motionstarting");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motionstarting = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // motionending
    PyObject * field = PyObject_GetAttrString(_pymsg, "motionending");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motionending = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // motionnum
    PyObject * field = PyObject_GetAttrString(_pymsg, "motionnum");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motionnum = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * humanoid_interfaces__msg__motionend__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Motionend */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("humanoid_interfaces.msg._motionend");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Motionend");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  humanoid_interfaces__msg__Motionend * ros_message = (humanoid_interfaces__msg__Motionend *)raw_ros_message;
  {  // motionend
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motionend);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motionend", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motioning
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motioning);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motioning", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motionstarting
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motionstarting);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motionstarting", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motionending
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motionending);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motionending", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motionnum
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motionnum);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motionnum", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
