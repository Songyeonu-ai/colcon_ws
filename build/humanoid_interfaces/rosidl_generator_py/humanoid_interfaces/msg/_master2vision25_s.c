// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from humanoid_interfaces:msg/Master2vision25.idl
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
#include "humanoid_interfaces/msg/detail/master2vision25__struct.h"
#include "humanoid_interfaces/msg/detail/master2vision25__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool humanoid_interfaces__msg__master2vision25__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[57];
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
    assert(strncmp("humanoid_interfaces.msg._master2vision25.Master2vision25", full_classname_dest, 56) == 0);
  }
  humanoid_interfaces__msg__Master2vision25 * ros_message = _ros_message;
  {  // scanmode
    PyObject * field = PyObject_GetAttrString(_pymsg, "scanmode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->scanmode = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // pan
    PyObject * field = PyObject_GetAttrString(_pymsg, "pan");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pan = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tilt
    PyObject * field = PyObject_GetAttrString(_pymsg, "tilt");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tilt = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * humanoid_interfaces__msg__master2vision25__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Master2vision25 */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("humanoid_interfaces.msg._master2vision25");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Master2vision25");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  humanoid_interfaces__msg__Master2vision25 * ros_message = (humanoid_interfaces__msg__Master2vision25 *)raw_ros_message;
  {  // scanmode
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->scanmode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "scanmode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pan
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pan);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pan", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tilt
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tilt);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tilt", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
