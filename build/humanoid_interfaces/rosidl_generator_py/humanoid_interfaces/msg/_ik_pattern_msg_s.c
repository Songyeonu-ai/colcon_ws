// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from humanoid_interfaces:msg/IkPatternMsg.idl
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
#include "humanoid_interfaces/msg/detail/ik_pattern_msg__struct.h"
#include "humanoid_interfaces/msg/detail/ik_pattern_msg__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool humanoid_interfaces__msg__ik_pattern_msg__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("humanoid_interfaces.msg._ik_pattern_msg.IkPatternMsg", full_classname_dest, 52) == 0);
  }
  humanoid_interfaces__msg__IkPatternMsg * ros_message = _ros_message;
  {  // xlpattern
    PyObject * field = PyObject_GetAttrString(_pymsg, "xlpattern");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->xlpattern = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ylpattern
    PyObject * field = PyObject_GetAttrString(_pymsg, "ylpattern");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ylpattern = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // zlpattern
    PyObject * field = PyObject_GetAttrString(_pymsg, "zlpattern");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->zlpattern = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // slpattern
    PyObject * field = PyObject_GetAttrString(_pymsg, "slpattern");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->slpattern = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tlpattern
    PyObject * field = PyObject_GetAttrString(_pymsg, "tlpattern");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tlpattern = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // xrpattern
    PyObject * field = PyObject_GetAttrString(_pymsg, "xrpattern");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->xrpattern = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yrpattern
    PyObject * field = PyObject_GetAttrString(_pymsg, "yrpattern");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yrpattern = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // zrpattern
    PyObject * field = PyObject_GetAttrString(_pymsg, "zrpattern");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->zrpattern = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // srpattern
    PyObject * field = PyObject_GetAttrString(_pymsg, "srpattern");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->srpattern = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // trpattern
    PyObject * field = PyObject_GetAttrString(_pymsg, "trpattern");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->trpattern = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * humanoid_interfaces__msg__ik_pattern_msg__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of IkPatternMsg */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("humanoid_interfaces.msg._ik_pattern_msg");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "IkPatternMsg");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  humanoid_interfaces__msg__IkPatternMsg * ros_message = (humanoid_interfaces__msg__IkPatternMsg *)raw_ros_message;
  {  // xlpattern
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->xlpattern);
    {
      int rc = PyObject_SetAttrString(_pymessage, "xlpattern", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ylpattern
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ylpattern);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ylpattern", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // zlpattern
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->zlpattern);
    {
      int rc = PyObject_SetAttrString(_pymessage, "zlpattern", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // slpattern
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->slpattern);
    {
      int rc = PyObject_SetAttrString(_pymessage, "slpattern", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tlpattern
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tlpattern);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tlpattern", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // xrpattern
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->xrpattern);
    {
      int rc = PyObject_SetAttrString(_pymessage, "xrpattern", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yrpattern
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yrpattern);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yrpattern", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // zrpattern
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->zrpattern);
    {
      int rc = PyObject_SetAttrString(_pymessage, "zrpattern", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // srpattern
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->srpattern);
    {
      int rc = PyObject_SetAttrString(_pymessage, "srpattern", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // trpattern
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->trpattern);
    {
      int rc = PyObject_SetAttrString(_pymessage, "trpattern", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
