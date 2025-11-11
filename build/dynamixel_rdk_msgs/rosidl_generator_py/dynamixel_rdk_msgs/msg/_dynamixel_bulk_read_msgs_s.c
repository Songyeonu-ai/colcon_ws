// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from dynamixel_rdk_msgs:msg/DynamixelBulkReadMsgs.idl
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
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_bulk_read_msgs__struct.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_bulk_read_msgs__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__functions.h"
// end nested array functions include
bool dynamixel_rdk_msgs__msg__dynamixel_status_msgs__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * dynamixel_rdk_msgs__msg__dynamixel_status_msgs__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool dynamixel_rdk_msgs__msg__dynamixel_bulk_read_msgs__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[71];
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
    assert(strncmp("dynamixel_rdk_msgs.msg._dynamixel_bulk_read_msgs.DynamixelBulkReadMsgs", full_classname_dest, 70) == 0);
  }
  dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs * ros_message = _ros_message;
  {  // status_msgs
    PyObject * field = PyObject_GetAttrString(_pymsg, "status_msgs");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'status_msgs'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence__init(&(ros_message->status_msgs), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * dest = ros_message->status_msgs.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!dynamixel_rdk_msgs__msg__dynamixel_status_msgs__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * dynamixel_rdk_msgs__msg__dynamixel_bulk_read_msgs__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DynamixelBulkReadMsgs */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("dynamixel_rdk_msgs.msg._dynamixel_bulk_read_msgs");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DynamixelBulkReadMsgs");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs * ros_message = (dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs *)raw_ros_message;
  {  // status_msgs
    PyObject * field = NULL;
    size_t size = ros_message->status_msgs.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->status_msgs.data[i]);
      PyObject * pyitem = dynamixel_rdk_msgs__msg__dynamixel_status_msgs__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "status_msgs", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
