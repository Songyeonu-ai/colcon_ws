// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from humanoid_interfaces:msg/Gamecontroldata.idl
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
#include "humanoid_interfaces/msg/detail/gamecontroldata__struct.h"
#include "humanoid_interfaces/msg/detail/gamecontroldata__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool humanoid_interfaces__msg__gamecontroldata__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("humanoid_interfaces.msg._gamecontroldata.Gamecontroldata", full_classname_dest, 56) == 0);
  }
  humanoid_interfaces__msg__Gamecontroldata * ros_message = _ros_message;
  {  // robotnum
    PyObject * field = PyObject_GetAttrString(_pymsg, "robotnum");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robotnum = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // position
    PyObject * field = PyObject_GetAttrString(_pymsg, "position");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->position = PyLong_AsLongLong(field);
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
  {  // state
    PyObject * field = PyObject_GetAttrString(_pymsg, "state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->state = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // myside
    PyObject * field = PyObject_GetAttrString(_pymsg, "myside");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->myside = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // iskickoff
    PyObject * field = PyObject_GetAttrString(_pymsg, "iskickoff");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->iskickoff = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // readytime
    PyObject * field = PyObject_GetAttrString(_pymsg, "readytime");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->readytime = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // penalty
    PyObject * field = PyObject_GetAttrString(_pymsg, "penalty");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->penalty = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // secondstate
    PyObject * field = PyObject_GetAttrString(_pymsg, "secondstate");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->secondstate = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // secondinfo
    PyObject * field = PyObject_GetAttrString(_pymsg, "secondinfo");
    if (!field) {
      return false;
    }
    if (PyObject_CheckBuffer(field)) {
      // Optimization for converting arrays of primitives
      Py_buffer view;
      int rc = PyObject_GetBuffer(field, &view, PyBUF_SIMPLE);
      if (rc < 0) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = view.len / sizeof(int64_t);
      if (!rosidl_runtime_c__int64__Sequence__init(&(ros_message->secondinfo), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create int64__Sequence ros_message");
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      int64_t * dest = ros_message->secondinfo.data;
      rc = PyBuffer_ToContiguous(dest, &view, view.len, 'C');
      if (rc < 0) {
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      PyBuffer_Release(&view);
    } else {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'secondinfo'");
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
      if (!rosidl_runtime_c__int64__Sequence__init(&(ros_message->secondinfo), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create int64__Sequence ros_message");
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      int64_t * dest = ros_message->secondinfo.data;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyLong_Check(item));
        int64_t tmp = PyLong_AsLongLong(item);
        memcpy(&dest[i], &tmp, sizeof(int64_t));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * humanoid_interfaces__msg__gamecontroldata__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Gamecontroldata */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("humanoid_interfaces.msg._gamecontroldata");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Gamecontroldata");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  humanoid_interfaces__msg__Gamecontroldata * ros_message = (humanoid_interfaces__msg__Gamecontroldata *)raw_ros_message;
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
  {  // position
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position", field);
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
  {  // state
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // myside
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->myside);
    {
      int rc = PyObject_SetAttrString(_pymessage, "myside", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // iskickoff
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->iskickoff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "iskickoff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // readytime
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->readytime);
    {
      int rc = PyObject_SetAttrString(_pymessage, "readytime", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // penalty
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->penalty);
    {
      int rc = PyObject_SetAttrString(_pymessage, "penalty", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // secondstate
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->secondstate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "secondstate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // secondinfo
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "secondinfo");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "array.array") == 0);
    // ensure that itemsize matches the sizeof of the ROS message field
    PyObject * itemsize_attr = PyObject_GetAttrString(field, "itemsize");
    assert(itemsize_attr != NULL);
    size_t itemsize = PyLong_AsSize_t(itemsize_attr);
    Py_DECREF(itemsize_attr);
    if (itemsize != sizeof(int64_t)) {
      PyErr_SetString(PyExc_RuntimeError, "itemsize doesn't match expectation");
      Py_DECREF(field);
      return NULL;
    }
    // clear the array, poor approach to remove potential default values
    Py_ssize_t length = PyObject_Length(field);
    if (-1 == length) {
      Py_DECREF(field);
      return NULL;
    }
    if (length > 0) {
      PyObject * pop = PyObject_GetAttrString(field, "pop");
      assert(pop != NULL);
      for (Py_ssize_t i = 0; i < length; ++i) {
        PyObject * ret = PyObject_CallFunctionObjArgs(pop, NULL);
        if (!ret) {
          Py_DECREF(pop);
          Py_DECREF(field);
          return NULL;
        }
        Py_DECREF(ret);
      }
      Py_DECREF(pop);
    }
    if (ros_message->secondinfo.size > 0) {
      // populating the array.array using the frombytes method
      PyObject * frombytes = PyObject_GetAttrString(field, "frombytes");
      assert(frombytes != NULL);
      int64_t * src = &(ros_message->secondinfo.data[0]);
      PyObject * data = PyBytes_FromStringAndSize((const char *)src, ros_message->secondinfo.size * sizeof(int64_t));
      assert(data != NULL);
      PyObject * ret = PyObject_CallFunctionObjArgs(frombytes, data, NULL);
      Py_DECREF(data);
      Py_DECREF(frombytes);
      if (!ret) {
        Py_DECREF(field);
        return NULL;
      }
      Py_DECREF(ret);
    }
    Py_DECREF(field);
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
