// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from humanoid_interfaces:msg/ImuMsg.idl
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
#include "humanoid_interfaces/msg/detail/imu_msg__struct.h"
#include "humanoid_interfaces/msg/detail/imu_msg__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool humanoid_interfaces__msg__imu_msg__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[40];
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
    assert(strncmp("humanoid_interfaces.msg._imu_msg.ImuMsg", full_classname_dest, 39) == 0);
  }
  humanoid_interfaces__msg__ImuMsg * ros_message = _ros_message;
  {  // pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // roll
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // q_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "q_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->q_x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // q_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "q_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->q_y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // q_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "q_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->q_z = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // q_w
    PyObject * field = PyObject_GetAttrString(_pymsg, "q_w");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->q_w = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // x_gyro
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_gyro");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_gyro = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_gyro
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_gyro");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_gyro = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_gyro
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_gyro");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_gyro = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // x_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_acc");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_acc = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_acc");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_acc = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_acc");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_acc = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // x_vel
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_vel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_vel = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_vel
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_vel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_vel = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_vel
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_vel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_vel = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // x_pos
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_pos");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_pos = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_pos
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_pos");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_pos = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_pos
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_pos");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_pos = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * humanoid_interfaces__msg__imu_msg__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ImuMsg */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("humanoid_interfaces.msg._imu_msg");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ImuMsg");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  humanoid_interfaces__msg__ImuMsg * ros_message = (humanoid_interfaces__msg__ImuMsg *)raw_ros_message;
  {  // pitch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll", field);
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
  {  // q_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->q_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "q_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // q_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->q_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "q_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // q_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->q_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "q_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // q_w
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->q_w);
    {
      int rc = PyObject_SetAttrString(_pymessage, "q_w", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_gyro
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_gyro);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_gyro", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_gyro
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_gyro);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_gyro", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_gyro
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_gyro);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_gyro", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_acc
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_acc
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_acc
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_vel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_vel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_vel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_vel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_vel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_vel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_vel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_vel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_vel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_pos
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_pos);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_pos", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_pos
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_pos);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_pos", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_pos
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_pos);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_pos", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
