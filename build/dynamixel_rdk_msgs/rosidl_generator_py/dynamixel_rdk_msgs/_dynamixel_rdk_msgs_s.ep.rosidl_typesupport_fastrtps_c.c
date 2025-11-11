// generated from rosidl_generator_py/resource/_idl_pkg_typesupport_entry_point.c.em
// generated code does not contain a copyright notice
#include <Python.h>

static PyMethodDef dynamixel_rdk_msgs__methods[] = {
  {NULL, NULL, 0, NULL}  /* sentinel */
};

static struct PyModuleDef dynamixel_rdk_msgs__module = {
  PyModuleDef_HEAD_INIT,
  "_dynamixel_rdk_msgs_support",
  "_dynamixel_rdk_msgs_doc",
  -1,  /* -1 means that the module keeps state in global variables */
  dynamixel_rdk_msgs__methods,
  NULL,
  NULL,
  NULL,
  NULL,
};

#include <stdbool.h>
#include <stdint.h>
#include "rosidl_runtime_c/visibility_control.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__type_support.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__struct.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__functions.h"

static void * dynamixel_rdk_msgs__msg__dynamixel_msgs__create_ros_message(void)
{
  return dynamixel_rdk_msgs__msg__DynamixelMsgs__create();
}

static void dynamixel_rdk_msgs__msg__dynamixel_msgs__destroy_ros_message(void * raw_ros_message)
{
  dynamixel_rdk_msgs__msg__DynamixelMsgs * ros_message = (dynamixel_rdk_msgs__msg__DynamixelMsgs *)raw_ros_message;
  dynamixel_rdk_msgs__msg__DynamixelMsgs__destroy(ros_message);
}

ROSIDL_GENERATOR_C_IMPORT
bool dynamixel_rdk_msgs__msg__dynamixel_msgs__convert_from_py(PyObject * _pymsg, void * ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * dynamixel_rdk_msgs__msg__dynamixel_msgs__convert_to_py(void * raw_ros_message);


ROSIDL_GENERATOR_C_IMPORT
const rosidl_message_type_support_t *
ROSIDL_GET_MSG_TYPE_SUPPORT(dynamixel_rdk_msgs, msg, DynamixelMsgs);

int8_t
_register_msg_type__msg__dynamixel_msgs(PyObject * pymodule)
{
  int8_t err;

  PyObject * pyobject_create_ros_message = NULL;
  pyobject_create_ros_message = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_msgs__create_ros_message,
    NULL, NULL);
  if (!pyobject_create_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "create_ros_message_msg__msg__dynamixel_msgs",
    pyobject_create_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_create_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_destroy_ros_message = NULL;
  pyobject_destroy_ros_message = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_msgs__destroy_ros_message,
    NULL, NULL);
  if (!pyobject_destroy_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "destroy_ros_message_msg__msg__dynamixel_msgs",
    pyobject_destroy_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_destroy_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_from_py = NULL;
  pyobject_convert_from_py = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_msgs__convert_from_py,
    NULL, NULL);
  if (!pyobject_convert_from_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_from_py_msg__msg__dynamixel_msgs",
    pyobject_convert_from_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_from_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_to_py = NULL;
  pyobject_convert_to_py = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_msgs__convert_to_py,
    NULL, NULL);
  if (!pyobject_convert_to_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_to_py_msg__msg__dynamixel_msgs",
    pyobject_convert_to_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_to_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_type_support = NULL;
  pyobject_type_support = PyCapsule_New(
    (void *)ROSIDL_GET_MSG_TYPE_SUPPORT(dynamixel_rdk_msgs, msg, DynamixelMsgs),
    NULL, NULL);
  if (!pyobject_type_support) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "type_support_msg__msg__dynamixel_msgs",
    pyobject_type_support);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_type_support);
    // previously added objects will be removed when the module is destroyed
    return err;
  }
  return 0;
}

// already included above
// #include <stdbool.h>
// already included above
// #include <stdint.h>
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_runtime_c/action_type_support_struct.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_control_msgs__type_support.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_control_msgs__struct.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_control_msgs__functions.h"

static void * dynamixel_rdk_msgs__msg__dynamixel_control_msgs__create_ros_message(void)
{
  return dynamixel_rdk_msgs__msg__DynamixelControlMsgs__create();
}

static void dynamixel_rdk_msgs__msg__dynamixel_control_msgs__destroy_ros_message(void * raw_ros_message)
{
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs * ros_message = (dynamixel_rdk_msgs__msg__DynamixelControlMsgs *)raw_ros_message;
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs__destroy(ros_message);
}

ROSIDL_GENERATOR_C_IMPORT
bool dynamixel_rdk_msgs__msg__dynamixel_control_msgs__convert_from_py(PyObject * _pymsg, void * ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * dynamixel_rdk_msgs__msg__dynamixel_control_msgs__convert_to_py(void * raw_ros_message);


ROSIDL_GENERATOR_C_IMPORT
const rosidl_message_type_support_t *
ROSIDL_GET_MSG_TYPE_SUPPORT(dynamixel_rdk_msgs, msg, DynamixelControlMsgs);

int8_t
_register_msg_type__msg__dynamixel_control_msgs(PyObject * pymodule)
{
  int8_t err;

  PyObject * pyobject_create_ros_message = NULL;
  pyobject_create_ros_message = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_control_msgs__create_ros_message,
    NULL, NULL);
  if (!pyobject_create_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "create_ros_message_msg__msg__dynamixel_control_msgs",
    pyobject_create_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_create_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_destroy_ros_message = NULL;
  pyobject_destroy_ros_message = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_control_msgs__destroy_ros_message,
    NULL, NULL);
  if (!pyobject_destroy_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "destroy_ros_message_msg__msg__dynamixel_control_msgs",
    pyobject_destroy_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_destroy_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_from_py = NULL;
  pyobject_convert_from_py = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_control_msgs__convert_from_py,
    NULL, NULL);
  if (!pyobject_convert_from_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_from_py_msg__msg__dynamixel_control_msgs",
    pyobject_convert_from_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_from_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_to_py = NULL;
  pyobject_convert_to_py = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_control_msgs__convert_to_py,
    NULL, NULL);
  if (!pyobject_convert_to_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_to_py_msg__msg__dynamixel_control_msgs",
    pyobject_convert_to_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_to_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_type_support = NULL;
  pyobject_type_support = PyCapsule_New(
    (void *)ROSIDL_GET_MSG_TYPE_SUPPORT(dynamixel_rdk_msgs, msg, DynamixelControlMsgs),
    NULL, NULL);
  if (!pyobject_type_support) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "type_support_msg__msg__dynamixel_control_msgs",
    pyobject_type_support);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_type_support);
    // previously added objects will be removed when the module is destroyed
    return err;
  }
  return 0;
}

// already included above
// #include <stdbool.h>
// already included above
// #include <stdint.h>
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_runtime_c/action_type_support_struct.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__type_support.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__struct.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__functions.h"

static void * dynamixel_rdk_msgs__msg__dynamixel_status_msgs__create_ros_message(void)
{
  return dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__create();
}

static void dynamixel_rdk_msgs__msg__dynamixel_status_msgs__destroy_ros_message(void * raw_ros_message)
{
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * ros_message = (dynamixel_rdk_msgs__msg__DynamixelStatusMsgs *)raw_ros_message;
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__destroy(ros_message);
}

ROSIDL_GENERATOR_C_IMPORT
bool dynamixel_rdk_msgs__msg__dynamixel_status_msgs__convert_from_py(PyObject * _pymsg, void * ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * dynamixel_rdk_msgs__msg__dynamixel_status_msgs__convert_to_py(void * raw_ros_message);


ROSIDL_GENERATOR_C_IMPORT
const rosidl_message_type_support_t *
ROSIDL_GET_MSG_TYPE_SUPPORT(dynamixel_rdk_msgs, msg, DynamixelStatusMsgs);

int8_t
_register_msg_type__msg__dynamixel_status_msgs(PyObject * pymodule)
{
  int8_t err;

  PyObject * pyobject_create_ros_message = NULL;
  pyobject_create_ros_message = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_status_msgs__create_ros_message,
    NULL, NULL);
  if (!pyobject_create_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "create_ros_message_msg__msg__dynamixel_status_msgs",
    pyobject_create_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_create_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_destroy_ros_message = NULL;
  pyobject_destroy_ros_message = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_status_msgs__destroy_ros_message,
    NULL, NULL);
  if (!pyobject_destroy_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "destroy_ros_message_msg__msg__dynamixel_status_msgs",
    pyobject_destroy_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_destroy_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_from_py = NULL;
  pyobject_convert_from_py = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_status_msgs__convert_from_py,
    NULL, NULL);
  if (!pyobject_convert_from_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_from_py_msg__msg__dynamixel_status_msgs",
    pyobject_convert_from_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_from_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_to_py = NULL;
  pyobject_convert_to_py = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_status_msgs__convert_to_py,
    NULL, NULL);
  if (!pyobject_convert_to_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_to_py_msg__msg__dynamixel_status_msgs",
    pyobject_convert_to_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_to_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_type_support = NULL;
  pyobject_type_support = PyCapsule_New(
    (void *)ROSIDL_GET_MSG_TYPE_SUPPORT(dynamixel_rdk_msgs, msg, DynamixelStatusMsgs),
    NULL, NULL);
  if (!pyobject_type_support) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "type_support_msg__msg__dynamixel_status_msgs",
    pyobject_type_support);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_type_support);
    // previously added objects will be removed when the module is destroyed
    return err;
  }
  return 0;
}

// already included above
// #include <stdbool.h>
// already included above
// #include <stdint.h>
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_runtime_c/action_type_support_struct.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_bulk_read_msgs__type_support.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_bulk_read_msgs__struct.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_bulk_read_msgs__functions.h"

static void * dynamixel_rdk_msgs__msg__dynamixel_bulk_read_msgs__create_ros_message(void)
{
  return dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__create();
}

static void dynamixel_rdk_msgs__msg__dynamixel_bulk_read_msgs__destroy_ros_message(void * raw_ros_message)
{
  dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs * ros_message = (dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs *)raw_ros_message;
  dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__destroy(ros_message);
}

ROSIDL_GENERATOR_C_IMPORT
bool dynamixel_rdk_msgs__msg__dynamixel_bulk_read_msgs__convert_from_py(PyObject * _pymsg, void * ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * dynamixel_rdk_msgs__msg__dynamixel_bulk_read_msgs__convert_to_py(void * raw_ros_message);


ROSIDL_GENERATOR_C_IMPORT
const rosidl_message_type_support_t *
ROSIDL_GET_MSG_TYPE_SUPPORT(dynamixel_rdk_msgs, msg, DynamixelBulkReadMsgs);

int8_t
_register_msg_type__msg__dynamixel_bulk_read_msgs(PyObject * pymodule)
{
  int8_t err;

  PyObject * pyobject_create_ros_message = NULL;
  pyobject_create_ros_message = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_bulk_read_msgs__create_ros_message,
    NULL, NULL);
  if (!pyobject_create_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "create_ros_message_msg__msg__dynamixel_bulk_read_msgs",
    pyobject_create_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_create_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_destroy_ros_message = NULL;
  pyobject_destroy_ros_message = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_bulk_read_msgs__destroy_ros_message,
    NULL, NULL);
  if (!pyobject_destroy_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "destroy_ros_message_msg__msg__dynamixel_bulk_read_msgs",
    pyobject_destroy_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_destroy_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_from_py = NULL;
  pyobject_convert_from_py = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_bulk_read_msgs__convert_from_py,
    NULL, NULL);
  if (!pyobject_convert_from_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_from_py_msg__msg__dynamixel_bulk_read_msgs",
    pyobject_convert_from_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_from_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_to_py = NULL;
  pyobject_convert_to_py = PyCapsule_New(
    (void *)&dynamixel_rdk_msgs__msg__dynamixel_bulk_read_msgs__convert_to_py,
    NULL, NULL);
  if (!pyobject_convert_to_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_to_py_msg__msg__dynamixel_bulk_read_msgs",
    pyobject_convert_to_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_to_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_type_support = NULL;
  pyobject_type_support = PyCapsule_New(
    (void *)ROSIDL_GET_MSG_TYPE_SUPPORT(dynamixel_rdk_msgs, msg, DynamixelBulkReadMsgs),
    NULL, NULL);
  if (!pyobject_type_support) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "type_support_msg__msg__dynamixel_bulk_read_msgs",
    pyobject_type_support);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_type_support);
    // previously added objects will be removed when the module is destroyed
    return err;
  }
  return 0;
}

PyMODINIT_FUNC
PyInit_dynamixel_rdk_msgs_s__rosidl_typesupport_fastrtps_c(void)
{
  PyObject * pymodule = NULL;
  pymodule = PyModule_Create(&dynamixel_rdk_msgs__module);
  if (!pymodule) {
    return NULL;
  }
  int8_t err;

  err = _register_msg_type__msg__dynamixel_msgs(pymodule);
  if (err) {
    Py_XDECREF(pymodule);
    return NULL;
  }

  err = _register_msg_type__msg__dynamixel_control_msgs(pymodule);
  if (err) {
    Py_XDECREF(pymodule);
    return NULL;
  }

  err = _register_msg_type__msg__dynamixel_status_msgs(pymodule);
  if (err) {
    Py_XDECREF(pymodule);
    return NULL;
  }

  err = _register_msg_type__msg__dynamixel_bulk_read_msgs(pymodule);
  if (err) {
    Py_XDECREF(pymodule);
    return NULL;
  }

  return pymodule;
}
