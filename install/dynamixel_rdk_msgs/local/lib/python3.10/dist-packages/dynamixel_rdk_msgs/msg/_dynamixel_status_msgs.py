# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dynamixel_rdk_msgs:msg/DynamixelStatusMsgs.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'min_max_position'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DynamixelStatusMsgs(type):
    """Metaclass of message 'DynamixelStatusMsgs'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('dynamixel_rdk_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dynamixel_rdk_msgs.msg.DynamixelStatusMsgs')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__dynamixel_status_msgs
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__dynamixel_status_msgs
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__dynamixel_status_msgs
            cls._TYPE_SUPPORT = module.type_support_msg__msg__dynamixel_status_msgs
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__dynamixel_status_msgs

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DynamixelStatusMsgs(metaclass=Metaclass_DynamixelStatusMsgs):
    """Message class 'DynamixelStatusMsgs'."""

    __slots__ = [
        '_header',
        '_id',
        '_torque_enabled',
        '_error_status',
        '_present_position',
        '_present_velocity',
        '_present_acceleration',
        '_present_current',
        '_present_voltage',
        '_present_temperature',
        '_min_max_position',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'id': 'int32',
        'torque_enabled': 'boolean',
        'error_status': 'uint8',
        'present_position': 'double',
        'present_velocity': 'double',
        'present_acceleration': 'double',
        'present_current': 'double',
        'present_voltage': 'double',
        'present_temperature': 'double',
        'min_max_position': 'double[2]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 2),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.id = kwargs.get('id', int())
        self.torque_enabled = kwargs.get('torque_enabled', bool())
        self.error_status = kwargs.get('error_status', int())
        self.present_position = kwargs.get('present_position', float())
        self.present_velocity = kwargs.get('present_velocity', float())
        self.present_acceleration = kwargs.get('present_acceleration', float())
        self.present_current = kwargs.get('present_current', float())
        self.present_voltage = kwargs.get('present_voltage', float())
        self.present_temperature = kwargs.get('present_temperature', float())
        if 'min_max_position' not in kwargs:
            self.min_max_position = numpy.zeros(2, dtype=numpy.float64)
        else:
            self.min_max_position = numpy.array(kwargs.get('min_max_position'), dtype=numpy.float64)
            assert self.min_max_position.shape == (2, )

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.id != other.id:
            return False
        if self.torque_enabled != other.torque_enabled:
            return False
        if self.error_status != other.error_status:
            return False
        if self.present_position != other.present_position:
            return False
        if self.present_velocity != other.present_velocity:
            return False
        if self.present_acceleration != other.present_acceleration:
            return False
        if self.present_current != other.present_current:
            return False
        if self.present_voltage != other.present_voltage:
            return False
        if self.present_temperature != other.present_temperature:
            return False
        if any(self.min_max_position != other.min_max_position):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'id' field must be an integer in [-2147483648, 2147483647]"
        self._id = value

    @builtins.property
    def torque_enabled(self):
        """Message field 'torque_enabled'."""
        return self._torque_enabled

    @torque_enabled.setter
    def torque_enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'torque_enabled' field must be of type 'bool'"
        self._torque_enabled = value

    @builtins.property
    def error_status(self):
        """Message field 'error_status'."""
        return self._error_status

    @error_status.setter
    def error_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'error_status' field must be an unsigned integer in [0, 255]"
        self._error_status = value

    @builtins.property
    def present_position(self):
        """Message field 'present_position'."""
        return self._present_position

    @present_position.setter
    def present_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'present_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'present_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._present_position = value

    @builtins.property
    def present_velocity(self):
        """Message field 'present_velocity'."""
        return self._present_velocity

    @present_velocity.setter
    def present_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'present_velocity' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'present_velocity' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._present_velocity = value

    @builtins.property
    def present_acceleration(self):
        """Message field 'present_acceleration'."""
        return self._present_acceleration

    @present_acceleration.setter
    def present_acceleration(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'present_acceleration' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'present_acceleration' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._present_acceleration = value

    @builtins.property
    def present_current(self):
        """Message field 'present_current'."""
        return self._present_current

    @present_current.setter
    def present_current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'present_current' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'present_current' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._present_current = value

    @builtins.property
    def present_voltage(self):
        """Message field 'present_voltage'."""
        return self._present_voltage

    @present_voltage.setter
    def present_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'present_voltage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'present_voltage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._present_voltage = value

    @builtins.property
    def present_temperature(self):
        """Message field 'present_temperature'."""
        return self._present_temperature

    @present_temperature.setter
    def present_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'present_temperature' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'present_temperature' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._present_temperature = value

    @builtins.property
    def min_max_position(self):
        """Message field 'min_max_position'."""
        return self._min_max_position

    @min_max_position.setter
    def min_max_position(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'min_max_position' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 2, \
                "The 'min_max_position' numpy.ndarray() must have a size of 2"
            self._min_max_position = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 2 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'min_max_position' field must be a set or sequence with length 2 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._min_max_position = numpy.array(value, dtype=numpy.float64)
