# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/Robocupvision25.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'robot_vec_x'
# Member 'robot_vec_y'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Robocupvision25(type):
    """Metaclass of message 'Robocupvision25'."""

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
            module = import_type_support('humanoid_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'humanoid_interfaces.msg.Robocupvision25')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robocupvision25
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robocupvision25
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robocupvision25
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robocupvision25
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robocupvision25

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Robocupvision25(metaclass=Metaclass_Robocupvision25):
    """Message class 'Robocupvision25'."""

    __slots__ = [
        '_ball_cam_x',
        '_ball_cam_y',
        '_ball_2d_x',
        '_ball_2d_y',
        '_ball_d',
        '_pan',
        '_tilt',
        '_ball_speed_x',
        '_ball_speed_y',
        '_robot_vec_x',
        '_robot_vec_y',
        '_ball_speed_level',
        '_scan_mode',
    ]

    _fields_and_field_types = {
        'ball_cam_x': 'int64',
        'ball_cam_y': 'int64',
        'ball_2d_x': 'double',
        'ball_2d_y': 'double',
        'ball_d': 'double',
        'pan': 'double',
        'tilt': 'double',
        'ball_speed_x': 'double',
        'ball_speed_y': 'double',
        'robot_vec_x': 'sequence<double>',
        'robot_vec_y': 'sequence<double>',
        'ball_speed_level': 'int64',
        'scan_mode': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ball_cam_x = kwargs.get('ball_cam_x', int())
        self.ball_cam_y = kwargs.get('ball_cam_y', int())
        self.ball_2d_x = kwargs.get('ball_2d_x', float())
        self.ball_2d_y = kwargs.get('ball_2d_y', float())
        self.ball_d = kwargs.get('ball_d', float())
        self.pan = kwargs.get('pan', float())
        self.tilt = kwargs.get('tilt', float())
        self.ball_speed_x = kwargs.get('ball_speed_x', float())
        self.ball_speed_y = kwargs.get('ball_speed_y', float())
        self.robot_vec_x = array.array('d', kwargs.get('robot_vec_x', []))
        self.robot_vec_y = array.array('d', kwargs.get('robot_vec_y', []))
        self.ball_speed_level = kwargs.get('ball_speed_level', int())
        self.scan_mode = kwargs.get('scan_mode', int())

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
        if self.ball_cam_x != other.ball_cam_x:
            return False
        if self.ball_cam_y != other.ball_cam_y:
            return False
        if self.ball_2d_x != other.ball_2d_x:
            return False
        if self.ball_2d_y != other.ball_2d_y:
            return False
        if self.ball_d != other.ball_d:
            return False
        if self.pan != other.pan:
            return False
        if self.tilt != other.tilt:
            return False
        if self.ball_speed_x != other.ball_speed_x:
            return False
        if self.ball_speed_y != other.ball_speed_y:
            return False
        if self.robot_vec_x != other.robot_vec_x:
            return False
        if self.robot_vec_y != other.robot_vec_y:
            return False
        if self.ball_speed_level != other.ball_speed_level:
            return False
        if self.scan_mode != other.scan_mode:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ball_cam_x(self):
        """Message field 'ball_cam_x'."""
        return self._ball_cam_x

    @ball_cam_x.setter
    def ball_cam_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ball_cam_x' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'ball_cam_x' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._ball_cam_x = value

    @builtins.property
    def ball_cam_y(self):
        """Message field 'ball_cam_y'."""
        return self._ball_cam_y

    @ball_cam_y.setter
    def ball_cam_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ball_cam_y' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'ball_cam_y' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._ball_cam_y = value

    @builtins.property
    def ball_2d_x(self):
        """Message field 'ball_2d_x'."""
        return self._ball_2d_x

    @ball_2d_x.setter
    def ball_2d_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ball_2d_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ball_2d_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ball_2d_x = value

    @builtins.property
    def ball_2d_y(self):
        """Message field 'ball_2d_y'."""
        return self._ball_2d_y

    @ball_2d_y.setter
    def ball_2d_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ball_2d_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ball_2d_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ball_2d_y = value

    @builtins.property
    def ball_d(self):
        """Message field 'ball_d'."""
        return self._ball_d

    @ball_d.setter
    def ball_d(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ball_d' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ball_d' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ball_d = value

    @builtins.property
    def pan(self):
        """Message field 'pan'."""
        return self._pan

    @pan.setter
    def pan(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pan' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pan' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pan = value

    @builtins.property
    def tilt(self):
        """Message field 'tilt'."""
        return self._tilt

    @tilt.setter
    def tilt(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tilt' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tilt' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tilt = value

    @builtins.property
    def ball_speed_x(self):
        """Message field 'ball_speed_x'."""
        return self._ball_speed_x

    @ball_speed_x.setter
    def ball_speed_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ball_speed_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ball_speed_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ball_speed_x = value

    @builtins.property
    def ball_speed_y(self):
        """Message field 'ball_speed_y'."""
        return self._ball_speed_y

    @ball_speed_y.setter
    def ball_speed_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ball_speed_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ball_speed_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ball_speed_y = value

    @builtins.property
    def robot_vec_x(self):
        """Message field 'robot_vec_x'."""
        return self._robot_vec_x

    @robot_vec_x.setter
    def robot_vec_x(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'robot_vec_x' array.array() must have the type code of 'd'"
            self._robot_vec_x = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'robot_vec_x' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._robot_vec_x = array.array('d', value)

    @builtins.property
    def robot_vec_y(self):
        """Message field 'robot_vec_y'."""
        return self._robot_vec_y

    @robot_vec_y.setter
    def robot_vec_y(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'robot_vec_y' array.array() must have the type code of 'd'"
            self._robot_vec_y = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'robot_vec_y' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._robot_vec_y = array.array('d', value)

    @builtins.property
    def ball_speed_level(self):
        """Message field 'ball_speed_level'."""
        return self._ball_speed_level

    @ball_speed_level.setter
    def ball_speed_level(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ball_speed_level' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'ball_speed_level' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._ball_speed_level = value

    @builtins.property
    def scan_mode(self):
        """Message field 'scan_mode'."""
        return self._scan_mode

    @scan_mode.setter
    def scan_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'scan_mode' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'scan_mode' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._scan_mode = value
