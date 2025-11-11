# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/CPWalk.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CPWalk(type):
    """Metaclass of message 'CPWalk'."""

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
                'humanoid_interfaces.msg.CPWalk')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__cp_walk
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__cp_walk
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__cp_walk
            cls._TYPE_SUPPORT = module.type_support_msg__msg__cp_walk
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__cp_walk

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CPWalk(metaclass=Metaclass_CPWalk):
    """Message class 'CPWalk'."""

    __slots__ = [
        '_cpend',
        '_cpflag',
        '_goal',
        '_start',
        '_goal_dist',
    ]

    _fields_and_field_types = {
        'cpend': 'int32',
        'cpflag': 'int32',
        'goal': 'double',
        'start': 'double',
        'goal_dist': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.cpend = kwargs.get('cpend', int())
        self.cpflag = kwargs.get('cpflag', int())
        self.goal = kwargs.get('goal', float())
        self.start = kwargs.get('start', float())
        self.goal_dist = kwargs.get('goal_dist', float())

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
        if self.cpend != other.cpend:
            return False
        if self.cpflag != other.cpflag:
            return False
        if self.goal != other.goal:
            return False
        if self.start != other.start:
            return False
        if self.goal_dist != other.goal_dist:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def cpend(self):
        """Message field 'cpend'."""
        return self._cpend

    @cpend.setter
    def cpend(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cpend' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'cpend' field must be an integer in [-2147483648, 2147483647]"
        self._cpend = value

    @builtins.property
    def cpflag(self):
        """Message field 'cpflag'."""
        return self._cpflag

    @cpflag.setter
    def cpflag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cpflag' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'cpflag' field must be an integer in [-2147483648, 2147483647]"
        self._cpflag = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'goal' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'goal' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._goal = value

    @builtins.property
    def start(self):
        """Message field 'start'."""
        return self._start

    @start.setter
    def start(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'start' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'start' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._start = value

    @builtins.property
    def goal_dist(self):
        """Message field 'goal_dist'."""
        return self._goal_dist

    @goal_dist.setter
    def goal_dist(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'goal_dist' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'goal_dist' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._goal_dist = value
