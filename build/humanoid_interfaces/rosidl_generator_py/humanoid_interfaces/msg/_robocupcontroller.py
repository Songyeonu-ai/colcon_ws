# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/Robocupcontroller.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'secondinfo'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Robocupcontroller(type):
    """Metaclass of message 'Robocupcontroller'."""

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
                'humanoid_interfaces.msg.Robocupcontroller')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robocupcontroller
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robocupcontroller
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robocupcontroller
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robocupcontroller
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robocupcontroller

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Robocupcontroller(metaclass=Metaclass_Robocupcontroller):
    """Message class 'Robocupcontroller'."""

    __slots__ = [
        '_robotnum',
        '_state',
        '_readytime',
        '_penalty',
        '_firsthalf',
        '_firstside',
        '_kickoffteam',
        '_secondstate',
        '_secondinfo',
    ]

    _fields_and_field_types = {
        'robotnum': 'int64',
        'state': 'int64',
        'readytime': 'int64',
        'penalty': 'int64',
        'firsthalf': 'int64',
        'firstside': 'int64',
        'kickoffteam': 'int64',
        'secondstate': 'int64',
        'secondinfo': 'sequence<int64>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int64')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.robotnum = kwargs.get('robotnum', int())
        self.state = kwargs.get('state', int())
        self.readytime = kwargs.get('readytime', int())
        self.penalty = kwargs.get('penalty', int())
        self.firsthalf = kwargs.get('firsthalf', int())
        self.firstside = kwargs.get('firstside', int())
        self.kickoffteam = kwargs.get('kickoffteam', int())
        self.secondstate = kwargs.get('secondstate', int())
        self.secondinfo = array.array('q', kwargs.get('secondinfo', []))

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
        if self.robotnum != other.robotnum:
            return False
        if self.state != other.state:
            return False
        if self.readytime != other.readytime:
            return False
        if self.penalty != other.penalty:
            return False
        if self.firsthalf != other.firsthalf:
            return False
        if self.firstside != other.firstside:
            return False
        if self.kickoffteam != other.kickoffteam:
            return False
        if self.secondstate != other.secondstate:
            return False
        if self.secondinfo != other.secondinfo:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def robotnum(self):
        """Message field 'robotnum'."""
        return self._robotnum

    @robotnum.setter
    def robotnum(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robotnum' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'robotnum' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._robotnum = value

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'state' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._state = value

    @builtins.property
    def readytime(self):
        """Message field 'readytime'."""
        return self._readytime

    @readytime.setter
    def readytime(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'readytime' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'readytime' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._readytime = value

    @builtins.property
    def penalty(self):
        """Message field 'penalty'."""
        return self._penalty

    @penalty.setter
    def penalty(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'penalty' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'penalty' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._penalty = value

    @builtins.property
    def firsthalf(self):
        """Message field 'firsthalf'."""
        return self._firsthalf

    @firsthalf.setter
    def firsthalf(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'firsthalf' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'firsthalf' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._firsthalf = value

    @builtins.property
    def firstside(self):
        """Message field 'firstside'."""
        return self._firstside

    @firstside.setter
    def firstside(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'firstside' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'firstside' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._firstside = value

    @builtins.property
    def kickoffteam(self):
        """Message field 'kickoffteam'."""
        return self._kickoffteam

    @kickoffteam.setter
    def kickoffteam(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kickoffteam' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'kickoffteam' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._kickoffteam = value

    @builtins.property
    def secondstate(self):
        """Message field 'secondstate'."""
        return self._secondstate

    @secondstate.setter
    def secondstate(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'secondstate' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'secondstate' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._secondstate = value

    @builtins.property
    def secondinfo(self):
        """Message field 'secondinfo'."""
        return self._secondinfo

    @secondinfo.setter
    def secondinfo(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'q', \
                "The 'secondinfo' array.array() must have the type code of 'q'"
            self._secondinfo = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -9223372036854775808 and val < 9223372036854775808 for val in value)), \
                "The 'secondinfo' field must be a set or sequence and each value of type 'int' and each integer in [-9223372036854775808, 9223372036854775807]"
        self._secondinfo = array.array('q', value)
