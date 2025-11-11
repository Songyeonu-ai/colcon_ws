# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/Master2udp.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Master2udp(type):
    """Metaclass of message 'Master2udp'."""

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
                'humanoid_interfaces.msg.Master2udp')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__master2udp
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__master2udp
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__master2udp
            cls._TYPE_SUPPORT = module.type_support_msg__msg__master2udp
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__master2udp

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Master2udp(metaclass=Metaclass_Master2udp):
    """Message class 'Master2udp'."""

    __slots__ = [
        '_robotnum',
        '_robotstate',
        '_robotcoorx',
        '_robotcoory',
        '_robotimuyaw',
        '_balldist',
        '_ballcoorx',
        '_ballcoory',
        '_myteam',
    ]

    _fields_and_field_types = {
        'robotnum': 'int64',
        'robotstate': 'int64',
        'robotcoorx': 'int64',
        'robotcoory': 'int64',
        'robotimuyaw': 'int64',
        'balldist': 'int64',
        'ballcoorx': 'int64',
        'ballcoory': 'int64',
        'myteam': 'int64',
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
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.robotnum = kwargs.get('robotnum', int())
        self.robotstate = kwargs.get('robotstate', int())
        self.robotcoorx = kwargs.get('robotcoorx', int())
        self.robotcoory = kwargs.get('robotcoory', int())
        self.robotimuyaw = kwargs.get('robotimuyaw', int())
        self.balldist = kwargs.get('balldist', int())
        self.ballcoorx = kwargs.get('ballcoorx', int())
        self.ballcoory = kwargs.get('ballcoory', int())
        self.myteam = kwargs.get('myteam', int())

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
        if self.robotstate != other.robotstate:
            return False
        if self.robotcoorx != other.robotcoorx:
            return False
        if self.robotcoory != other.robotcoory:
            return False
        if self.robotimuyaw != other.robotimuyaw:
            return False
        if self.balldist != other.balldist:
            return False
        if self.ballcoorx != other.ballcoorx:
            return False
        if self.ballcoory != other.ballcoory:
            return False
        if self.myteam != other.myteam:
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
    def robotstate(self):
        """Message field 'robotstate'."""
        return self._robotstate

    @robotstate.setter
    def robotstate(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robotstate' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'robotstate' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._robotstate = value

    @builtins.property
    def robotcoorx(self):
        """Message field 'robotcoorx'."""
        return self._robotcoorx

    @robotcoorx.setter
    def robotcoorx(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robotcoorx' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'robotcoorx' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._robotcoorx = value

    @builtins.property
    def robotcoory(self):
        """Message field 'robotcoory'."""
        return self._robotcoory

    @robotcoory.setter
    def robotcoory(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robotcoory' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'robotcoory' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._robotcoory = value

    @builtins.property
    def robotimuyaw(self):
        """Message field 'robotimuyaw'."""
        return self._robotimuyaw

    @robotimuyaw.setter
    def robotimuyaw(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robotimuyaw' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'robotimuyaw' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._robotimuyaw = value

    @builtins.property
    def balldist(self):
        """Message field 'balldist'."""
        return self._balldist

    @balldist.setter
    def balldist(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'balldist' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'balldist' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._balldist = value

    @builtins.property
    def ballcoorx(self):
        """Message field 'ballcoorx'."""
        return self._ballcoorx

    @ballcoorx.setter
    def ballcoorx(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ballcoorx' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'ballcoorx' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._ballcoorx = value

    @builtins.property
    def ballcoory(self):
        """Message field 'ballcoory'."""
        return self._ballcoory

    @ballcoory.setter
    def ballcoory(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ballcoory' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'ballcoory' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._ballcoory = value

    @builtins.property
    def myteam(self):
        """Message field 'myteam'."""
        return self._myteam

    @myteam.setter
    def myteam(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'myteam' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'myteam' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._myteam = value
