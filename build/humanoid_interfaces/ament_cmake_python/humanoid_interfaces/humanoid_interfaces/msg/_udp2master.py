# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/Udp2master.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Udp2master(type):
    """Metaclass of message 'Udp2master'."""

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
                'humanoid_interfaces.msg.Udp2master')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__udp2master
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__udp2master
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__udp2master
            cls._TYPE_SUPPORT = module.type_support_msg__msg__udp2master
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__udp2master

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Udp2master(metaclass=Metaclass_Udp2master):
    """Message class 'Udp2master'."""

    __slots__ = [
        '_robotnum',
        '_robotcase',
        '_localx',
        '_localy',
        '_localyaw',
        '_ballx',
        '_bally',
        '_balldist',
        '_balltheta',
    ]

    _fields_and_field_types = {
        'robotnum': 'int64',
        'robotcase': 'int64',
        'localx': 'int64',
        'localy': 'int64',
        'localyaw': 'int64',
        'ballx': 'int64',
        'bally': 'int64',
        'balldist': 'int64',
        'balltheta': 'int64',
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
        self.robotcase = kwargs.get('robotcase', int())
        self.localx = kwargs.get('localx', int())
        self.localy = kwargs.get('localy', int())
        self.localyaw = kwargs.get('localyaw', int())
        self.ballx = kwargs.get('ballx', int())
        self.bally = kwargs.get('bally', int())
        self.balldist = kwargs.get('balldist', int())
        self.balltheta = kwargs.get('balltheta', int())

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
        if self.robotcase != other.robotcase:
            return False
        if self.localx != other.localx:
            return False
        if self.localy != other.localy:
            return False
        if self.localyaw != other.localyaw:
            return False
        if self.ballx != other.ballx:
            return False
        if self.bally != other.bally:
            return False
        if self.balldist != other.balldist:
            return False
        if self.balltheta != other.balltheta:
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
    def robotcase(self):
        """Message field 'robotcase'."""
        return self._robotcase

    @robotcase.setter
    def robotcase(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robotcase' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'robotcase' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._robotcase = value

    @builtins.property
    def localx(self):
        """Message field 'localx'."""
        return self._localx

    @localx.setter
    def localx(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'localx' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'localx' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._localx = value

    @builtins.property
    def localy(self):
        """Message field 'localy'."""
        return self._localy

    @localy.setter
    def localy(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'localy' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'localy' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._localy = value

    @builtins.property
    def localyaw(self):
        """Message field 'localyaw'."""
        return self._localyaw

    @localyaw.setter
    def localyaw(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'localyaw' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'localyaw' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._localyaw = value

    @builtins.property
    def ballx(self):
        """Message field 'ballx'."""
        return self._ballx

    @ballx.setter
    def ballx(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ballx' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'ballx' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._ballx = value

    @builtins.property
    def bally(self):
        """Message field 'bally'."""
        return self._bally

    @bally.setter
    def bally(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'bally' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'bally' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._bally = value

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
    def balltheta(self):
        """Message field 'balltheta'."""
        return self._balltheta

    @balltheta.setter
    def balltheta(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'balltheta' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'balltheta' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._balltheta = value
