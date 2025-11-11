# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/Master2localization25.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Master2localization25(type):
    """Metaclass of message 'Master2localization25'."""

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
                'humanoid_interfaces.msg.Master2localization25')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__master2localization25
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__master2localization25
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__master2localization25
            cls._TYPE_SUPPORT = module.type_support_msg__msg__master2localization25
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__master2localization25

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Master2localization25(metaclass=Metaclass_Master2localization25):
    """Message class 'Master2localization25'."""

    __slots__ = [
        '_goalx',
        '_goaly',
        '_targetx',
        '_targety',
        '_robotallowanceerror',
        '_robotcaliradius',
    ]

    _fields_and_field_types = {
        'goalx': 'int64',
        'goaly': 'int64',
        'targetx': 'int64',
        'targety': 'int64',
        'robotallowanceerror': 'int64',
        'robotcaliradius': 'int64',
    }

    SLOT_TYPES = (
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
        self.goalx = kwargs.get('goalx', int())
        self.goaly = kwargs.get('goaly', int())
        self.targetx = kwargs.get('targetx', int())
        self.targety = kwargs.get('targety', int())
        self.robotallowanceerror = kwargs.get('robotallowanceerror', int())
        self.robotcaliradius = kwargs.get('robotcaliradius', int())

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
        if self.goalx != other.goalx:
            return False
        if self.goaly != other.goaly:
            return False
        if self.targetx != other.targetx:
            return False
        if self.targety != other.targety:
            return False
        if self.robotallowanceerror != other.robotallowanceerror:
            return False
        if self.robotcaliradius != other.robotcaliradius:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goalx(self):
        """Message field 'goalx'."""
        return self._goalx

    @goalx.setter
    def goalx(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'goalx' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'goalx' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._goalx = value

    @builtins.property
    def goaly(self):
        """Message field 'goaly'."""
        return self._goaly

    @goaly.setter
    def goaly(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'goaly' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'goaly' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._goaly = value

    @builtins.property
    def targetx(self):
        """Message field 'targetx'."""
        return self._targetx

    @targetx.setter
    def targetx(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'targetx' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'targetx' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._targetx = value

    @builtins.property
    def targety(self):
        """Message field 'targety'."""
        return self._targety

    @targety.setter
    def targety(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'targety' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'targety' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._targety = value

    @builtins.property
    def robotallowanceerror(self):
        """Message field 'robotallowanceerror'."""
        return self._robotallowanceerror

    @robotallowanceerror.setter
    def robotallowanceerror(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robotallowanceerror' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'robotallowanceerror' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._robotallowanceerror = value

    @builtins.property
    def robotcaliradius(self):
        """Message field 'robotcaliradius'."""
        return self._robotcaliradius

    @robotcaliradius.setter
    def robotcaliradius(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robotcaliradius' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'robotcaliradius' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._robotcaliradius = value
