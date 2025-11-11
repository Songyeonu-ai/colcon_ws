# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/IkLTCMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_IkLTCMsg(type):
    """Metaclass of message 'IkLTCMsg'."""

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
                'humanoid_interfaces.msg.IkLTCMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ik_ltc_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ik_ltc_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ik_ltc_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ik_ltc_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ik_ltc_msg

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IkLTCMsg(metaclass=Metaclass_IkLTCMsg):
    """Message class 'IkLTCMsg'."""

    __slots__ = [
        '_entire_time',
        '_swing_gain_l',
        '_swing_gain_r',
        '_warning',
        '_safe',
        '_landing_time_l',
        '_landing_time_r',
        '_landing_error_l',
        '_landing_error_r',
    ]

    _fields_and_field_types = {
        'entire_time': 'double',
        'swing_gain_l': 'double',
        'swing_gain_r': 'double',
        'warning': 'int64',
        'safe': 'int64',
        'landing_time_l': 'double',
        'landing_time_r': 'double',
        'landing_error_l': 'double',
        'landing_error_r': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.entire_time = kwargs.get('entire_time', float())
        self.swing_gain_l = kwargs.get('swing_gain_l', float())
        self.swing_gain_r = kwargs.get('swing_gain_r', float())
        self.warning = kwargs.get('warning', int())
        self.safe = kwargs.get('safe', int())
        self.landing_time_l = kwargs.get('landing_time_l', float())
        self.landing_time_r = kwargs.get('landing_time_r', float())
        self.landing_error_l = kwargs.get('landing_error_l', float())
        self.landing_error_r = kwargs.get('landing_error_r', float())

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
        if self.entire_time != other.entire_time:
            return False
        if self.swing_gain_l != other.swing_gain_l:
            return False
        if self.swing_gain_r != other.swing_gain_r:
            return False
        if self.warning != other.warning:
            return False
        if self.safe != other.safe:
            return False
        if self.landing_time_l != other.landing_time_l:
            return False
        if self.landing_time_r != other.landing_time_r:
            return False
        if self.landing_error_l != other.landing_error_l:
            return False
        if self.landing_error_r != other.landing_error_r:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def entire_time(self):
        """Message field 'entire_time'."""
        return self._entire_time

    @entire_time.setter
    def entire_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'entire_time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'entire_time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._entire_time = value

    @builtins.property
    def swing_gain_l(self):
        """Message field 'swing_gain_l'."""
        return self._swing_gain_l

    @swing_gain_l.setter
    def swing_gain_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'swing_gain_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'swing_gain_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._swing_gain_l = value

    @builtins.property
    def swing_gain_r(self):
        """Message field 'swing_gain_r'."""
        return self._swing_gain_r

    @swing_gain_r.setter
    def swing_gain_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'swing_gain_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'swing_gain_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._swing_gain_r = value

    @builtins.property
    def warning(self):
        """Message field 'warning'."""
        return self._warning

    @warning.setter
    def warning(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'warning' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'warning' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._warning = value

    @builtins.property
    def safe(self):
        """Message field 'safe'."""
        return self._safe

    @safe.setter
    def safe(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'safe' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'safe' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._safe = value

    @builtins.property
    def landing_time_l(self):
        """Message field 'landing_time_l'."""
        return self._landing_time_l

    @landing_time_l.setter
    def landing_time_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'landing_time_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'landing_time_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._landing_time_l = value

    @builtins.property
    def landing_time_r(self):
        """Message field 'landing_time_r'."""
        return self._landing_time_r

    @landing_time_r.setter
    def landing_time_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'landing_time_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'landing_time_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._landing_time_r = value

    @builtins.property
    def landing_error_l(self):
        """Message field 'landing_error_l'."""
        return self._landing_error_l

    @landing_error_l.setter
    def landing_error_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'landing_error_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'landing_error_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._landing_error_l = value

    @builtins.property
    def landing_error_r(self):
        """Message field 'landing_error_r'."""
        return self._landing_error_r

    @landing_error_r.setter
    def landing_error_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'landing_error_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'landing_error_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._landing_error_r = value
