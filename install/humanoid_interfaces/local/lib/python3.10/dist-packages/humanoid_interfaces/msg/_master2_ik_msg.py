# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/Master2IkMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Master2IkMsg(type):
    """Metaclass of message 'Master2IkMsg'."""

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
                'humanoid_interfaces.msg.Master2IkMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__master2_ik_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__master2_ik_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__master2_ik_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__master2_ik_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__master2_ik_msg

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Master2IkMsg(metaclass=Metaclass_Master2IkMsg):
    """Message class 'Master2IkMsg'."""

    __slots__ = [
        '_x_length',
        '_y_length',
        '_yaw',
        '_flag',
        '_one_x_length',
        '_one_y_length',
        '_one_yaw',
        '_one_step_flag',
    ]

    _fields_and_field_types = {
        'x_length': 'double',
        'y_length': 'double',
        'yaw': 'double',
        'flag': 'double',
        'one_x_length': 'int32',
        'one_y_length': 'int32',
        'one_yaw': 'int32',
        'one_step_flag': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.x_length = kwargs.get('x_length', float())
        self.y_length = kwargs.get('y_length', float())
        self.yaw = kwargs.get('yaw', float())
        self.flag = kwargs.get('flag', float())
        self.one_x_length = kwargs.get('one_x_length', int())
        self.one_y_length = kwargs.get('one_y_length', int())
        self.one_yaw = kwargs.get('one_yaw', int())
        self.one_step_flag = kwargs.get('one_step_flag', int())

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
        if self.x_length != other.x_length:
            return False
        if self.y_length != other.y_length:
            return False
        if self.yaw != other.yaw:
            return False
        if self.flag != other.flag:
            return False
        if self.one_x_length != other.one_x_length:
            return False
        if self.one_y_length != other.one_y_length:
            return False
        if self.one_yaw != other.one_yaw:
            return False
        if self.one_step_flag != other.one_step_flag:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def x_length(self):
        """Message field 'x_length'."""
        return self._x_length

    @x_length.setter
    def x_length(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_length' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x_length' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x_length = value

    @builtins.property
    def y_length(self):
        """Message field 'y_length'."""
        return self._y_length

    @y_length.setter
    def y_length(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_length' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y_length' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y_length = value

    @builtins.property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._yaw = value

    @builtins.property
    def flag(self):
        """Message field 'flag'."""
        return self._flag

    @flag.setter
    def flag(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'flag' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'flag' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._flag = value

    @builtins.property
    def one_x_length(self):
        """Message field 'one_x_length'."""
        return self._one_x_length

    @one_x_length.setter
    def one_x_length(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'one_x_length' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'one_x_length' field must be an integer in [-2147483648, 2147483647]"
        self._one_x_length = value

    @builtins.property
    def one_y_length(self):
        """Message field 'one_y_length'."""
        return self._one_y_length

    @one_y_length.setter
    def one_y_length(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'one_y_length' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'one_y_length' field must be an integer in [-2147483648, 2147483647]"
        self._one_y_length = value

    @builtins.property
    def one_yaw(self):
        """Message field 'one_yaw'."""
        return self._one_yaw

    @one_yaw.setter
    def one_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'one_yaw' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'one_yaw' field must be an integer in [-2147483648, 2147483647]"
        self._one_yaw = value

    @builtins.property
    def one_step_flag(self):
        """Message field 'one_step_flag'."""
        return self._one_step_flag

    @one_step_flag.setter
    def one_step_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'one_step_flag' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'one_step_flag' field must be an integer in [-2147483648, 2147483647]"
        self._one_step_flag = value
