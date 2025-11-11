# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/ZmpMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ZmpMsg(type):
    """Metaclass of message 'ZmpMsg'."""

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
                'humanoid_interfaces.msg.ZmpMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__zmp_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__zmp_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__zmp_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__zmp_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__zmp_msg

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ZmpMsg(metaclass=Metaclass_ZmpMsg):
    """Message class 'ZmpMsg'."""

    __slots__ = [
        '_left_x_zmp',
        '_left_y_zmp',
        '_right_x_zmp',
        '_right_y_zmp',
        '_total_x_zmp',
        '_total_y_zmp',
        '_left_foot',
        '_right_foot',
        '_both_feet',
    ]

    _fields_and_field_types = {
        'left_x_zmp': 'double',
        'left_y_zmp': 'double',
        'right_x_zmp': 'double',
        'right_y_zmp': 'double',
        'total_x_zmp': 'double',
        'total_y_zmp': 'double',
        'left_foot': 'boolean',
        'right_foot': 'boolean',
        'both_feet': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.left_x_zmp = kwargs.get('left_x_zmp', float())
        self.left_y_zmp = kwargs.get('left_y_zmp', float())
        self.right_x_zmp = kwargs.get('right_x_zmp', float())
        self.right_y_zmp = kwargs.get('right_y_zmp', float())
        self.total_x_zmp = kwargs.get('total_x_zmp', float())
        self.total_y_zmp = kwargs.get('total_y_zmp', float())
        self.left_foot = kwargs.get('left_foot', bool())
        self.right_foot = kwargs.get('right_foot', bool())
        self.both_feet = kwargs.get('both_feet', bool())

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
        if self.left_x_zmp != other.left_x_zmp:
            return False
        if self.left_y_zmp != other.left_y_zmp:
            return False
        if self.right_x_zmp != other.right_x_zmp:
            return False
        if self.right_y_zmp != other.right_y_zmp:
            return False
        if self.total_x_zmp != other.total_x_zmp:
            return False
        if self.total_y_zmp != other.total_y_zmp:
            return False
        if self.left_foot != other.left_foot:
            return False
        if self.right_foot != other.right_foot:
            return False
        if self.both_feet != other.both_feet:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def left_x_zmp(self):
        """Message field 'left_x_zmp'."""
        return self._left_x_zmp

    @left_x_zmp.setter
    def left_x_zmp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_x_zmp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_x_zmp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_x_zmp = value

    @builtins.property
    def left_y_zmp(self):
        """Message field 'left_y_zmp'."""
        return self._left_y_zmp

    @left_y_zmp.setter
    def left_y_zmp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_y_zmp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_y_zmp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_y_zmp = value

    @builtins.property
    def right_x_zmp(self):
        """Message field 'right_x_zmp'."""
        return self._right_x_zmp

    @right_x_zmp.setter
    def right_x_zmp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_x_zmp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_x_zmp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_x_zmp = value

    @builtins.property
    def right_y_zmp(self):
        """Message field 'right_y_zmp'."""
        return self._right_y_zmp

    @right_y_zmp.setter
    def right_y_zmp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_y_zmp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_y_zmp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_y_zmp = value

    @builtins.property
    def total_x_zmp(self):
        """Message field 'total_x_zmp'."""
        return self._total_x_zmp

    @total_x_zmp.setter
    def total_x_zmp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_x_zmp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'total_x_zmp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._total_x_zmp = value

    @builtins.property
    def total_y_zmp(self):
        """Message field 'total_y_zmp'."""
        return self._total_y_zmp

    @total_y_zmp.setter
    def total_y_zmp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_y_zmp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'total_y_zmp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._total_y_zmp = value

    @builtins.property
    def left_foot(self):
        """Message field 'left_foot'."""
        return self._left_foot

    @left_foot.setter
    def left_foot(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'left_foot' field must be of type 'bool'"
        self._left_foot = value

    @builtins.property
    def right_foot(self):
        """Message field 'right_foot'."""
        return self._right_foot

    @right_foot.setter
    def right_foot(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'right_foot' field must be of type 'bool'"
        self._right_foot = value

    @builtins.property
    def both_feet(self):
        """Message field 'both_feet'."""
        return self._both_feet

    @both_feet.setter
    def both_feet(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'both_feet' field must be of type 'bool'"
        self._both_feet = value
