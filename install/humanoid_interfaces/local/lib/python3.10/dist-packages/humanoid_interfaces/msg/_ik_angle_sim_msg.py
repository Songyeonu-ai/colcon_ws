# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/IkAngleSimMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_IkAngleSimMsg(type):
    """Metaclass of message 'IkAngleSimMsg'."""

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
                'humanoid_interfaces.msg.IkAngleSimMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ik_angle_sim_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ik_angle_sim_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ik_angle_sim_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ik_angle_sim_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ik_angle_sim_msg

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IkAngleSimMsg(metaclass=Metaclass_IkAngleSimMsg):
    """Message class 'IkAngleSimMsg'."""

    __slots__ = [
        '_ang_l_0',
        '_ang_l_1',
        '_ang_l_2',
        '_ang_l_3',
        '_ang_l_4',
        '_ang_l_5',
        '_ang_l_6',
        '_ang_l_7',
        '_ang_l_8',
        '_ang_r_0',
        '_ang_r_1',
        '_ang_r_2',
        '_ang_r_3',
        '_ang_r_4',
        '_ang_r_5',
        '_ang_r_6',
        '_ang_r_7',
        '_ang_r_8',
    ]

    _fields_and_field_types = {
        'ang_l_0': 'double',
        'ang_l_1': 'double',
        'ang_l_2': 'double',
        'ang_l_3': 'double',
        'ang_l_4': 'double',
        'ang_l_5': 'double',
        'ang_l_6': 'double',
        'ang_l_7': 'double',
        'ang_l_8': 'double',
        'ang_r_0': 'double',
        'ang_r_1': 'double',
        'ang_r_2': 'double',
        'ang_r_3': 'double',
        'ang_r_4': 'double',
        'ang_r_5': 'double',
        'ang_r_6': 'double',
        'ang_r_7': 'double',
        'ang_r_8': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ang_l_0 = kwargs.get('ang_l_0', float())
        self.ang_l_1 = kwargs.get('ang_l_1', float())
        self.ang_l_2 = kwargs.get('ang_l_2', float())
        self.ang_l_3 = kwargs.get('ang_l_3', float())
        self.ang_l_4 = kwargs.get('ang_l_4', float())
        self.ang_l_5 = kwargs.get('ang_l_5', float())
        self.ang_l_6 = kwargs.get('ang_l_6', float())
        self.ang_l_7 = kwargs.get('ang_l_7', float())
        self.ang_l_8 = kwargs.get('ang_l_8', float())
        self.ang_r_0 = kwargs.get('ang_r_0', float())
        self.ang_r_1 = kwargs.get('ang_r_1', float())
        self.ang_r_2 = kwargs.get('ang_r_2', float())
        self.ang_r_3 = kwargs.get('ang_r_3', float())
        self.ang_r_4 = kwargs.get('ang_r_4', float())
        self.ang_r_5 = kwargs.get('ang_r_5', float())
        self.ang_r_6 = kwargs.get('ang_r_6', float())
        self.ang_r_7 = kwargs.get('ang_r_7', float())
        self.ang_r_8 = kwargs.get('ang_r_8', float())

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
        if self.ang_l_0 != other.ang_l_0:
            return False
        if self.ang_l_1 != other.ang_l_1:
            return False
        if self.ang_l_2 != other.ang_l_2:
            return False
        if self.ang_l_3 != other.ang_l_3:
            return False
        if self.ang_l_4 != other.ang_l_4:
            return False
        if self.ang_l_5 != other.ang_l_5:
            return False
        if self.ang_l_6 != other.ang_l_6:
            return False
        if self.ang_l_7 != other.ang_l_7:
            return False
        if self.ang_l_8 != other.ang_l_8:
            return False
        if self.ang_r_0 != other.ang_r_0:
            return False
        if self.ang_r_1 != other.ang_r_1:
            return False
        if self.ang_r_2 != other.ang_r_2:
            return False
        if self.ang_r_3 != other.ang_r_3:
            return False
        if self.ang_r_4 != other.ang_r_4:
            return False
        if self.ang_r_5 != other.ang_r_5:
            return False
        if self.ang_r_6 != other.ang_r_6:
            return False
        if self.ang_r_7 != other.ang_r_7:
            return False
        if self.ang_r_8 != other.ang_r_8:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ang_l_0(self):
        """Message field 'ang_l_0'."""
        return self._ang_l_0

    @ang_l_0.setter
    def ang_l_0(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_l_0' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_l_0' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_l_0 = value

    @builtins.property
    def ang_l_1(self):
        """Message field 'ang_l_1'."""
        return self._ang_l_1

    @ang_l_1.setter
    def ang_l_1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_l_1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_l_1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_l_1 = value

    @builtins.property
    def ang_l_2(self):
        """Message field 'ang_l_2'."""
        return self._ang_l_2

    @ang_l_2.setter
    def ang_l_2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_l_2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_l_2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_l_2 = value

    @builtins.property
    def ang_l_3(self):
        """Message field 'ang_l_3'."""
        return self._ang_l_3

    @ang_l_3.setter
    def ang_l_3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_l_3' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_l_3' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_l_3 = value

    @builtins.property
    def ang_l_4(self):
        """Message field 'ang_l_4'."""
        return self._ang_l_4

    @ang_l_4.setter
    def ang_l_4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_l_4' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_l_4' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_l_4 = value

    @builtins.property
    def ang_l_5(self):
        """Message field 'ang_l_5'."""
        return self._ang_l_5

    @ang_l_5.setter
    def ang_l_5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_l_5' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_l_5' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_l_5 = value

    @builtins.property
    def ang_l_6(self):
        """Message field 'ang_l_6'."""
        return self._ang_l_6

    @ang_l_6.setter
    def ang_l_6(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_l_6' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_l_6' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_l_6 = value

    @builtins.property
    def ang_l_7(self):
        """Message field 'ang_l_7'."""
        return self._ang_l_7

    @ang_l_7.setter
    def ang_l_7(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_l_7' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_l_7' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_l_7 = value

    @builtins.property
    def ang_l_8(self):
        """Message field 'ang_l_8'."""
        return self._ang_l_8

    @ang_l_8.setter
    def ang_l_8(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_l_8' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_l_8' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_l_8 = value

    @builtins.property
    def ang_r_0(self):
        """Message field 'ang_r_0'."""
        return self._ang_r_0

    @ang_r_0.setter
    def ang_r_0(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_r_0' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_r_0' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_r_0 = value

    @builtins.property
    def ang_r_1(self):
        """Message field 'ang_r_1'."""
        return self._ang_r_1

    @ang_r_1.setter
    def ang_r_1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_r_1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_r_1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_r_1 = value

    @builtins.property
    def ang_r_2(self):
        """Message field 'ang_r_2'."""
        return self._ang_r_2

    @ang_r_2.setter
    def ang_r_2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_r_2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_r_2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_r_2 = value

    @builtins.property
    def ang_r_3(self):
        """Message field 'ang_r_3'."""
        return self._ang_r_3

    @ang_r_3.setter
    def ang_r_3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_r_3' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_r_3' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_r_3 = value

    @builtins.property
    def ang_r_4(self):
        """Message field 'ang_r_4'."""
        return self._ang_r_4

    @ang_r_4.setter
    def ang_r_4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_r_4' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_r_4' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_r_4 = value

    @builtins.property
    def ang_r_5(self):
        """Message field 'ang_r_5'."""
        return self._ang_r_5

    @ang_r_5.setter
    def ang_r_5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_r_5' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_r_5' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_r_5 = value

    @builtins.property
    def ang_r_6(self):
        """Message field 'ang_r_6'."""
        return self._ang_r_6

    @ang_r_6.setter
    def ang_r_6(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_r_6' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_r_6' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_r_6 = value

    @builtins.property
    def ang_r_7(self):
        """Message field 'ang_r_7'."""
        return self._ang_r_7

    @ang_r_7.setter
    def ang_r_7(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_r_7' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_r_7' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_r_7 = value

    @builtins.property
    def ang_r_8(self):
        """Message field 'ang_r_8'."""
        return self._ang_r_8

    @ang_r_8.setter
    def ang_r_8(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ang_r_8' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ang_r_8' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ang_r_8 = value
