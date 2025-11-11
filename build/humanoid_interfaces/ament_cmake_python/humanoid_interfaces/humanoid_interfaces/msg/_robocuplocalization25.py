# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/Robocuplocalization25.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Robocuplocalization25(type):
    """Metaclass of message 'Robocuplocalization25'."""

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
                'humanoid_interfaces.msg.Robocuplocalization25')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robocuplocalization25
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robocuplocalization25
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robocuplocalization25
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robocuplocalization25
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robocuplocalization25

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Robocuplocalization25(metaclass=Metaclass_Robocuplocalization25):
    """Message class 'Robocuplocalization25'."""

    __slots__ = [
        '_ball_x',
        '_ball_y',
        '_ball_speed_x',
        '_ball_speed_y',
        '_robot_x',
        '_robot_y',
        '_obstacle_x',
        '_obstacle_y',
        '_obstacle0_x',
        '_obstacle0_y',
        '_obstacle1_x',
        '_obstacle1_y',
        '_obstacle2_x',
        '_obstacle2_y',
        '_obstacle3_x',
        '_obstacle3_y',
    ]

    _fields_and_field_types = {
        'ball_x': 'double',
        'ball_y': 'double',
        'ball_speed_x': 'double',
        'ball_speed_y': 'double',
        'robot_x': 'double',
        'robot_y': 'double',
        'obstacle_x': 'double',
        'obstacle_y': 'double',
        'obstacle0_x': 'double',
        'obstacle0_y': 'double',
        'obstacle1_x': 'double',
        'obstacle1_y': 'double',
        'obstacle2_x': 'double',
        'obstacle2_y': 'double',
        'obstacle3_x': 'double',
        'obstacle3_y': 'double',
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
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ball_x = kwargs.get('ball_x', float())
        self.ball_y = kwargs.get('ball_y', float())
        self.ball_speed_x = kwargs.get('ball_speed_x', float())
        self.ball_speed_y = kwargs.get('ball_speed_y', float())
        self.robot_x = kwargs.get('robot_x', float())
        self.robot_y = kwargs.get('robot_y', float())
        self.obstacle_x = kwargs.get('obstacle_x', float())
        self.obstacle_y = kwargs.get('obstacle_y', float())
        self.obstacle0_x = kwargs.get('obstacle0_x', float())
        self.obstacle0_y = kwargs.get('obstacle0_y', float())
        self.obstacle1_x = kwargs.get('obstacle1_x', float())
        self.obstacle1_y = kwargs.get('obstacle1_y', float())
        self.obstacle2_x = kwargs.get('obstacle2_x', float())
        self.obstacle2_y = kwargs.get('obstacle2_y', float())
        self.obstacle3_x = kwargs.get('obstacle3_x', float())
        self.obstacle3_y = kwargs.get('obstacle3_y', float())

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
        if self.ball_x != other.ball_x:
            return False
        if self.ball_y != other.ball_y:
            return False
        if self.ball_speed_x != other.ball_speed_x:
            return False
        if self.ball_speed_y != other.ball_speed_y:
            return False
        if self.robot_x != other.robot_x:
            return False
        if self.robot_y != other.robot_y:
            return False
        if self.obstacle_x != other.obstacle_x:
            return False
        if self.obstacle_y != other.obstacle_y:
            return False
        if self.obstacle0_x != other.obstacle0_x:
            return False
        if self.obstacle0_y != other.obstacle0_y:
            return False
        if self.obstacle1_x != other.obstacle1_x:
            return False
        if self.obstacle1_y != other.obstacle1_y:
            return False
        if self.obstacle2_x != other.obstacle2_x:
            return False
        if self.obstacle2_y != other.obstacle2_y:
            return False
        if self.obstacle3_x != other.obstacle3_x:
            return False
        if self.obstacle3_y != other.obstacle3_y:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ball_x(self):
        """Message field 'ball_x'."""
        return self._ball_x

    @ball_x.setter
    def ball_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ball_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ball_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ball_x = value

    @builtins.property
    def ball_y(self):
        """Message field 'ball_y'."""
        return self._ball_y

    @ball_y.setter
    def ball_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ball_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ball_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ball_y = value

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
    def robot_x(self):
        """Message field 'robot_x'."""
        return self._robot_x

    @robot_x.setter
    def robot_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'robot_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'robot_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._robot_x = value

    @builtins.property
    def robot_y(self):
        """Message field 'robot_y'."""
        return self._robot_y

    @robot_y.setter
    def robot_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'robot_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'robot_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._robot_y = value

    @builtins.property
    def obstacle_x(self):
        """Message field 'obstacle_x'."""
        return self._obstacle_x

    @obstacle_x.setter
    def obstacle_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'obstacle_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'obstacle_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._obstacle_x = value

    @builtins.property
    def obstacle_y(self):
        """Message field 'obstacle_y'."""
        return self._obstacle_y

    @obstacle_y.setter
    def obstacle_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'obstacle_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'obstacle_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._obstacle_y = value

    @builtins.property
    def obstacle0_x(self):
        """Message field 'obstacle0_x'."""
        return self._obstacle0_x

    @obstacle0_x.setter
    def obstacle0_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'obstacle0_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'obstacle0_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._obstacle0_x = value

    @builtins.property
    def obstacle0_y(self):
        """Message field 'obstacle0_y'."""
        return self._obstacle0_y

    @obstacle0_y.setter
    def obstacle0_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'obstacle0_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'obstacle0_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._obstacle0_y = value

    @builtins.property
    def obstacle1_x(self):
        """Message field 'obstacle1_x'."""
        return self._obstacle1_x

    @obstacle1_x.setter
    def obstacle1_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'obstacle1_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'obstacle1_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._obstacle1_x = value

    @builtins.property
    def obstacle1_y(self):
        """Message field 'obstacle1_y'."""
        return self._obstacle1_y

    @obstacle1_y.setter
    def obstacle1_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'obstacle1_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'obstacle1_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._obstacle1_y = value

    @builtins.property
    def obstacle2_x(self):
        """Message field 'obstacle2_x'."""
        return self._obstacle2_x

    @obstacle2_x.setter
    def obstacle2_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'obstacle2_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'obstacle2_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._obstacle2_x = value

    @builtins.property
    def obstacle2_y(self):
        """Message field 'obstacle2_y'."""
        return self._obstacle2_y

    @obstacle2_y.setter
    def obstacle2_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'obstacle2_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'obstacle2_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._obstacle2_y = value

    @builtins.property
    def obstacle3_x(self):
        """Message field 'obstacle3_x'."""
        return self._obstacle3_x

    @obstacle3_x.setter
    def obstacle3_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'obstacle3_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'obstacle3_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._obstacle3_x = value

    @builtins.property
    def obstacle3_y(self):
        """Message field 'obstacle3_y'."""
        return self._obstacle3_y

    @obstacle3_y.setter
    def obstacle3_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'obstacle3_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'obstacle3_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._obstacle3_y = value
