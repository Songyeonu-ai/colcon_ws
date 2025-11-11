# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/ImuMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ImuMsg(type):
    """Metaclass of message 'ImuMsg'."""

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
                'humanoid_interfaces.msg.ImuMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__imu_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__imu_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__imu_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__imu_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__imu_msg

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ImuMsg(metaclass=Metaclass_ImuMsg):
    """Message class 'ImuMsg'."""

    __slots__ = [
        '_pitch',
        '_roll',
        '_yaw',
        '_q_x',
        '_q_y',
        '_q_z',
        '_q_w',
        '_x_gyro',
        '_y_gyro',
        '_z_gyro',
        '_x_acc',
        '_y_acc',
        '_z_acc',
        '_x_vel',
        '_y_vel',
        '_z_vel',
        '_x_pos',
        '_y_pos',
        '_z_pos',
    ]

    _fields_and_field_types = {
        'pitch': 'float',
        'roll': 'float',
        'yaw': 'float',
        'q_x': 'float',
        'q_y': 'float',
        'q_z': 'float',
        'q_w': 'float',
        'x_gyro': 'float',
        'y_gyro': 'float',
        'z_gyro': 'float',
        'x_acc': 'float',
        'y_acc': 'float',
        'z_acc': 'float',
        'x_vel': 'float',
        'y_vel': 'float',
        'z_vel': 'float',
        'x_pos': 'float',
        'y_pos': 'float',
        'z_pos': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.pitch = kwargs.get('pitch', float())
        self.roll = kwargs.get('roll', float())
        self.yaw = kwargs.get('yaw', float())
        self.q_x = kwargs.get('q_x', float())
        self.q_y = kwargs.get('q_y', float())
        self.q_z = kwargs.get('q_z', float())
        self.q_w = kwargs.get('q_w', float())
        self.x_gyro = kwargs.get('x_gyro', float())
        self.y_gyro = kwargs.get('y_gyro', float())
        self.z_gyro = kwargs.get('z_gyro', float())
        self.x_acc = kwargs.get('x_acc', float())
        self.y_acc = kwargs.get('y_acc', float())
        self.z_acc = kwargs.get('z_acc', float())
        self.x_vel = kwargs.get('x_vel', float())
        self.y_vel = kwargs.get('y_vel', float())
        self.z_vel = kwargs.get('z_vel', float())
        self.x_pos = kwargs.get('x_pos', float())
        self.y_pos = kwargs.get('y_pos', float())
        self.z_pos = kwargs.get('z_pos', float())

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
        if self.pitch != other.pitch:
            return False
        if self.roll != other.roll:
            return False
        if self.yaw != other.yaw:
            return False
        if self.q_x != other.q_x:
            return False
        if self.q_y != other.q_y:
            return False
        if self.q_z != other.q_z:
            return False
        if self.q_w != other.q_w:
            return False
        if self.x_gyro != other.x_gyro:
            return False
        if self.y_gyro != other.y_gyro:
            return False
        if self.z_gyro != other.z_gyro:
            return False
        if self.x_acc != other.x_acc:
            return False
        if self.y_acc != other.y_acc:
            return False
        if self.z_acc != other.z_acc:
            return False
        if self.x_vel != other.x_vel:
            return False
        if self.y_vel != other.y_vel:
            return False
        if self.z_vel != other.z_vel:
            return False
        if self.x_pos != other.x_pos:
            return False
        if self.y_pos != other.y_pos:
            return False
        if self.z_pos != other.z_pos:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pitch(self):
        """Message field 'pitch'."""
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch = value

    @builtins.property
    def roll(self):
        """Message field 'roll'."""
        return self._roll

    @roll.setter
    def roll(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'roll' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._roll = value

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
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw = value

    @builtins.property
    def q_x(self):
        """Message field 'q_x'."""
        return self._q_x

    @q_x.setter
    def q_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'q_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'q_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._q_x = value

    @builtins.property
    def q_y(self):
        """Message field 'q_y'."""
        return self._q_y

    @q_y.setter
    def q_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'q_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'q_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._q_y = value

    @builtins.property
    def q_z(self):
        """Message field 'q_z'."""
        return self._q_z

    @q_z.setter
    def q_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'q_z' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'q_z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._q_z = value

    @builtins.property
    def q_w(self):
        """Message field 'q_w'."""
        return self._q_w

    @q_w.setter
    def q_w(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'q_w' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'q_w' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._q_w = value

    @builtins.property
    def x_gyro(self):
        """Message field 'x_gyro'."""
        return self._x_gyro

    @x_gyro.setter
    def x_gyro(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_gyro' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x_gyro' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x_gyro = value

    @builtins.property
    def y_gyro(self):
        """Message field 'y_gyro'."""
        return self._y_gyro

    @y_gyro.setter
    def y_gyro(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_gyro' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y_gyro' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y_gyro = value

    @builtins.property
    def z_gyro(self):
        """Message field 'z_gyro'."""
        return self._z_gyro

    @z_gyro.setter
    def z_gyro(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_gyro' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z_gyro' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z_gyro = value

    @builtins.property
    def x_acc(self):
        """Message field 'x_acc'."""
        return self._x_acc

    @x_acc.setter
    def x_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_acc' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x_acc' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x_acc = value

    @builtins.property
    def y_acc(self):
        """Message field 'y_acc'."""
        return self._y_acc

    @y_acc.setter
    def y_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_acc' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y_acc' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y_acc = value

    @builtins.property
    def z_acc(self):
        """Message field 'z_acc'."""
        return self._z_acc

    @z_acc.setter
    def z_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_acc' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z_acc' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z_acc = value

    @builtins.property
    def x_vel(self):
        """Message field 'x_vel'."""
        return self._x_vel

    @x_vel.setter
    def x_vel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_vel' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x_vel' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x_vel = value

    @builtins.property
    def y_vel(self):
        """Message field 'y_vel'."""
        return self._y_vel

    @y_vel.setter
    def y_vel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_vel' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y_vel' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y_vel = value

    @builtins.property
    def z_vel(self):
        """Message field 'z_vel'."""
        return self._z_vel

    @z_vel.setter
    def z_vel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_vel' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z_vel' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z_vel = value

    @builtins.property
    def x_pos(self):
        """Message field 'x_pos'."""
        return self._x_pos

    @x_pos.setter
    def x_pos(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_pos' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x_pos' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x_pos = value

    @builtins.property
    def y_pos(self):
        """Message field 'y_pos'."""
        return self._y_pos

    @y_pos.setter
    def y_pos(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_pos' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y_pos' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y_pos = value

    @builtins.property
    def z_pos(self):
        """Message field 'z_pos'."""
        return self._z_pos

    @z_pos.setter
    def z_pos(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_pos' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z_pos' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z_pos = value
