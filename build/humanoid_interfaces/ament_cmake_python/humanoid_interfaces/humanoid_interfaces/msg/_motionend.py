# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/Motionend.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Motionend(type):
    """Metaclass of message 'Motionend'."""

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
                'humanoid_interfaces.msg.Motionend')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motionend
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motionend
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motionend
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motionend
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motionend

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Motionend(metaclass=Metaclass_Motionend):
    """Message class 'Motionend'."""

    __slots__ = [
        '_motionend',
        '_motioning',
        '_motionstarting',
        '_motionending',
        '_motionnum',
    ]

    _fields_and_field_types = {
        'motionend': 'int32',
        'motioning': 'int32',
        'motionstarting': 'int32',
        'motionending': 'int32',
        'motionnum': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.motionend = kwargs.get('motionend', int())
        self.motioning = kwargs.get('motioning', int())
        self.motionstarting = kwargs.get('motionstarting', int())
        self.motionending = kwargs.get('motionending', int())
        self.motionnum = kwargs.get('motionnum', int())

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
        if self.motionend != other.motionend:
            return False
        if self.motioning != other.motioning:
            return False
        if self.motionstarting != other.motionstarting:
            return False
        if self.motionending != other.motionending:
            return False
        if self.motionnum != other.motionnum:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def motionend(self):
        """Message field 'motionend'."""
        return self._motionend

    @motionend.setter
    def motionend(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motionend' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'motionend' field must be an integer in [-2147483648, 2147483647]"
        self._motionend = value

    @builtins.property
    def motioning(self):
        """Message field 'motioning'."""
        return self._motioning

    @motioning.setter
    def motioning(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motioning' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'motioning' field must be an integer in [-2147483648, 2147483647]"
        self._motioning = value

    @builtins.property
    def motionstarting(self):
        """Message field 'motionstarting'."""
        return self._motionstarting

    @motionstarting.setter
    def motionstarting(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motionstarting' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'motionstarting' field must be an integer in [-2147483648, 2147483647]"
        self._motionstarting = value

    @builtins.property
    def motionending(self):
        """Message field 'motionending'."""
        return self._motionending

    @motionending.setter
    def motionending(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motionending' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'motionending' field must be an integer in [-2147483648, 2147483647]"
        self._motionending = value

    @builtins.property
    def motionnum(self):
        """Message field 'motionnum'."""
        return self._motionnum

    @motionnum.setter
    def motionnum(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motionnum' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'motionnum' field must be an integer in [-2147483648, 2147483647]"
        self._motionnum = value
