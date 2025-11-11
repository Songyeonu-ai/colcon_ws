# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/IkEndMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_IkEndMsg(type):
    """Metaclass of message 'IkEndMsg'."""

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
                'humanoid_interfaces.msg.IkEndMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ik_end_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ik_end_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ik_end_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ik_end_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ik_end_msg

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IkEndMsg(metaclass=Metaclass_IkEndMsg):
    """Message class 'IkEndMsg'."""

    __slots__ = [
        '_ikend',
        '_onestep_end',
    ]

    _fields_and_field_types = {
        'ikend': 'int32',
        'onestep_end': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ikend = kwargs.get('ikend', int())
        self.onestep_end = kwargs.get('onestep_end', int())

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
        if self.ikend != other.ikend:
            return False
        if self.onestep_end != other.onestep_end:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ikend(self):
        """Message field 'ikend'."""
        return self._ikend

    @ikend.setter
    def ikend(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ikend' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'ikend' field must be an integer in [-2147483648, 2147483647]"
        self._ikend = value

    @builtins.property
    def onestep_end(self):
        """Message field 'onestep_end'."""
        return self._onestep_end

    @onestep_end.setter
    def onestep_end(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'onestep_end' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'onestep_end' field must be an integer in [-2147483648, 2147483647]"
        self._onestep_end = value
