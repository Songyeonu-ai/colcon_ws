# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/IkPatternMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_IkPatternMsg(type):
    """Metaclass of message 'IkPatternMsg'."""

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
                'humanoid_interfaces.msg.IkPatternMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ik_pattern_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ik_pattern_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ik_pattern_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ik_pattern_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ik_pattern_msg

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IkPatternMsg(metaclass=Metaclass_IkPatternMsg):
    """Message class 'IkPatternMsg'."""

    __slots__ = [
        '_xlpattern',
        '_ylpattern',
        '_zlpattern',
        '_slpattern',
        '_tlpattern',
        '_xrpattern',
        '_yrpattern',
        '_zrpattern',
        '_srpattern',
        '_trpattern',
    ]

    _fields_and_field_types = {
        'xlpattern': 'double',
        'ylpattern': 'double',
        'zlpattern': 'double',
        'slpattern': 'double',
        'tlpattern': 'double',
        'xrpattern': 'double',
        'yrpattern': 'double',
        'zrpattern': 'double',
        'srpattern': 'double',
        'trpattern': 'double',
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
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.xlpattern = kwargs.get('xlpattern', float())
        self.ylpattern = kwargs.get('ylpattern', float())
        self.zlpattern = kwargs.get('zlpattern', float())
        self.slpattern = kwargs.get('slpattern', float())
        self.tlpattern = kwargs.get('tlpattern', float())
        self.xrpattern = kwargs.get('xrpattern', float())
        self.yrpattern = kwargs.get('yrpattern', float())
        self.zrpattern = kwargs.get('zrpattern', float())
        self.srpattern = kwargs.get('srpattern', float())
        self.trpattern = kwargs.get('trpattern', float())

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
        if self.xlpattern != other.xlpattern:
            return False
        if self.ylpattern != other.ylpattern:
            return False
        if self.zlpattern != other.zlpattern:
            return False
        if self.slpattern != other.slpattern:
            return False
        if self.tlpattern != other.tlpattern:
            return False
        if self.xrpattern != other.xrpattern:
            return False
        if self.yrpattern != other.yrpattern:
            return False
        if self.zrpattern != other.zrpattern:
            return False
        if self.srpattern != other.srpattern:
            return False
        if self.trpattern != other.trpattern:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def xlpattern(self):
        """Message field 'xlpattern'."""
        return self._xlpattern

    @xlpattern.setter
    def xlpattern(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'xlpattern' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'xlpattern' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._xlpattern = value

    @builtins.property
    def ylpattern(self):
        """Message field 'ylpattern'."""
        return self._ylpattern

    @ylpattern.setter
    def ylpattern(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ylpattern' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ylpattern' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ylpattern = value

    @builtins.property
    def zlpattern(self):
        """Message field 'zlpattern'."""
        return self._zlpattern

    @zlpattern.setter
    def zlpattern(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'zlpattern' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'zlpattern' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._zlpattern = value

    @builtins.property
    def slpattern(self):
        """Message field 'slpattern'."""
        return self._slpattern

    @slpattern.setter
    def slpattern(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'slpattern' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'slpattern' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._slpattern = value

    @builtins.property
    def tlpattern(self):
        """Message field 'tlpattern'."""
        return self._tlpattern

    @tlpattern.setter
    def tlpattern(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tlpattern' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tlpattern' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tlpattern = value

    @builtins.property
    def xrpattern(self):
        """Message field 'xrpattern'."""
        return self._xrpattern

    @xrpattern.setter
    def xrpattern(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'xrpattern' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'xrpattern' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._xrpattern = value

    @builtins.property
    def yrpattern(self):
        """Message field 'yrpattern'."""
        return self._yrpattern

    @yrpattern.setter
    def yrpattern(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yrpattern' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'yrpattern' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._yrpattern = value

    @builtins.property
    def zrpattern(self):
        """Message field 'zrpattern'."""
        return self._zrpattern

    @zrpattern.setter
    def zrpattern(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'zrpattern' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'zrpattern' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._zrpattern = value

    @builtins.property
    def srpattern(self):
        """Message field 'srpattern'."""
        return self._srpattern

    @srpattern.setter
    def srpattern(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'srpattern' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'srpattern' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._srpattern = value

    @builtins.property
    def trpattern(self):
        """Message field 'trpattern'."""
        return self._trpattern

    @trpattern.setter
    def trpattern(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'trpattern' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'trpattern' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._trpattern = value
