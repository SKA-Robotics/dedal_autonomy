# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_msgs:msg/TagLocation.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TagLocation(type):
    """Metaclass of message 'TagLocation'."""

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
            module = import_type_support('custom_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_msgs.msg.TagLocation')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__tag_location
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__tag_location
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__tag_location
            cls._TYPE_SUPPORT = module.type_support_msg__msg__tag_location
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__tag_location

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TagLocation(metaclass=Metaclass_TagLocation):
    """Message class 'TagLocation'."""

    __slots__ = [
        '_x_distance',
        '_y_distance',
        '_z_distance',
        '_pitch_rads',
        '_roll_rads',
        '_yaw_rads',
    ]

    _fields_and_field_types = {
        'x_distance': 'double',
        'y_distance': 'double',
        'z_distance': 'double',
        'pitch_rads': 'double',
        'roll_rads': 'double',
        'yaw_rads': 'double',
    }

    SLOT_TYPES = (
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
        self.x_distance = kwargs.get('x_distance', float())
        self.y_distance = kwargs.get('y_distance', float())
        self.z_distance = kwargs.get('z_distance', float())
        self.pitch_rads = kwargs.get('pitch_rads', float())
        self.roll_rads = kwargs.get('roll_rads', float())
        self.yaw_rads = kwargs.get('yaw_rads', float())

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
        if self.x_distance != other.x_distance:
            return False
        if self.y_distance != other.y_distance:
            return False
        if self.z_distance != other.z_distance:
            return False
        if self.pitch_rads != other.pitch_rads:
            return False
        if self.roll_rads != other.roll_rads:
            return False
        if self.yaw_rads != other.yaw_rads:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def x_distance(self):
        """Message field 'x_distance'."""
        return self._x_distance

    @x_distance.setter
    def x_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_distance' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x_distance' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x_distance = value

    @builtins.property
    def y_distance(self):
        """Message field 'y_distance'."""
        return self._y_distance

    @y_distance.setter
    def y_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_distance' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y_distance' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y_distance = value

    @builtins.property
    def z_distance(self):
        """Message field 'z_distance'."""
        return self._z_distance

    @z_distance.setter
    def z_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_distance' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'z_distance' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._z_distance = value

    @builtins.property
    def pitch_rads(self):
        """Message field 'pitch_rads'."""
        return self._pitch_rads

    @pitch_rads.setter
    def pitch_rads(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch_rads' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pitch_rads' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pitch_rads = value

    @builtins.property
    def roll_rads(self):
        """Message field 'roll_rads'."""
        return self._roll_rads

    @roll_rads.setter
    def roll_rads(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll_rads' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'roll_rads' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._roll_rads = value

    @builtins.property
    def yaw_rads(self):
        """Message field 'yaw_rads'."""
        return self._yaw_rads

    @yaw_rads.setter
    def yaw_rads(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_rads' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'yaw_rads' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._yaw_rads = value
