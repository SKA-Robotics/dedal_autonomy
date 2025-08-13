# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_msgs:msg/ImuData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ImuData(type):
    """Metaclass of message 'ImuData'."""

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
                'custom_msgs.msg.ImuData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__imu_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__imu_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__imu_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__imu_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__imu_data

            from custom_msgs.msg import AccelData
            if AccelData.__class__._TYPE_SUPPORT is None:
                AccelData.__class__.__import_type_support__()

            from custom_msgs.msg import GyroData
            if GyroData.__class__._TYPE_SUPPORT is None:
                GyroData.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ImuData(metaclass=Metaclass_ImuData):
    """Message class 'ImuData'."""

    __slots__ = [
        '_timestamp',
        '_accel',
        '_gyro',
    ]

    _fields_and_field_types = {
        'timestamp': 'int64',
        'accel': 'custom_msgs/AccelData',
        'gyro': 'custom_msgs/GyroData',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['custom_msgs', 'msg'], 'AccelData'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['custom_msgs', 'msg'], 'GyroData'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.timestamp = kwargs.get('timestamp', int())
        from custom_msgs.msg import AccelData
        self.accel = kwargs.get('accel', AccelData())
        from custom_msgs.msg import GyroData
        self.gyro = kwargs.get('gyro', GyroData())

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
        if self.timestamp != other.timestamp:
            return False
        if self.accel != other.accel:
            return False
        if self.gyro != other.gyro:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timestamp' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'timestamp' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._timestamp = value

    @builtins.property
    def accel(self):
        """Message field 'accel'."""
        return self._accel

    @accel.setter
    def accel(self, value):
        if __debug__:
            from custom_msgs.msg import AccelData
            assert \
                isinstance(value, AccelData), \
                "The 'accel' field must be a sub message of type 'AccelData'"
        self._accel = value

    @builtins.property
    def gyro(self):
        """Message field 'gyro'."""
        return self._gyro

    @gyro.setter
    def gyro(self, value):
        if __debug__:
            from custom_msgs.msg import GyroData
            assert \
                isinstance(value, GyroData), \
                "The 'gyro' field must be a sub message of type 'GyroData'"
        self._gyro = value
