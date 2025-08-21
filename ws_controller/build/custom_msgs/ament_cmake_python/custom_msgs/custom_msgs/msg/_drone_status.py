# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_msgs:msg/DroneStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DroneStatus(type):
    """Metaclass of message 'DroneStatus'."""

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
                'custom_msgs.msg.DroneStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__drone_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__drone_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__drone_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__drone_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__drone_status

            from custom_msgs.msg import GeoData
            if GeoData.__class__._TYPE_SUPPORT is None:
                GeoData.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DroneStatus(metaclass=Metaclass_DroneStatus):
    """Message class 'DroneStatus'."""

    __slots__ = [
        '_is_autonomy_active',
        '_is_moving',
        '_battery_voltage',
        '_ekf_position',
    ]

    _fields_and_field_types = {
        'is_autonomy_active': 'boolean',
        'is_moving': 'boolean',
        'battery_voltage': 'float',
        'ekf_position': 'custom_msgs/GeoData',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['custom_msgs', 'msg'], 'GeoData'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.is_autonomy_active = kwargs.get('is_autonomy_active', bool())
        self.is_moving = kwargs.get('is_moving', bool())
        self.battery_voltage = kwargs.get('battery_voltage', float())
        from custom_msgs.msg import GeoData
        self.ekf_position = kwargs.get('ekf_position', GeoData())

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
        if self.is_autonomy_active != other.is_autonomy_active:
            return False
        if self.is_moving != other.is_moving:
            return False
        if self.battery_voltage != other.battery_voltage:
            return False
        if self.ekf_position != other.ekf_position:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def is_autonomy_active(self):
        """Message field 'is_autonomy_active'."""
        return self._is_autonomy_active

    @is_autonomy_active.setter
    def is_autonomy_active(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_autonomy_active' field must be of type 'bool'"
        self._is_autonomy_active = value

    @builtins.property
    def is_moving(self):
        """Message field 'is_moving'."""
        return self._is_moving

    @is_moving.setter
    def is_moving(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_moving' field must be of type 'bool'"
        self._is_moving = value

    @builtins.property
    def battery_voltage(self):
        """Message field 'battery_voltage'."""
        return self._battery_voltage

    @battery_voltage.setter
    def battery_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'battery_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._battery_voltage = value

    @builtins.property
    def ekf_position(self):
        """Message field 'ekf_position'."""
        return self._ekf_position

    @ekf_position.setter
    def ekf_position(self, value):
        if __debug__:
            from custom_msgs.msg import GeoData
            assert \
                isinstance(value, GeoData), \
                "The 'ekf_position' field must be a sub message of type 'GeoData'"
        self._ekf_position = value
