# generated from rosidl_generator_py/resource/_idl.py.em
# with input from lidar_msgs:msg/Metadata.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Metadata(type):
    """Metaclass of message 'Metadata'."""

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
            module = import_type_support('lidar_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'lidar_msgs.msg.Metadata')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__metadata
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__metadata
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__metadata
            cls._TYPE_SUPPORT = module.type_support_msg__msg__metadata
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__metadata

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Metadata(metaclass=Metaclass_Metadata):
    """Message class 'Metadata'."""

    __slots__ = [
        '_computer_ip',
        '_lidar_ip',
        '_imu_port',
        '_lidar_port',
    ]

    _fields_and_field_types = {
        'computer_ip': 'string',
        'lidar_ip': 'string',
        'imu_port': 'int8',
        'lidar_port': 'int8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.computer_ip = kwargs.get('computer_ip', str())
        self.lidar_ip = kwargs.get('lidar_ip', str())
        self.imu_port = kwargs.get('imu_port', int())
        self.lidar_port = kwargs.get('lidar_port', int())

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
        if self.computer_ip != other.computer_ip:
            return False
        if self.lidar_ip != other.lidar_ip:
            return False
        if self.imu_port != other.imu_port:
            return False
        if self.lidar_port != other.lidar_port:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def computer_ip(self):
        """Message field 'computer_ip'."""
        return self._computer_ip

    @computer_ip.setter
    def computer_ip(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'computer_ip' field must be of type 'str'"
        self._computer_ip = value

    @property
    def lidar_ip(self):
        """Message field 'lidar_ip'."""
        return self._lidar_ip

    @lidar_ip.setter
    def lidar_ip(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'lidar_ip' field must be of type 'str'"
        self._lidar_ip = value

    @property
    def imu_port(self):
        """Message field 'imu_port'."""
        return self._imu_port

    @imu_port.setter
    def imu_port(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'imu_port' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'imu_port' field must be an integer in [-128, 127]"
        self._imu_port = value

    @property
    def lidar_port(self):
        """Message field 'lidar_port'."""
        return self._lidar_port

    @lidar_port.setter
    def lidar_port(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'lidar_port' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'lidar_port' field must be an integer in [-128, 127]"
        self._lidar_port = value
