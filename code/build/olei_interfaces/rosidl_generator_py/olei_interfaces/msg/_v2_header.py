# generated from rosidl_generator_py/resource/_idl.py.em
# with input from olei_interfaces:msg/V2Header.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'brand'
# Member 'vommercial'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_V2Header(type):
    """Metaclass of message 'V2Header'."""

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
            module = import_type_support('olei_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'olei_interfaces.msg.V2Header')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__v2_header
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__v2_header
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__v2_header
            cls._TYPE_SUPPORT = module.type_support_msg__msg__v2_header
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__v2_header

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class V2Header(metaclass=Metaclass_V2Header):
    """Message class 'V2Header'."""

    __slots__ = [
        '_magic',
        '_version',
        '_distance_ratio',
        '_brand',
        '_vommercial',
        '_internal',
        '_hardware',
        '_software',
        '_timestamp',
        '_scan_frequency',
        '_safe_status',
        '_error_status',
        '_status_flags',
    ]

    _fields_and_field_types = {
        'magic': 'uint32',
        'version': 'uint16',
        'distance_ratio': 'uint8',
        'brand': 'uint8[3]',
        'vommercial': 'uint8[12]',
        'internal': 'uint16',
        'hardware': 'uint16',
        'software': 'uint16',
        'timestamp': 'uint32',
        'scan_frequency': 'uint16',
        'safe_status': 'uint8',
        'error_status': 'uint8',
        'status_flags': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 12),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.magic = kwargs.get('magic', int())
        self.version = kwargs.get('version', int())
        self.distance_ratio = kwargs.get('distance_ratio', int())
        if 'brand' not in kwargs:
            self.brand = numpy.zeros(3, dtype=numpy.uint8)
        else:
            self.brand = numpy.array(kwargs.get('brand'), dtype=numpy.uint8)
            assert self.brand.shape == (3, )
        if 'vommercial' not in kwargs:
            self.vommercial = numpy.zeros(12, dtype=numpy.uint8)
        else:
            self.vommercial = numpy.array(kwargs.get('vommercial'), dtype=numpy.uint8)
            assert self.vommercial.shape == (12, )
        self.internal = kwargs.get('internal', int())
        self.hardware = kwargs.get('hardware', int())
        self.software = kwargs.get('software', int())
        self.timestamp = kwargs.get('timestamp', int())
        self.scan_frequency = kwargs.get('scan_frequency', int())
        self.safe_status = kwargs.get('safe_status', int())
        self.error_status = kwargs.get('error_status', int())
        self.status_flags = kwargs.get('status_flags', int())

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
        if self.magic != other.magic:
            return False
        if self.version != other.version:
            return False
        if self.distance_ratio != other.distance_ratio:
            return False
        if all(self.brand != other.brand):
            return False
        if all(self.vommercial != other.vommercial):
            return False
        if self.internal != other.internal:
            return False
        if self.hardware != other.hardware:
            return False
        if self.software != other.software:
            return False
        if self.timestamp != other.timestamp:
            return False
        if self.scan_frequency != other.scan_frequency:
            return False
        if self.safe_status != other.safe_status:
            return False
        if self.error_status != other.error_status:
            return False
        if self.status_flags != other.status_flags:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def magic(self):
        """Message field 'magic'."""
        return self._magic

    @magic.setter
    def magic(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'magic' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'magic' field must be an unsigned integer in [0, 4294967295]"
        self._magic = value

    @property
    def version(self):
        """Message field 'version'."""
        return self._version

    @version.setter
    def version(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'version' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'version' field must be an unsigned integer in [0, 65535]"
        self._version = value

    @property
    def distance_ratio(self):
        """Message field 'distance_ratio'."""
        return self._distance_ratio

    @distance_ratio.setter
    def distance_ratio(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'distance_ratio' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'distance_ratio' field must be an unsigned integer in [0, 255]"
        self._distance_ratio = value

    @property
    def brand(self):
        """Message field 'brand'."""
        return self._brand

    @brand.setter
    def brand(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'brand' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 3, \
                "The 'brand' numpy.ndarray() must have a size of 3"
            self._brand = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'brand' field must be a set or sequence with length 3 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._brand = numpy.array(value, dtype=numpy.uint8)

    @property
    def vommercial(self):
        """Message field 'vommercial'."""
        return self._vommercial

    @vommercial.setter
    def vommercial(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'vommercial' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 12, \
                "The 'vommercial' numpy.ndarray() must have a size of 12"
            self._vommercial = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 12 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'vommercial' field must be a set or sequence with length 12 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._vommercial = numpy.array(value, dtype=numpy.uint8)

    @property
    def internal(self):
        """Message field 'internal'."""
        return self._internal

    @internal.setter
    def internal(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'internal' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'internal' field must be an unsigned integer in [0, 65535]"
        self._internal = value

    @property
    def hardware(self):
        """Message field 'hardware'."""
        return self._hardware

    @hardware.setter
    def hardware(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'hardware' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'hardware' field must be an unsigned integer in [0, 65535]"
        self._hardware = value

    @property
    def software(self):
        """Message field 'software'."""
        return self._software

    @software.setter
    def software(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'software' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'software' field must be an unsigned integer in [0, 65535]"
        self._software = value

    @property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timestamp' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'timestamp' field must be an unsigned integer in [0, 4294967295]"
        self._timestamp = value

    @property
    def scan_frequency(self):
        """Message field 'scan_frequency'."""
        return self._scan_frequency

    @scan_frequency.setter
    def scan_frequency(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'scan_frequency' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'scan_frequency' field must be an unsigned integer in [0, 65535]"
        self._scan_frequency = value

    @property
    def safe_status(self):
        """Message field 'safe_status'."""
        return self._safe_status

    @safe_status.setter
    def safe_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'safe_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'safe_status' field must be an unsigned integer in [0, 255]"
        self._safe_status = value

    @property
    def error_status(self):
        """Message field 'error_status'."""
        return self._error_status

    @error_status.setter
    def error_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'error_status' field must be an unsigned integer in [0, 255]"
        self._error_status = value

    @property
    def status_flags(self):
        """Message field 'status_flags'."""
        return self._status_flags

    @status_flags.setter
    def status_flags(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status_flags' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'status_flags' field must be an unsigned integer in [0, 4294967295]"
        self._status_flags = value
