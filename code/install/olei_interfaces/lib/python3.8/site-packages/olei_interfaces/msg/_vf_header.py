# generated from rosidl_generator_py/resource/_idl.py.em
# with input from olei_interfaces:msg/VFHeader.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VFHeader(type):
    """Metaclass of message 'VFHeader'."""

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
                'olei_interfaces.msg.VFHeader')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__vf_header
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__vf_header
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__vf_header
            cls._TYPE_SUPPORT = module.type_support_msg__msg__vf_header
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__vf_header

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class VFHeader(metaclass=Metaclass_VFHeader):
    """Message class 'VFHeader'."""

    __slots__ = [
        '_magic',
        '_version',
        '_packet_size',
        '_header_size',
        '_distance_ratio',
        '_types',
        '_scan_number',
        '_packet_number',
        '_timestamp_decimal',
        '_timestamp_integer',
        '_scan_frequency',
        '_num_points_scan',
        '_iutput_status',
        '_output_status',
        '_field_status',
        '_start_index',
        '_end_index',
        '_first_index',
        '_num_points_packet',
        '_status_flags',
    ]

    _fields_and_field_types = {
        'magic': 'uint16',
        'version': 'uint16',
        'packet_size': 'uint32',
        'header_size': 'uint16',
        'distance_ratio': 'uint8',
        'types': 'uint8',
        'scan_number': 'uint16',
        'packet_number': 'uint16',
        'timestamp_decimal': 'uint32',
        'timestamp_integer': 'uint32',
        'scan_frequency': 'uint16',
        'num_points_scan': 'uint16',
        'iutput_status': 'uint16',
        'output_status': 'uint16',
        'field_status': 'uint32',
        'start_index': 'uint16',
        'end_index': 'uint16',
        'first_index': 'uint16',
        'num_points_packet': 'uint16',
        'status_flags': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.magic = kwargs.get('magic', int())
        self.version = kwargs.get('version', int())
        self.packet_size = kwargs.get('packet_size', int())
        self.header_size = kwargs.get('header_size', int())
        self.distance_ratio = kwargs.get('distance_ratio', int())
        self.types = kwargs.get('types', int())
        self.scan_number = kwargs.get('scan_number', int())
        self.packet_number = kwargs.get('packet_number', int())
        self.timestamp_decimal = kwargs.get('timestamp_decimal', int())
        self.timestamp_integer = kwargs.get('timestamp_integer', int())
        self.scan_frequency = kwargs.get('scan_frequency', int())
        self.num_points_scan = kwargs.get('num_points_scan', int())
        self.iutput_status = kwargs.get('iutput_status', int())
        self.output_status = kwargs.get('output_status', int())
        self.field_status = kwargs.get('field_status', int())
        self.start_index = kwargs.get('start_index', int())
        self.end_index = kwargs.get('end_index', int())
        self.first_index = kwargs.get('first_index', int())
        self.num_points_packet = kwargs.get('num_points_packet', int())
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
        if self.packet_size != other.packet_size:
            return False
        if self.header_size != other.header_size:
            return False
        if self.distance_ratio != other.distance_ratio:
            return False
        if self.types != other.types:
            return False
        if self.scan_number != other.scan_number:
            return False
        if self.packet_number != other.packet_number:
            return False
        if self.timestamp_decimal != other.timestamp_decimal:
            return False
        if self.timestamp_integer != other.timestamp_integer:
            return False
        if self.scan_frequency != other.scan_frequency:
            return False
        if self.num_points_scan != other.num_points_scan:
            return False
        if self.iutput_status != other.iutput_status:
            return False
        if self.output_status != other.output_status:
            return False
        if self.field_status != other.field_status:
            return False
        if self.start_index != other.start_index:
            return False
        if self.end_index != other.end_index:
            return False
        if self.first_index != other.first_index:
            return False
        if self.num_points_packet != other.num_points_packet:
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
            assert value >= 0 and value < 65536, \
                "The 'magic' field must be an unsigned integer in [0, 65535]"
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
    def packet_size(self):
        """Message field 'packet_size'."""
        return self._packet_size

    @packet_size.setter
    def packet_size(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'packet_size' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'packet_size' field must be an unsigned integer in [0, 4294967295]"
        self._packet_size = value

    @property
    def header_size(self):
        """Message field 'header_size'."""
        return self._header_size

    @header_size.setter
    def header_size(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'header_size' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'header_size' field must be an unsigned integer in [0, 65535]"
        self._header_size = value

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
    def types(self):
        """Message field 'types'."""
        return self._types

    @types.setter
    def types(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'types' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'types' field must be an unsigned integer in [0, 255]"
        self._types = value

    @property
    def scan_number(self):
        """Message field 'scan_number'."""
        return self._scan_number

    @scan_number.setter
    def scan_number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'scan_number' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'scan_number' field must be an unsigned integer in [0, 65535]"
        self._scan_number = value

    @property
    def packet_number(self):
        """Message field 'packet_number'."""
        return self._packet_number

    @packet_number.setter
    def packet_number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'packet_number' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'packet_number' field must be an unsigned integer in [0, 65535]"
        self._packet_number = value

    @property
    def timestamp_decimal(self):
        """Message field 'timestamp_decimal'."""
        return self._timestamp_decimal

    @timestamp_decimal.setter
    def timestamp_decimal(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timestamp_decimal' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'timestamp_decimal' field must be an unsigned integer in [0, 4294967295]"
        self._timestamp_decimal = value

    @property
    def timestamp_integer(self):
        """Message field 'timestamp_integer'."""
        return self._timestamp_integer

    @timestamp_integer.setter
    def timestamp_integer(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timestamp_integer' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'timestamp_integer' field must be an unsigned integer in [0, 4294967295]"
        self._timestamp_integer = value

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
    def num_points_scan(self):
        """Message field 'num_points_scan'."""
        return self._num_points_scan

    @num_points_scan.setter
    def num_points_scan(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_points_scan' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'num_points_scan' field must be an unsigned integer in [0, 65535]"
        self._num_points_scan = value

    @property
    def iutput_status(self):
        """Message field 'iutput_status'."""
        return self._iutput_status

    @iutput_status.setter
    def iutput_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'iutput_status' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'iutput_status' field must be an unsigned integer in [0, 65535]"
        self._iutput_status = value

    @property
    def output_status(self):
        """Message field 'output_status'."""
        return self._output_status

    @output_status.setter
    def output_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'output_status' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'output_status' field must be an unsigned integer in [0, 65535]"
        self._output_status = value

    @property
    def field_status(self):
        """Message field 'field_status'."""
        return self._field_status

    @field_status.setter
    def field_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'field_status' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'field_status' field must be an unsigned integer in [0, 4294967295]"
        self._field_status = value

    @property
    def start_index(self):
        """Message field 'start_index'."""
        return self._start_index

    @start_index.setter
    def start_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'start_index' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'start_index' field must be an unsigned integer in [0, 65535]"
        self._start_index = value

    @property
    def end_index(self):
        """Message field 'end_index'."""
        return self._end_index

    @end_index.setter
    def end_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'end_index' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'end_index' field must be an unsigned integer in [0, 65535]"
        self._end_index = value

    @property
    def first_index(self):
        """Message field 'first_index'."""
        return self._first_index

    @first_index.setter
    def first_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'first_index' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'first_index' field must be an unsigned integer in [0, 65535]"
        self._first_index = value

    @property
    def num_points_packet(self):
        """Message field 'num_points_packet'."""
        return self._num_points_packet

    @num_points_packet.setter
    def num_points_packet(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_points_packet' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'num_points_packet' field must be an unsigned integer in [0, 65535]"
        self._num_points_packet = value

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
