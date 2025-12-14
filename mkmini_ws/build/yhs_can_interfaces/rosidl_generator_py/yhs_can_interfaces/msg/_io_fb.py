# generated from rosidl_generator_py/resource/_idl.py.em
# with input from yhs_can_interfaces:msg/IoFb.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_IoFb(type):
    """Metaclass of message 'IoFb'."""

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
            module = import_type_support('yhs_can_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'yhs_can_interfaces.msg.IoFb')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__io_fb
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__io_fb
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__io_fb
            cls._TYPE_SUPPORT = module.type_support_msg__msg__io_fb
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__io_fb

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IoFb(metaclass=Metaclass_IoFb):
    """Message class 'IoFb'."""

    __slots__ = [
        '_io_fb_enable',
        '_io_fb_turn_lamp',
        '_io_fb_braking_lamp',
        '_io_fb_fm_impact_sensor',
        '_io_fb_rm_impact_sensor',
        '_io_fb_dis_charge',
        '_io_fb_charge_en',
        '_io_fb_scram_st',
    ]

    _fields_and_field_types = {
        'io_fb_enable': 'boolean',
        'io_fb_turn_lamp': 'int8',
        'io_fb_braking_lamp': 'boolean',
        'io_fb_fm_impact_sensor': 'boolean',
        'io_fb_rm_impact_sensor': 'boolean',
        'io_fb_dis_charge': 'boolean',
        'io_fb_charge_en': 'boolean',
        'io_fb_scram_st': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.io_fb_enable = kwargs.get('io_fb_enable', bool())
        self.io_fb_turn_lamp = kwargs.get('io_fb_turn_lamp', int())
        self.io_fb_braking_lamp = kwargs.get('io_fb_braking_lamp', bool())
        self.io_fb_fm_impact_sensor = kwargs.get('io_fb_fm_impact_sensor', bool())
        self.io_fb_rm_impact_sensor = kwargs.get('io_fb_rm_impact_sensor', bool())
        self.io_fb_dis_charge = kwargs.get('io_fb_dis_charge', bool())
        self.io_fb_charge_en = kwargs.get('io_fb_charge_en', bool())
        self.io_fb_scram_st = kwargs.get('io_fb_scram_st', bool())

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
        if self.io_fb_enable != other.io_fb_enable:
            return False
        if self.io_fb_turn_lamp != other.io_fb_turn_lamp:
            return False
        if self.io_fb_braking_lamp != other.io_fb_braking_lamp:
            return False
        if self.io_fb_fm_impact_sensor != other.io_fb_fm_impact_sensor:
            return False
        if self.io_fb_rm_impact_sensor != other.io_fb_rm_impact_sensor:
            return False
        if self.io_fb_dis_charge != other.io_fb_dis_charge:
            return False
        if self.io_fb_charge_en != other.io_fb_charge_en:
            return False
        if self.io_fb_scram_st != other.io_fb_scram_st:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def io_fb_enable(self):
        """Message field 'io_fb_enable'."""
        return self._io_fb_enable

    @io_fb_enable.setter
    def io_fb_enable(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'io_fb_enable' field must be of type 'bool'"
        self._io_fb_enable = value

    @builtins.property
    def io_fb_turn_lamp(self):
        """Message field 'io_fb_turn_lamp'."""
        return self._io_fb_turn_lamp

    @io_fb_turn_lamp.setter
    def io_fb_turn_lamp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'io_fb_turn_lamp' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'io_fb_turn_lamp' field must be an integer in [-128, 127]"
        self._io_fb_turn_lamp = value

    @builtins.property
    def io_fb_braking_lamp(self):
        """Message field 'io_fb_braking_lamp'."""
        return self._io_fb_braking_lamp

    @io_fb_braking_lamp.setter
    def io_fb_braking_lamp(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'io_fb_braking_lamp' field must be of type 'bool'"
        self._io_fb_braking_lamp = value

    @builtins.property
    def io_fb_fm_impact_sensor(self):
        """Message field 'io_fb_fm_impact_sensor'."""
        return self._io_fb_fm_impact_sensor

    @io_fb_fm_impact_sensor.setter
    def io_fb_fm_impact_sensor(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'io_fb_fm_impact_sensor' field must be of type 'bool'"
        self._io_fb_fm_impact_sensor = value

    @builtins.property
    def io_fb_rm_impact_sensor(self):
        """Message field 'io_fb_rm_impact_sensor'."""
        return self._io_fb_rm_impact_sensor

    @io_fb_rm_impact_sensor.setter
    def io_fb_rm_impact_sensor(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'io_fb_rm_impact_sensor' field must be of type 'bool'"
        self._io_fb_rm_impact_sensor = value

    @builtins.property
    def io_fb_dis_charge(self):
        """Message field 'io_fb_dis_charge'."""
        return self._io_fb_dis_charge

    @io_fb_dis_charge.setter
    def io_fb_dis_charge(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'io_fb_dis_charge' field must be of type 'bool'"
        self._io_fb_dis_charge = value

    @builtins.property
    def io_fb_charge_en(self):
        """Message field 'io_fb_charge_en'."""
        return self._io_fb_charge_en

    @io_fb_charge_en.setter
    def io_fb_charge_en(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'io_fb_charge_en' field must be of type 'bool'"
        self._io_fb_charge_en = value

    @builtins.property
    def io_fb_scram_st(self):
        """Message field 'io_fb_scram_st'."""
        return self._io_fb_scram_st

    @io_fb_scram_st.setter
    def io_fb_scram_st(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'io_fb_scram_st' field must be of type 'bool'"
        self._io_fb_scram_st = value
