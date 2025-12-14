# generated from rosidl_generator_py/resource/_idl.py.em
# with input from yhs_can_interfaces:msg/ChassisInfoFb.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ChassisInfoFb(type):
    """Metaclass of message 'ChassisInfoFb'."""

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
                'yhs_can_interfaces.msg.ChassisInfoFb')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__chassis_info_fb
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__chassis_info_fb
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__chassis_info_fb
            cls._TYPE_SUPPORT = module.type_support_msg__msg__chassis_info_fb
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__chassis_info_fb

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from yhs_can_interfaces.msg import BmsFlagInfoFb
            if BmsFlagInfoFb.__class__._TYPE_SUPPORT is None:
                BmsFlagInfoFb.__class__.__import_type_support__()

            from yhs_can_interfaces.msg import BmsInfoFb
            if BmsInfoFb.__class__._TYPE_SUPPORT is None:
                BmsInfoFb.__class__.__import_type_support__()

            from yhs_can_interfaces.msg import CtrlFb
            if CtrlFb.__class__._TYPE_SUPPORT is None:
                CtrlFb.__class__.__import_type_support__()

            from yhs_can_interfaces.msg import DriveMcuEcodeFb
            if DriveMcuEcodeFb.__class__._TYPE_SUPPORT is None:
                DriveMcuEcodeFb.__class__.__import_type_support__()

            from yhs_can_interfaces.msg import IoFb
            if IoFb.__class__._TYPE_SUPPORT is None:
                IoFb.__class__.__import_type_support__()

            from yhs_can_interfaces.msg import LrWheelFb
            if LrWheelFb.__class__._TYPE_SUPPORT is None:
                LrWheelFb.__class__.__import_type_support__()

            from yhs_can_interfaces.msg import RrWheelFb
            if RrWheelFb.__class__._TYPE_SUPPORT is None:
                RrWheelFb.__class__.__import_type_support__()

            from yhs_can_interfaces.msg import VehDiagFb
            if VehDiagFb.__class__._TYPE_SUPPORT is None:
                VehDiagFb.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ChassisInfoFb(metaclass=Metaclass_ChassisInfoFb):
    """Message class 'ChassisInfoFb'."""

    __slots__ = [
        '_header',
        '_ctrl_fb',
        '_io_fb',
        '_lr_wheel_fb',
        '_rr_wheel_fb',
        '_bms_flag_info_fb',
        '_bms_info_fb',
        '_drive_mcu_ecode_fb',
        '_veh_diag_fb',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'ctrl_fb': 'yhs_can_interfaces/CtrlFb',
        'io_fb': 'yhs_can_interfaces/IoFb',
        'lr_wheel_fb': 'yhs_can_interfaces/LrWheelFb',
        'rr_wheel_fb': 'yhs_can_interfaces/RrWheelFb',
        'bms_flag_info_fb': 'yhs_can_interfaces/BmsFlagInfoFb',
        'bms_info_fb': 'yhs_can_interfaces/BmsInfoFb',
        'drive_mcu_ecode_fb': 'yhs_can_interfaces/DriveMcuEcodeFb',
        'veh_diag_fb': 'yhs_can_interfaces/VehDiagFb',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['yhs_can_interfaces', 'msg'], 'CtrlFb'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['yhs_can_interfaces', 'msg'], 'IoFb'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['yhs_can_interfaces', 'msg'], 'LrWheelFb'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['yhs_can_interfaces', 'msg'], 'RrWheelFb'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['yhs_can_interfaces', 'msg'], 'BmsFlagInfoFb'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['yhs_can_interfaces', 'msg'], 'BmsInfoFb'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['yhs_can_interfaces', 'msg'], 'DriveMcuEcodeFb'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['yhs_can_interfaces', 'msg'], 'VehDiagFb'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from yhs_can_interfaces.msg import CtrlFb
        self.ctrl_fb = kwargs.get('ctrl_fb', CtrlFb())
        from yhs_can_interfaces.msg import IoFb
        self.io_fb = kwargs.get('io_fb', IoFb())
        from yhs_can_interfaces.msg import LrWheelFb
        self.lr_wheel_fb = kwargs.get('lr_wheel_fb', LrWheelFb())
        from yhs_can_interfaces.msg import RrWheelFb
        self.rr_wheel_fb = kwargs.get('rr_wheel_fb', RrWheelFb())
        from yhs_can_interfaces.msg import BmsFlagInfoFb
        self.bms_flag_info_fb = kwargs.get('bms_flag_info_fb', BmsFlagInfoFb())
        from yhs_can_interfaces.msg import BmsInfoFb
        self.bms_info_fb = kwargs.get('bms_info_fb', BmsInfoFb())
        from yhs_can_interfaces.msg import DriveMcuEcodeFb
        self.drive_mcu_ecode_fb = kwargs.get('drive_mcu_ecode_fb', DriveMcuEcodeFb())
        from yhs_can_interfaces.msg import VehDiagFb
        self.veh_diag_fb = kwargs.get('veh_diag_fb', VehDiagFb())

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
        if self.header != other.header:
            return False
        if self.ctrl_fb != other.ctrl_fb:
            return False
        if self.io_fb != other.io_fb:
            return False
        if self.lr_wheel_fb != other.lr_wheel_fb:
            return False
        if self.rr_wheel_fb != other.rr_wheel_fb:
            return False
        if self.bms_flag_info_fb != other.bms_flag_info_fb:
            return False
        if self.bms_info_fb != other.bms_info_fb:
            return False
        if self.drive_mcu_ecode_fb != other.drive_mcu_ecode_fb:
            return False
        if self.veh_diag_fb != other.veh_diag_fb:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def ctrl_fb(self):
        """Message field 'ctrl_fb'."""
        return self._ctrl_fb

    @ctrl_fb.setter
    def ctrl_fb(self, value):
        if __debug__:
            from yhs_can_interfaces.msg import CtrlFb
            assert \
                isinstance(value, CtrlFb), \
                "The 'ctrl_fb' field must be a sub message of type 'CtrlFb'"
        self._ctrl_fb = value

    @builtins.property
    def io_fb(self):
        """Message field 'io_fb'."""
        return self._io_fb

    @io_fb.setter
    def io_fb(self, value):
        if __debug__:
            from yhs_can_interfaces.msg import IoFb
            assert \
                isinstance(value, IoFb), \
                "The 'io_fb' field must be a sub message of type 'IoFb'"
        self._io_fb = value

    @builtins.property
    def lr_wheel_fb(self):
        """Message field 'lr_wheel_fb'."""
        return self._lr_wheel_fb

    @lr_wheel_fb.setter
    def lr_wheel_fb(self, value):
        if __debug__:
            from yhs_can_interfaces.msg import LrWheelFb
            assert \
                isinstance(value, LrWheelFb), \
                "The 'lr_wheel_fb' field must be a sub message of type 'LrWheelFb'"
        self._lr_wheel_fb = value

    @builtins.property
    def rr_wheel_fb(self):
        """Message field 'rr_wheel_fb'."""
        return self._rr_wheel_fb

    @rr_wheel_fb.setter
    def rr_wheel_fb(self, value):
        if __debug__:
            from yhs_can_interfaces.msg import RrWheelFb
            assert \
                isinstance(value, RrWheelFb), \
                "The 'rr_wheel_fb' field must be a sub message of type 'RrWheelFb'"
        self._rr_wheel_fb = value

    @builtins.property
    def bms_flag_info_fb(self):
        """Message field 'bms_flag_info_fb'."""
        return self._bms_flag_info_fb

    @bms_flag_info_fb.setter
    def bms_flag_info_fb(self, value):
        if __debug__:
            from yhs_can_interfaces.msg import BmsFlagInfoFb
            assert \
                isinstance(value, BmsFlagInfoFb), \
                "The 'bms_flag_info_fb' field must be a sub message of type 'BmsFlagInfoFb'"
        self._bms_flag_info_fb = value

    @builtins.property
    def bms_info_fb(self):
        """Message field 'bms_info_fb'."""
        return self._bms_info_fb

    @bms_info_fb.setter
    def bms_info_fb(self, value):
        if __debug__:
            from yhs_can_interfaces.msg import BmsInfoFb
            assert \
                isinstance(value, BmsInfoFb), \
                "The 'bms_info_fb' field must be a sub message of type 'BmsInfoFb'"
        self._bms_info_fb = value

    @builtins.property
    def drive_mcu_ecode_fb(self):
        """Message field 'drive_mcu_ecode_fb'."""
        return self._drive_mcu_ecode_fb

    @drive_mcu_ecode_fb.setter
    def drive_mcu_ecode_fb(self, value):
        if __debug__:
            from yhs_can_interfaces.msg import DriveMcuEcodeFb
            assert \
                isinstance(value, DriveMcuEcodeFb), \
                "The 'drive_mcu_ecode_fb' field must be a sub message of type 'DriveMcuEcodeFb'"
        self._drive_mcu_ecode_fb = value

    @builtins.property
    def veh_diag_fb(self):
        """Message field 'veh_diag_fb'."""
        return self._veh_diag_fb

    @veh_diag_fb.setter
    def veh_diag_fb(self, value):
        if __debug__:
            from yhs_can_interfaces.msg import VehDiagFb
            assert \
                isinstance(value, VehDiagFb), \
                "The 'veh_diag_fb' field must be a sub message of type 'VehDiagFb'"
        self._veh_diag_fb = value
