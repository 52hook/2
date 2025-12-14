# generated from rosidl_generator_py/resource/_idl.py.em
# with input from yhs_can_interfaces:msg/VehDiagFb.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VehDiagFb(type):
    """Metaclass of message 'VehDiagFb'."""

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
                'yhs_can_interfaces.msg.VehDiagFb')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__veh_diag_fb
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__veh_diag_fb
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__veh_diag_fb
            cls._TYPE_SUPPORT = module.type_support_msg__msg__veh_diag_fb
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__veh_diag_fb

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class VehDiagFb(metaclass=Metaclass_VehDiagFb):
    """Message class 'VehDiagFb'."""

    __slots__ = [
        '_veh_fb_fault_level',
        '_veh_fb_auto_can_ctrl_cmd',
        '_veh_fb_auto_io_can_cmd',
        '_veh_fb_eps_dis_on_line',
        '_veh_fb_eps_fault',
        '_veh_fb_eps_mosf_et_ot',
        '_veh_fb_eps_warning',
        '_veh_fb_eps_dis_work',
        '_veh_fb_eps_over_current',
        '_veh_fb_st_reserve',
        '_veh_fb_ehb_ecu_fault',
        '_veh_fb_ehb_dis_on_line',
        '_veh_fb_ehb_work_model_fault',
        '_veh_fb_ehb_dis_en',
        '_veh_fb_ehb_anguler_fault',
        '_veh_fb_ehb_ot',
        '_veh_fb_ehb_power_fault',
        '_veh_fb_ehb_sensor_abnomal',
        '_veh_fb_ehb_motor_fault',
        '_veh_fb_ehb_oil_press_sensor_fault',
        '_veh_fb_ehb_oil_fault',
        '_veh_fb_ld_rv_mcu_fault',
        '_veh_fb_rd_rv_mcu_fault',
        '_veh_fb_aux_bms_dis_on_line',
        '_veh_fb_aux_scram',
        '_veh_fb_aux_remote_close',
        '_veh_fb_aux_remote_dis_on_line',
    ]

    _fields_and_field_types = {
        'veh_fb_fault_level': 'uint8',
        'veh_fb_auto_can_ctrl_cmd': 'boolean',
        'veh_fb_auto_io_can_cmd': 'boolean',
        'veh_fb_eps_dis_on_line': 'boolean',
        'veh_fb_eps_fault': 'boolean',
        'veh_fb_eps_mosf_et_ot': 'boolean',
        'veh_fb_eps_warning': 'boolean',
        'veh_fb_eps_dis_work': 'boolean',
        'veh_fb_eps_over_current': 'boolean',
        'veh_fb_st_reserve': 'boolean',
        'veh_fb_ehb_ecu_fault': 'boolean',
        'veh_fb_ehb_dis_on_line': 'boolean',
        'veh_fb_ehb_work_model_fault': 'boolean',
        'veh_fb_ehb_dis_en': 'boolean',
        'veh_fb_ehb_anguler_fault': 'boolean',
        'veh_fb_ehb_ot': 'boolean',
        'veh_fb_ehb_power_fault': 'boolean',
        'veh_fb_ehb_sensor_abnomal': 'boolean',
        'veh_fb_ehb_motor_fault': 'boolean',
        'veh_fb_ehb_oil_press_sensor_fault': 'boolean',
        'veh_fb_ehb_oil_fault': 'boolean',
        'veh_fb_ld_rv_mcu_fault': 'uint8',
        'veh_fb_rd_rv_mcu_fault': 'uint8',
        'veh_fb_aux_bms_dis_on_line': 'boolean',
        'veh_fb_aux_scram': 'boolean',
        'veh_fb_aux_remote_close': 'boolean',
        'veh_fb_aux_remote_dis_on_line': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.veh_fb_fault_level = kwargs.get('veh_fb_fault_level', int())
        self.veh_fb_auto_can_ctrl_cmd = kwargs.get('veh_fb_auto_can_ctrl_cmd', bool())
        self.veh_fb_auto_io_can_cmd = kwargs.get('veh_fb_auto_io_can_cmd', bool())
        self.veh_fb_eps_dis_on_line = kwargs.get('veh_fb_eps_dis_on_line', bool())
        self.veh_fb_eps_fault = kwargs.get('veh_fb_eps_fault', bool())
        self.veh_fb_eps_mosf_et_ot = kwargs.get('veh_fb_eps_mosf_et_ot', bool())
        self.veh_fb_eps_warning = kwargs.get('veh_fb_eps_warning', bool())
        self.veh_fb_eps_dis_work = kwargs.get('veh_fb_eps_dis_work', bool())
        self.veh_fb_eps_over_current = kwargs.get('veh_fb_eps_over_current', bool())
        self.veh_fb_st_reserve = kwargs.get('veh_fb_st_reserve', bool())
        self.veh_fb_ehb_ecu_fault = kwargs.get('veh_fb_ehb_ecu_fault', bool())
        self.veh_fb_ehb_dis_on_line = kwargs.get('veh_fb_ehb_dis_on_line', bool())
        self.veh_fb_ehb_work_model_fault = kwargs.get('veh_fb_ehb_work_model_fault', bool())
        self.veh_fb_ehb_dis_en = kwargs.get('veh_fb_ehb_dis_en', bool())
        self.veh_fb_ehb_anguler_fault = kwargs.get('veh_fb_ehb_anguler_fault', bool())
        self.veh_fb_ehb_ot = kwargs.get('veh_fb_ehb_ot', bool())
        self.veh_fb_ehb_power_fault = kwargs.get('veh_fb_ehb_power_fault', bool())
        self.veh_fb_ehb_sensor_abnomal = kwargs.get('veh_fb_ehb_sensor_abnomal', bool())
        self.veh_fb_ehb_motor_fault = kwargs.get('veh_fb_ehb_motor_fault', bool())
        self.veh_fb_ehb_oil_press_sensor_fault = kwargs.get('veh_fb_ehb_oil_press_sensor_fault', bool())
        self.veh_fb_ehb_oil_fault = kwargs.get('veh_fb_ehb_oil_fault', bool())
        self.veh_fb_ld_rv_mcu_fault = kwargs.get('veh_fb_ld_rv_mcu_fault', int())
        self.veh_fb_rd_rv_mcu_fault = kwargs.get('veh_fb_rd_rv_mcu_fault', int())
        self.veh_fb_aux_bms_dis_on_line = kwargs.get('veh_fb_aux_bms_dis_on_line', bool())
        self.veh_fb_aux_scram = kwargs.get('veh_fb_aux_scram', bool())
        self.veh_fb_aux_remote_close = kwargs.get('veh_fb_aux_remote_close', bool())
        self.veh_fb_aux_remote_dis_on_line = kwargs.get('veh_fb_aux_remote_dis_on_line', bool())

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
        if self.veh_fb_fault_level != other.veh_fb_fault_level:
            return False
        if self.veh_fb_auto_can_ctrl_cmd != other.veh_fb_auto_can_ctrl_cmd:
            return False
        if self.veh_fb_auto_io_can_cmd != other.veh_fb_auto_io_can_cmd:
            return False
        if self.veh_fb_eps_dis_on_line != other.veh_fb_eps_dis_on_line:
            return False
        if self.veh_fb_eps_fault != other.veh_fb_eps_fault:
            return False
        if self.veh_fb_eps_mosf_et_ot != other.veh_fb_eps_mosf_et_ot:
            return False
        if self.veh_fb_eps_warning != other.veh_fb_eps_warning:
            return False
        if self.veh_fb_eps_dis_work != other.veh_fb_eps_dis_work:
            return False
        if self.veh_fb_eps_over_current != other.veh_fb_eps_over_current:
            return False
        if self.veh_fb_st_reserve != other.veh_fb_st_reserve:
            return False
        if self.veh_fb_ehb_ecu_fault != other.veh_fb_ehb_ecu_fault:
            return False
        if self.veh_fb_ehb_dis_on_line != other.veh_fb_ehb_dis_on_line:
            return False
        if self.veh_fb_ehb_work_model_fault != other.veh_fb_ehb_work_model_fault:
            return False
        if self.veh_fb_ehb_dis_en != other.veh_fb_ehb_dis_en:
            return False
        if self.veh_fb_ehb_anguler_fault != other.veh_fb_ehb_anguler_fault:
            return False
        if self.veh_fb_ehb_ot != other.veh_fb_ehb_ot:
            return False
        if self.veh_fb_ehb_power_fault != other.veh_fb_ehb_power_fault:
            return False
        if self.veh_fb_ehb_sensor_abnomal != other.veh_fb_ehb_sensor_abnomal:
            return False
        if self.veh_fb_ehb_motor_fault != other.veh_fb_ehb_motor_fault:
            return False
        if self.veh_fb_ehb_oil_press_sensor_fault != other.veh_fb_ehb_oil_press_sensor_fault:
            return False
        if self.veh_fb_ehb_oil_fault != other.veh_fb_ehb_oil_fault:
            return False
        if self.veh_fb_ld_rv_mcu_fault != other.veh_fb_ld_rv_mcu_fault:
            return False
        if self.veh_fb_rd_rv_mcu_fault != other.veh_fb_rd_rv_mcu_fault:
            return False
        if self.veh_fb_aux_bms_dis_on_line != other.veh_fb_aux_bms_dis_on_line:
            return False
        if self.veh_fb_aux_scram != other.veh_fb_aux_scram:
            return False
        if self.veh_fb_aux_remote_close != other.veh_fb_aux_remote_close:
            return False
        if self.veh_fb_aux_remote_dis_on_line != other.veh_fb_aux_remote_dis_on_line:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def veh_fb_fault_level(self):
        """Message field 'veh_fb_fault_level'."""
        return self._veh_fb_fault_level

    @veh_fb_fault_level.setter
    def veh_fb_fault_level(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'veh_fb_fault_level' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'veh_fb_fault_level' field must be an unsigned integer in [0, 255]"
        self._veh_fb_fault_level = value

    @builtins.property
    def veh_fb_auto_can_ctrl_cmd(self):
        """Message field 'veh_fb_auto_can_ctrl_cmd'."""
        return self._veh_fb_auto_can_ctrl_cmd

    @veh_fb_auto_can_ctrl_cmd.setter
    def veh_fb_auto_can_ctrl_cmd(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_auto_can_ctrl_cmd' field must be of type 'bool'"
        self._veh_fb_auto_can_ctrl_cmd = value

    @builtins.property
    def veh_fb_auto_io_can_cmd(self):
        """Message field 'veh_fb_auto_io_can_cmd'."""
        return self._veh_fb_auto_io_can_cmd

    @veh_fb_auto_io_can_cmd.setter
    def veh_fb_auto_io_can_cmd(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_auto_io_can_cmd' field must be of type 'bool'"
        self._veh_fb_auto_io_can_cmd = value

    @builtins.property
    def veh_fb_eps_dis_on_line(self):
        """Message field 'veh_fb_eps_dis_on_line'."""
        return self._veh_fb_eps_dis_on_line

    @veh_fb_eps_dis_on_line.setter
    def veh_fb_eps_dis_on_line(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_eps_dis_on_line' field must be of type 'bool'"
        self._veh_fb_eps_dis_on_line = value

    @builtins.property
    def veh_fb_eps_fault(self):
        """Message field 'veh_fb_eps_fault'."""
        return self._veh_fb_eps_fault

    @veh_fb_eps_fault.setter
    def veh_fb_eps_fault(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_eps_fault' field must be of type 'bool'"
        self._veh_fb_eps_fault = value

    @builtins.property
    def veh_fb_eps_mosf_et_ot(self):
        """Message field 'veh_fb_eps_mosf_et_ot'."""
        return self._veh_fb_eps_mosf_et_ot

    @veh_fb_eps_mosf_et_ot.setter
    def veh_fb_eps_mosf_et_ot(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_eps_mosf_et_ot' field must be of type 'bool'"
        self._veh_fb_eps_mosf_et_ot = value

    @builtins.property
    def veh_fb_eps_warning(self):
        """Message field 'veh_fb_eps_warning'."""
        return self._veh_fb_eps_warning

    @veh_fb_eps_warning.setter
    def veh_fb_eps_warning(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_eps_warning' field must be of type 'bool'"
        self._veh_fb_eps_warning = value

    @builtins.property
    def veh_fb_eps_dis_work(self):
        """Message field 'veh_fb_eps_dis_work'."""
        return self._veh_fb_eps_dis_work

    @veh_fb_eps_dis_work.setter
    def veh_fb_eps_dis_work(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_eps_dis_work' field must be of type 'bool'"
        self._veh_fb_eps_dis_work = value

    @builtins.property
    def veh_fb_eps_over_current(self):
        """Message field 'veh_fb_eps_over_current'."""
        return self._veh_fb_eps_over_current

    @veh_fb_eps_over_current.setter
    def veh_fb_eps_over_current(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_eps_over_current' field must be of type 'bool'"
        self._veh_fb_eps_over_current = value

    @builtins.property
    def veh_fb_st_reserve(self):
        """Message field 'veh_fb_st_reserve'."""
        return self._veh_fb_st_reserve

    @veh_fb_st_reserve.setter
    def veh_fb_st_reserve(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_st_reserve' field must be of type 'bool'"
        self._veh_fb_st_reserve = value

    @builtins.property
    def veh_fb_ehb_ecu_fault(self):
        """Message field 'veh_fb_ehb_ecu_fault'."""
        return self._veh_fb_ehb_ecu_fault

    @veh_fb_ehb_ecu_fault.setter
    def veh_fb_ehb_ecu_fault(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_ehb_ecu_fault' field must be of type 'bool'"
        self._veh_fb_ehb_ecu_fault = value

    @builtins.property
    def veh_fb_ehb_dis_on_line(self):
        """Message field 'veh_fb_ehb_dis_on_line'."""
        return self._veh_fb_ehb_dis_on_line

    @veh_fb_ehb_dis_on_line.setter
    def veh_fb_ehb_dis_on_line(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_ehb_dis_on_line' field must be of type 'bool'"
        self._veh_fb_ehb_dis_on_line = value

    @builtins.property
    def veh_fb_ehb_work_model_fault(self):
        """Message field 'veh_fb_ehb_work_model_fault'."""
        return self._veh_fb_ehb_work_model_fault

    @veh_fb_ehb_work_model_fault.setter
    def veh_fb_ehb_work_model_fault(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_ehb_work_model_fault' field must be of type 'bool'"
        self._veh_fb_ehb_work_model_fault = value

    @builtins.property
    def veh_fb_ehb_dis_en(self):
        """Message field 'veh_fb_ehb_dis_en'."""
        return self._veh_fb_ehb_dis_en

    @veh_fb_ehb_dis_en.setter
    def veh_fb_ehb_dis_en(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_ehb_dis_en' field must be of type 'bool'"
        self._veh_fb_ehb_dis_en = value

    @builtins.property
    def veh_fb_ehb_anguler_fault(self):
        """Message field 'veh_fb_ehb_anguler_fault'."""
        return self._veh_fb_ehb_anguler_fault

    @veh_fb_ehb_anguler_fault.setter
    def veh_fb_ehb_anguler_fault(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_ehb_anguler_fault' field must be of type 'bool'"
        self._veh_fb_ehb_anguler_fault = value

    @builtins.property
    def veh_fb_ehb_ot(self):
        """Message field 'veh_fb_ehb_ot'."""
        return self._veh_fb_ehb_ot

    @veh_fb_ehb_ot.setter
    def veh_fb_ehb_ot(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_ehb_ot' field must be of type 'bool'"
        self._veh_fb_ehb_ot = value

    @builtins.property
    def veh_fb_ehb_power_fault(self):
        """Message field 'veh_fb_ehb_power_fault'."""
        return self._veh_fb_ehb_power_fault

    @veh_fb_ehb_power_fault.setter
    def veh_fb_ehb_power_fault(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_ehb_power_fault' field must be of type 'bool'"
        self._veh_fb_ehb_power_fault = value

    @builtins.property
    def veh_fb_ehb_sensor_abnomal(self):
        """Message field 'veh_fb_ehb_sensor_abnomal'."""
        return self._veh_fb_ehb_sensor_abnomal

    @veh_fb_ehb_sensor_abnomal.setter
    def veh_fb_ehb_sensor_abnomal(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_ehb_sensor_abnomal' field must be of type 'bool'"
        self._veh_fb_ehb_sensor_abnomal = value

    @builtins.property
    def veh_fb_ehb_motor_fault(self):
        """Message field 'veh_fb_ehb_motor_fault'."""
        return self._veh_fb_ehb_motor_fault

    @veh_fb_ehb_motor_fault.setter
    def veh_fb_ehb_motor_fault(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_ehb_motor_fault' field must be of type 'bool'"
        self._veh_fb_ehb_motor_fault = value

    @builtins.property
    def veh_fb_ehb_oil_press_sensor_fault(self):
        """Message field 'veh_fb_ehb_oil_press_sensor_fault'."""
        return self._veh_fb_ehb_oil_press_sensor_fault

    @veh_fb_ehb_oil_press_sensor_fault.setter
    def veh_fb_ehb_oil_press_sensor_fault(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_ehb_oil_press_sensor_fault' field must be of type 'bool'"
        self._veh_fb_ehb_oil_press_sensor_fault = value

    @builtins.property
    def veh_fb_ehb_oil_fault(self):
        """Message field 'veh_fb_ehb_oil_fault'."""
        return self._veh_fb_ehb_oil_fault

    @veh_fb_ehb_oil_fault.setter
    def veh_fb_ehb_oil_fault(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_ehb_oil_fault' field must be of type 'bool'"
        self._veh_fb_ehb_oil_fault = value

    @builtins.property
    def veh_fb_ld_rv_mcu_fault(self):
        """Message field 'veh_fb_ld_rv_mcu_fault'."""
        return self._veh_fb_ld_rv_mcu_fault

    @veh_fb_ld_rv_mcu_fault.setter
    def veh_fb_ld_rv_mcu_fault(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'veh_fb_ld_rv_mcu_fault' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'veh_fb_ld_rv_mcu_fault' field must be an unsigned integer in [0, 255]"
        self._veh_fb_ld_rv_mcu_fault = value

    @builtins.property
    def veh_fb_rd_rv_mcu_fault(self):
        """Message field 'veh_fb_rd_rv_mcu_fault'."""
        return self._veh_fb_rd_rv_mcu_fault

    @veh_fb_rd_rv_mcu_fault.setter
    def veh_fb_rd_rv_mcu_fault(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'veh_fb_rd_rv_mcu_fault' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'veh_fb_rd_rv_mcu_fault' field must be an unsigned integer in [0, 255]"
        self._veh_fb_rd_rv_mcu_fault = value

    @builtins.property
    def veh_fb_aux_bms_dis_on_line(self):
        """Message field 'veh_fb_aux_bms_dis_on_line'."""
        return self._veh_fb_aux_bms_dis_on_line

    @veh_fb_aux_bms_dis_on_line.setter
    def veh_fb_aux_bms_dis_on_line(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_aux_bms_dis_on_line' field must be of type 'bool'"
        self._veh_fb_aux_bms_dis_on_line = value

    @builtins.property
    def veh_fb_aux_scram(self):
        """Message field 'veh_fb_aux_scram'."""
        return self._veh_fb_aux_scram

    @veh_fb_aux_scram.setter
    def veh_fb_aux_scram(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_aux_scram' field must be of type 'bool'"
        self._veh_fb_aux_scram = value

    @builtins.property
    def veh_fb_aux_remote_close(self):
        """Message field 'veh_fb_aux_remote_close'."""
        return self._veh_fb_aux_remote_close

    @veh_fb_aux_remote_close.setter
    def veh_fb_aux_remote_close(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_aux_remote_close' field must be of type 'bool'"
        self._veh_fb_aux_remote_close = value

    @builtins.property
    def veh_fb_aux_remote_dis_on_line(self):
        """Message field 'veh_fb_aux_remote_dis_on_line'."""
        return self._veh_fb_aux_remote_dis_on_line

    @veh_fb_aux_remote_dis_on_line.setter
    def veh_fb_aux_remote_dis_on_line(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'veh_fb_aux_remote_dis_on_line' field must be of type 'bool'"
        self._veh_fb_aux_remote_dis_on_line = value
