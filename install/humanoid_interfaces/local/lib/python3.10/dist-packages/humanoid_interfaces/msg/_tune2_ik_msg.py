# generated from rosidl_generator_py/resource/_idl.py.em
# with input from humanoid_interfaces:msg/Tune2IkMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'offset_motor'
# Member 'percentage_of_ik_motor'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Tune2IkMsg(type):
    """Metaclass of message 'Tune2IkMsg'."""

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
                'humanoid_interfaces.msg.Tune2IkMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__tune2_ik_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__tune2_ik_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__tune2_ik_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__tune2_ik_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__tune2_ik_msg

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Tune2IkMsg(metaclass=Metaclass_Tune2IkMsg):
    """Message class 'Tune2IkMsg'."""

    __slots__ = [
        '_ik_flag',
        '_entire_time',
        '_frequency',
        '_default_x_left',
        '_default_y_left',
        '_default_z_left',
        '_default_x_right',
        '_default_y_right',
        '_default_z_right',
        '_offset_motor',
        '_swing_shoulder_left',
        '_swing_shoulder_right',
        '_swing_leg_left',
        '_swing_leg_right',
        '_rise_leg_left',
        '_rise_leg_right',
        '_start_entire_time',
        '_start_swing',
        '_start_rise',
        '_end_entire_time',
        '_end_swing',
        '_end_rise',
        '_test_x',
        '_test_side',
        '_test_yaw',
        '_tuning_x',
        '_tuning_side',
        '_tuning_yaw',
        '_balance_value_0',
        '_balance_value_1',
        '_balance_value_2',
        '_balance_value_3',
        '_balance_pitch_gp',
        '_balance_pitch_gi',
        '_balance_pitch_gd',
        '_balance_pitch_elimit',
        '_balance_pitch_olimit',
        '_balance_pitch_neg_target',
        '_balance_pitch_pos_target',
        '_balance_roll_gp',
        '_balance_roll_gi',
        '_balance_roll_gd',
        '_balance_roll_elimit',
        '_balance_roll_olimit',
        '_balance_roll_neg_target',
        '_balance_roll_pos_target',
        '_balance_angle_pitch_gp',
        '_balance_angle_pitch_gi',
        '_balance_angle_pitch_gd',
        '_balance_angle_pitch_elimit',
        '_balance_angle_pitch_olimit',
        '_balance_angle_pitch_neg_target',
        '_balance_angle_pitch_pos_target',
        '_balance_value_4',
        '_balance_value_5',
        '_balance_pitch_flag_imu',
        '_balance_pitch_gp_imu',
        '_balance_pitch_gi_imu',
        '_balance_pitch_gd_imu',
        '_balance_pitch_neg_target_imu',
        '_balance_pitch_pos_target_imu',
        '_balance_pitch_elimit_imu',
        '_balance_pitch_olimit_imu',
        '_balance_roll_flag_imu',
        '_balance_roll_gp_imu',
        '_balance_roll_gi_imu',
        '_balance_roll_gd_imu',
        '_balance_roll_neg_target_imu',
        '_balance_roll_pos_target_imu',
        '_balance_roll_elimit_imu',
        '_balance_roll_olimit_imu',
        '_balance_pitch_flag',
        '_balance_ankle_pitch_flag',
        '_balance_roll_flag',
        '_ratio_check_flag',
        '_center2leg',
        '_link2link',
        '_init_z_up',
        '_percentage_of_ik_motor',
        '_first_pos_xr',
        '_first_neg_xr',
        '_first_pos_side_r',
        '_first_neg_side_r',
        '_first_pos_yaw_r',
        '_first_neg_yaw_r',
        '_first_pos_xl',
        '_first_neg_xl',
        '_first_pos_side_l',
        '_first_neg_side_l',
        '_first_pos_yaw_l',
        '_first_neg_yaw_l',
        '_first_pos_side_r_swing_minus',
        '_first_neg_side_r_swing_minus',
        '_first_pos_side_l_swing_minus',
        '_first_neg_side_l_swing_minus',
        '_first_min',
        '_first_max',
        '_second_pos_xr',
        '_second_neg_xr',
        '_second_pos_side_r',
        '_second_neg_side_r',
        '_second_pos_yaw_r',
        '_second_neg_yaw_r',
        '_second_pos_xl',
        '_second_neg_xl',
        '_second_pos_side_l',
        '_second_neg_side_l',
        '_second_pos_yaw_l',
        '_second_neg_yaw_l',
        '_second_pos_side_r_swing_minus',
        '_second_neg_side_r_swing_minus',
        '_second_pos_side_l_swing_minus',
        '_second_neg_side_l_swing_minus',
        '_second_min',
        '_second_max',
        '_third_pos_xr',
        '_third_neg_xr',
        '_third_pos_side_r',
        '_third_neg_side_r',
        '_third_pos_yaw_r',
        '_third_neg_yaw_r',
        '_third_pos_xl',
        '_third_neg_xl',
        '_third_pos_side_l',
        '_third_neg_side_l',
        '_third_pos_yaw_l',
        '_third_neg_yaw_l',
        '_third_pos_side_r_swing_minus',
        '_third_neg_side_r_swing_minus',
        '_third_pos_side_l_swing_minus',
        '_third_neg_side_l_swing_minus',
        '_third_min',
        '_third_max',
        '_fourth_pos_xr',
        '_fourth_neg_xr',
        '_fourth_pos_side_r',
        '_fourth_neg_side_r',
        '_fourth_pos_yaw_r',
        '_fourth_neg_yaw_r',
        '_fourth_pos_xl',
        '_fourth_neg_xl',
        '_fourth_pos_side_l',
        '_fourth_neg_side_l',
        '_fourth_pos_yaw_l',
        '_fourth_neg_yaw_l',
        '_fourth_pos_side_r_swing_minus',
        '_fourth_neg_side_r_swing_minus',
        '_fourth_pos_side_l_swing_minus',
        '_fourth_neg_side_l_swing_minus',
        '_fourth_min',
        '_fourth_max',
        '_landing_time_control_flag',
        '_control_system_flag',
    ]

    _fields_and_field_types = {
        'ik_flag': 'boolean',
        'entire_time': 'double',
        'frequency': 'double',
        'default_x_left': 'double',
        'default_y_left': 'double',
        'default_z_left': 'double',
        'default_x_right': 'double',
        'default_y_right': 'double',
        'default_z_right': 'double',
        'offset_motor': 'sequence<double>',
        'swing_shoulder_left': 'double',
        'swing_shoulder_right': 'double',
        'swing_leg_left': 'double',
        'swing_leg_right': 'double',
        'rise_leg_left': 'double',
        'rise_leg_right': 'double',
        'start_entire_time': 'double',
        'start_swing': 'double',
        'start_rise': 'double',
        'end_entire_time': 'double',
        'end_swing': 'double',
        'end_rise': 'double',
        'test_x': 'double',
        'test_side': 'double',
        'test_yaw': 'double',
        'tuning_x': 'double',
        'tuning_side': 'double',
        'tuning_yaw': 'double',
        'balance_value_0': 'double',
        'balance_value_1': 'double',
        'balance_value_2': 'double',
        'balance_value_3': 'double',
        'balance_pitch_gp': 'double',
        'balance_pitch_gi': 'double',
        'balance_pitch_gd': 'double',
        'balance_pitch_elimit': 'double',
        'balance_pitch_olimit': 'double',
        'balance_pitch_neg_target': 'double',
        'balance_pitch_pos_target': 'double',
        'balance_roll_gp': 'double',
        'balance_roll_gi': 'double',
        'balance_roll_gd': 'double',
        'balance_roll_elimit': 'double',
        'balance_roll_olimit': 'double',
        'balance_roll_neg_target': 'double',
        'balance_roll_pos_target': 'double',
        'balance_angle_pitch_gp': 'double',
        'balance_angle_pitch_gi': 'double',
        'balance_angle_pitch_gd': 'double',
        'balance_angle_pitch_elimit': 'double',
        'balance_angle_pitch_olimit': 'double',
        'balance_angle_pitch_neg_target': 'double',
        'balance_angle_pitch_pos_target': 'double',
        'balance_value_4': 'double',
        'balance_value_5': 'double',
        'balance_pitch_flag_imu': 'int64',
        'balance_pitch_gp_imu': 'double',
        'balance_pitch_gi_imu': 'double',
        'balance_pitch_gd_imu': 'double',
        'balance_pitch_neg_target_imu': 'double',
        'balance_pitch_pos_target_imu': 'double',
        'balance_pitch_elimit_imu': 'double',
        'balance_pitch_olimit_imu': 'double',
        'balance_roll_flag_imu': 'int64',
        'balance_roll_gp_imu': 'double',
        'balance_roll_gi_imu': 'double',
        'balance_roll_gd_imu': 'double',
        'balance_roll_neg_target_imu': 'double',
        'balance_roll_pos_target_imu': 'double',
        'balance_roll_elimit_imu': 'double',
        'balance_roll_olimit_imu': 'double',
        'balance_pitch_flag': 'int64',
        'balance_ankle_pitch_flag': 'int64',
        'balance_roll_flag': 'int64',
        'ratio_check_flag': 'int64',
        'center2leg': 'double',
        'link2link': 'double',
        'init_z_up': 'double',
        'percentage_of_ik_motor': 'sequence<double>',
        'first_pos_xr': 'double',
        'first_neg_xr': 'double',
        'first_pos_side_r': 'double',
        'first_neg_side_r': 'double',
        'first_pos_yaw_r': 'double',
        'first_neg_yaw_r': 'double',
        'first_pos_xl': 'double',
        'first_neg_xl': 'double',
        'first_pos_side_l': 'double',
        'first_neg_side_l': 'double',
        'first_pos_yaw_l': 'double',
        'first_neg_yaw_l': 'double',
        'first_pos_side_r_swing_minus': 'double',
        'first_neg_side_r_swing_minus': 'double',
        'first_pos_side_l_swing_minus': 'double',
        'first_neg_side_l_swing_minus': 'double',
        'first_min': 'int64',
        'first_max': 'int64',
        'second_pos_xr': 'double',
        'second_neg_xr': 'double',
        'second_pos_side_r': 'double',
        'second_neg_side_r': 'double',
        'second_pos_yaw_r': 'double',
        'second_neg_yaw_r': 'double',
        'second_pos_xl': 'double',
        'second_neg_xl': 'double',
        'second_pos_side_l': 'double',
        'second_neg_side_l': 'double',
        'second_pos_yaw_l': 'double',
        'second_neg_yaw_l': 'double',
        'second_pos_side_r_swing_minus': 'double',
        'second_neg_side_r_swing_minus': 'double',
        'second_pos_side_l_swing_minus': 'double',
        'second_neg_side_l_swing_minus': 'double',
        'second_min': 'int64',
        'second_max': 'int64',
        'third_pos_xr': 'double',
        'third_neg_xr': 'double',
        'third_pos_side_r': 'double',
        'third_neg_side_r': 'double',
        'third_pos_yaw_r': 'double',
        'third_neg_yaw_r': 'double',
        'third_pos_xl': 'double',
        'third_neg_xl': 'double',
        'third_pos_side_l': 'double',
        'third_neg_side_l': 'double',
        'third_pos_yaw_l': 'double',
        'third_neg_yaw_l': 'double',
        'third_pos_side_r_swing_minus': 'double',
        'third_neg_side_r_swing_minus': 'double',
        'third_pos_side_l_swing_minus': 'double',
        'third_neg_side_l_swing_minus': 'double',
        'third_min': 'int64',
        'third_max': 'int64',
        'fourth_pos_xr': 'double',
        'fourth_neg_xr': 'double',
        'fourth_pos_side_r': 'double',
        'fourth_neg_side_r': 'double',
        'fourth_pos_yaw_r': 'double',
        'fourth_neg_yaw_r': 'double',
        'fourth_pos_xl': 'double',
        'fourth_neg_xl': 'double',
        'fourth_pos_side_l': 'double',
        'fourth_neg_side_l': 'double',
        'fourth_pos_yaw_l': 'double',
        'fourth_neg_yaw_l': 'double',
        'fourth_pos_side_r_swing_minus': 'double',
        'fourth_neg_side_r_swing_minus': 'double',
        'fourth_pos_side_l_swing_minus': 'double',
        'fourth_neg_side_l_swing_minus': 'double',
        'fourth_min': 'int64',
        'fourth_max': 'int64',
        'landing_time_control_flag': 'int64',
        'control_system_flag': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
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
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
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
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
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
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
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
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
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
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ik_flag = kwargs.get('ik_flag', bool())
        self.entire_time = kwargs.get('entire_time', float())
        self.frequency = kwargs.get('frequency', float())
        self.default_x_left = kwargs.get('default_x_left', float())
        self.default_y_left = kwargs.get('default_y_left', float())
        self.default_z_left = kwargs.get('default_z_left', float())
        self.default_x_right = kwargs.get('default_x_right', float())
        self.default_y_right = kwargs.get('default_y_right', float())
        self.default_z_right = kwargs.get('default_z_right', float())
        self.offset_motor = array.array('d', kwargs.get('offset_motor', []))
        self.swing_shoulder_left = kwargs.get('swing_shoulder_left', float())
        self.swing_shoulder_right = kwargs.get('swing_shoulder_right', float())
        self.swing_leg_left = kwargs.get('swing_leg_left', float())
        self.swing_leg_right = kwargs.get('swing_leg_right', float())
        self.rise_leg_left = kwargs.get('rise_leg_left', float())
        self.rise_leg_right = kwargs.get('rise_leg_right', float())
        self.start_entire_time = kwargs.get('start_entire_time', float())
        self.start_swing = kwargs.get('start_swing', float())
        self.start_rise = kwargs.get('start_rise', float())
        self.end_entire_time = kwargs.get('end_entire_time', float())
        self.end_swing = kwargs.get('end_swing', float())
        self.end_rise = kwargs.get('end_rise', float())
        self.test_x = kwargs.get('test_x', float())
        self.test_side = kwargs.get('test_side', float())
        self.test_yaw = kwargs.get('test_yaw', float())
        self.tuning_x = kwargs.get('tuning_x', float())
        self.tuning_side = kwargs.get('tuning_side', float())
        self.tuning_yaw = kwargs.get('tuning_yaw', float())
        self.balance_value_0 = kwargs.get('balance_value_0', float())
        self.balance_value_1 = kwargs.get('balance_value_1', float())
        self.balance_value_2 = kwargs.get('balance_value_2', float())
        self.balance_value_3 = kwargs.get('balance_value_3', float())
        self.balance_pitch_gp = kwargs.get('balance_pitch_gp', float())
        self.balance_pitch_gi = kwargs.get('balance_pitch_gi', float())
        self.balance_pitch_gd = kwargs.get('balance_pitch_gd', float())
        self.balance_pitch_elimit = kwargs.get('balance_pitch_elimit', float())
        self.balance_pitch_olimit = kwargs.get('balance_pitch_olimit', float())
        self.balance_pitch_neg_target = kwargs.get('balance_pitch_neg_target', float())
        self.balance_pitch_pos_target = kwargs.get('balance_pitch_pos_target', float())
        self.balance_roll_gp = kwargs.get('balance_roll_gp', float())
        self.balance_roll_gi = kwargs.get('balance_roll_gi', float())
        self.balance_roll_gd = kwargs.get('balance_roll_gd', float())
        self.balance_roll_elimit = kwargs.get('balance_roll_elimit', float())
        self.balance_roll_olimit = kwargs.get('balance_roll_olimit', float())
        self.balance_roll_neg_target = kwargs.get('balance_roll_neg_target', float())
        self.balance_roll_pos_target = kwargs.get('balance_roll_pos_target', float())
        self.balance_angle_pitch_gp = kwargs.get('balance_angle_pitch_gp', float())
        self.balance_angle_pitch_gi = kwargs.get('balance_angle_pitch_gi', float())
        self.balance_angle_pitch_gd = kwargs.get('balance_angle_pitch_gd', float())
        self.balance_angle_pitch_elimit = kwargs.get('balance_angle_pitch_elimit', float())
        self.balance_angle_pitch_olimit = kwargs.get('balance_angle_pitch_olimit', float())
        self.balance_angle_pitch_neg_target = kwargs.get('balance_angle_pitch_neg_target', float())
        self.balance_angle_pitch_pos_target = kwargs.get('balance_angle_pitch_pos_target', float())
        self.balance_value_4 = kwargs.get('balance_value_4', float())
        self.balance_value_5 = kwargs.get('balance_value_5', float())
        self.balance_pitch_flag_imu = kwargs.get('balance_pitch_flag_imu', int())
        self.balance_pitch_gp_imu = kwargs.get('balance_pitch_gp_imu', float())
        self.balance_pitch_gi_imu = kwargs.get('balance_pitch_gi_imu', float())
        self.balance_pitch_gd_imu = kwargs.get('balance_pitch_gd_imu', float())
        self.balance_pitch_neg_target_imu = kwargs.get('balance_pitch_neg_target_imu', float())
        self.balance_pitch_pos_target_imu = kwargs.get('balance_pitch_pos_target_imu', float())
        self.balance_pitch_elimit_imu = kwargs.get('balance_pitch_elimit_imu', float())
        self.balance_pitch_olimit_imu = kwargs.get('balance_pitch_olimit_imu', float())
        self.balance_roll_flag_imu = kwargs.get('balance_roll_flag_imu', int())
        self.balance_roll_gp_imu = kwargs.get('balance_roll_gp_imu', float())
        self.balance_roll_gi_imu = kwargs.get('balance_roll_gi_imu', float())
        self.balance_roll_gd_imu = kwargs.get('balance_roll_gd_imu', float())
        self.balance_roll_neg_target_imu = kwargs.get('balance_roll_neg_target_imu', float())
        self.balance_roll_pos_target_imu = kwargs.get('balance_roll_pos_target_imu', float())
        self.balance_roll_elimit_imu = kwargs.get('balance_roll_elimit_imu', float())
        self.balance_roll_olimit_imu = kwargs.get('balance_roll_olimit_imu', float())
        self.balance_pitch_flag = kwargs.get('balance_pitch_flag', int())
        self.balance_ankle_pitch_flag = kwargs.get('balance_ankle_pitch_flag', int())
        self.balance_roll_flag = kwargs.get('balance_roll_flag', int())
        self.ratio_check_flag = kwargs.get('ratio_check_flag', int())
        self.center2leg = kwargs.get('center2leg', float())
        self.link2link = kwargs.get('link2link', float())
        self.init_z_up = kwargs.get('init_z_up', float())
        self.percentage_of_ik_motor = array.array('d', kwargs.get('percentage_of_ik_motor', []))
        self.first_pos_xr = kwargs.get('first_pos_xr', float())
        self.first_neg_xr = kwargs.get('first_neg_xr', float())
        self.first_pos_side_r = kwargs.get('first_pos_side_r', float())
        self.first_neg_side_r = kwargs.get('first_neg_side_r', float())
        self.first_pos_yaw_r = kwargs.get('first_pos_yaw_r', float())
        self.first_neg_yaw_r = kwargs.get('first_neg_yaw_r', float())
        self.first_pos_xl = kwargs.get('first_pos_xl', float())
        self.first_neg_xl = kwargs.get('first_neg_xl', float())
        self.first_pos_side_l = kwargs.get('first_pos_side_l', float())
        self.first_neg_side_l = kwargs.get('first_neg_side_l', float())
        self.first_pos_yaw_l = kwargs.get('first_pos_yaw_l', float())
        self.first_neg_yaw_l = kwargs.get('first_neg_yaw_l', float())
        self.first_pos_side_r_swing_minus = kwargs.get('first_pos_side_r_swing_minus', float())
        self.first_neg_side_r_swing_minus = kwargs.get('first_neg_side_r_swing_minus', float())
        self.first_pos_side_l_swing_minus = kwargs.get('first_pos_side_l_swing_minus', float())
        self.first_neg_side_l_swing_minus = kwargs.get('first_neg_side_l_swing_minus', float())
        self.first_min = kwargs.get('first_min', int())
        self.first_max = kwargs.get('first_max', int())
        self.second_pos_xr = kwargs.get('second_pos_xr', float())
        self.second_neg_xr = kwargs.get('second_neg_xr', float())
        self.second_pos_side_r = kwargs.get('second_pos_side_r', float())
        self.second_neg_side_r = kwargs.get('second_neg_side_r', float())
        self.second_pos_yaw_r = kwargs.get('second_pos_yaw_r', float())
        self.second_neg_yaw_r = kwargs.get('second_neg_yaw_r', float())
        self.second_pos_xl = kwargs.get('second_pos_xl', float())
        self.second_neg_xl = kwargs.get('second_neg_xl', float())
        self.second_pos_side_l = kwargs.get('second_pos_side_l', float())
        self.second_neg_side_l = kwargs.get('second_neg_side_l', float())
        self.second_pos_yaw_l = kwargs.get('second_pos_yaw_l', float())
        self.second_neg_yaw_l = kwargs.get('second_neg_yaw_l', float())
        self.second_pos_side_r_swing_minus = kwargs.get('second_pos_side_r_swing_minus', float())
        self.second_neg_side_r_swing_minus = kwargs.get('second_neg_side_r_swing_minus', float())
        self.second_pos_side_l_swing_minus = kwargs.get('second_pos_side_l_swing_minus', float())
        self.second_neg_side_l_swing_minus = kwargs.get('second_neg_side_l_swing_minus', float())
        self.second_min = kwargs.get('second_min', int())
        self.second_max = kwargs.get('second_max', int())
        self.third_pos_xr = kwargs.get('third_pos_xr', float())
        self.third_neg_xr = kwargs.get('third_neg_xr', float())
        self.third_pos_side_r = kwargs.get('third_pos_side_r', float())
        self.third_neg_side_r = kwargs.get('third_neg_side_r', float())
        self.third_pos_yaw_r = kwargs.get('third_pos_yaw_r', float())
        self.third_neg_yaw_r = kwargs.get('third_neg_yaw_r', float())
        self.third_pos_xl = kwargs.get('third_pos_xl', float())
        self.third_neg_xl = kwargs.get('third_neg_xl', float())
        self.third_pos_side_l = kwargs.get('third_pos_side_l', float())
        self.third_neg_side_l = kwargs.get('third_neg_side_l', float())
        self.third_pos_yaw_l = kwargs.get('third_pos_yaw_l', float())
        self.third_neg_yaw_l = kwargs.get('third_neg_yaw_l', float())
        self.third_pos_side_r_swing_minus = kwargs.get('third_pos_side_r_swing_minus', float())
        self.third_neg_side_r_swing_minus = kwargs.get('third_neg_side_r_swing_minus', float())
        self.third_pos_side_l_swing_minus = kwargs.get('third_pos_side_l_swing_minus', float())
        self.third_neg_side_l_swing_minus = kwargs.get('third_neg_side_l_swing_minus', float())
        self.third_min = kwargs.get('third_min', int())
        self.third_max = kwargs.get('third_max', int())
        self.fourth_pos_xr = kwargs.get('fourth_pos_xr', float())
        self.fourth_neg_xr = kwargs.get('fourth_neg_xr', float())
        self.fourth_pos_side_r = kwargs.get('fourth_pos_side_r', float())
        self.fourth_neg_side_r = kwargs.get('fourth_neg_side_r', float())
        self.fourth_pos_yaw_r = kwargs.get('fourth_pos_yaw_r', float())
        self.fourth_neg_yaw_r = kwargs.get('fourth_neg_yaw_r', float())
        self.fourth_pos_xl = kwargs.get('fourth_pos_xl', float())
        self.fourth_neg_xl = kwargs.get('fourth_neg_xl', float())
        self.fourth_pos_side_l = kwargs.get('fourth_pos_side_l', float())
        self.fourth_neg_side_l = kwargs.get('fourth_neg_side_l', float())
        self.fourth_pos_yaw_l = kwargs.get('fourth_pos_yaw_l', float())
        self.fourth_neg_yaw_l = kwargs.get('fourth_neg_yaw_l', float())
        self.fourth_pos_side_r_swing_minus = kwargs.get('fourth_pos_side_r_swing_minus', float())
        self.fourth_neg_side_r_swing_minus = kwargs.get('fourth_neg_side_r_swing_minus', float())
        self.fourth_pos_side_l_swing_minus = kwargs.get('fourth_pos_side_l_swing_minus', float())
        self.fourth_neg_side_l_swing_minus = kwargs.get('fourth_neg_side_l_swing_minus', float())
        self.fourth_min = kwargs.get('fourth_min', int())
        self.fourth_max = kwargs.get('fourth_max', int())
        self.landing_time_control_flag = kwargs.get('landing_time_control_flag', int())
        self.control_system_flag = kwargs.get('control_system_flag', int())

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
        if self.ik_flag != other.ik_flag:
            return False
        if self.entire_time != other.entire_time:
            return False
        if self.frequency != other.frequency:
            return False
        if self.default_x_left != other.default_x_left:
            return False
        if self.default_y_left != other.default_y_left:
            return False
        if self.default_z_left != other.default_z_left:
            return False
        if self.default_x_right != other.default_x_right:
            return False
        if self.default_y_right != other.default_y_right:
            return False
        if self.default_z_right != other.default_z_right:
            return False
        if self.offset_motor != other.offset_motor:
            return False
        if self.swing_shoulder_left != other.swing_shoulder_left:
            return False
        if self.swing_shoulder_right != other.swing_shoulder_right:
            return False
        if self.swing_leg_left != other.swing_leg_left:
            return False
        if self.swing_leg_right != other.swing_leg_right:
            return False
        if self.rise_leg_left != other.rise_leg_left:
            return False
        if self.rise_leg_right != other.rise_leg_right:
            return False
        if self.start_entire_time != other.start_entire_time:
            return False
        if self.start_swing != other.start_swing:
            return False
        if self.start_rise != other.start_rise:
            return False
        if self.end_entire_time != other.end_entire_time:
            return False
        if self.end_swing != other.end_swing:
            return False
        if self.end_rise != other.end_rise:
            return False
        if self.test_x != other.test_x:
            return False
        if self.test_side != other.test_side:
            return False
        if self.test_yaw != other.test_yaw:
            return False
        if self.tuning_x != other.tuning_x:
            return False
        if self.tuning_side != other.tuning_side:
            return False
        if self.tuning_yaw != other.tuning_yaw:
            return False
        if self.balance_value_0 != other.balance_value_0:
            return False
        if self.balance_value_1 != other.balance_value_1:
            return False
        if self.balance_value_2 != other.balance_value_2:
            return False
        if self.balance_value_3 != other.balance_value_3:
            return False
        if self.balance_pitch_gp != other.balance_pitch_gp:
            return False
        if self.balance_pitch_gi != other.balance_pitch_gi:
            return False
        if self.balance_pitch_gd != other.balance_pitch_gd:
            return False
        if self.balance_pitch_elimit != other.balance_pitch_elimit:
            return False
        if self.balance_pitch_olimit != other.balance_pitch_olimit:
            return False
        if self.balance_pitch_neg_target != other.balance_pitch_neg_target:
            return False
        if self.balance_pitch_pos_target != other.balance_pitch_pos_target:
            return False
        if self.balance_roll_gp != other.balance_roll_gp:
            return False
        if self.balance_roll_gi != other.balance_roll_gi:
            return False
        if self.balance_roll_gd != other.balance_roll_gd:
            return False
        if self.balance_roll_elimit != other.balance_roll_elimit:
            return False
        if self.balance_roll_olimit != other.balance_roll_olimit:
            return False
        if self.balance_roll_neg_target != other.balance_roll_neg_target:
            return False
        if self.balance_roll_pos_target != other.balance_roll_pos_target:
            return False
        if self.balance_angle_pitch_gp != other.balance_angle_pitch_gp:
            return False
        if self.balance_angle_pitch_gi != other.balance_angle_pitch_gi:
            return False
        if self.balance_angle_pitch_gd != other.balance_angle_pitch_gd:
            return False
        if self.balance_angle_pitch_elimit != other.balance_angle_pitch_elimit:
            return False
        if self.balance_angle_pitch_olimit != other.balance_angle_pitch_olimit:
            return False
        if self.balance_angle_pitch_neg_target != other.balance_angle_pitch_neg_target:
            return False
        if self.balance_angle_pitch_pos_target != other.balance_angle_pitch_pos_target:
            return False
        if self.balance_value_4 != other.balance_value_4:
            return False
        if self.balance_value_5 != other.balance_value_5:
            return False
        if self.balance_pitch_flag_imu != other.balance_pitch_flag_imu:
            return False
        if self.balance_pitch_gp_imu != other.balance_pitch_gp_imu:
            return False
        if self.balance_pitch_gi_imu != other.balance_pitch_gi_imu:
            return False
        if self.balance_pitch_gd_imu != other.balance_pitch_gd_imu:
            return False
        if self.balance_pitch_neg_target_imu != other.balance_pitch_neg_target_imu:
            return False
        if self.balance_pitch_pos_target_imu != other.balance_pitch_pos_target_imu:
            return False
        if self.balance_pitch_elimit_imu != other.balance_pitch_elimit_imu:
            return False
        if self.balance_pitch_olimit_imu != other.balance_pitch_olimit_imu:
            return False
        if self.balance_roll_flag_imu != other.balance_roll_flag_imu:
            return False
        if self.balance_roll_gp_imu != other.balance_roll_gp_imu:
            return False
        if self.balance_roll_gi_imu != other.balance_roll_gi_imu:
            return False
        if self.balance_roll_gd_imu != other.balance_roll_gd_imu:
            return False
        if self.balance_roll_neg_target_imu != other.balance_roll_neg_target_imu:
            return False
        if self.balance_roll_pos_target_imu != other.balance_roll_pos_target_imu:
            return False
        if self.balance_roll_elimit_imu != other.balance_roll_elimit_imu:
            return False
        if self.balance_roll_olimit_imu != other.balance_roll_olimit_imu:
            return False
        if self.balance_pitch_flag != other.balance_pitch_flag:
            return False
        if self.balance_ankle_pitch_flag != other.balance_ankle_pitch_flag:
            return False
        if self.balance_roll_flag != other.balance_roll_flag:
            return False
        if self.ratio_check_flag != other.ratio_check_flag:
            return False
        if self.center2leg != other.center2leg:
            return False
        if self.link2link != other.link2link:
            return False
        if self.init_z_up != other.init_z_up:
            return False
        if self.percentage_of_ik_motor != other.percentage_of_ik_motor:
            return False
        if self.first_pos_xr != other.first_pos_xr:
            return False
        if self.first_neg_xr != other.first_neg_xr:
            return False
        if self.first_pos_side_r != other.first_pos_side_r:
            return False
        if self.first_neg_side_r != other.first_neg_side_r:
            return False
        if self.first_pos_yaw_r != other.first_pos_yaw_r:
            return False
        if self.first_neg_yaw_r != other.first_neg_yaw_r:
            return False
        if self.first_pos_xl != other.first_pos_xl:
            return False
        if self.first_neg_xl != other.first_neg_xl:
            return False
        if self.first_pos_side_l != other.first_pos_side_l:
            return False
        if self.first_neg_side_l != other.first_neg_side_l:
            return False
        if self.first_pos_yaw_l != other.first_pos_yaw_l:
            return False
        if self.first_neg_yaw_l != other.first_neg_yaw_l:
            return False
        if self.first_pos_side_r_swing_minus != other.first_pos_side_r_swing_minus:
            return False
        if self.first_neg_side_r_swing_minus != other.first_neg_side_r_swing_minus:
            return False
        if self.first_pos_side_l_swing_minus != other.first_pos_side_l_swing_minus:
            return False
        if self.first_neg_side_l_swing_minus != other.first_neg_side_l_swing_minus:
            return False
        if self.first_min != other.first_min:
            return False
        if self.first_max != other.first_max:
            return False
        if self.second_pos_xr != other.second_pos_xr:
            return False
        if self.second_neg_xr != other.second_neg_xr:
            return False
        if self.second_pos_side_r != other.second_pos_side_r:
            return False
        if self.second_neg_side_r != other.second_neg_side_r:
            return False
        if self.second_pos_yaw_r != other.second_pos_yaw_r:
            return False
        if self.second_neg_yaw_r != other.second_neg_yaw_r:
            return False
        if self.second_pos_xl != other.second_pos_xl:
            return False
        if self.second_neg_xl != other.second_neg_xl:
            return False
        if self.second_pos_side_l != other.second_pos_side_l:
            return False
        if self.second_neg_side_l != other.second_neg_side_l:
            return False
        if self.second_pos_yaw_l != other.second_pos_yaw_l:
            return False
        if self.second_neg_yaw_l != other.second_neg_yaw_l:
            return False
        if self.second_pos_side_r_swing_minus != other.second_pos_side_r_swing_minus:
            return False
        if self.second_neg_side_r_swing_minus != other.second_neg_side_r_swing_minus:
            return False
        if self.second_pos_side_l_swing_minus != other.second_pos_side_l_swing_minus:
            return False
        if self.second_neg_side_l_swing_minus != other.second_neg_side_l_swing_minus:
            return False
        if self.second_min != other.second_min:
            return False
        if self.second_max != other.second_max:
            return False
        if self.third_pos_xr != other.third_pos_xr:
            return False
        if self.third_neg_xr != other.third_neg_xr:
            return False
        if self.third_pos_side_r != other.third_pos_side_r:
            return False
        if self.third_neg_side_r != other.third_neg_side_r:
            return False
        if self.third_pos_yaw_r != other.third_pos_yaw_r:
            return False
        if self.third_neg_yaw_r != other.third_neg_yaw_r:
            return False
        if self.third_pos_xl != other.third_pos_xl:
            return False
        if self.third_neg_xl != other.third_neg_xl:
            return False
        if self.third_pos_side_l != other.third_pos_side_l:
            return False
        if self.third_neg_side_l != other.third_neg_side_l:
            return False
        if self.third_pos_yaw_l != other.third_pos_yaw_l:
            return False
        if self.third_neg_yaw_l != other.third_neg_yaw_l:
            return False
        if self.third_pos_side_r_swing_minus != other.third_pos_side_r_swing_minus:
            return False
        if self.third_neg_side_r_swing_minus != other.third_neg_side_r_swing_minus:
            return False
        if self.third_pos_side_l_swing_minus != other.third_pos_side_l_swing_minus:
            return False
        if self.third_neg_side_l_swing_minus != other.third_neg_side_l_swing_minus:
            return False
        if self.third_min != other.third_min:
            return False
        if self.third_max != other.third_max:
            return False
        if self.fourth_pos_xr != other.fourth_pos_xr:
            return False
        if self.fourth_neg_xr != other.fourth_neg_xr:
            return False
        if self.fourth_pos_side_r != other.fourth_pos_side_r:
            return False
        if self.fourth_neg_side_r != other.fourth_neg_side_r:
            return False
        if self.fourth_pos_yaw_r != other.fourth_pos_yaw_r:
            return False
        if self.fourth_neg_yaw_r != other.fourth_neg_yaw_r:
            return False
        if self.fourth_pos_xl != other.fourth_pos_xl:
            return False
        if self.fourth_neg_xl != other.fourth_neg_xl:
            return False
        if self.fourth_pos_side_l != other.fourth_pos_side_l:
            return False
        if self.fourth_neg_side_l != other.fourth_neg_side_l:
            return False
        if self.fourth_pos_yaw_l != other.fourth_pos_yaw_l:
            return False
        if self.fourth_neg_yaw_l != other.fourth_neg_yaw_l:
            return False
        if self.fourth_pos_side_r_swing_minus != other.fourth_pos_side_r_swing_minus:
            return False
        if self.fourth_neg_side_r_swing_minus != other.fourth_neg_side_r_swing_minus:
            return False
        if self.fourth_pos_side_l_swing_minus != other.fourth_pos_side_l_swing_minus:
            return False
        if self.fourth_neg_side_l_swing_minus != other.fourth_neg_side_l_swing_minus:
            return False
        if self.fourth_min != other.fourth_min:
            return False
        if self.fourth_max != other.fourth_max:
            return False
        if self.landing_time_control_flag != other.landing_time_control_flag:
            return False
        if self.control_system_flag != other.control_system_flag:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ik_flag(self):
        """Message field 'ik_flag'."""
        return self._ik_flag

    @ik_flag.setter
    def ik_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'ik_flag' field must be of type 'bool'"
        self._ik_flag = value

    @builtins.property
    def entire_time(self):
        """Message field 'entire_time'."""
        return self._entire_time

    @entire_time.setter
    def entire_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'entire_time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'entire_time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._entire_time = value

    @builtins.property
    def frequency(self):
        """Message field 'frequency'."""
        return self._frequency

    @frequency.setter
    def frequency(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'frequency' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'frequency' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._frequency = value

    @builtins.property
    def default_x_left(self):
        """Message field 'default_x_left'."""
        return self._default_x_left

    @default_x_left.setter
    def default_x_left(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'default_x_left' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'default_x_left' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._default_x_left = value

    @builtins.property
    def default_y_left(self):
        """Message field 'default_y_left'."""
        return self._default_y_left

    @default_y_left.setter
    def default_y_left(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'default_y_left' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'default_y_left' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._default_y_left = value

    @builtins.property
    def default_z_left(self):
        """Message field 'default_z_left'."""
        return self._default_z_left

    @default_z_left.setter
    def default_z_left(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'default_z_left' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'default_z_left' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._default_z_left = value

    @builtins.property
    def default_x_right(self):
        """Message field 'default_x_right'."""
        return self._default_x_right

    @default_x_right.setter
    def default_x_right(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'default_x_right' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'default_x_right' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._default_x_right = value

    @builtins.property
    def default_y_right(self):
        """Message field 'default_y_right'."""
        return self._default_y_right

    @default_y_right.setter
    def default_y_right(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'default_y_right' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'default_y_right' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._default_y_right = value

    @builtins.property
    def default_z_right(self):
        """Message field 'default_z_right'."""
        return self._default_z_right

    @default_z_right.setter
    def default_z_right(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'default_z_right' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'default_z_right' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._default_z_right = value

    @builtins.property
    def offset_motor(self):
        """Message field 'offset_motor'."""
        return self._offset_motor

    @offset_motor.setter
    def offset_motor(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'offset_motor' array.array() must have the type code of 'd'"
            self._offset_motor = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'offset_motor' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._offset_motor = array.array('d', value)

    @builtins.property
    def swing_shoulder_left(self):
        """Message field 'swing_shoulder_left'."""
        return self._swing_shoulder_left

    @swing_shoulder_left.setter
    def swing_shoulder_left(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'swing_shoulder_left' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'swing_shoulder_left' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._swing_shoulder_left = value

    @builtins.property
    def swing_shoulder_right(self):
        """Message field 'swing_shoulder_right'."""
        return self._swing_shoulder_right

    @swing_shoulder_right.setter
    def swing_shoulder_right(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'swing_shoulder_right' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'swing_shoulder_right' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._swing_shoulder_right = value

    @builtins.property
    def swing_leg_left(self):
        """Message field 'swing_leg_left'."""
        return self._swing_leg_left

    @swing_leg_left.setter
    def swing_leg_left(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'swing_leg_left' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'swing_leg_left' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._swing_leg_left = value

    @builtins.property
    def swing_leg_right(self):
        """Message field 'swing_leg_right'."""
        return self._swing_leg_right

    @swing_leg_right.setter
    def swing_leg_right(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'swing_leg_right' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'swing_leg_right' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._swing_leg_right = value

    @builtins.property
    def rise_leg_left(self):
        """Message field 'rise_leg_left'."""
        return self._rise_leg_left

    @rise_leg_left.setter
    def rise_leg_left(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rise_leg_left' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rise_leg_left' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rise_leg_left = value

    @builtins.property
    def rise_leg_right(self):
        """Message field 'rise_leg_right'."""
        return self._rise_leg_right

    @rise_leg_right.setter
    def rise_leg_right(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rise_leg_right' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rise_leg_right' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rise_leg_right = value

    @builtins.property
    def start_entire_time(self):
        """Message field 'start_entire_time'."""
        return self._start_entire_time

    @start_entire_time.setter
    def start_entire_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'start_entire_time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'start_entire_time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._start_entire_time = value

    @builtins.property
    def start_swing(self):
        """Message field 'start_swing'."""
        return self._start_swing

    @start_swing.setter
    def start_swing(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'start_swing' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'start_swing' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._start_swing = value

    @builtins.property
    def start_rise(self):
        """Message field 'start_rise'."""
        return self._start_rise

    @start_rise.setter
    def start_rise(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'start_rise' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'start_rise' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._start_rise = value

    @builtins.property
    def end_entire_time(self):
        """Message field 'end_entire_time'."""
        return self._end_entire_time

    @end_entire_time.setter
    def end_entire_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'end_entire_time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'end_entire_time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._end_entire_time = value

    @builtins.property
    def end_swing(self):
        """Message field 'end_swing'."""
        return self._end_swing

    @end_swing.setter
    def end_swing(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'end_swing' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'end_swing' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._end_swing = value

    @builtins.property
    def end_rise(self):
        """Message field 'end_rise'."""
        return self._end_rise

    @end_rise.setter
    def end_rise(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'end_rise' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'end_rise' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._end_rise = value

    @builtins.property
    def test_x(self):
        """Message field 'test_x'."""
        return self._test_x

    @test_x.setter
    def test_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'test_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'test_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._test_x = value

    @builtins.property
    def test_side(self):
        """Message field 'test_side'."""
        return self._test_side

    @test_side.setter
    def test_side(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'test_side' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'test_side' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._test_side = value

    @builtins.property
    def test_yaw(self):
        """Message field 'test_yaw'."""
        return self._test_yaw

    @test_yaw.setter
    def test_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'test_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'test_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._test_yaw = value

    @builtins.property
    def tuning_x(self):
        """Message field 'tuning_x'."""
        return self._tuning_x

    @tuning_x.setter
    def tuning_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tuning_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tuning_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tuning_x = value

    @builtins.property
    def tuning_side(self):
        """Message field 'tuning_side'."""
        return self._tuning_side

    @tuning_side.setter
    def tuning_side(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tuning_side' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tuning_side' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tuning_side = value

    @builtins.property
    def tuning_yaw(self):
        """Message field 'tuning_yaw'."""
        return self._tuning_yaw

    @tuning_yaw.setter
    def tuning_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tuning_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tuning_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tuning_yaw = value

    @builtins.property
    def balance_value_0(self):
        """Message field 'balance_value_0'."""
        return self._balance_value_0

    @balance_value_0.setter
    def balance_value_0(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_value_0' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_value_0' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_value_0 = value

    @builtins.property
    def balance_value_1(self):
        """Message field 'balance_value_1'."""
        return self._balance_value_1

    @balance_value_1.setter
    def balance_value_1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_value_1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_value_1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_value_1 = value

    @builtins.property
    def balance_value_2(self):
        """Message field 'balance_value_2'."""
        return self._balance_value_2

    @balance_value_2.setter
    def balance_value_2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_value_2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_value_2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_value_2 = value

    @builtins.property
    def balance_value_3(self):
        """Message field 'balance_value_3'."""
        return self._balance_value_3

    @balance_value_3.setter
    def balance_value_3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_value_3' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_value_3' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_value_3 = value

    @builtins.property
    def balance_pitch_gp(self):
        """Message field 'balance_pitch_gp'."""
        return self._balance_pitch_gp

    @balance_pitch_gp.setter
    def balance_pitch_gp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_pitch_gp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_pitch_gp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_pitch_gp = value

    @builtins.property
    def balance_pitch_gi(self):
        """Message field 'balance_pitch_gi'."""
        return self._balance_pitch_gi

    @balance_pitch_gi.setter
    def balance_pitch_gi(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_pitch_gi' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_pitch_gi' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_pitch_gi = value

    @builtins.property
    def balance_pitch_gd(self):
        """Message field 'balance_pitch_gd'."""
        return self._balance_pitch_gd

    @balance_pitch_gd.setter
    def balance_pitch_gd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_pitch_gd' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_pitch_gd' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_pitch_gd = value

    @builtins.property
    def balance_pitch_elimit(self):
        """Message field 'balance_pitch_elimit'."""
        return self._balance_pitch_elimit

    @balance_pitch_elimit.setter
    def balance_pitch_elimit(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_pitch_elimit' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_pitch_elimit' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_pitch_elimit = value

    @builtins.property
    def balance_pitch_olimit(self):
        """Message field 'balance_pitch_olimit'."""
        return self._balance_pitch_olimit

    @balance_pitch_olimit.setter
    def balance_pitch_olimit(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_pitch_olimit' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_pitch_olimit' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_pitch_olimit = value

    @builtins.property
    def balance_pitch_neg_target(self):
        """Message field 'balance_pitch_neg_target'."""
        return self._balance_pitch_neg_target

    @balance_pitch_neg_target.setter
    def balance_pitch_neg_target(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_pitch_neg_target' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_pitch_neg_target' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_pitch_neg_target = value

    @builtins.property
    def balance_pitch_pos_target(self):
        """Message field 'balance_pitch_pos_target'."""
        return self._balance_pitch_pos_target

    @balance_pitch_pos_target.setter
    def balance_pitch_pos_target(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_pitch_pos_target' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_pitch_pos_target' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_pitch_pos_target = value

    @builtins.property
    def balance_roll_gp(self):
        """Message field 'balance_roll_gp'."""
        return self._balance_roll_gp

    @balance_roll_gp.setter
    def balance_roll_gp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_roll_gp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_roll_gp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_roll_gp = value

    @builtins.property
    def balance_roll_gi(self):
        """Message field 'balance_roll_gi'."""
        return self._balance_roll_gi

    @balance_roll_gi.setter
    def balance_roll_gi(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_roll_gi' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_roll_gi' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_roll_gi = value

    @builtins.property
    def balance_roll_gd(self):
        """Message field 'balance_roll_gd'."""
        return self._balance_roll_gd

    @balance_roll_gd.setter
    def balance_roll_gd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_roll_gd' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_roll_gd' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_roll_gd = value

    @builtins.property
    def balance_roll_elimit(self):
        """Message field 'balance_roll_elimit'."""
        return self._balance_roll_elimit

    @balance_roll_elimit.setter
    def balance_roll_elimit(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_roll_elimit' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_roll_elimit' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_roll_elimit = value

    @builtins.property
    def balance_roll_olimit(self):
        """Message field 'balance_roll_olimit'."""
        return self._balance_roll_olimit

    @balance_roll_olimit.setter
    def balance_roll_olimit(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_roll_olimit' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_roll_olimit' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_roll_olimit = value

    @builtins.property
    def balance_roll_neg_target(self):
        """Message field 'balance_roll_neg_target'."""
        return self._balance_roll_neg_target

    @balance_roll_neg_target.setter
    def balance_roll_neg_target(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_roll_neg_target' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_roll_neg_target' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_roll_neg_target = value

    @builtins.property
    def balance_roll_pos_target(self):
        """Message field 'balance_roll_pos_target'."""
        return self._balance_roll_pos_target

    @balance_roll_pos_target.setter
    def balance_roll_pos_target(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_roll_pos_target' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_roll_pos_target' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_roll_pos_target = value

    @builtins.property
    def balance_angle_pitch_gp(self):
        """Message field 'balance_angle_pitch_gp'."""
        return self._balance_angle_pitch_gp

    @balance_angle_pitch_gp.setter
    def balance_angle_pitch_gp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_angle_pitch_gp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_angle_pitch_gp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_angle_pitch_gp = value

    @builtins.property
    def balance_angle_pitch_gi(self):
        """Message field 'balance_angle_pitch_gi'."""
        return self._balance_angle_pitch_gi

    @balance_angle_pitch_gi.setter
    def balance_angle_pitch_gi(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_angle_pitch_gi' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_angle_pitch_gi' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_angle_pitch_gi = value

    @builtins.property
    def balance_angle_pitch_gd(self):
        """Message field 'balance_angle_pitch_gd'."""
        return self._balance_angle_pitch_gd

    @balance_angle_pitch_gd.setter
    def balance_angle_pitch_gd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_angle_pitch_gd' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_angle_pitch_gd' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_angle_pitch_gd = value

    @builtins.property
    def balance_angle_pitch_elimit(self):
        """Message field 'balance_angle_pitch_elimit'."""
        return self._balance_angle_pitch_elimit

    @balance_angle_pitch_elimit.setter
    def balance_angle_pitch_elimit(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_angle_pitch_elimit' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_angle_pitch_elimit' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_angle_pitch_elimit = value

    @builtins.property
    def balance_angle_pitch_olimit(self):
        """Message field 'balance_angle_pitch_olimit'."""
        return self._balance_angle_pitch_olimit

    @balance_angle_pitch_olimit.setter
    def balance_angle_pitch_olimit(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_angle_pitch_olimit' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_angle_pitch_olimit' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_angle_pitch_olimit = value

    @builtins.property
    def balance_angle_pitch_neg_target(self):
        """Message field 'balance_angle_pitch_neg_target'."""
        return self._balance_angle_pitch_neg_target

    @balance_angle_pitch_neg_target.setter
    def balance_angle_pitch_neg_target(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_angle_pitch_neg_target' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_angle_pitch_neg_target' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_angle_pitch_neg_target = value

    @builtins.property
    def balance_angle_pitch_pos_target(self):
        """Message field 'balance_angle_pitch_pos_target'."""
        return self._balance_angle_pitch_pos_target

    @balance_angle_pitch_pos_target.setter
    def balance_angle_pitch_pos_target(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_angle_pitch_pos_target' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_angle_pitch_pos_target' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_angle_pitch_pos_target = value

    @builtins.property
    def balance_value_4(self):
        """Message field 'balance_value_4'."""
        return self._balance_value_4

    @balance_value_4.setter
    def balance_value_4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_value_4' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_value_4' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_value_4 = value

    @builtins.property
    def balance_value_5(self):
        """Message field 'balance_value_5'."""
        return self._balance_value_5

    @balance_value_5.setter
    def balance_value_5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_value_5' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_value_5' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_value_5 = value

    @builtins.property
    def balance_pitch_flag_imu(self):
        """Message field 'balance_pitch_flag_imu'."""
        return self._balance_pitch_flag_imu

    @balance_pitch_flag_imu.setter
    def balance_pitch_flag_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'balance_pitch_flag_imu' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'balance_pitch_flag_imu' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._balance_pitch_flag_imu = value

    @builtins.property
    def balance_pitch_gp_imu(self):
        """Message field 'balance_pitch_gp_imu'."""
        return self._balance_pitch_gp_imu

    @balance_pitch_gp_imu.setter
    def balance_pitch_gp_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_pitch_gp_imu' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_pitch_gp_imu' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_pitch_gp_imu = value

    @builtins.property
    def balance_pitch_gi_imu(self):
        """Message field 'balance_pitch_gi_imu'."""
        return self._balance_pitch_gi_imu

    @balance_pitch_gi_imu.setter
    def balance_pitch_gi_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_pitch_gi_imu' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_pitch_gi_imu' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_pitch_gi_imu = value

    @builtins.property
    def balance_pitch_gd_imu(self):
        """Message field 'balance_pitch_gd_imu'."""
        return self._balance_pitch_gd_imu

    @balance_pitch_gd_imu.setter
    def balance_pitch_gd_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_pitch_gd_imu' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_pitch_gd_imu' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_pitch_gd_imu = value

    @builtins.property
    def balance_pitch_neg_target_imu(self):
        """Message field 'balance_pitch_neg_target_imu'."""
        return self._balance_pitch_neg_target_imu

    @balance_pitch_neg_target_imu.setter
    def balance_pitch_neg_target_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_pitch_neg_target_imu' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_pitch_neg_target_imu' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_pitch_neg_target_imu = value

    @builtins.property
    def balance_pitch_pos_target_imu(self):
        """Message field 'balance_pitch_pos_target_imu'."""
        return self._balance_pitch_pos_target_imu

    @balance_pitch_pos_target_imu.setter
    def balance_pitch_pos_target_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_pitch_pos_target_imu' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_pitch_pos_target_imu' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_pitch_pos_target_imu = value

    @builtins.property
    def balance_pitch_elimit_imu(self):
        """Message field 'balance_pitch_elimit_imu'."""
        return self._balance_pitch_elimit_imu

    @balance_pitch_elimit_imu.setter
    def balance_pitch_elimit_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_pitch_elimit_imu' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_pitch_elimit_imu' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_pitch_elimit_imu = value

    @builtins.property
    def balance_pitch_olimit_imu(self):
        """Message field 'balance_pitch_olimit_imu'."""
        return self._balance_pitch_olimit_imu

    @balance_pitch_olimit_imu.setter
    def balance_pitch_olimit_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_pitch_olimit_imu' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_pitch_olimit_imu' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_pitch_olimit_imu = value

    @builtins.property
    def balance_roll_flag_imu(self):
        """Message field 'balance_roll_flag_imu'."""
        return self._balance_roll_flag_imu

    @balance_roll_flag_imu.setter
    def balance_roll_flag_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'balance_roll_flag_imu' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'balance_roll_flag_imu' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._balance_roll_flag_imu = value

    @builtins.property
    def balance_roll_gp_imu(self):
        """Message field 'balance_roll_gp_imu'."""
        return self._balance_roll_gp_imu

    @balance_roll_gp_imu.setter
    def balance_roll_gp_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_roll_gp_imu' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_roll_gp_imu' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_roll_gp_imu = value

    @builtins.property
    def balance_roll_gi_imu(self):
        """Message field 'balance_roll_gi_imu'."""
        return self._balance_roll_gi_imu

    @balance_roll_gi_imu.setter
    def balance_roll_gi_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_roll_gi_imu' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_roll_gi_imu' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_roll_gi_imu = value

    @builtins.property
    def balance_roll_gd_imu(self):
        """Message field 'balance_roll_gd_imu'."""
        return self._balance_roll_gd_imu

    @balance_roll_gd_imu.setter
    def balance_roll_gd_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_roll_gd_imu' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_roll_gd_imu' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_roll_gd_imu = value

    @builtins.property
    def balance_roll_neg_target_imu(self):
        """Message field 'balance_roll_neg_target_imu'."""
        return self._balance_roll_neg_target_imu

    @balance_roll_neg_target_imu.setter
    def balance_roll_neg_target_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_roll_neg_target_imu' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_roll_neg_target_imu' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_roll_neg_target_imu = value

    @builtins.property
    def balance_roll_pos_target_imu(self):
        """Message field 'balance_roll_pos_target_imu'."""
        return self._balance_roll_pos_target_imu

    @balance_roll_pos_target_imu.setter
    def balance_roll_pos_target_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_roll_pos_target_imu' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_roll_pos_target_imu' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_roll_pos_target_imu = value

    @builtins.property
    def balance_roll_elimit_imu(self):
        """Message field 'balance_roll_elimit_imu'."""
        return self._balance_roll_elimit_imu

    @balance_roll_elimit_imu.setter
    def balance_roll_elimit_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_roll_elimit_imu' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_roll_elimit_imu' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_roll_elimit_imu = value

    @builtins.property
    def balance_roll_olimit_imu(self):
        """Message field 'balance_roll_olimit_imu'."""
        return self._balance_roll_olimit_imu

    @balance_roll_olimit_imu.setter
    def balance_roll_olimit_imu(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'balance_roll_olimit_imu' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'balance_roll_olimit_imu' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._balance_roll_olimit_imu = value

    @builtins.property
    def balance_pitch_flag(self):
        """Message field 'balance_pitch_flag'."""
        return self._balance_pitch_flag

    @balance_pitch_flag.setter
    def balance_pitch_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'balance_pitch_flag' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'balance_pitch_flag' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._balance_pitch_flag = value

    @builtins.property
    def balance_ankle_pitch_flag(self):
        """Message field 'balance_ankle_pitch_flag'."""
        return self._balance_ankle_pitch_flag

    @balance_ankle_pitch_flag.setter
    def balance_ankle_pitch_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'balance_ankle_pitch_flag' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'balance_ankle_pitch_flag' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._balance_ankle_pitch_flag = value

    @builtins.property
    def balance_roll_flag(self):
        """Message field 'balance_roll_flag'."""
        return self._balance_roll_flag

    @balance_roll_flag.setter
    def balance_roll_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'balance_roll_flag' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'balance_roll_flag' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._balance_roll_flag = value

    @builtins.property
    def ratio_check_flag(self):
        """Message field 'ratio_check_flag'."""
        return self._ratio_check_flag

    @ratio_check_flag.setter
    def ratio_check_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ratio_check_flag' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'ratio_check_flag' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._ratio_check_flag = value

    @builtins.property
    def center2leg(self):
        """Message field 'center2leg'."""
        return self._center2leg

    @center2leg.setter
    def center2leg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'center2leg' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'center2leg' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._center2leg = value

    @builtins.property
    def link2link(self):
        """Message field 'link2link'."""
        return self._link2link

    @link2link.setter
    def link2link(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'link2link' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'link2link' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._link2link = value

    @builtins.property
    def init_z_up(self):
        """Message field 'init_z_up'."""
        return self._init_z_up

    @init_z_up.setter
    def init_z_up(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'init_z_up' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'init_z_up' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._init_z_up = value

    @builtins.property
    def percentage_of_ik_motor(self):
        """Message field 'percentage_of_ik_motor'."""
        return self._percentage_of_ik_motor

    @percentage_of_ik_motor.setter
    def percentage_of_ik_motor(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'percentage_of_ik_motor' array.array() must have the type code of 'd'"
            self._percentage_of_ik_motor = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'percentage_of_ik_motor' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._percentage_of_ik_motor = array.array('d', value)

    @builtins.property
    def first_pos_xr(self):
        """Message field 'first_pos_xr'."""
        return self._first_pos_xr

    @first_pos_xr.setter
    def first_pos_xr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_pos_xr' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_pos_xr' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_pos_xr = value

    @builtins.property
    def first_neg_xr(self):
        """Message field 'first_neg_xr'."""
        return self._first_neg_xr

    @first_neg_xr.setter
    def first_neg_xr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_neg_xr' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_neg_xr' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_neg_xr = value

    @builtins.property
    def first_pos_side_r(self):
        """Message field 'first_pos_side_r'."""
        return self._first_pos_side_r

    @first_pos_side_r.setter
    def first_pos_side_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_pos_side_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_pos_side_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_pos_side_r = value

    @builtins.property
    def first_neg_side_r(self):
        """Message field 'first_neg_side_r'."""
        return self._first_neg_side_r

    @first_neg_side_r.setter
    def first_neg_side_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_neg_side_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_neg_side_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_neg_side_r = value

    @builtins.property
    def first_pos_yaw_r(self):
        """Message field 'first_pos_yaw_r'."""
        return self._first_pos_yaw_r

    @first_pos_yaw_r.setter
    def first_pos_yaw_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_pos_yaw_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_pos_yaw_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_pos_yaw_r = value

    @builtins.property
    def first_neg_yaw_r(self):
        """Message field 'first_neg_yaw_r'."""
        return self._first_neg_yaw_r

    @first_neg_yaw_r.setter
    def first_neg_yaw_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_neg_yaw_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_neg_yaw_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_neg_yaw_r = value

    @builtins.property
    def first_pos_xl(self):
        """Message field 'first_pos_xl'."""
        return self._first_pos_xl

    @first_pos_xl.setter
    def first_pos_xl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_pos_xl' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_pos_xl' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_pos_xl = value

    @builtins.property
    def first_neg_xl(self):
        """Message field 'first_neg_xl'."""
        return self._first_neg_xl

    @first_neg_xl.setter
    def first_neg_xl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_neg_xl' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_neg_xl' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_neg_xl = value

    @builtins.property
    def first_pos_side_l(self):
        """Message field 'first_pos_side_l'."""
        return self._first_pos_side_l

    @first_pos_side_l.setter
    def first_pos_side_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_pos_side_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_pos_side_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_pos_side_l = value

    @builtins.property
    def first_neg_side_l(self):
        """Message field 'first_neg_side_l'."""
        return self._first_neg_side_l

    @first_neg_side_l.setter
    def first_neg_side_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_neg_side_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_neg_side_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_neg_side_l = value

    @builtins.property
    def first_pos_yaw_l(self):
        """Message field 'first_pos_yaw_l'."""
        return self._first_pos_yaw_l

    @first_pos_yaw_l.setter
    def first_pos_yaw_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_pos_yaw_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_pos_yaw_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_pos_yaw_l = value

    @builtins.property
    def first_neg_yaw_l(self):
        """Message field 'first_neg_yaw_l'."""
        return self._first_neg_yaw_l

    @first_neg_yaw_l.setter
    def first_neg_yaw_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_neg_yaw_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_neg_yaw_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_neg_yaw_l = value

    @builtins.property
    def first_pos_side_r_swing_minus(self):
        """Message field 'first_pos_side_r_swing_minus'."""
        return self._first_pos_side_r_swing_minus

    @first_pos_side_r_swing_minus.setter
    def first_pos_side_r_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_pos_side_r_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_pos_side_r_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_pos_side_r_swing_minus = value

    @builtins.property
    def first_neg_side_r_swing_minus(self):
        """Message field 'first_neg_side_r_swing_minus'."""
        return self._first_neg_side_r_swing_minus

    @first_neg_side_r_swing_minus.setter
    def first_neg_side_r_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_neg_side_r_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_neg_side_r_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_neg_side_r_swing_minus = value

    @builtins.property
    def first_pos_side_l_swing_minus(self):
        """Message field 'first_pos_side_l_swing_minus'."""
        return self._first_pos_side_l_swing_minus

    @first_pos_side_l_swing_minus.setter
    def first_pos_side_l_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_pos_side_l_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_pos_side_l_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_pos_side_l_swing_minus = value

    @builtins.property
    def first_neg_side_l_swing_minus(self):
        """Message field 'first_neg_side_l_swing_minus'."""
        return self._first_neg_side_l_swing_minus

    @first_neg_side_l_swing_minus.setter
    def first_neg_side_l_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'first_neg_side_l_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'first_neg_side_l_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._first_neg_side_l_swing_minus = value

    @builtins.property
    def first_min(self):
        """Message field 'first_min'."""
        return self._first_min

    @first_min.setter
    def first_min(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'first_min' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'first_min' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._first_min = value

    @builtins.property
    def first_max(self):
        """Message field 'first_max'."""
        return self._first_max

    @first_max.setter
    def first_max(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'first_max' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'first_max' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._first_max = value

    @builtins.property
    def second_pos_xr(self):
        """Message field 'second_pos_xr'."""
        return self._second_pos_xr

    @second_pos_xr.setter
    def second_pos_xr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_pos_xr' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_pos_xr' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_pos_xr = value

    @builtins.property
    def second_neg_xr(self):
        """Message field 'second_neg_xr'."""
        return self._second_neg_xr

    @second_neg_xr.setter
    def second_neg_xr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_neg_xr' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_neg_xr' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_neg_xr = value

    @builtins.property
    def second_pos_side_r(self):
        """Message field 'second_pos_side_r'."""
        return self._second_pos_side_r

    @second_pos_side_r.setter
    def second_pos_side_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_pos_side_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_pos_side_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_pos_side_r = value

    @builtins.property
    def second_neg_side_r(self):
        """Message field 'second_neg_side_r'."""
        return self._second_neg_side_r

    @second_neg_side_r.setter
    def second_neg_side_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_neg_side_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_neg_side_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_neg_side_r = value

    @builtins.property
    def second_pos_yaw_r(self):
        """Message field 'second_pos_yaw_r'."""
        return self._second_pos_yaw_r

    @second_pos_yaw_r.setter
    def second_pos_yaw_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_pos_yaw_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_pos_yaw_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_pos_yaw_r = value

    @builtins.property
    def second_neg_yaw_r(self):
        """Message field 'second_neg_yaw_r'."""
        return self._second_neg_yaw_r

    @second_neg_yaw_r.setter
    def second_neg_yaw_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_neg_yaw_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_neg_yaw_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_neg_yaw_r = value

    @builtins.property
    def second_pos_xl(self):
        """Message field 'second_pos_xl'."""
        return self._second_pos_xl

    @second_pos_xl.setter
    def second_pos_xl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_pos_xl' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_pos_xl' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_pos_xl = value

    @builtins.property
    def second_neg_xl(self):
        """Message field 'second_neg_xl'."""
        return self._second_neg_xl

    @second_neg_xl.setter
    def second_neg_xl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_neg_xl' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_neg_xl' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_neg_xl = value

    @builtins.property
    def second_pos_side_l(self):
        """Message field 'second_pos_side_l'."""
        return self._second_pos_side_l

    @second_pos_side_l.setter
    def second_pos_side_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_pos_side_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_pos_side_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_pos_side_l = value

    @builtins.property
    def second_neg_side_l(self):
        """Message field 'second_neg_side_l'."""
        return self._second_neg_side_l

    @second_neg_side_l.setter
    def second_neg_side_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_neg_side_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_neg_side_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_neg_side_l = value

    @builtins.property
    def second_pos_yaw_l(self):
        """Message field 'second_pos_yaw_l'."""
        return self._second_pos_yaw_l

    @second_pos_yaw_l.setter
    def second_pos_yaw_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_pos_yaw_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_pos_yaw_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_pos_yaw_l = value

    @builtins.property
    def second_neg_yaw_l(self):
        """Message field 'second_neg_yaw_l'."""
        return self._second_neg_yaw_l

    @second_neg_yaw_l.setter
    def second_neg_yaw_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_neg_yaw_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_neg_yaw_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_neg_yaw_l = value

    @builtins.property
    def second_pos_side_r_swing_minus(self):
        """Message field 'second_pos_side_r_swing_minus'."""
        return self._second_pos_side_r_swing_minus

    @second_pos_side_r_swing_minus.setter
    def second_pos_side_r_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_pos_side_r_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_pos_side_r_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_pos_side_r_swing_minus = value

    @builtins.property
    def second_neg_side_r_swing_minus(self):
        """Message field 'second_neg_side_r_swing_minus'."""
        return self._second_neg_side_r_swing_minus

    @second_neg_side_r_swing_minus.setter
    def second_neg_side_r_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_neg_side_r_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_neg_side_r_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_neg_side_r_swing_minus = value

    @builtins.property
    def second_pos_side_l_swing_minus(self):
        """Message field 'second_pos_side_l_swing_minus'."""
        return self._second_pos_side_l_swing_minus

    @second_pos_side_l_swing_minus.setter
    def second_pos_side_l_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_pos_side_l_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_pos_side_l_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_pos_side_l_swing_minus = value

    @builtins.property
    def second_neg_side_l_swing_minus(self):
        """Message field 'second_neg_side_l_swing_minus'."""
        return self._second_neg_side_l_swing_minus

    @second_neg_side_l_swing_minus.setter
    def second_neg_side_l_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'second_neg_side_l_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'second_neg_side_l_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._second_neg_side_l_swing_minus = value

    @builtins.property
    def second_min(self):
        """Message field 'second_min'."""
        return self._second_min

    @second_min.setter
    def second_min(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'second_min' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'second_min' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._second_min = value

    @builtins.property
    def second_max(self):
        """Message field 'second_max'."""
        return self._second_max

    @second_max.setter
    def second_max(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'second_max' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'second_max' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._second_max = value

    @builtins.property
    def third_pos_xr(self):
        """Message field 'third_pos_xr'."""
        return self._third_pos_xr

    @third_pos_xr.setter
    def third_pos_xr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_pos_xr' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_pos_xr' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_pos_xr = value

    @builtins.property
    def third_neg_xr(self):
        """Message field 'third_neg_xr'."""
        return self._third_neg_xr

    @third_neg_xr.setter
    def third_neg_xr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_neg_xr' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_neg_xr' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_neg_xr = value

    @builtins.property
    def third_pos_side_r(self):
        """Message field 'third_pos_side_r'."""
        return self._third_pos_side_r

    @third_pos_side_r.setter
    def third_pos_side_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_pos_side_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_pos_side_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_pos_side_r = value

    @builtins.property
    def third_neg_side_r(self):
        """Message field 'third_neg_side_r'."""
        return self._third_neg_side_r

    @third_neg_side_r.setter
    def third_neg_side_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_neg_side_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_neg_side_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_neg_side_r = value

    @builtins.property
    def third_pos_yaw_r(self):
        """Message field 'third_pos_yaw_r'."""
        return self._third_pos_yaw_r

    @third_pos_yaw_r.setter
    def third_pos_yaw_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_pos_yaw_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_pos_yaw_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_pos_yaw_r = value

    @builtins.property
    def third_neg_yaw_r(self):
        """Message field 'third_neg_yaw_r'."""
        return self._third_neg_yaw_r

    @third_neg_yaw_r.setter
    def third_neg_yaw_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_neg_yaw_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_neg_yaw_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_neg_yaw_r = value

    @builtins.property
    def third_pos_xl(self):
        """Message field 'third_pos_xl'."""
        return self._third_pos_xl

    @third_pos_xl.setter
    def third_pos_xl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_pos_xl' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_pos_xl' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_pos_xl = value

    @builtins.property
    def third_neg_xl(self):
        """Message field 'third_neg_xl'."""
        return self._third_neg_xl

    @third_neg_xl.setter
    def third_neg_xl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_neg_xl' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_neg_xl' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_neg_xl = value

    @builtins.property
    def third_pos_side_l(self):
        """Message field 'third_pos_side_l'."""
        return self._third_pos_side_l

    @third_pos_side_l.setter
    def third_pos_side_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_pos_side_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_pos_side_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_pos_side_l = value

    @builtins.property
    def third_neg_side_l(self):
        """Message field 'third_neg_side_l'."""
        return self._third_neg_side_l

    @third_neg_side_l.setter
    def third_neg_side_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_neg_side_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_neg_side_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_neg_side_l = value

    @builtins.property
    def third_pos_yaw_l(self):
        """Message field 'third_pos_yaw_l'."""
        return self._third_pos_yaw_l

    @third_pos_yaw_l.setter
    def third_pos_yaw_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_pos_yaw_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_pos_yaw_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_pos_yaw_l = value

    @builtins.property
    def third_neg_yaw_l(self):
        """Message field 'third_neg_yaw_l'."""
        return self._third_neg_yaw_l

    @third_neg_yaw_l.setter
    def third_neg_yaw_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_neg_yaw_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_neg_yaw_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_neg_yaw_l = value

    @builtins.property
    def third_pos_side_r_swing_minus(self):
        """Message field 'third_pos_side_r_swing_minus'."""
        return self._third_pos_side_r_swing_minus

    @third_pos_side_r_swing_minus.setter
    def third_pos_side_r_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_pos_side_r_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_pos_side_r_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_pos_side_r_swing_minus = value

    @builtins.property
    def third_neg_side_r_swing_minus(self):
        """Message field 'third_neg_side_r_swing_minus'."""
        return self._third_neg_side_r_swing_minus

    @third_neg_side_r_swing_minus.setter
    def third_neg_side_r_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_neg_side_r_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_neg_side_r_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_neg_side_r_swing_minus = value

    @builtins.property
    def third_pos_side_l_swing_minus(self):
        """Message field 'third_pos_side_l_swing_minus'."""
        return self._third_pos_side_l_swing_minus

    @third_pos_side_l_swing_minus.setter
    def third_pos_side_l_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_pos_side_l_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_pos_side_l_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_pos_side_l_swing_minus = value

    @builtins.property
    def third_neg_side_l_swing_minus(self):
        """Message field 'third_neg_side_l_swing_minus'."""
        return self._third_neg_side_l_swing_minus

    @third_neg_side_l_swing_minus.setter
    def third_neg_side_l_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'third_neg_side_l_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'third_neg_side_l_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._third_neg_side_l_swing_minus = value

    @builtins.property
    def third_min(self):
        """Message field 'third_min'."""
        return self._third_min

    @third_min.setter
    def third_min(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'third_min' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'third_min' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._third_min = value

    @builtins.property
    def third_max(self):
        """Message field 'third_max'."""
        return self._third_max

    @third_max.setter
    def third_max(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'third_max' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'third_max' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._third_max = value

    @builtins.property
    def fourth_pos_xr(self):
        """Message field 'fourth_pos_xr'."""
        return self._fourth_pos_xr

    @fourth_pos_xr.setter
    def fourth_pos_xr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_pos_xr' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_pos_xr' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_pos_xr = value

    @builtins.property
    def fourth_neg_xr(self):
        """Message field 'fourth_neg_xr'."""
        return self._fourth_neg_xr

    @fourth_neg_xr.setter
    def fourth_neg_xr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_neg_xr' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_neg_xr' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_neg_xr = value

    @builtins.property
    def fourth_pos_side_r(self):
        """Message field 'fourth_pos_side_r'."""
        return self._fourth_pos_side_r

    @fourth_pos_side_r.setter
    def fourth_pos_side_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_pos_side_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_pos_side_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_pos_side_r = value

    @builtins.property
    def fourth_neg_side_r(self):
        """Message field 'fourth_neg_side_r'."""
        return self._fourth_neg_side_r

    @fourth_neg_side_r.setter
    def fourth_neg_side_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_neg_side_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_neg_side_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_neg_side_r = value

    @builtins.property
    def fourth_pos_yaw_r(self):
        """Message field 'fourth_pos_yaw_r'."""
        return self._fourth_pos_yaw_r

    @fourth_pos_yaw_r.setter
    def fourth_pos_yaw_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_pos_yaw_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_pos_yaw_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_pos_yaw_r = value

    @builtins.property
    def fourth_neg_yaw_r(self):
        """Message field 'fourth_neg_yaw_r'."""
        return self._fourth_neg_yaw_r

    @fourth_neg_yaw_r.setter
    def fourth_neg_yaw_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_neg_yaw_r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_neg_yaw_r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_neg_yaw_r = value

    @builtins.property
    def fourth_pos_xl(self):
        """Message field 'fourth_pos_xl'."""
        return self._fourth_pos_xl

    @fourth_pos_xl.setter
    def fourth_pos_xl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_pos_xl' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_pos_xl' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_pos_xl = value

    @builtins.property
    def fourth_neg_xl(self):
        """Message field 'fourth_neg_xl'."""
        return self._fourth_neg_xl

    @fourth_neg_xl.setter
    def fourth_neg_xl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_neg_xl' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_neg_xl' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_neg_xl = value

    @builtins.property
    def fourth_pos_side_l(self):
        """Message field 'fourth_pos_side_l'."""
        return self._fourth_pos_side_l

    @fourth_pos_side_l.setter
    def fourth_pos_side_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_pos_side_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_pos_side_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_pos_side_l = value

    @builtins.property
    def fourth_neg_side_l(self):
        """Message field 'fourth_neg_side_l'."""
        return self._fourth_neg_side_l

    @fourth_neg_side_l.setter
    def fourth_neg_side_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_neg_side_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_neg_side_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_neg_side_l = value

    @builtins.property
    def fourth_pos_yaw_l(self):
        """Message field 'fourth_pos_yaw_l'."""
        return self._fourth_pos_yaw_l

    @fourth_pos_yaw_l.setter
    def fourth_pos_yaw_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_pos_yaw_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_pos_yaw_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_pos_yaw_l = value

    @builtins.property
    def fourth_neg_yaw_l(self):
        """Message field 'fourth_neg_yaw_l'."""
        return self._fourth_neg_yaw_l

    @fourth_neg_yaw_l.setter
    def fourth_neg_yaw_l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_neg_yaw_l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_neg_yaw_l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_neg_yaw_l = value

    @builtins.property
    def fourth_pos_side_r_swing_minus(self):
        """Message field 'fourth_pos_side_r_swing_minus'."""
        return self._fourth_pos_side_r_swing_minus

    @fourth_pos_side_r_swing_minus.setter
    def fourth_pos_side_r_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_pos_side_r_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_pos_side_r_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_pos_side_r_swing_minus = value

    @builtins.property
    def fourth_neg_side_r_swing_minus(self):
        """Message field 'fourth_neg_side_r_swing_minus'."""
        return self._fourth_neg_side_r_swing_minus

    @fourth_neg_side_r_swing_minus.setter
    def fourth_neg_side_r_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_neg_side_r_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_neg_side_r_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_neg_side_r_swing_minus = value

    @builtins.property
    def fourth_pos_side_l_swing_minus(self):
        """Message field 'fourth_pos_side_l_swing_minus'."""
        return self._fourth_pos_side_l_swing_minus

    @fourth_pos_side_l_swing_minus.setter
    def fourth_pos_side_l_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_pos_side_l_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_pos_side_l_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_pos_side_l_swing_minus = value

    @builtins.property
    def fourth_neg_side_l_swing_minus(self):
        """Message field 'fourth_neg_side_l_swing_minus'."""
        return self._fourth_neg_side_l_swing_minus

    @fourth_neg_side_l_swing_minus.setter
    def fourth_neg_side_l_swing_minus(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fourth_neg_side_l_swing_minus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fourth_neg_side_l_swing_minus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fourth_neg_side_l_swing_minus = value

    @builtins.property
    def fourth_min(self):
        """Message field 'fourth_min'."""
        return self._fourth_min

    @fourth_min.setter
    def fourth_min(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fourth_min' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'fourth_min' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._fourth_min = value

    @builtins.property
    def fourth_max(self):
        """Message field 'fourth_max'."""
        return self._fourth_max

    @fourth_max.setter
    def fourth_max(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fourth_max' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'fourth_max' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._fourth_max = value

    @builtins.property
    def landing_time_control_flag(self):
        """Message field 'landing_time_control_flag'."""
        return self._landing_time_control_flag

    @landing_time_control_flag.setter
    def landing_time_control_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'landing_time_control_flag' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'landing_time_control_flag' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._landing_time_control_flag = value

    @builtins.property
    def control_system_flag(self):
        """Message field 'control_system_flag'."""
        return self._control_system_flag

    @control_system_flag.setter
    def control_system_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'control_system_flag' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'control_system_flag' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._control_system_flag = value
