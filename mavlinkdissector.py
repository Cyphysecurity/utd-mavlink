#!/usr/bin/env python3
# -*- coding: iso-8859-1 -*-

import sys
from struct import unpack
from struct import error as stex
from binascii import hexlify
import json
from logging import getLogger, ERROR
getLogger("scapy.runtime").setLevel(ERROR)
from scapy.all import PcapReader
from base64 import b64encode, b64decode
from time import time

MAVLINK_VERSIONS = {
    0x55: 'MAVLink 0.9',
    0xfe: 'MAVLink 1.0',
    0xfd: 'MAVLink 2.0'
}

# MAVLink enums

MAVLINK_AUTOPILOT = {
    0: 'MAV_AUTOPILOT_GENERIC',
    1: 'MAV_AUTOPILOT_RESERVED',
    2: 'MAV_AUTOPILOT_SLUGS',
    3: 'MAV_AUTOPILOT_ARDUPILOTMEGA',
    4: 'MAV_AUTOPILOT_OPENPILOT',
    5: 'MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY',
    6: 'MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY',
    7: 'MAV_AUTOPILOT_GENERIC_MISSION_FULL',
    8: 'MAV_AUTOPILOT_INVALID',
    9: 'MAV_AUTOPILOT_PPZ',
    10: 'MAV_AUTOPILOT_UDB',
    11: 'MAV_AUTOPILOT_FP',
    12: 'MAV_AUTOPILOT_PX4',
    13: 'MAV_AUTOPILOT_SMACCMPILOT',
    14: 'MAV_AUTOPILOT_AUTOQUAD',
    15: 'MAV_AUTOPILOT_ARMAZILA',
    16: 'MAV_AUTOPILOT_AEROB',
    17: 'MAV_AUTOPILOT_ASLUAV',
    18: 'MAV_AUTOPILOT_SMARTAP',
    19: 'MAV_AUTOPILOT_AIRRAILS',
}

MAVLINK_BATTERY_CHARGE_STATE = {
    0: 'MAV_BATTERY_CHARGE_STATE_UNDEFINED',
    1: 'MAV_BATTERY_CHARGE_STATE_OK',
    2: 'MAV_BATTERY_CHARGE_STATE_LOW',
    3: 'MAV_BATTERY_CHARGE_STATE_CRITICAL',
    4: 'MAV_BATTERY_CHARGE_STATE_EMERGENCY',
    5: 'MAV_BATTERY_CHARGE_STATE_FAILED',
    6: 'MAV_BATTERY_CHARGE_STATE_UNHEALTHY',
}

MAVLINK_BATTERY_FUNCTION = {
    0: 'MAV_BATTERY_FUNCTION_UNKNOWN',
    1: 'MAV_BATTERY_FUNCTION_ALL',
    2: 'MAV_BATTERY_FUNCTION_PROPULSION',
    3: 'MAV_BATTERY_FUNCTION_AVIONICS',
    4: 'MAV_BATTERY_TYPE_PAYLOAD',
}

MAVLINK_BATTERY_TYPE = {
    0: 'MAV_BATTERY_TYPE_UNKNOWN',
    1: 'MAV_BATTERY_TYPE_LIPO',
    2: 'MAV_BATTERY_TYPE_LIFE',
    3: 'MAV_BATTERY_TYPE_LION',
    4: 'MAV_BATTERY_TYPE_NIMH',
}

MAVLINK_CAMERA_STATUS_TYPES = {
    0: 'CAMERA_STATUS_TYPE_HEARTBEAT',
    1: 'CAMERA_STATUS_TYPE_TRIGGER',
    2: 'CAMERA_STATUS_TYPE_DISCONNECT',
    3: 'CAMERA_STATUS_TYPE_ERROR',
    4: 'CAMERA_STATUS_TYPE_LOWBATT',
    5: 'CAMERA_STATUS_TYPE_LOWSTORE',
    6: 'CAMERA_STATUS_TYPE_LOWSTOREV',
}

MAVLINK_CMD = {
    16: 'MAV_CMD_NAV_WAYPOINT',
    17: 'MAV_CMD_NAV_LOITER_UNLIM',
    18: 'MAV_CMD_NAV_LOITER_TURNS',
    19: 'MAV_CMD_NAV_LOITER_TIME',
    20: 'MAV_CMD_NAV_RETURN_TO_LAUNCH',
    21: 'MAV_CMD_NAV_LAND',
    22: 'MAV_CMD_NAV_TAKEOFF',
    23: 'MAV_CMD_NAV_LAND_LOCAL',
    24: 'MAV_CMD_NAV_TAKEOFF_LOCAL',
    25: 'MAV_CMD_NAV_FOLLOW',
    30: 'MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT',
    31: 'MAV_CMD_NAV_LOITER_TO_ALT',
    32: 'MAV_CMD_DO_FOLLOW',
    33: 'MAV_CMD_DO_FOLLOW_REPOSITION',
    80: 'MAV_CMD_NAV_ROI',
    81: 'MAV_CMD_NAV_PATHPLANNING',
    82: 'MAV_CMD_NAV_SPLINE_WAYPOINT',
    84: 'MAV_CMD_NAV_VTOL_TAKEOFF',
    85: 'MAV_CMD_NAV_VTOL_LAND',
    92: 'MAV_CMD_NAV_GUIDED_ENABLE',
    93: 'MAV_CMD_NAV_DELAY',
    94: 'MAV_CMD_NAV_PAYLOAD_PLACE',
    95: 'MAV_CMD_NAV_LAST',
    112: 'MAV_CMD_CONDITION_DELAY',
    113: 'MAV_CMD_CONDITION_CHANGE_ALT',
    114: 'MAV_CMD_CONDITION_DISTANCE',
    115: 'MAV_CMD_CONDITION_YAW',
    159: 'MAV_CMD_CONDITION_LAST',
    176: 'MAV_CMD_DO_SET_MODE',
    177: 'MAV_CMD_DO_JUMP',
    178: 'MAV_CMD_DO_CHANGE_SPEED',
    179: 'MAV_CMD_DO_SET_HOME',
    180: 'MAV_CMD_DO_SET_PARAMETER',
    181: 'MAV_CMD_DO_SET_RELAY',
    182: 'MAV_CMD_DO_REPEAT_RELAY',
    183: 'MAV_CMD_DO_SET_SERVO',
    184: 'MAV_CMD_DO_REPEAT_SERVO',
    185: 'MAV_CMD_DO_FLIGHTTERMINATION',
    186: 'MAV_CMD_DO_CHANGE_ALTITUDE',
    189: 'MAV_CMD_DO_LAND_START',
    190: 'MAV_CMD_DO_RALLY_LAND',
    191: 'MAV_CMD_DO_GO_AROUND',
    192: 'MAV_CMD_DO_REPOSITION',
    193: 'MAV_CMD_DO_PAUSE_CONTINUE',
    194: 'MAV_CMD_DO_SET_REVERSE',
    195: 'MAV_CMD_DO_SET_ROI_LOCATION',
    196: 'MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET',
    197: 'MAV_CMD_DO_SET_ROI_NONE',
    200: 'MAV_CMD_DO_CONTROL_VIDEO',
    201: 'MAV_CMD_DO_SET_ROI',
    202: 'MAV_CMD_DO_DIGICAM_CONFIGURE',
    203: 'MAV_CMD_DO_DIGICAM_CONTROL',
    204: 'MAV_CMD_DO_MOUNT_CONFIGURE',
    205: 'MAV_CMD_DO_MOUNT_CONTROL',
    206: 'MAV_CMD_DO_SET_CAM_TRIGG_DIST',
    207: 'MAV_CMD_DO_FENCE_ENABLE',
    208: 'MAV_CMD_DO_PARACHUTE',
    209: 'MAV_CMD_DO_MOTOR_TEST',
    210: 'MAV_CMD_DO_INVERTED_FLIGHT',
    213: 'MAV_CMD_NAV_SET_YAW_SPEED',
    214: 'MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL',
    220: 'MAV_CMD_DO_MOUNT_CONTROL_QUAT',
    221: 'MAV_CMD_DO_GUIDED_MASTER',
    222: 'MAV_CMD_DO_GUIDED_LIMITS',
    223: 'MAV_CMD_DO_ENGINE_CONTROL',
    240: 'MAV_CMD_DO_LAST',
    241: 'MAV_CMD_PREFLIGHT_CALIBRATION',
    242: 'MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS',
    243: 'MAV_CMD_PREFLIGHT_UAVCAN',
    245: 'MAV_CMD_PREFLIGHT_STORAGE',
    246: 'MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN',
    252: 'MAV_CMD_OVERRIDE_GOTO',
    300: 'MAV_CMD_MISSION_START',
    400: 'MAV_CMD_COMPONENT_ARM_DISARM',
    410: 'MAV_CMD_GET_HOME_POSITION',
    500: 'MAV_CMD_START_RX_PAIR',
    510: 'MAV_CMD_GET_MESSAGE_INTERVAL',
    511: 'MAV_CMD_SET_MESSAGE_INTERVAL',
    519: 'MAV_CMD_REQUEST_PROTOCOL_VERSION',
    520: 'MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES',
    521: 'MAV_CMD_REQUEST_CAMERA_INFORMATION',
    522: 'MAV_CMD_REQUEST_CAMERA_SETTINGS',
    525: 'MAV_CMD_REQUEST_STORAGE_INFORMATION',
    526: 'MAV_CMD_STORAGE_FORMAT',
    527: 'MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS',
    528: 'MAV_CMD_REQUEST_FLIGHT_INFORMATION',
    529: 'MAV_CMD_RESET_CAMERA_SETTINGS',
    530: 'MAV_CMD_SET_CAMERA_MODE',
    2000: 'MAV_CMD_IMAGE_START_CAPTURE',
    2001: 'MAV_CMD_IMAGE_STOP_CAPTURE',
    2002: 'MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE',
    2003: 'MAV_CMD_DO_TRIGGER_CONTROL',
    2500: 'MAV_CMD_VIDEO_START_CAPTURE',
    2501: 'MAV_CMD_VIDEO_STOP_CAPTURE',
    2502: 'MAV_CMD_VIDEO_START_STREAMING',
    2503: 'MAV_CMD_VIDEO_STOP_STREAMING',
    2504: 'MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION',
    2510: 'MAV_CMD_LOGGING_START',
    2511: 'MAV_CMD_LOGGING_STOP',
    2520: 'MAV_CMD_AIRFRAME_CONFIGURATION',
    2600: 'MAV_CMD_CONTROL_HIGH_LATENCY',
    2800: 'MAV_CMD_PANORAMA_CREATE',
    3000: 'MAV_CMD_DO_VTOL_TRANSITION',
    4000: 'MAV_CMD_SET_GUIDED_SUBMODE_STANDARD',
    4001: 'MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE',
    4501: 'MAV_CMD_CONDITION_GATE',
    3001: 'MAV_CMD_ARM_AUTHORIZATION_REQUEST',
    5000: 'MAV_CMD_NAV_FENCE_RETURN_POINT',
    5001: 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION',
    5002: 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION',
    5003: 'MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION',
    5004: 'MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION',
    5100: 'MAV_CMD_NAV_RALLY_POINT',
    5200: 'MAV_CMD_UAVCAN_GET_NODE_INFO',
    30001: 'MAV_CMD_PAYLOAD_PREPARE_DEPLOY',
    30002: 'MAV_CMD_PAYLOAD_CONTROL_DEPLOY',
    31000: 'MAV_CMD_WAYPOINT_USER_1',
    31001: 'MAV_CMD_WAYPOINT_USER_2',
    31002: 'MAV_CMD_WAYPOINT_USER_3',
    31003: 'MAV_CMD_WAYPOINT_USER_4',
    31004: 'MAV_CMD_WAYPOINT_USER_5',
    31005: 'MAV_CMD_SPATIAL_USER_1',
    31006: 'MAV_CMD_SPATIAL_USER_2',
    31007: 'MAV_CMD_SPATIAL_USER_3',
    31008: 'MAV_CMD_SPATIAL_USER_4',
    31009: 'MAV_CMD_SPATIAL_USER_5',
    31010: 'MAV_CMD_USER_1',
    31011: 'MAV_CMD_USER_2',
    31012: 'MAV_CMD_USER_3',
    31013: 'MAV_CMD_USER_4',
    31014: 'MAV_CMD_USER_5',
}

MAVLINK_CRC = {
    0:  50,
    1:  124,
    2:  137,
    4:  237,
    5:  217,
    6:  104,
    7:  119,
    11:  89,
    20:  214,
    21:  159,
    22:  220,
    23:  168,
    24:  24,
    25:  23,
    26:  170,
    27:  144,
    28:  67,
    29:  115,
    30:  39,
    31:  246,
    32:  185,
    33:  104,
    34:  237,
    35:  244,
    36:  222,
    37:  212,
    38:  9,
    39:  254,
    40:  230,
    41:  28,
    42:  28,
    43:  132,
    44:  221,
    45:  232,
    46:  11,
    47:  153,
    48:  41,
    49:  39,
    50:  78,
    51:  196,
    54:  15,
    55:  3,
    61:  167,
    62:  183,
    63:  119,
    64:  191,
    65:  118,
    66:  148,
    67:  21,
    69:  243,
    70:  124,
    73:  38,
    74:  20,
    75:  158,
    76:  152,
    77:  143,
    81:  106,
    82:  49,
    83:  22,
    84:  143,
    85:  140,
    86:  5,
    87:  150,
    89:  231,
    90:  183,
    91:  63,
    92:  54,
    93:  47,
    100:  175,
    101:  102,
    102:  158,
    103:  208,
    104:  56,
    105:  93,
    106:  138,
    107:  108,
    108:  32,
    109:  185,
    110:  84,
    111:  34,
    112:  174,
    113:  124,
    114:  237,
    115:  4,
    116:  76,
    117:  128,
    118:  56,
    119:  116,
    120:  134,
    121:  237,
    122:  203,
    123:  250,
    124:  87,
    125:  203,
    126:  220,
    127:  25,
    128:  226,
    129:  46,
    130:  29,
    131:  223,
    132:  85,
    133:  6,
    134:  229,
    135:  203,
    136:  1,
    137:  195,
    138:  109,
    139:  168,
    140:  181,
    141:  47,
    142:  72,
    143:  131,
    144:  127,
    146:  103,
    147:  154,
    148:  178,
    149:  200,
    230:  163,
    231:  105,
    232:  151,
    233:  35,
    234:  150,
    235:  179,
    241:  90,
    242:  104,
    243:  85,
    244:  95,
    245:  130,
    246:  184,
    247:  81,
    248:  8,
    249:  204,
    250:  49,
    251:  170,
    252:  44,
    253:  83,
    254:  46,
    256:  71,
    257:  131,
    258:  187,
    259:  92,
    260:  146,
    261:  179,
    262:  12,
    263:  133,
    264:  49,
    265:  26,
    266:  193,
    267:  35,
    268:  14,
    269:  58,
    270:  232,
    299:  19,
    300:  217,
    310:  28,
    311:  95,
    320:  243,
    321:  88,
    322:  243,
    323:  78,
    324:  132,
    330:  23,
    331:  58,
    332:  131,
}

MAVLINK_ESTIMATOR_STATUS_FLAGS = {
    1: 'ESTIMATOR_ATTITUDE',
    2: 'ESTIMATOR_VELOCITY_HORIZ',
    4: 'ESTIMATOR_VELOCITY_VERT',
    8: 'ESTIMATOR_POS_HORIZ_REL',
    16: 'ESTIMATOR_POS_HORIZ_ABS',
    32: 'ESTIMATOR_POS_VERT_ABS',
    64: 'ESTIMATOR_POS_VERT_AGL',
    128: 'ESTIMATOR_CONST_POS_MODE',
    256: 'ESTIMATOR_PRED_POS_HORIZ_REL',
    512: 'ESTIMATOR_PRED_POS_HORIZ_ABS',
    1024: 'ESTIMATOR_GPS_GLITCH',
    2048: 'ESTIMATOR_ACCEL_ERROR',
}

MAVLINK_FRAME = {
    0: 'MAV_FRAME_GLOBAL',
    1: 'MAV_FRAME_LOCAL_NED',
    2: 'MAV_FRAME_MISSION',
    3: 'MAV_FRAME_GLOBAL_RELATIVE_ALT',
    4: 'MAV_FRAME_LOCAL_ENU',
    5: 'MAV_FRAME_GLOBAL_INT',
    6: 'MAV_FRAME_GLOBAL_RELATIVE_ALT_INT',
    7: 'MAV_FRAME_LOCAL_OFFSET_NED',
    8: 'MAV_FRAME_BODY_NED',
    9: 'MAV_FRAME_BODY_OFFSET_NED',
    10: 'MAV_FRAME_GLOBAL_TERRAIN_ALT',
    11: 'MAV_FRAME_GLOBAL_TERRAIN_ALT_INT',
    12: 'MAV_FRAME_BODY_FRD',
    13: 'MAV_FRAME_BODY_FLU',
    14: 'MAV_FRAME_MOCAP_NED',
    15: 'MAV_FRAME_MOCAP_ENU',
    16: 'MAV_FRAME_VISION_NED',
    17: 'MAV_FRAME_VISION_ENU',
    18: 'MAV_FRAME_ESTIM_NED',
    19: 'MAV_FRAME_ESTIM_ENU',
}

MAVLINK_GPS_FIX_TYPE = {
    0: 'GPS_FIX_TYPE_NO_GPS',
    1: 'GPS_FIX_TYPE_NO_FIX',
    2: 'GPS_FIX_TYPE_2D_FIX',
    3: 'GPS_FIX_TYPE_3D_FIX',
    4: 'GPS_FIX_TYPE_DGPS',
    5: 'GPS_FIX_TYPE_RTK_FLOAT',
    6: 'GPS_FIX_TYPE_RTK_FIXED',
    7: 'GPS_FIX_TYPE_STATIC',
    8: 'GPS_FIX_TYPE_PPP',
}

MAVLINK_LANDED_STATE = {
    0: 'MAV_LANDED_STATE_UNDEFINED',
    1: 'MAV_LANDED_STATE_ON_GROUND',
    2: 'MAV_LANDED_STATE_IN_AIR',
    3: 'MAV_LANDED_STATE_TAKEOFF',
    4: 'MAV_LANDED_STATE_LANDING',
}

MAVLINK_MISSION_RESULT = {
    0: 'MAV_MISSION_ACCEPTED',
    1: 'MAV_MISSION_ERROR',
    2: 'MAV_MISSION_UNSUPPORTED_FRAME',
    3: 'MAV_MISSION_UNSUPPORTED',
    4: 'MAV_MISSION_NO_SPACE',
    5: 'MAV_MISSION_INVALID',
    6: 'MAV_MISSION_INVALID_PARAM1',
    7: 'MAV_MISSION_INVALID_PARAM2',
    8: 'MAV_MISSION_INVALID_PARAM3',
    9: 'MAV_MISSION_INVALID_PARAM4',
    10: 'MAV_MISSION_INVALID_PARAM5_X',
    11: 'MAV_MISSION_INVALID_PARAM6_Y',
    12: 'MAV_MISSION_INVALID_PARAM7',
    13: 'MAV_MISSION_INVALID_SEQUENCE',
    14: 'MAV_MISSION_DENIED',
}

MAVLINK_MISSION_TYPE = {
    0: 'MAV_MISSION_TYPE_MISSION',
    1: 'MAV_MISSION_TYPE_FENCE',
    2: 'MAV_MISSION_TYPE_RALLY',
    255: 'MAV_MISSION_TYPE_ALL',
}

MAVLINK_MODE = {
    0: 'MAV_MODE_PREFLIGHT',
    64: 'MAV_MODE_MANUAL_DISARMED',
    66: 'MAV_MODE_TEST_DISARMED',
    80: 'MAV_MODE_STABILIZE_DISARMED',
    88: 'MAV_MODE_GUIDED_DISARMED',
    92: 'MAV_MODE_AUTO_DISARMED',
    192: 'MAV_MODE_MANUAL_ARMED',
    194: 'MAV_MODE_TEST_ARMED',
    208: 'MAV_MODE_STABILIZE_ARMED',
    216: 'MAV_MODE_GUIDED_ARMED',
    220: 'MAV_MODE_AUTO_ARMED',
}

MAVLINK_MODE_FLAG = {
    1: 'MAV_MODE_FLAG_CUSTOM_MODE_ENABLED',
    2: 'MAV_MODE_FLAG_TEST_ENABLED',
    4: 'MAV_MODE_FLAG_AUTO_ENABLED',
    8: 'MAV_MODE_FLAG_GUIDED_ENABLED',
    16: 'MAV_MODE_FLAG_STABILIZE_ENABLED',
    32: 'MAV_MODE_FLAG_HIL_ENABLED',
    64: 'MAV_MODE_FLAG_MANUAL_INPUT_ENABLED',
    128: 'MAV_MODE_FLAG_SAFETY_ARMED',
}

MAVLINK_PARAM_TYPE = {
    1: 'MAV_PARAM_TYPE_UINT8',
    2: 'MAV_PARAM_TYPE_INT8',
    3: 'MAV_PARAM_TYPE_UINT16',
    4: 'MAV_PARAM_TYPE_INT16',
    5: 'MAV_PARAM_TYPE_UINT32',
    6: 'MAV_PARAM_TYPE_INT32',
    7: 'MAV_PARAM_TYPE_UINT64',
    8: 'MAV_PARAM_TYPE_INT64',
    9: 'MAV_PARAM_TYPE_REAL32',
    10: 'MAV_PARAM_TYPE_REAL64',
}

MAVLINK_PROTOCOL_CAPABILITY = {
    1: 'MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT',
    2: 'MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT',
    4: 'MAV_PROTOCOL_CAPABILITY_MISSION_INT',
    8: 'MAV_PROTOCOL_CAPABILITY_COMMAND_INT',
    16: 'MAV_PROTOCOL_CAPABILITY_PARAM_UNION',
    32: 'MAV_PROTOCOL_CAPABILITY_FTP',
    64: 'MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET',
    128: 'MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED',
    256: 'MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT',
    512: 'MAV_PROTOCOL_CAPABILITY_TERRAIN',
    1024: 'MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET',
    2048: 'MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION',
    4096: 'MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION',
    8192: 'MAV_PROTOCOL_CAPABILITY_MAVLINK2',
    16384: 'MAV_PROTOCOL_CAPABILITY_MISSION_FENCE',
    32768: 'MAV_PROTOCOL_CAPABILITY_MISSION_RALLY',
    65536: 'MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION',
}

MAVLINK_RESULT = {
    0: 'MAV_RESULT_ACCEPTED',
    1: 'MAV_RESULT_TEMPORARILY_REJECTED',
    2: 'MAV_RESULT_DENIED',
    3: 'MAV_RESULT_UNSUPPORTED',
    4: 'MAV_RESULT_FAILED',
    5: 'MAV_RESULT_IN_PROGRESS',
}

MAVLINK_SEVERITY = {
    0: 'MAV_SEVERITY_EMERGENCY',
    1: 'MAV_SEVERITY_ALERT',
    2: 'MAV_SEVERITY_CRITICAL',
    3: 'MAV_SEVERITY_ERROR',
    4: 'MAV_SEVERITY_WARNING',
    5: 'MAV_SEVERITY_NOTICE',
    6: 'MAV_SEVERITY_INFO',
    7: 'MAV_SEVERITY_DEBUG',
}

MAVLINK_STATE = {
    0: 'MAV_STATE_UNINIT',
    1: 'MAV_STATE_BOOT',
    2: 'MAV_STATE_CALIBRATING',
    3: 'MAV_STATE_STANDBY',
    4: 'MAV_STATE_ACTIVE',
    5: 'MAV_STATE_CRITICAL',
    6: 'MAV_STATE_EMERGENCY',
    7: 'MAV_STATE_POWEROFF',
    8: 'MAV_STATE_FLIGHT_TERMINATION',
}

MAVLINK_SYS_STATUS_SENSOR = {
    1: 'MAV_SYS_STATUS_SENSOR_3D_GYRO',
    2: 'MAV_SYS_STATUS_SENSOR_3D_ACCEL',
    4: 'MAV_SYS_STATUS_SENSOR_3D_MAG',
    8: 'MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE',
    16: 'MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE',
    32: 'MAV_SYS_STATUS_SENSOR_GPS',
    64: 'MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW',
    128: 'MAV_SYS_STATUS_SENSOR_VISION_POSITION',
    256: 'MAV_SYS_STATUS_SENSOR_LASER_POSITION',
    512: 'MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH',
    1024: 'MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL',
    2048: 'MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION',
    4096: 'MAV_SYS_STATUS_SENSOR_YAW_POSITION',
    8192: 'MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL',
    16384: 'MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL',
    32768: 'MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS',
    65536: 'MAV_SYS_STATUS_SENSOR_RC_RECEIVER',
    131072: 'MAV_SYS_STATUS_SENSOR_3D_GYRO2',
    262144: 'MAV_SYS_STATUS_SENSOR_3D_ACCEL2',
    524288: 'MAV_SYS_STATUS_SENSOR_3D_MAG2',
    33554432: 'MAV_SYS_STATUS_SENSOR_BATTERY',
}

MAVLINK_TYPE = {
    0: 'MAV_TYPE_GENERIC',
    1: 'MAV_TYPE_FIXED_WING',
    2: 'MAV_TYPE_QUADROTOR',
    3: 'MAV_TYPE_COAXIAL',
    4: 'MAV_TYPE_HELICOPTER',
    5: 'MAV_TYPE_ANTENNA_TRACKER',
    6: 'MAV_TYPE_GCS',
    7: 'MAV_TYPE_AIRSHIP',
    8: 'MAV_TYPE_FREE_BALLOON',
    9: 'MAV_TYPE_ROCKET',
    10: 'MAV_TYPE_GROUND_ROVER',
    11: 'MAV_TYPE_SURFACE_BOAT',
    12: 'MAV_TYPE_SUBMARINE',
    13: 'MAV_TYPE_HEXAROTOR',
    14: 'MAV_TYPE_OCTOROTOR',
    15: 'MAV_TYPE_TRICOPTER',
    16: 'MAV_TYPE_FLAPPING_WING',
    17: 'MAV_TYPE_KITE',
    18: 'MAV_TYPE_ONBOARD_CONTROLLER',
    19: 'MAV_TYPE_VTOL_DUOROTOR',
    20: 'MAV_TYPE_VTOL_QUADROTOR',
    21: 'MAV_TYPE_VTOL_TILTROTOR',
    22: 'MAV_TYPE_VTOL_RESERVED2',
    23: 'MAV_TYPE_VTOL_RESERVED3',
    24: 'MAV_TYPE_VTOL_RESERVED4',
    25: 'MAV_TYPE_VTOL_RESERVED5',
    26: 'MAV_TYPE_GIMBAL',
    27: 'MAV_TYPE_ADSB',
    28: 'MAV_TYPE_PARAFOIL',
    29: 'MAV_TYPE_DODECAROTOR',
    30: 'MAV_TYPE_CAMERA',
    31: 'MAV_TYPE_CHARGING_STATION',
}

MAVLINK_VTOL_STATE = {
    0: 'MAV_VTOL_STATE_UNDEFINED',
    1: 'MAV_VTOL_STATE_TRANSITION_TO_FW',
    2: 'MAV_VTOL_STATE_TRANSITION_TO_MC',
    3: 'MAV_VTOL_STATE_MC',
    4: 'MAV_VTOL_STATE_FW',
}

# Dissect HEARTBEAT
def dissect_0 (buffer):
    message = {}
    if len(buffer) == 9:
        unpacked_data = unpack('<BBBIBB', bytes(buffer))
        if unpacked_data[0] in MAVLINK_TYPE:
            message['type'] = MAVLINK_TYPE[unpacked_data[0]]
        else:
            message['type'] = 'UNKNOWN ({0:d})'.format(unpacked_data[0])
        if unpacked_data[1] in MAVLINK_AUTOPILOT:
            message['autopilot'] = MAVLINK_AUTOPILOT[unpacked_data[1]]
        else:
            message['autopilot'] = 'UNKNOWN ({0:d})'.format(unpacked_data[1])
        message['base_mode'] = {}
        for i in range(0, 8):
            message['base_mode'][MAVLINK_MODE_FLAG[2**i]] = bool(unpacked_data[2] & 2**i > 0)
        message['custom_mode'] = '0x{0:04x} [{0:032b}]'.format(unpacked_data[3])
        if unpacked_data[4] in MAVLINK_STATE:
            message['system_status'] = MAVLINK_STATE[unpacked_data[4]]
        else:
            message['system_status'] = 'UNKNOWN ({0:d})'.format(unpacked_data[4])
        message['mavlink_version'] = unpacked_data[5]
    else:
        message['error'] = 'Malformed HEARTBEAT'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect SYS_STATUS
def dissect_1 (buffer):
    message = {}
    if len(buffer) == 31:
        unpacked_data = unpack('<IIIHHhbHHHHHH', bytes(buffer))
        message['onboard_control_sensors_present'] = {}
        for i in range(0, 20):
            message['onboard_control_sensors_present'][MAVLINK_SYS_STATUS_SENSOR[2**i]] = bool(unpacked_data[0] & 2**i > 0)
        message['onboard_control_sensors_present'][MAVLINK_SYS_STATUS_SENSOR[2**25]] = bool(unpacked_data[0] & 2**25 > 0)
        message['onboard_control_sensors_enabled'] = {}
        for i in range(0, 20):
            message['onboard_control_sensors_enabled'][MAVLINK_SYS_STATUS_SENSOR[2**i]] = bool(unpacked_data[1] & 2**i > 0)
        message['onboard_control_sensors_enabled'][MAVLINK_SYS_STATUS_SENSOR[2**25]] = bool(unpacked_data[0] & 2**25 > 0)
        message['onboard_control_sensors_health'] = {}
        for i in range(0, 20):
            message['onboard_control_sensors_health'][MAVLINK_SYS_STATUS_SENSOR[2**i]] = bool(unpacked_data[2] & 2**i > 0)
        message['onboard_control_sensors_health'][MAVLINK_SYS_STATUS_SENSOR[2**25]] = bool(unpacked_data[0] & 2**25 > 0)
        message['load'] = unpacked_data[3]
        message['voltage_battery'] = unpacked_data[4]
        message['current_battery'] = unpacked_data[5]
        message['battery_remaining'] = unpacked_data[6]
        message['drop_rate_comm'] = unpacked_data[7]
        message['errors_comm'] = unpacked_data[8]
        message['errors_count1'] = unpacked_data[9]
        message['errors_count2'] = unpacked_data[10]
        message['errors_count3'] = unpacked_data[11]
        message['errors_count4'] = unpacked_data[12]
    else:
        message['error'] = 'Malformed SYS_STATUS'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect SYSTEM_TIME
def dissect_2 (buffer):
    message = {}
    if len(buffer) == 12:
        unpacked_data = unpack('<QI', bytes(buffer))
        message['time_unix_usec'] = unpacked_data[0]
        message['time_boot_ms'] = unpacked_data[1]
    else:
        message['error'] = 'Malformed SYSTEM_TIME'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect SET_MODE
def dissect_11 (buffer):
    message = {}
    if len(buffer) == 6:
        unpacked_data = unpack('<BBI', bytes(buffer))
        message['tagert_system'] = unpacked_data[0]
        if unpacked_data[1] in MAVLINK_MODE:
            message['base_mode'] = MAVLINK_MODE[unpacked_data[1]]
        else:
            message['base_mode'] = 'UNKNOWN ({0:d})'.format(unpacked_data[1])
        message['custom_mode'] = unpacked_data[2]
    else:
        message['error'] = 'Malformed SET_MODE'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect PARAM_REQUEST_LIST
def dissect_21 (buffer):
    message = {}
    if len(buffer) == 2:
        unpacked_data = unpack('BB', bytes(buffer))
        message['target_system'] = unpacked_data[0]
        message['target_component'] = unpacked_data[1]
    else:
        message['error'] = 'Malformed PARAM_REQUEST_LIST'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect PARAM_VALUE
def dissect_22 (buffer):
    message = {}
    if len(buffer) == 25:
        unpacked_data = unpack('<16BfBHH', bytes(buffer))
        message['param_id'] = list(bytes(unpacked_data[0:16]).decode('iso-8859-1'))
        message['param_value'] = unpacked_data[16]
        if unpacked_data[17] in MAVLINK_PARAM_TYPE:
            message['param_type'] = MAVLINK_PARAM_TYPE[unpacked_data[17]]
        else:
            message['param_type'] = 'UNKNOWN ({0:d})'.format(unpacked_data[17])
        message['param_count'] = unpacked_data[18]
        message['param_index'] = unpacked_data[19]
    else:
        message['error'] = 'Malformed PARAM_VALUE'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect PARAM_SET
def dissect_23 (buffer):
    message = {}
    if len(buffer) == 23:
        unpacked_data = unpack('<BB16BfB', bytes(buffer))
        message['target_system'] = unpacked_data[0]
        message['target_component'] = unpacked_data[1]
        message['param_id'] = list(bytes(unpacked_data[2:18]).decode('iso-8859-1'))
        message['param_value'] = unpacked_data[18]
        if unpacked_data[19] in MAVLINK_PARAM_TYPE:
            message['param_type'] = MAVLINK_PARAM_TYPE[unpacked_data[19]]
        else:
            message['param_type'] = 'UNKNOWN ({0:d})'.format(unpacked_data[19])
    else:
        message['error'] = 'Malformed PARAM_SET'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect GPS_RAW_INT
def dissect_24 (buffer):
    message = {}
    if len(buffer) == 30 or len(buffer) == 50:
        if len(buffer) == 30:
            unpacked_data = unpack('<QBiiiHHHHB', bytes(buffer))
        else:
            unpacked_data = unpack('<QBiiiHHHHBiIIII', bytes(buffer))
        message['time_usec'] = unpacked_data[0]
        if unpacked_data[1] in MAVLINK_GPS_FIX_TYPE:
            message['fix_type'] = MAVLINK_GPS_FIX_TYPE[unpacked_data[1]]
        else:
            message['fix_type'] = 'UNKNOWN ({0:d})'.format(unpacked_data[1])
        message['lat'] = unpacked_data[2]
        message['lon'] = unpacked_data[3]
        message['alt'] = unpacked_data[4]
        message['eph'] = unpacked_data[5]
        message['epv'] = unpacked_data[6]
        message['vel'] = unpacked_data[7]
        message['cog'] = unpacked_data[8]
        message['satellites_visible'] = unpacked_data[9]
        if len(buffer) == 50:
            message['alt_ellipsoid'] = unpacked_data[10]
            message['h_acc'] = unpacked_data[11]
            message['v_acc'] = unpacked_data[12]
            message['vel_acc'] = unpacked_data[13]
            message['hdg_acc'] = unpacked_data[14]
    else:
        message['error'] = 'Malformed GPS_RAW_INT'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect ATTITUDE
def dissect_30 (buffer):
    message = {}
    if len(buffer) == 28:
        unpacked_data = unpack('<Iffffff', bytes(buffer))
        message['time_boot_ms'] = unpacked_data[0]
        message['roll'] = unpacked_data[1]
        message['pitch'] = unpacked_data[2]
        message['yaw'] = unpacked_data[3]
        message['rollspeed'] = unpacked_data[4]
        message['pitchspeed'] = unpacked_data[5]
        message['yawspeed'] = unpacked_data[6]
    else:
        message['error'] = 'Malformed ATTITUDE'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect ATTITUDE_QUATERNION
def dissect_31 (buffer):
    message = {}
    if len(buffer) == 32:
        unpacked_data = unpack('<Ifffffff', bytes(buffer))
        message['time_boot_ms'] = unpacked_data[0]
        message['W'] = unpacked_data[1]
        message['X'] = unpacked_data[2]
        message['Y'] = unpacked_data[3]
        message['Z'] = unpacked_data[4]
        message['rollspeed'] = unpacked_data[5]
        message['pitchspeed'] = unpacked_data[6]
        message['yawspeed'] = unpacked_data[7]
    else:
        message['error'] = 'Malformed ATTITUDE_QUATERNION'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect LOCAL_POSITION_NED
def dissect_32 (buffer):
    message = {}
    if len(buffer) == 28:
        unpacked_data = unpack('<Iffffff', bytes(buffer))
        message['time_boot_ms'] = unpacked_data[0]
        message['x_position'] = unpacked_data[1]
        message['y_position'] = unpacked_data[2]
        message['z_position'] = unpacked_data[3]
        message['x_speed'] = unpacked_data[4]
        message['y_speed'] = unpacked_data[5]
        message['z_speed'] = unpacked_data[6]
    else:
        message['error'] = 'Malformed LOCAL_POSITION_NED'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect GLOBAL_POSITION_INT
def dissect_33 (buffer):
    message = {}
    if len(buffer) == 28:
        unpacked_data = unpack('<IiiiihhhH', bytes(buffer))
        message['time_boot_ms'] = unpacked_data[0]
        message['lat'] = unpacked_data[1]
        message['lon'] = unpacked_data[2]
        message['alt'] = unpacked_data[3]
        message['relative_alt'] = unpacked_data[4]
        message['vx'] = unpacked_data[5]
        message['vy'] = unpacked_data[6]
        message['vz'] = unpacked_data[7]
        message['hdg'] = unpacked_data[8]
    else:
        message['error'] = 'Malformed GLOBAL_POSITION_INT'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect SERVO_OUTPUT_RAW
def dissect_36 (buffer):
    message = {}
    if len(buffer) == 21 or len(buffer) == 37:
        if len(buffer) == 21:
            unpacked_data = unpack('<IBHHHHHHHH',bytes(buffer))
        else:
            unpacked_data = unpack('<IBHHHHHHHHHHHHHHHH',bytes(buffer))
        message['time_usec'] = unpacked_data[0]
        message['port'] = unpacked_data[1]
        message['servo 1'] = unpacked_data[2]
        message['servo 2'] = unpacked_data[3]
        message['servo 3'] = unpacked_data[4]
        message['servo 4'] = unpacked_data[5]
        message['servo 5'] = unpacked_data[6]
        message['servo 6'] = unpacked_data[7]
        message['servo 7'] = unpacked_data[8]
        message['servo 8'] = unpacked_data[9]
        if len(buffer) == 37:
            message['servo 9'] = unpacked_data[10]
            message['servo 10'] = unpacked_data[11]
            message['servo 11'] = unpacked_data[12]
            message['servo 12'] = unpacked_data[13]
            message['servo 13'] = unpacked_data[14]
            message['servo 14'] = unpacked_data[15]
            message['servo 15'] = unpacked_data[16]
            message['servo 16'] = unpacked_data[17]
    else:
        message['error'] = 'Malformed SERVO_OUTPUT_RAW'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect MISSION_ITEM
def dissect_39 (buffer):
    message = {}
    if len(buffer) == 37 or len(buffer) == 38:
        if len(buffer) == 37:
            unpacked_data = unpack('<BBHBHBBfffffff', bytes(buffer))
        else:
            unpacked_data = unpack('<BBHBHBBfffffffB', bytes(buffer))
        message['target_system'] = unpacked_data[0]
        message['target_component'] = unpacked_data[1]
        message['seq'] = unpacked_data[2]
        if unpacked_data[3] in MAVLINK_FRAME:
            message['frame'] = MAVLINK_FRAME[unpacked_data[3]]
        else:
            message['frame'] = 'UNKNOWN ({0:d})'.format(unpacked_data[3])
        if unpacked_data[4] in MAVLINK_CMD:
            message['command'] = MAVLINK_CMD[unpacked_data[4]]
        else:
            message['command'] = 'UNKNOWN ({0:d})'.format(unpacked_data[4])
        message['current'] = unpacked_data[5]
        message['autocontinue'] = unpacked_data[6]
        message['param1'] = unpacked_data[7]
        message['param2'] = unpacked_data[8]
        message['param3'] = unpacked_data[9]
        message['param4'] = unpacked_data[10]
        message['x'] = unpacked_data[11]
        message['y'] = unpacked_data[12]
        message['z'] = unpacked_data[13]
        if len(buffer) == 38:
            if unpacked_data[14] in MAVLINK_MISSION_TYPE:
                message['mission_type'] = MAVLINK_MISSION_TYPE[unpacked_data[14]]
            else:
                message['mission_type'] = 'UNKNOWN ({0:d})'.format(unpacked_data[14])
    else:
        message['error'] = 'Malformed MISSION_ITEM'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect MISSION_REQUEST
def dissect_40 (buffer):
    message = {}
    if len(buffer) == 4 or len(buffer) == 5:
        if len(buffer) == 4:
            unpacked_data = unpack('<BBH', bytes(buffer))
        else:
            unpacked_data = unpack('<BBHB', bytes(buffer))
        message['target_system'] = unpacked_data[0]
        message['target_component'] = unpacked_data[1]
        message['seq'] = unpacked_data[2]
        if len(buffer) == 5:
            if unpacked_data[3] in MAVLINK_MISSION_TYPE:
                message['mission_type'] = MAVLINK_MISSION_TYPE[unpacked_data[3]]
            else:
                message['mission_type'] = 'UNKNOWN ({0:d})'.format(unpacked_data[3])
    else:
        message['error'] = 'Malformed MISSION_REQUEST'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect MISSION_CURRENT
def dissect_42 (buffer):
    message = {}
    if len(buffer) == 2:
        message['seq'] = unpack('<H', bytes(buffer))[0]
    else:
        message['error'] = 'Malformed MISSION_CURRENT'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect MISSION_REQUEST_LIST
def dissect_43 (buffer):
    message = {}
    if len(buffer) == 2 or len(buffer) == 3:
        if len(buffer) == 2:
            unpacked_data = unpack('BB', bytes(buffer))
        else:
            unpacked_data = unpack('BBB', bytes(buffer))
        message['target_system'] = unpacked_data[0]
        message['target_component'] = unpacked_data[1]
        if len(buffer) == 3:
            if unpacked_data[2] in MAVLINK_MISSION_TYPE:
                message['mission_type'] = MAVLINK_MISSION_TYPE[unpacked_data[2]]
            else:
                message['mission_type'] = 'UNKNOWN ({0:d})'.format(unpacked_data[2])
    else:
        message['error'] = 'Malformed MISSION_REQUEST_LIST'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect MISSION_COUNT
def dissect_44 (buffer):
    message = {}
    if len(buffer) == 4 or len(buffer) == 5:
        if len(buffer) == 4:
            unpacked_data = unpack('<BBH', bytes(buffer))
        else:
            unpacked_data = unpack('<BBHB', bytes(buffer))
        message['target_system'] = unpacked_data[0]
        message['target_component'] = unpacked_data[1]
        message['count'] = unpacked_data[2]
        if len(buffer) == 5:
            if unpacked_data[3] in MAVLINK_MISSION_TYPE:
                message['mission_type'] = MAVLINK_MISSION_TYPE[unpacked_data[3]]
            else:
                message['mission_type'] = 'UNKNOWN ({0:d})'.format(unpacked_data[3])
    else:
        message['error'] = 'Malformed MISSION_COUNT'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect MISSION_ITEM_REACHED
def dissect_46 (buffer):
    message = {}
    if len(buffer) == 2:
        message['seq'] = unpack('<H', bytes(buffer))[0]
    else:
        message['error'] = 'Malformed MISSION_ITEM_REACHED'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect MISSION_ACK
def dissect_47 (buffer):
    message = {}
    if len(buffer) == 3 or len(buffer) == 4:
        if len(buffer) == 3:
            unpacked_data = unpack('3B', bytes(buffer))
        else:
            unpacked_data = unpack('4B', bytes(buffer))
        message['target_system'] = unpacked_data[0]
        message['target_component'] = unpacked_data[1]
        if unpacked_data[2] in MAVLINK_MISSION_RESULT:
            message['type'] = MAVLINK_MISSION_RESULT[unpacked_data[2]]
        else:
            message['type'] = 'UNKNOWN ({0:d})'.format(unpacked_data[2])
        if len(buffer) == 4:
            if unpacked_data[3] in MAVLINK_MISSION_TYPE:
                message['mission_type'] = MAVLINK_MISSION_TYPE[unpacked_data[3]]
            else:
                message['mission_type'] = 'UNKNOWN ({0:d})'.format(unpacked_data[3])
    else:
        message['error'] = 'Malformed MISSION_ACK'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect RC_CHANNELS
def dissect_65 (buffer):
    message = {}
    if len(buffer) == 42:
        unpacked_data = unpack('<IB18HB', bytes(buffer))
        message['time_boot_ms'] = unpacked_data[0]
        message['chancount'] = unpacked_data[1]
        for i in range(1, 19):
            message['chan{0:d}_raw'.format(i)] = unpacked_data[i + 1]
        message['rssi'] = unpacked_data[20]
    else:
        message['error'] = 'Malformed RC_CHANNELS'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect MANUAL_CONTROL
def dissect_69 (buffer):
    message = {}
    if len(buffer) == 11:
        unpacked_data = unpack('<BhhhhH', bytes(buffer))
        message['target'] = unpacked_data[0]
        message['x'] = unpacked_data[1]
        message['y'] = unpacked_data[2]
        message['z'] = unpacked_data[3]
        message['r'] = unpacked_data[4]
        message['buttons'] = unpacked_data[5]
    else:
        message['error'] = 'Malformed MANUAL_CONTROL'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect VFR_HUD
def dissect_74 (buffer):
    message = {}
    if len(buffer) == 20:
        unpacked_data = unpack('<ffhHff', bytes(buffer))
        message['airspeed'] = unpacked_data[0]
        message['groundspeed'] = unpacked_data[1]
        message['heading'] = unpacked_data[2]
        message['throttle'] = unpacked_data[3]
        message['alt'] = unpacked_data[4]
        message['climb'] = unpacked_data[5]
    else:
        message['error'] = 'Malformed VFR_HUD'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect COMMAND_LONG
def dissect_76 (buffer):
    message = {}
    if len(buffer) == 33:
        unpacked_data = unpack('<BBHBfffffff', bytes(buffer))
        message['target_system'] = unpacked_data[0]
        message['target_component'] = unpacked_data[1]
        if unpacked_data[2] in MAVLINK_CMD:
            message['command'] = MAVLINK_CMD[unpacked_data[2]]
        else:
            message['command'] = 'UNKNOWN ({0:d})'.format(unpacked_data[2])
        message['confirmation'] = unpacked_data[3]
        for i in range(1, 8):
            message['param{0:d}'.format(i)] = unpacked_data[i+3]
    else:
        message['error'] = 'Malformed COMMAND_LONG'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect COMMAND_ACK
def dissect_77 (buffer):
    message = {}
    if len(buffer) == 3 or len(buffer) == 10:
        if len(buffer) == 3:
            unpacked_data = unpack('<HB', bytes(buffer))
        else:
            unpacked_data = unpack('<HBBiBB', bytes(buffer))
        if unpacked_data[0] in MAVLINK_CMD:
            message['command'] = MAVLINK_CMD[unpacked_data[0]]
        else:
            message['command'] = 'UNKNOWN ({0:d})'.format(unpacked_data[0])
        if unpacked_data[1] in MAVLINK_RESULT:
            message['result'] = MAVLINK_RESULT[unpacked_data[1]]
        else:
            message['result'] = 'UNKNOWN ({0:d})'.format(unpacked_data[1])
        if len(buffer) == 10:
            message['progress'] = unpacked_data[2]
            message['result_param2'] = unpacked_data[3]
            message['target_system'] = unpacked_data[4]
            message['target_component'] = unpacked_data[5]
    else:
        message['error'] = 'Malformed COMMAND_ACK'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect ATTITUDE_TARGET
def dissect_83 (buffer):
    message = {}
    if len(buffer) == 37:
        unpacked_data = unpack('<IBffffffff', bytes(buffer))
        message['time_boot_ms'] = unpacked_data[0]
        message['type_mask'] = {}
        message['type_mask']['body_roll_rate'] = bool(unpacked_data[1] & 0x1 > 0)
        message['type_mask']['body_pitch_rate'] = bool(unpacked_data[1] & 0x2 > 0)
        message['type_mask']['body_yaw_rate'] = bool(unpacked_data[1] & 0x4 > 0)
        message['type_mask']['attitude'] = bool(unpacked_data[1] & 0x80 > 0)
        message['Q'] = {}
        message['Q']['W'] = unpacked_data[2]
        message['Q']['X'] = unpacked_data[3]
        message['Q']['Y'] = unpacked_data[4]
        message['Q']['Z'] = unpacked_data[5]
        message['body_roll_rate'] = unpacked_data[6]
        message['body_pitch_rate'] = unpacked_data[7]
        message['body_yaw_rate'] = unpacked_data[8]
        message['thrust'] = unpacked_data[9]
    else:
        message['error'] = 'Malformed ATTITUDE_TARGET'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect POSITION_TARGET_LOCAL_NED
def dissect_85 (buffer):
    message = {}
    if len(buffer) == 51:
        unpacked_data = unpack('<IBHfffffffffff', bytes(buffer))
        message['time_boot_ms'] = unpacked_data[0]
        message['coordinate_frame'] = unpacked_data[1]
        message['type_mask'] = unpacked_data[2]
        message['x'] = unpacked_data[3]
        message['y'] = unpacked_data[4]
        message['z'] = unpacked_data[5]
        message['vx'] = unpacked_data[6]
        message['vy'] = unpacked_data[7]
        message['vz'] = unpacked_data[8]
        message['afx'] = unpacked_data[9]
        message['afy'] = unpacked_data[10]
        message['afz'] = unpacked_data[11]
        message['yaw'] = unpacked_data[12]
        message['yaw_rate'] = unpacked_data[13]
    else:
        message['error'] = 'Malformed POSITION_TARGET_LOCAL_NED'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect POSITION_TARGET_GLOBAL_INT
def dissect_87 (buffer):
    message = {}
    if len(buffer) == 51:
        unpacked_data = unpack('<IBHiifffffffff', bytes(buffer))
        message['time_boot_ms'] = unpacked_data[0]
        if unpacked_data[1] in MAVLINK_FRAME:
            message['coordinate_frame'] = MAVLINK_FRAME[unpacked_data[1]]
        else:
            message['coordinate_frame'] = 'UNKNOWN ({0:d})'.format(unpacked_data[1])
        message['type_mask'] = unpacked_data[2]
        message['lat_int'] = unpacked_data[3]
        message['lon_int'] = unpacked_data[4]
        message['alt'] = unpacked_data[5]
        message['vx'] = unpacked_data[6]
        message['vy'] = unpacked_data[7]
        message['vz'] = unpacked_data[8]
        message['afx'] = unpacked_data[9]
        message['afy'] = unpacked_data[10]
        message['afz'] = unpacked_data[11]
        message['yaw'] = unpacked_data[12]
        message['yaw_rate'] = unpacked_data[13]
    else:
        message['error'] = 'Malformed POSITION_TARGET_GLOBAL_INT'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect HIL_ACTUATOR_CONTROLS
def dissect_93 (buffer):
    message = {}
    if len(buffer) == 81:
        unpacked_data = unpack('<Q16fBQ', bytes(buffer))
        message['time_usec'] = unpacked_data[0]
        message['controls'] = unpacked_data[1:17]
        if unpacked_data[17] in MAVLINK_MODE:
            message['mode'] = MAVLINK_MODE[unpacked_data[17]]
        else:
            message['mode'] = 'UNKNOWN ({0:d})'.format(unpacked_data[17])
        message['flags'] = unpacked_data[18]
    else:
        message['error'] = 'Malformed HIL_ACTUATOR_CONTROLS'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect HIGHRES_IMU
def dissect_105 (buffer):
    message = {}
    if len(buffer) == 62:
        unpacked_data = unpack('<QfffffffffffffH', bytes(buffer))
        message['time_usec'] = unpacked_data[0]
        message['xacc'] = unpacked_data[1]
        message['yacc'] = unpacked_data[2]
        message['zacc'] = unpacked_data[3]
        message['xgyro'] = unpacked_data[4]
        message['ygyro'] = unpacked_data[5]
        message['zgyro'] = unpacked_data[6]
        message['xmag'] = unpacked_data[7]
        message['ymag'] = unpacked_data[8]
        message['zmag'] = unpacked_data[9]
        message['abs_pressure'] = unpacked_data[10]
        message['diff_pressure'] = unpacked_data[11]
        message['pressure_alt'] = unpacked_data[12]
        message['temperature'] = unpacked_data[13]
        message['fields_updated'] = {}
        message['fields_updated']['xacc'] = bool((unpacked_data[14] & 0x1) > 0)
        message['fields_updated']['yacc'] = bool((unpacked_data[14] & 0x2) > 0)
        message['fields_updated']['zacc'] = bool((unpacked_data[14] & 0x4) > 0)
        message['fields_updated']['xgyro'] = bool((unpacked_data[14] & 0x8) > 0)
        message['fields_updated']['ygyro'] = bool((unpacked_data[14] & 0x10) > 0)
        message['fields_updated']['zgyro'] = bool((unpacked_data[14] & 0x20) > 0)
        message['fields_updated']['xmag'] = bool((unpacked_data[14] & 0x40) > 0)
        message['fields_updated']['ymag'] = bool((unpacked_data[14] & 0x80) > 0)
        message['fields_updated']['zmag'] = bool((unpacked_data[14] & 0x100) > 0)
        message['fields_updated']['abs_pressure'] = bool((unpacked_data[14] & 0x200) > 0)
        message['fields_updated']['diff_pressure'] = bool((unpacked_data[14] & 0x400) > 0)
        message['fields_updated']['pressure_alt'] = bool((unpacked_data[14] & 0x800) > 0)
        message['fields_updated']['temperature'] = bool((unpacked_data[14] & 0x1000) > 0)
    else:
        message['error'] = 'Malformed HIGHRES_IMU'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect HIL_SENSOR
def dissect_107 (buffer):
    message = {}
    if len(buffer) == 64:
        unpacked_data = unpack('<QfffffffffffffI', bytes(buffer))
        message['time_usec'] = unpacked_data[0]
        message['xacc'] = unpacked_data[1]
        message['yacc'] = unpacked_data[2]
        message['zacc'] = unpacked_data[3]
        message['xgyro'] = unpacked_data[4]
        message['ygyro'] = unpacked_data[5]
        message['zgyro'] = unpacked_data[6]
        message['xmag'] = unpacked_data[7]
        message['ymag'] = unpacked_data[8]
        message['zmag'] = unpacked_data[9]
        message['abs_pressure'] = unpacked_data[10]
        message['diff_pressure'] = unpacked_data[11]
        message['pressure_alt'] = unpacked_data[12]
        message['temperature'] = unpacked_data[13]
        message['fields_updated'] = {}
        message['fields_updated']['time_usec'] = bool(unpacked_data[14] & 2**0 > 0)
        message['fields_updated']['xacc'] = bool(unpacked_data[14] & 2**1 > 0)
        message['fields_updated']['yacc'] = bool(unpacked_data[14] & 2**2 > 0)
        message['fields_updated']['zacc'] = bool(unpacked_data[14] & 2**3 > 0)
        message['fields_updated']['xgyro'] = bool(unpacked_data[14] & 2**4 > 0)
        message['fields_updated']['ygyro'] = bool(unpacked_data[14] & 2**5 > 0)
        message['fields_updated']['zgyro'] = bool(unpacked_data[14] & 2**6 > 0)
        message['fields_updated']['xmag'] = bool(unpacked_data[14] & 2**7 > 0)
        message['fields_updated']['ymag'] = bool(unpacked_data[14] & 2**8 > 0)
        message['fields_updated']['zmag'] = bool(unpacked_data[14] & 2**9 > 0)
        message['fields_updated']['abs_pressure'] = bool(unpacked_data[14] & 2**10 > 0)
        message['fields_updated']['diff_pressure'] = bool(unpacked_data[14] & 2**11 > 0)
        message['fields_updated']['pressure_alt'] = bool(unpacked_data[14] & 2**12 > 0)
        message['fields_updated']['temperature'] = bool(unpacked_data[14] & 2**13 > 0)
        message['fields_updated']['full_reset'] = bool(unpacked_data[14] & 2**31 > 0)
    else:
        message['error'] = 'Malformed HIL_SENSOR'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect TIMESYNC
def dissect_111 (buffer):
    message = {}
    if len(buffer) == 16:
        unpacked_data = unpack('<QQ', bytes(buffer))
        message['tc1'] = unpacked_data[0]
        message['ts1'] = unpacked_data[1]
    else:
        message['error'] = 'Malformed TIMESYNC'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect HIL_GPS
def dissect_113 (buffer):
    message = {}
    if len(buffer) == 36:
        unpacked_data = unpack('<QBiiiHHHhhhHB', bytes(buffer))
        message['time_usec'] = unpacked_data[0]
        message['fix_type'] = unpacked_data[1]
        message['lat'] = unpacked_data[2]
        message['lon'] = unpacked_data[3]
        message['alt'] = unpacked_data[4]
        message['eph'] = unpacked_data[5]
        message['epv'] = unpacked_data[6]
        message['vel'] = unpacked_data[7]
        message['vn'] = unpacked_data[8]
        message['ve'] = unpacked_data[9]
        message['vd'] = unpacked_data[10]
        message['cog'] = unpacked_data[11]
        message['satellites_visible'] = unpacked_data[12]
    else:
        message['error'] = 'Malformed HIL_GPS'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect HIL_STATE_QUATERNION
def dissect_115 (buffer):
    message={}
    if len(buffer) == 64:
        unpacked_data = unpack('<Q4ffffiiihhhHHhhh', bytes(buffer))
        message['time_usec'] = unpacked_data[0]
        message['attitude_quaternion'] = {}
        message['attitude_quaternion']['w'] = unpacked_data[1]
        message['attitude_quaternion']['x'] = unpacked_data[2]
        message['attitude_quaternion']['y'] = unpacked_data[3]
        message['attitude_quaternion']['z'] = unpacked_data[4]
        message['rollspeed'] = unpacked_data[5]
        message['pitchspeed'] = unpacked_data[6]
        message['yawspeed'] = unpacked_data[7]
        message['lat'] = unpacked_data[8]
        message['lon'] = unpacked_data[9]
        message['alt'] = unpacked_data[10]
        message['vx'] = unpacked_data[11]
        message['vy'] = unpacked_data[12]
        message['vz'] = unpacked_data[13]
        message['ind_airspeed'] = unpacked_data[14]
        message['true_airspeed'] = unpacked_data[15]
        message['xacc'] = unpacked_data[16]
        message['yacc'] = unpacked_data[17]
        message['zacc'] = unpacked_data[18]
    else:
        message['error'] = 'Malformed HIL_STATE_QUATERNION'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect ACTUATOR_CONTROL_TARGET
def dissect_140 (buffer):
    message = {}
    if len(buffer) == 41:
        unpacked_data = unpack('<QBffffffff', bytes(buffer))
        message['time_usec'] = unpacked_data[0]
        message['group_mlx'] = unpacked_data[1]
        message['controls'] = {}
        for i in range(0, 8):
            message['controls'][i] = unpacked_data[i+2]
    else:
        message['error'] = 'Malformed ACTUATOR_CONTROL_TARGET'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect ALTITUDE
def dissect_141 (buffer):
    message = {}
    if len(buffer) == 32:
        unpacked_data = unpack('<Qffffff', bytes(buffer))
        message['time_usec'] = unpacked_data[0]
        message['altitude_monotonic'] = unpacked_data[1]
        message['altitude_amsl'] = unpacked_data[2]
        message['altitude_local'] = unpacked_data[3]
        message['altitude_relative'] = unpacked_data[4]
        message['altitude_terrain'] = unpacked_data[5]
        message['bottom_clearance'] = unpacked_data[6]
    else:
        message['error'] = 'Malformed ALTITUDE'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect BATTERY_STATUS
def dissect_147 (buffer):
    message = {}
    if len(buffer) == 36 or len(buffer) == 41:
        if len(buffer) == 36:
            unpacked_data = unpack('<BBBhHHHHHHHHHHhiib', bytes(buffer))
        else:
            unpacked_data = unpack('<BBBhHHHHHHHHHHhiibiB', bytes(buffer))
        message['id'] = unpacked_data[0]
        if unpacked_data[1] in MAVLINK_BATTERY_FUNCTION:
            message['battery_function'] = MAVLINK_BATTERY_FUNCTION[unpacked_data[1]]
        else:
            message['battery_function'] = 'UNKNOWN ({0:d})'.format(unpacked_data[1])
        if unpacked_data[2] in MAVLINK_BATTERY_TYPE:
            message['type'] = MAVLINK_BATTERY_TYPE[unpacked_data[2]]
        else:
            message['type'] = 'UNKNOWN ({0:d})'.format(unpacked_data[2])
        message['temperature'] = unpacked_data[3]
        message['voltages'] = {}
        for i in range(0, 10):
            message['voltages'][i] = unpacked_data[i+4]
        message['current_battery'] = unpacked_data[14]
        message['energy_consumed'] = unpacked_data[15]
        message['battery_remaining'] = unpacked_data[16]
        if len(buffer) == 41:
            message['time_remaining'] = unpacked_data[17]
            if unpacked_data[18] in MAVLINK_BATTERY_CHARGE_STATE:
                message['charge_state'] = MAVLINK_BATTERY_CHARGE_STATE[unpacked_data[18]]
            else:
                message['charge_state'] = 'UNKNOWN ({0:d})'.format(unpacked_data[18])
    else:
        message['error'] = 'Malformed BATTERY_STATUS'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect AUTOPILOT_VERSION
def dissect_148 (buffer):
    message = {}
    if len(buffer) == 52 or len(buffer) == 60 or len(buffer) == 78:
        if len(buffer) == 52:
            unpacked_data = unpack('<QIIII8B8B8BHH', bytes(buffer))
        elif len(buffer) == 60:
            unpacked_data = unpack('<QIIII8B8B8BHHQ', bytes(buffer))
        else:
            unpacked_data = unpack('<QIIII8B8B8BHHQ18B', bytes(buffer))
        message['capabilities'] = {}
        for i in MAVLINK_PROTOCOL_CAPABILITY:
            message['capabilities'][MAVLINK_PROTOCOL_CAPABILITY[i]] = bool(unpacked_data[0] & i > 0)
        message['flight_sw_version'] = unpacked_data[1]
        message['middleware_sw_version'] = unpacked_data[2]
        message['os_sw_version'] = unpacked_data[3]
        message['board_version'] = unpacked_data[4]
        message['flight_custom_version'] = unpacked_data[5:13]
        message['middleware_custom_version'] = unpacked_data[13:21]
        message['os_custom_version'] = unpacked_data[21:29]
        message['vendor_id'] = unpacked_data[29]
        message['product_id'] = unpacked_data[30]
        if len(buffer) == 52:
            message['uid'] = 'NO_UID_PROVIDED'
        else:
            message['uid'] = unpacked_data[31]
        if len(buffer) == 78:
            message['uid2'] = unpacked_data[32:]
    else:
        message['error'] = 'Malformed AUTOPILOT_VERSION'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect CAMERA_STATUS
def dissect_179 (buffer):
    message = {}
    if len(buffer) == 29:
        unpacked_data = unpack('<QBBHBffff', bytes(buffer))
        message['time_usec'] = unpacked_data[0]
        message['target_system'] = unpacked_data[1]
        message['cam_idx'] = unpacked_data[2]
        message['img_idx'] = unpacked_data[3]
        if unpacked_data[4] in MAVLINK_CAMERA_STATUS_TYPES:
            message['event_id'] = MAVLINK_CAMERA_STATUS_TYPES[unpacked_data[4]]
        else:
            message['event_id'] = 'UNKNOWN ({0:d})'.format(unpacked_data[4])
        message['p1'] = unpacked_data[5]
        message['p2'] = unpacked_data[6]
        message['p3'] = unpacked_data[7]
        message['p4'] = unpacked_data[8]
    else:
        message['error'] = 'Malformed CAMERA_STATUS'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect ESTIMATOR_STATUS
def dissect_230 (buffer):
    message = {}
    if len(buffer) == 42:
        unpacked_data = unpack('<QHffffffff', bytes(buffer))
        message['time_usec'] = unpacked_data[0]
        message['flags'] = {}
        for i in MAVLINK_ESTIMATOR_STATUS_FLAGS:
            message['flags'][MAVLINK_ESTIMATOR_STATUS_FLAGS[i]] = bool(unpacked_data[1] & i > 0)
        message['vel_ratio'] = unpacked_data[2]
        message['pos_horiz_ratio'] = unpacked_data[3]
        message['pos_vert_ratio'] = unpacked_data[4]
        message['mag_ratio'] = unpacked_data[5]
        message['hagl_ratio'] = unpacked_data[6]
        message['tas_ratio'] = unpacked_data[7]
        message['pos_horiz_accuracy'] = unpacked_data[8]
        message['pos_vert_accuracy'] = unpacked_data[9]
    else:
        message['error'] = 'Malformed ESTIMATOR_STATUS'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect WIND_COV
def dissect_231 (buffer):
    message = {}
    if len(buffer) == 40:
        unpacked_data = unpack('<Qffffffff', bytes(buffer))
        message['time_usec'] = unpacked_data[0]
        message['wind_x'] = unpacked_data[1]
        message['wind_y'] = unpacked_data[2]
        message['wind_z'] = unpacked_data[3]
        message['var_horiz'] = unpacked_data[4]
        message['var_vert'] = unpacked_data[5]
        message['wind_alt'] = unpacked_data[6]
        message['horiz_accuracy'] = unpacked_data[7]
        message['vert_accuracy'] = unpacked_data[8]
    else:
        message['error'] = 'Malformed WIND_COV'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect VIBRATION
def dissect_241 (buffer):
    message = {}
    if len(buffer) == 32:
        unpacked_data = unpack('<QfffIII', bytes(buffer))
        message['time_usec'] = unpacked_data[0]
        message['vibration_x'] = unpacked_data[1]
        message['vibration_y'] = unpacked_data[2]
        message['vibration_z'] = unpacked_data[3]
        message['clipping_0'] = unpacked_data[4]
        message['clipping_1'] = unpacked_data[5]
        message['clipping_2'] = unpacked_data[6]
    else:
        message['error'] = 'Malformed VIBRATION'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect HOME_POSITION
def dissect_242 (buffer):
    message = {}
    if len(buffer) == 52 or len(buffer) == 60:
        if len(buffer) == 52:
            unpacked_data = unpack('<iiifff4ffff', bytes(buffer))
        else:
            unpacked_data = unpack('<iiifff4ffffQ', bytes(buffer))
        message['latitude'] = unpacked_data[0]
        message['longitude'] = unpacked_data[1]
        message['altitude'] = unpacked_data[2]
        message['x'] = unpacked_data[3]
        message['y'] = unpacked_data[4]
        message['z'] = unpacked_data[5]
        message['q'] = unpacked_data[6:9]
        message['approach_x'] = unpacked_data[10]
        message['approach_y'] = unpacked_data[11]
        message['approach_z'] = unpacked_data[12]
        if len(buffer) == 60:
            message['time_usec'] = unpacked_data[13]
    else:
        message['error'] = 'Malformed HOME_POSITION'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect EXTENDED_SYS_STATE
def dissect_245 (buffer):
    message = {}
    if len(buffer) == 2:
        unpacked_data = unpack('<BB', bytes(buffer))
        if unpacked_data[0] in MAVLINK_VTOL_STATE:
            message['vtol_state'] = MAVLINK_VTOL_STATE[unpacked_data[0]]
        else:
            message['vtol_state'] = 'UNKNOWN ({0:d})'.format(unpacked_data[0])
        if unpacked_data[1] in MAVLINK_LANDED_STATE:
            message['landed_state'] = MAVLINK_LANDED_STATE[unpacked_data[1]]
        else:
            message['landed_state'] = 'UNKNOWN ({0:d})'.format(unpacked_data[1])
    else:
        message['error'] = 'Malformed EXTENDED_SYS_STATE'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect STATUSTEXT
def dissect_253 (buffer):
    message = {}
    if len(buffer) >=2 and len(buffer) <= 51:
        strlen = len(buffer) - 1
        unpacked_data = unpack('B{0:d}s'.format(strlen), bytes(buffer))
        if unpacked_data[0] in MAVLINK_SEVERITY:
            message['severity'] = MAVLINK_SEVERITY[unpacked_data[0]]
        else:
            message['severity'] = 'UNKNOWN ({0:d})'.format(unpacked_data[0])
        message['text'] = unpacked_data[1].decode('iso-8859-1')
    else:
        message['error'] = 'Malformed STATUSTEXT'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissect LOGGING_DATA
def dissect_266 (buffer):
    message = {}
    if len(buffer) == 255:
        unpacked_data = unpack('<BBHBB249B', bytes(buffer))
        message['target_system'] = unpacked_data[0]
        message['target_component'] = unpacked_data[1]
        message['sequence'] = unpacked_data[2]
        message['length'] = unpacked_data[3]
        message['first_message_offet'] = unpacked_data[4]
        message['data'] = list(bytes(unpacked_data[5:]).decode('iso-8859-1'))
    else:
        message['error'] = 'Malformed LOGGING_DATA'
        message['unparseable'] = '0x' + hexlify(buffer).decode('ascii')
    return message

# Dissector enums

MAVLINK_MESSAGE_NAME = {
    0: 'HEARTBEAT',
    1: 'SYS_STATUS',
    2: 'SYSTEM_TIME',
    11: 'SET_MODE',
    21: 'PARAM_REQUEST_LIST',
    22: 'PARAM_VALUE',
    23: 'PARAM_SET',
    24: 'GPS_RAW_INT',
    30: 'ATTITUDE',
    31: 'ATTITUDE_QUATERNION',
    32: 'LOCAL_POSITION_NED',
    33: 'GLOBAL_POSITION_INT',
    36: 'SERVO_OUTPUT_RAW',
    39: 'MISSION_ITEM',
    40: 'MISSION_REQUEST',
    42: 'MISSION_CURRENT',
    43: 'MISSION_REQUEST_LIST',
    44: 'MISSION_COUNT',
    46: 'MISSION_ITEM_REACHED',
    47: 'MISSION_ACK',
    65: 'RC_CHANNELS',
    69: 'MANUAL_CONTROL',
    74: 'VFR_HUD',
    76: 'COMMAND_LONG',
    77: 'COMMAND_ACK',
    83: 'ATTITUDE_TARGET',
    85: 'POSITION_TARGET_LOCAL_NED',
    87: 'POSITION_TARGET_GLOBAL_INT',
    93: 'HIL_ACTUATOR_CONTROLS',
    105: 'HIGHRES_IMU',
    107: 'HIL_SENSOR',
    111: 'TIMESYNC',
    113: 'HIL_GPS',
    115: 'HIL_STATE_QUATERNION',
    140: 'ACTUATOR_CONTROL_TARGET',
    141: 'ALTITUDE',
    147: 'BATTERY_STATUS',
    148: 'AUTOPILOT_VERSION',
    179: 'CAMERA_STATUS',
    230: 'ESTIMATOR_STATUS',
    231: 'WIND_COV',
    241: 'VIBRATION',
    242: 'HOME_POSITION',
    245: 'EXTENDED_SYS_STATE',
    253: 'STATUSTEXT',
    266: 'LOGGING_DATA',
}

PAYLOAD_DISSECTORS = {
    0: dissect_0,
    1: dissect_1,
    2: dissect_2,
    11: dissect_11,
    21: dissect_21,
    22: dissect_22,
    23: dissect_23,
    24: dissect_24,
    30: dissect_30,
    31: dissect_31,
    32: dissect_32,
    33: dissect_33,
    36: dissect_36,
    39: dissect_39,
    40: dissect_40,
    42: dissect_42,
    43: dissect_43,
    44: dissect_44,
    46: dissect_46,
    47: dissect_47,
    65: dissect_65,
    69: dissect_69,
    74: dissect_74,
    76: dissect_76,
    77: dissect_77,
    83: dissect_83,
    85: dissect_85,
    87: dissect_87,
    93: dissect_93,
    105: dissect_105,
    107: dissect_107,
    111: dissect_111,
    113: dissect_113,
    115: dissect_115,
    140: dissect_140,
    141: dissect_141,
    147: dissect_147,
    148: dissect_148,
    179: dissect_179,
    230: dissect_230,
    231: dissect_231,
    241: dissect_241,
    242: dissect_242,
    245: dissect_245,
    253: dissect_253,
    266: dissect_266,
}

# Check message CRC
def message_crc(data, msgid):
    '''x25 CRC based on checksum.h'''
    crc = 0xffff
    buf = list(data)
    buf.append(MAVLINK_CRC[msgid])
    for b in buf:
        tmp = b ^ (crc & 0xff)
        tmp = (tmp ^ (tmp << 4)) & 0xff
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    return crc

# Dissect entire data buffer
def dissect_mavlink(data):
    offset = 0
    messages = []
    last = 0
    curr = 0
    mps = 0
    print('Dissecting MAVLink messages ...')
    while(offset < len(data)):
        proto_version = data[offset]
        unknownBegin = offset
        unknownEnd = len(data)
        proto_string = None
        found = False
        message = {}
        while not found and offset <= len(data):
            if proto_version in [ 0xfd, 0xfe, 0x55 ]:
                proto_string = 'MAVLink'
                found = True
            else:
                if unknownEnd < offset:
                    unknownBegin = offset
                    unknownEnd = len(data)
                offset += 1
                if offset < len(data):
                    proto_version = data[offset]
                    if proto_version in [ 0xfd, 0xfe, 0x55 ]:
                        unknownEnd = offset
                        found = True
        if unknownEnd < len(data):
            message['protocol'] = 'UNKNOWN'
            message['payload'] = '0x' + hexlify(data[unknownBegin:unknownEnd]).decode('ascii')
        elif proto_string is None and unknownEnd == len(data):
            message['protocol'] = 'UNKNOWN'
            message['payload'] = '0x' + hexlify(data[unknownBegin:]).decode('ascii')
        elif offset + 3 < len(data):
            message['protocol'] = MAVLINK_VERSIONS.get(proto_version)
            offset += 1
            length = int(data[offset])
            if ( len(data) - 2 - offset > 10 and proto_version == 0xfd ) or ( len(data) - 2 - offset > 6 and proto_version in [0xfe, 0x55] ):
                message['length'] = length
                if proto_version == 0xfd:
                    crc_payload = bytes(data[offset : offset + 9 + length])
                else:
                    crc_payload = bytes(data[offset : offset +  5 + length])
                offset += 1
                if proto_version == 0xfd:
                    message['incompat_flags'] = data[offset]
                    offset += 1
                    message['compat_flags'] = data[offset]
                    offset += 1
                message['sequence'] = int(data[offset])
                offset += 1
                message['sysid'] = data[offset]
                offset += 1
                message['compid'] = data[offset]
                offset += 1
                if proto_version == 0xfd:
                    message['id'] = int(unpack('<I',bytes(data[offset:offset+3]) + b'\x00')[0])
                    offset += 3
                else:
                    message['id'] = int(data[offset])
                    offset += 1
                msgid = message['id']
                # Dissect payload if any
                if msgid in PAYLOAD_DISSECTORS:
                    message['payload'] = PAYLOAD_DISSECTORS[msgid](data[offset:offset+length])
                    message['name'] = MAVLINK_MESSAGE_NAME[msgid]
                else:
                    message['payload'] = '0x' + hexlify(data[offset : offset + length]).decode('ascii')
                    message['name'] = 'UNKNOWN MESSAGE'
                offset += length
                try:
                    message['CRC'] = unpack('<H',bytes(data[offset : offset + 2]))[0]
                    if message['id'] in MAVLINK_CRC:
                        message['CRC OK'] = (message['CRC'] == message_crc(crc_payload, message['id']))
                    else:
                        message['CRC OK'] = False
                except stex:
                    message['CRC'] = None
                    message['CRC OK'] = False
                offset += 2
            else:
                # Truncated header
                hsize = len(data) - 2 - offset
                message['TRUNCATED HEADER'] = data[offset : offset + hsize]
                offset += hsize
            messages.append(message)
            if (len(messages) % 1000) == 0:
                curr = time() * 1e6
                diff = curr - last
                mps = int(1e9 / diff)
                last = curr
            sys.stdout.write('\r' + ' ' * 50)
            sys.stdout.write('\r{0:,d} messages dissected ({1:,d} mps)...'.format(len(messages), mps))
        else:
            offset += 1
    print('')
    return messages

# Read the pcap and start the dissection
def main():
    print('Reading pcap ...')
    packets = PcapReader(sys.argv[1])
    mavlink_pl = b''
    count = 0
    last = 0
    curr = 0
    pps = 0
    for packet in packets:
        if packet.haslayer('TCP') and ( packet['TCP'].dport == 5760 or packet['TCP'].sport == 5760 ):
            payload = bytes(packet['TCP'].payload)
            if len(payload) > 0:
                mavlink_pl += payload
        elif packet.haslayer('UDP') and ( packet['UDP'].dport in [14550, 14556, 14557, 14560] or packet['UDP'].sport in [14550, 14556, 14557, 14560] ):
            payload = bytes(packet['UDP'].payload)
            if len(payload) > 0:
                mavlink_pl += payload
        count += 1
        if (count % 100) == 0:
            curr = time() * 1e6
            diff = curr - last
            pps = int(1e8 / diff)
        last = curr
        sys.stdout.write('\r' + ' ' * 50)
        sys.stdout.write('\r{0:,d} packets processed ({1:,d} pps)...'.format(count, pps))
    sys.stdout.write('\r\n')
    print('Read {0:,d} bytes'.format(len(mavlink_pl)))
    packets = None
    messages = dissect_mavlink(mavlink_pl)
    if len(sys.argv) >= 3:
        print('Writing output to {0:s} ...'.format(sys.argv[2]))
        f = open(sys.argv[2], mode='w')
        f.seek(0)
        json.dump(messages, f, indent=2)
        f.close()
    print('done!')


if __name__ == '__main__':
    main()
