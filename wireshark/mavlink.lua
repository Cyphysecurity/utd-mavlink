-- Wireshark dissector

-- luacheck: read_globals Proto ProtoField base DissectorTable PI_MALFORMED PI_ERROR wtap
-- luacheck: allow_defined

mavlink_proto = Proto("mavlink", "MAVLink Protocol")

-- Message names
messageName = {
    [0] = 'HEARTBEAT',
    [1] = 'SYS_STATUS',
    [2] = 'SYSTEM_TIME',
    [4] = 'PING',
    [5] = 'CHANGE_OPERATOR_CONTROL',
    [6] = 'CHANGE_OPERATOR_CONTROL_ACK',
    [7] = 'AUTH_KEY',
    [11] = 'SET_MODE',
    [20] = 'PARAM_REQUEST_READ',
    [21] = 'PARAM_REQUEST_LIST',
    [22] = 'PARAM_VALUE',
    [23] = 'PARAM_SET',
    [24] = 'GPS_RAW_INT',
    [25] = 'GPS_STATUS',
    [26] = 'SCALED_IMU',
    [27] = 'RAW_IMU',
    [28] = 'RAW_PRESSURE',
    [29] = 'SCALED_PRESSURE',
    [30] = 'ATTITUDE',
    [31] = 'ATTITUDE_QUATERNION',
    [32] = 'LOCAL_POSITION_NED',
    [33] = 'GLOBAL_POSITION_INT',
    [34] = 'RC_CHANNELS_SCALED',
    [35] = 'RC_CHANNELS_RAW',
    [36] = 'SERVO_OUTPUT_RAW',
    [37] = 'MISSION_REQUEST_PARTIAL_LIST',
    [38] = 'MISSION_WRITE_PARTIAL_LIST',
    [39] = 'MISSION_ITEM',
    [40] = 'MISSION_REQUEST',
    [41] = 'MISSION_SET_CURRENT',
    [42] = 'MISSION_CURRENT',
    [43] = 'MISSION_REQUEST_LIST',
    [44] = 'MISSION_COUNT',
    [45] = 'MISSION_CLEAR_ALL',
    [46] = 'MISSION_ITEM_REACHED',
    [47] = 'MISSION_ACK',
    [48] = 'SET_GPS_GLOBAL_ORIGIN',
    [49] = 'GPS_GLOBAL_ORIGIN',
    [50] = 'PARAM_MAP_RC',
    [51] = 'MISSION_REQUEST_INT',
    [54] = 'SAFETY_SET_ALLOWED_AREA',
    [55] = 'SAFETY_ALLOWED_AREA',
    [61] = 'ATTITUDE_QUATERNION_COV',
    [62] = 'NAV_CONTROLLER_OUTPUT',
    [63] = 'GLOBAL_POSITION_INT_COV',
    [64] = 'LOCAL_POSITION_NED_COV',
    [65] = 'RC_CHANNELS',
    [66] = 'REQUEST_DATA_STREAM',
    [67] = 'DATA_STREAM',
    [69] = 'MANUAL_CONTROL',
    [70] = 'RC_CHANNELS_OVERRIDE',
    [73] = 'MISSION_ITEM_INT',
    [74] = 'VFR_HUD',
    [75] = 'COMMAND_INT',
    [76] = 'COMMAND_LONG',
    [77] = 'COMMAND_ACK',
    [81] = 'MANUAL_SETPOINT',
    [82] = 'SET_ATTITUDE_TARGET',
    [83] = 'ATTITUDE_TARGET',
    [84] = 'SET_POSITION_TARGET_LOCAL_NED',
    [85] = 'POSITION_TARGET_LOCAL_NED',
    [86] = 'SET_POSITION_TARGET_GLOBAL_INT',
    [87] = 'POSITION_TARGET_GLOBAL_INT',
    [89] = 'LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET',
    [90] = 'HIL_STATE',
    [91] = 'HIL_CONTROLS',
    [92] = 'HIL_RC_INPUTS_RAW',
    [93] = 'HIL_ACTUATOR_CONTROLS',
    [100] = 'OPTICAL_FLOW',
    [101] = 'GLOBAL_VISION_POSITION_ESTIMATE',
    [102] = 'VISION_POSITION_ESTIMATE',
    [103] = 'VISION_SPEED_ESTIMATE',
    [104] = 'VICON_POSITION_ESTIMATE',
    [105] = 'HIGHRES_IMU',
    [106] = 'OPTICAL_FLOW_RAD',
    [107] = 'HIL_SENSOR',
    [108] = 'SIM_STATE',
    [109] = 'RADIO_STATUS',
    [110] = 'FILE_TRANSFER_PROTOCOL',
    [111] = 'TIMESYNC',
    [112] = 'CAMERA_TRIGGER',
    [113] = 'HIL_GPS',
    [114] = 'HIL_OPTICAL_FLOW',
    [115] = 'HIL_STATE_QUATERNION',
    [116] = 'SCALED_IMU2',
    [117] = 'LOG_REQUEST_LIST',
    [118] = 'LOG_ENTRY',
    [119] = 'LOG_REQUEST_DATA',
    [120] = 'LOG_DATA',
    [121] = 'LOG_ERASE',
    [122] = 'LOG_REQUEST_END',
    [123] = 'GPS_INJECT_DATA',
    [124] = 'GPS2_RAW',
    [125] = 'POWER_STATUS',
    [126] = 'SERIAL_CONTROL',
    [127] = 'GPS_RTK',
    [128] = 'GPS2_RTK',
    [129] = 'SCALED_IMU3',
    [130] = 'DATA_TRANSMISSION_HANDSHAKE',
    [131] = 'ENCAPSULATED_DATA',
    [132] = 'DISTANCE_SENSOR',
    [133] = 'TERRAIN_REQUEST',
    [134] = 'TERRAIN_DATA',
    [135] = 'TERRAIN_CHECK',
    [136] = 'TERRAIN_REPORT',
    [137] = 'SCALED_PRESSURE2',
    [138] = 'ATT_POS_MOCAP',
    [139] = 'SET_ACTUATOR_CONTROL_TARGET',
    [140] = 'ACTUATOR_CONTROL_TARGET',
    [141] = 'ALTITUDE',
    [142] = 'RESOURCE_REQUEST',
    [143] = 'SCALED_PRESSURE3',
    [144] = 'FOLLOW_TARGET',
    [146] = 'CONTROL_SYSTEM_STATE',
    [147] = 'BATTERY_STATUS',
    [148] = 'AUTOPILOT_VERSION',
    [149] = 'LANDING_TARGET',
    [150] = 'SENSOR_OFFSETS',
    [151] = 'SET_MAG_OFFSETS',
    [152] = 'MEMINFO',
    [153] = 'AP_ADC',
    [154] = 'DIGICAM_CONFIGURE',
    [155] = 'DIGICAM_CONTROL',
    [156] = 'MOUNT_CONFIGURE',
    [157] = 'MOUNT_CONTROL',
    [158] = 'MOUNT_STATUS',
    [160] = 'FENCE_POINT',
    [161] = 'FENCE_FETCH_POINT',
    [162] = 'FENCE_STATUS',
    [163] = 'AHRS',
    [164] = 'SIMSTATE',
    [165] = 'HWSTATUS',
    [166] = 'RADIO',
    [167] = 'LIMITS_STATUS',
    [168] = 'WIND',
    [169] = 'DATA16',
    [170] = 'DATA32',
    [171] = 'DATA64',
    [172] = 'DATA96',
    [173] = 'RANGEFINDER',
    [174] = 'AIRSPEED_AUTOCAL',
    [175] = 'RALLY_POINT',
    [176] = 'RALLY_FETCH_POINT',
    [177] = 'COMPASSMOT_STATUS',
    [178] = 'AHRS2',
    [179] = 'CAMERA_STATUS',
    [180] = 'CAMERA_FEEDBACK',
    [181] = 'BATTERY2',
    [182] = 'AHRS3',
    [183] = 'AUTOPILOT_VERSION_REQUEST',
    [184] = 'REMOTE_LOG_DATA_BLOCK',
    [185] = 'REMOTE_LOG_BLOCK_STATUS',
    [186] = 'LED_CONTROL',
    [191] = 'MAG_CAL_PROGRESS',
    [192] = 'MAG_CAL_REPORT',
    [193] = 'EKF_STATUS_REPORT',
    [194] = 'PID_TUNING',
    [195] = 'DEEPSTALL',
    [200] = 'GIMBAL_REPORT',
    [201] = 'SENS_POWER',
    [202] = 'SENS_MPPT',
    [203] = 'ASLCTRL_DATA',
    [204] = 'ASLCTRL_DEBUG',
    [205] = 'ASLUAV_STATUS',
    [206] = 'EKF_EXT',
    [207] = 'ASL_OBCTRL',
    [208] = 'SENS_ATMOS',
    [209] = 'SENS_BATMON',
    [210] = 'FW_SOARING_DATA',
    [211] = 'SENSORPOD_STATUS',
    [212] = 'SENS_POWER_BOARD',
    [214] = 'GIMBAL_TORQUE_CMD_REPORT',
    [215] = 'GOPRO_HEARTBEAT',
    [216] = 'GOPRO_GET_REQUEST',
    [217] = 'GOPRO_GET_RESPONSE',
    [218] = 'GOPRO_SET_REQUEST',
    [219] = 'GOPRO_SET_RESPONSE',
    [220] = 'NAV_FILTER_BIAS',
    [221] = 'RADIO_CALIBRATION',
    [222] = 'UALBERTA_SYS_STATUS',
    [226] = 'RPM',
    [230] = 'ESTIMATOR_STATUS',
    [231] = 'WIND_COV',
    [232] = 'GPS_INPUT',
    [233] = 'GPS_RTCM_DATA',
    [234] = 'HIGH_LATENCY',
    [235] = 'HIGH_LATENCY2',
    [241] = 'VIBRATION',
    [242] = 'HOME_POSITION',
    [243] = 'SET_HOME_POSITION',
    [244] = 'MESSAGE_INTERVAL',
    [245] = 'EXTENDED_SYS_STATE',
    [246] = 'ADSB_VEHICLE',
    [247] = 'COLLISION',
    [248] = 'V2_EXTENSION',
    [249] = 'MEMORY_VECT',
    [250] = 'DEBUG_VECT',
    [251] = 'NAMED_VALUE_FLOAT',
    [252] = 'NAMED_VALUE_INT',
    [253] = 'STATUSTEXT',
    [254] = 'DEBUG',
    [256] = 'SETUP_SIGNING',
    [257] = 'BUTTON_CHANGE',
    [258] = 'PLAY_TUNE',
    [259] = 'CAMERA_INFORMATION',
    [260] = 'CAMERA_SETTINGS',
    [261] = 'STORAGE_INFORMATION',
    [262] = 'CAMERA_CAPTURE_STATUS',
    [263] = 'CAMERA_IMAGE_CAPTURED',
    [264] = 'FLIGHT_INFORMATION',
    [265] = 'MOUNT_ORIENTATION',
    [266] = 'LOGGING_DATA',
    [267] = 'LOGGING_DATA_ACKED',
    [268] = 'LOGGING_ACK',
    [269] = 'VIDEO_STREAM_INFORMATION',
    [270] = 'SET_VIDEO_STREAM_SETTINGS',
    [299] = 'WIFI_CONFIG_AP',
    [300] = 'PROTOCOL_VERSION',
    [310] = 'UAVCAN_NODE_STATUS',
    [311] = 'UAVCAN_NODE_INFO',
    [320] = 'PARAM_EXT_REQUEST_READ',
    [321] = 'PARAM_EXT_REQUEST_LIST',
    [322] = 'PARAM_EXT_VALUE',
    [323] = 'PARAM_EXT_SET',
    [324] = 'PARAM_EXT_ACK',
    [330] = 'OBSTACLE_DISTANCE',
    [10001] = 'UAVIONIX_ADSB_OUT_CFG',
    [10002] = 'UAVIONIX_ADSB_OUT_DYNAMIC',
    [10003] = 'UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT',
    [11000] = 'DEVICE_OP_READ',
    [11001] = 'DEVICE_OP_READ_REPLY',
    [11002] = 'DEVICE_OP_WRITE',
    [11003] = 'DEVICE_OP_WRITE_REPLY',
    [11010] = 'ADAP_TUNING',
    [11011] = 'VISION_POSITION_DELTA',
    [11020] = 'AOA_SSA',
    [42000] = 'ICAROUS_HEARTBEAT',
    [42001] = 'ICAROUS_KINEMATIC_BANDS',
}

-- MAVLink autopilot
mavlinkAutopilot = {
    [0] = 'MAV_AUTOPILOT_GENERIC',
    [1] = 'MAV_AUTOPILOT_RESERVED',
    [2] = 'MAV_AUTOPILOT_SLUGS',
    [3] = 'MAV_AUTOPILOT_ARDUPILOTMEGA',
    [4] = 'MAV_AUTOPILOT_OPENPILOT',
    [5] = 'MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY',
    [6] = 'MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY',
    [7] = 'MAV_AUTOPILOT_GENERIC_MISSION_FULL',
    [8] = 'MAV_AUTOPILOT_INVALID',
    [9] = 'MAV_AUTOPILOT_PPZ',
    [10] = 'MAV_AUTOPILOT_UDB',
    [11] = 'MAV_AUTOPILOT_FP',
    [12] = 'MAV_AUTOPILOT_PX4',
    [13] = 'MAV_AUTOPILOT_SMACCMPILOT',
    [14] = 'MAV_AUTOPILOT_AUTOQUAD',
    [15] = 'MAV_AUTOPILOT_ARMAZILA',
    [16] = 'MAV_AUTOPILOT_AEROB',
    [17] = 'MAV_AUTOPILOT_ASLUAV',
    [18] = 'MAV_AUTOPILOT_SMARTAP',
    [19] = 'MAV_AUTOPILOT_AIRRAILS',
}

-- MAVLink frames
mavlinkFrame = {
    [0] = "MAV_FRAME_GLOBAL",
    [1] = "MAV_FRAME_LOCAL_NED",
    [2] = "MAV_FRAME_MISSION",
    [3] = "MAV_FRAME_GLOBAL_RELATIVE_ALT",
    [4] = "MAV_FRAME_LOCAL_ENU",
    [5] = "MAV_FRAME_GLOBAL_INT",
    [6] = "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT",
    [7] = "MAV_FRAME_LOCAL_OFFSET_NED",
    [8] = "MAV_FRAME_BODY_NED",
    [9] = "MAV_FRAME_BODY_OFFSET_NED",
    [10] = "MAV_FRAME_GLOBAL_TERRAIN_ALT",
    [11] = "MAV_FRAME_GLOBAL_TERRAIN_ALT_INT",
    [12] = "MAV_FRAME_BODY_FRD",
    [13] = "MAV_FRAME_BODY_FLU",
    [14] = "MAV_FRAME_MOCAP_NED",
    [15] = "MAV_FRAME_MOCAP_ENU",
    [16] = "MAV_FRAME_VISION_NED",
    [17] = "MAV_FRAME_VISION_ENU",
    [18] = "MAV_FRAME_ESTIM_NED",
    [19] = "MAV_FRAME_ESTIM_ENU",
}

-- MAVLink battery type
mavlinkBatteryType = {
    [0] = 'MAV_BATTERY_TYPE_UNKNOWN',
    [1] = 'MAV_BATTERY_TYPE_LIPO',
    [2] = 'MAV_BATTERY_TYPE_LIFE',
    [3] = 'MAV_BATTERY_TYPE_LION',
    [4] = 'MAV_BATTERY_TYPE_NIMH',
}

-- MAVLink battery function
mavlinkBatteryFunction = {
    [0] = 'MAV_BATTERY_FUNCTION_UNKNOWN',
    [1] = 'MAV_BATTERY_FUNCTION_ALL',
    [2] = 'MAV_BATTERY_FUNCTION_PROPULSION',
    [3] = 'MAV_BATTERY_FUNCTION_AVIONICS',
}

-- MAVLink battery charge state
mavlinkBatteryChargeState = {
    [0] = 'MAV_BATTERY_CHARGE_STATE_UNDEFINED',
    [1] = 'MAV_BATTERY_CHARGE_STATE_OK',
    [2] = 'MAV_BATTERY_CHARGE_STATE_LOW',
    [3] = 'MAV_BATTERY_CHARGE_STATE_CRITICAL',
    [4] = 'MAV_BATTERY_CHARGE_STATE_EMERGENCY',
    [5] = 'MAV_BATTERY_CHARGE_STATE_FAILED',
    [6] = 'MAV_BATTERY_CHARGE_STATE_UNHEALTHY',
}

-- MAVLink types
mavlinkType = {
    [0] = 'MAV_TYPE_GENERIC',
    [1] = 'MAV_TYPE_FIXED_WING',
    [2] = 'MAV_TYPE_QUADROTOR',
    [3] = 'MAV_TYPE_COAXIAL',
    [4] = 'MAV_TYPE_HELICOPTER',
    [5] = 'MAV_TYPE_ANTENNA_TRACKER',
    [6] = 'MAV_TYPE_GCS',
    [7] = 'MAV_TYPE_AIRSHIP',
    [8] = 'MAV_TYPE_FREE_BALLOON',
    [9] = 'MAV_TYPE_ROCKET',
    [10] = 'MAV_TYPE_GROUND_ROVER',
    [11] = 'MAV_TYPE_SURFACE_BOAT',
    [12] = 'MAV_TYPE_SUBMARINE',
    [13] = 'MAV_TYPE_HEXAROTOR',
    [14] = 'MAV_TYPE_OCTOROTOR',
    [15] = 'MAV_TYPE_TRICOPTER',
    [16] = 'MAV_TYPE_FLAPPING_WING',
    [17] = 'MAV_TYPE_KITE',
    [18] = 'MAV_TYPE_ONBOARD_CONTROLLER',
    [19] = 'MAV_TYPE_VTOL_DUOROTOR',
    [20] = 'MAV_TYPE_VTOL_QUADROTOR',
    [21] = 'MAV_TYPE_VTOL_TILTROTOR',
    [22] = 'MAV_TYPE_VTOL_RESERVED2',
    [23] = 'MAV_TYPE_VTOL_RESERVED3',
    [24] = 'MAV_TYPE_VTOL_RESERVED4',
    [25] = 'MAV_TYPE_VTOL_RESERVED5',
    [26] = 'MAV_TYPE_GIMBAL',
    [27] = 'MAV_TYPE_ADSB',
    [28] = 'MAV_TYPE_PARAFOIL',
    [29] = 'MAV_TYPE_DODECAROTOR',
    [30] = 'MAV_TYPE_CAMERA',
    [31] = 'MAV_TYPE_CHARGING_STATION',
}

-- MAVLink mode
mavlinkMode = {
    [0] = 'MAV_MODE_PREFLIGHT',
    [80] = 'MAV_MODE_STABILIZE_DISARMED',
    [208] = 'MAV_MODE_STABILIZE_ARMED',
    [64] = 'MAV_MODE_MANUAL_DISARMED',
    [192] = 'MAV_MODE_MANUAL_ARMED',
    [88] = 'MAV_MODE_GUIDED_DISARMED',
    [216] = 'MAV_MODE_GUIDED_ARMED',
    [92] = 'MAV_MODE_AUTO_DISARMED',
    [220] = 'MAV_MODE_AUTO_ARMED',
    [66] = 'MAV_MODE_TEST_DISARMED',
    [194] = 'MAV_MODE_TEST_ARMED',
}

-- MAVLink state
mavlinkState = {
    [0] = 'MAV_STATE_UINIT',
    [1] = 'MAV_STATE_BOOT',
    [2] = 'MAV_STATE_CALIBRATING',
    [3] = 'MAV_STATE_STANDBY',
    [4] = 'MAV_STATE_ACTIVE',
    [5] = 'MAV_STATE_CRITICAL',
    [6] = 'MAV_STATE_EMERGENCY',
    [7] = 'MAV_STATE_POWEROFF',
    [8] = 'MAV_STATE_FLIGHT_TERMINATION',
}

-- MAVLink command
mavlinkCommand = {
    [16] = 'MAV_CMD_NAV_WAYPOINT',
    [17] = 'MAV_CMD_NAV_LOITER_UNLIM',
    [18] = 'MAV_CMD_NAV_LOITER_TURNS',
    [19] = 'MAV_CMD_NAV_LOITER_TIME',
    [20] = 'MAV_CMD_NAV_RETURN_TO_LAUNCH',
    [21] = 'MAV_CMD_NAV_LAND',
    [22] = 'MAV_CMD_NAV_TAKEOFF',
    [23] = 'MAV_CMD_NAV_LAND_LOCAL',
    [24] = 'MAV_CMD_NAV_TAKEOFF_LOCAL',
    [25] = 'MAV_CMD_NAV_FOLLOW',
    [30] = 'MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT',
    [31] = 'MAV_CMD_NAV_LOITER_TO_ALT',
    [32] = 'MAV_CMD_DO_FOLLOW',
    [33] = 'MAV_CMD_DO_FOLLOW_REPOSITION',
    [80] = 'MAV_CMD_NAV_ROI',
    [81] = 'MAV_CMD_NAV_PATHPLANNING',
    [82] = 'MAV_CMD_NAV_SPLINE_WAYPOINT',
    [84] = 'MAV_CMD_NAV_VTOL_TAKEOFF',
    [85] = 'MAV_CMD_NAV_VTOL_LAND',
    [92] = 'MAV_CMD_NAV_GUIDED_ENABLE',
    [93] = 'MAV_CMD_NAV_DELAY',
    [94] = 'MAV_CMD_NAV_PAYLOAD_PLACE',
    [95] = 'MAV_CMD_NAV_LAST',
    [112] = 'MAV_CMD_CONDITION_DELAY',
    [113] = 'MAV_CMD_CONDITION_CHANGE_ALT',
    [114] = 'MAV_CMD_CONDITION_DISTANCE',
    [115] = 'MAV_CMD_CONDITION_YAW',
    [159] = 'MAV_CMD_CONDITION_LAST',
    [176] = 'MAV_CMD_DO_SET_MODE',
    [177] = 'MAV_CMD_DO_JUMP',
    [178] = 'MAV_CMD_DO_CHANGE_SPEED',
    [179] = 'MAV_CMD_DO_SET_HOME',
    [180] = 'MAV_CMD_DO_SET_PARAMETER',
    [181] = 'MAV_CMD_DO_SET_RELAY',
    [182] = 'MAV_CMD_DO_REPEAT_RELAY',
    [183] = 'MAV_CMD_DO_SET_SERVO',
    [184] = 'MAV_CMD_DO_REPEAT_SERVO',
    [185] = 'MAV_CMD_DO_FLIGHTTERMINATION',
    [186] = 'MAV_CMD_DO_CHANGE_ALTITUDE',
    [189] = 'MAV_CMD_DO_LAND_START',
    [190] = 'MAV_CMD_DO_RALLY_LAND',
    [191] = 'MAV_CMD_DO_GO_AROUND',
    [192] = 'MAV_CMD_DO_REPOSITION',
    [193] = 'MAV_CMD_DO_PAUSE_CONTINUE',
    [194] = 'MAV_CMD_DO_SET_REVERSE',
    [195] = 'MAV_CMD_DO_SET_ROI_LOCATION',
    [196] = 'MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET',
    [197] = 'MAV_CMD_DO_SET_ROI_NONE',
    [200] = 'MAV_CMD_DO_CONTROL_VIDEO',
    [201] = 'MAV_CMD_DO_SET_ROI',
    [202] = 'MAV_CMD_DO_DIGICAM_CONFIGURE',
    [203] = 'MAV_CMD_DO_DIGICAM_CONTROL',
    [204] = 'MAV_CMD_DO_MOUNT_CONFIGURE',
    [205] = 'MAV_CMD_DO_MOUNT_CONTROL',
    [206] = 'MAV_CMD_DO_SET_CAM_TRIGG_DIST',
    [207] = 'MAV_CMD_DO_FENCE_ENABLE',
    [208] = 'MAV_CMD_DO_PARACHUTE',
    [209] = 'MAV_CMD_DO_MOTOR_TEST',
    [210] = 'MAV_CMD_DO_INVERTED_FLIGHT',
    [213] = 'MAV_CMD_NAV_SET_YAW_SPEED',
    [214] = 'MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL',
    [220] = 'MAV_CMD_DO_MOUNT_CONTROL_QUAT',
    [221] = 'MAV_CMD_DO_GUIDED_MASTER',
    [222] = 'MAV_CMD_DO_GUIDED_LIMITS',
    [223] = 'MAV_CMD_DO_ENGINE_CONTROL',
    [240] = 'MAV_CMD_DO_LAST',
    [241] = 'MAV_CMD_PREFLIGHT_CALIBRATION',
    [242] = 'MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS',
    [243] = 'MAV_CMD_PREFLIGHT_UAVCAN',
    [245] = 'MAV_CMD_PREFLIGHT_STORAGE',
    [246] = 'MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN',
    [252] = 'MAV_CMD_OVERRIDE_GOTO',
    [300] = 'MAV_CMD_MISSION_START',
    [400] = 'MAV_CMD_COMPONENT_ARM_DISARM',
    [410] = 'MAV_CMD_GET_HOME_POSITION',
    [500] = 'MAV_CMD_START_RX_PAIR',
    [510] = 'MAV_CMD_GET_MESSAGE_INTERVAL',
    [511] = 'MAV_CMD_SET_MESSAGE_INTERVAL',
    [519] = 'MAV_CMD_REQUEST_PROTOCOL_VERSION',
    [520] = 'MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES',
    [521] = 'MAV_CMD_REQUEST_CAMERA_INFORMATION',
    [522] = 'MAV_CMD_REQUEST_CAMERA_SETTINGS',
    [525] = 'MAV_CMD_REQUEST_STORAGE_INFORMATION',
    [526] = 'MAV_CMD_STORAGE_FORMAT',
    [527] = 'MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS',
    [528] = 'MAV_CMD_REQUEST_FLIGHT_INFORMATION',
    [529] = 'MAV_CMD_RESET_CAMERA_SETTINGS',
    [530] = 'MAV_CMD_SET_CAMERA_MODE',
    [2000] = 'MAV_CMD_IMAGE_START_CAPTURE',
    [2001] = 'MAV_CMD_IMAGE_STOP_CAPTURE',
    [2002] = 'MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE',
    [2003] = 'MAV_CMD_DO_TRIGGER_CONTROL',
    [2500] = 'MAV_CMD_VIDEO_START_CAPTURE',
    [2501] = 'MAV_CMD_VIDEO_STOP_CAPTURE',
    [2502] = 'MAV_CMD_VIDEO_START_STREAMING',
    [2503] = 'MAV_CMD_VIDEO_STOP_STREAMING',
    [2504] = 'MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION',
    [2510] = 'MAV_CMD_LOGGING_START',
    [2511] = 'MAV_CMD_LOGGING_STOP',
    [2520] = 'MAV_CMD_AIRFRAME_CONFIGURATION',
    [2600] = 'MAV_CMD_CONTROL_HIGH_LATENCY',
    [2800] = 'MAV_CMD_PANORAMA_CREATE',
    [3000] = 'MAV_CMD_DO_VTOL_TRANSITION',
    [4000] = 'MAV_CMD_SET_GUIDED_SUBMODE_STANDARD',
    [4001] = 'MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE',
    [4501] = 'MAV_CMD_CONDITION_GATE',
    [3001] = 'MAV_CMD_ARM_AUTHORIZATION_REQUEST',
    [5000] = 'MAV_CMD_NAV_FENCE_RETURN_POINT',
    [5001] = 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION',
    [5002] = 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION',
    [5003] = 'MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION',
    [5004] = 'MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION',
    [5100] = 'MAV_CMD_NAV_RALLY_POINT',
    [5200] = 'MAV_CMD_UAVCAN_GET_NODE_INFO',
    [30001] = 'MAV_CMD_PAYLOAD_PREPARE_DEPLOY',
    [30002] = 'MAV_CMD_PAYLOAD_CONTROL_DEPLOY',
    [31000] = 'MAV_CMD_WAYPOINT_USER_1',
    [31001] = 'MAV_CMD_WAYPOINT_USER_2',
    [31002] = 'MAV_CMD_WAYPOINT_USER_3',
    [31003] = 'MAV_CMD_WAYPOINT_USER_4',
    [31004] = 'MAV_CMD_WAYPOINT_USER_5',
    [31005] = 'MAV_CMD_SPATIAL_USER_1',
    [31006] = 'MAV_CMD_SPATIAL_USER_2',
    [31007] = 'MAV_CMD_SPATIAL_USER_3',
    [31008] = 'MAV_CMD_SPATIAL_USER_4',
    [31009] = 'MAV_CMD_SPATIAL_USER_5',
    [31010] = 'MAV_CMD_USER_1',
    [31011] = 'MAV_CMD_USER_2',
    [31012] = 'MAV_CMD_USER_3',
    [31013] = 'MAV_CMD_USER_4',
    [31014] = 'MAV_CMD_USER_5',
}

-- MAVLink Severity
mavlinkSeverity = {
    [0] = 'MAV_SEVERITY_EMERGENCY',
    [1] = 'MAV_SEVERITY_ALERT',
    [2] = 'MAV_SEVERITY_CRITICAL',
    [3] = 'MAV_SEVERITY_ERROR',
    [4] = 'MAV_SEVERITY_WARNING',
    [5] = 'MAV_SEVERITY_NOTICE',
    [6] = 'MAV_SEVERITY_INFO',
    [7] = 'MAV_SEVERITY_DEBUG',
}

-- MAVLink GPS fix type
gpsFixType = {
    [0] = 'GPS_FIX_TYPE_NO_GPS',
    [1] = 'GPS_FIX_TYPE_NO_FIX',
    [2] = 'GPS_FIX_TYPE_2D_FIX',
    [3] = 'GPS_FIX_TYPE_3D_FIX',
    [4] = 'GPS_FIX_TYPE_DGPS',
    [5] = 'GPS_FIX_TYPE_RTK_FLOAT',
    [6] = 'GPS_FIX_TYPE_RTK_FIXED',
    [7] = 'GPS_FIX_TYPE_STATIC',
    [8] = 'GPS_FIX_TYPE_PPP',
}

-- MAVLink component ID
mavlinkCompId = {
    [0] = 'MAV_COMP_ID_ALL',
    [1] = 'MAV_COMP_ID_AUTOPILOT1',
    [100] = 'MAV_COMP_ID_CAMERA',
    [101] = 'MAV_COMP_ID_CAMERA2',
    [102] = 'MAV_COMP_ID_CAMERA3',
    [103] = 'MAV_COMP_ID_CAMERA4',
    [104] = 'MAV_COMP_ID_CAMERA5',
    [105] = 'MAV_COMP_ID_CAMERA6',
    [140] = 'MAV_COMP_ID_SERVO1',
    [141] = 'MAV_COMP_ID_SERVO2',
    [142] = 'MAV_COMP_ID_SERVO3',
    [143] = 'MAV_COMP_ID_SERVO4',
    [144] = 'MAV_COMP_ID_SERVO5',
    [145] = 'MAV_COMP_ID_SERVO6',
    [146] = 'MAV_COMP_ID_SERVO7',
    [147] = 'MAV_COMP_ID_SERVO8',
    [148] = 'MAV_COMP_ID_SERVO9',
    [149] = 'MAV_COMP_ID_SERVO10',
    [150] = 'MAV_COMP_ID_SERVO11',
    [151] = 'MAV_COMP_ID_SERVO12',
    [152] = 'MAV_COMP_ID_SERVO13',
    [153] = 'MAV_COMP_ID_SERVO14',
    [154] = 'MAV_COMP_ID_GIMBAL',
    [155] = 'MAV_COMP_ID_LOG',
    [156] = 'MAV_COMP_ID_ADSB',
    [157] = 'MAV_COMP_ID_OSD',
    [158] = 'MAV_COMP_ID_PERIPHERAL',
    [159] = 'MAV_COMP_ID_QX1_GIMBAL',
    [180] = 'MAV_COMP_ID_MAPPER',
    [190] = 'MAV_COMP_ID_MISSIONPLANNER',
    [195] = 'MAV_COMP_ID_PATHPLANNER',
    [200] = 'MAV_COMP_ID_IMU',
    [201] = 'MAV_COMP_ID_IMU_2',
    [202] = 'MAV_COMP_ID_IMU_3',
    [220] = 'MAV_COMP_ID_GPS',
    [221] = 'MAV_COMP_ID_GPS2',
    [240] = 'MAV_COMP_ID_UDP_BRIDGE',
    [241] = 'MAV_COMP_ID_UART_BRIDGE',
    [250] = 'MAV_COMP_ID_SYSTEM_CONTROL',
}

-- MAVLink VTOL state
mavlinkVTOLState = {
    [0] = 'MAV_VTOL_STATE_UNDEFINED',
    [1] = 'MAV_VTOL_STATE_TRANSITION_TO_FW',
    [2] = 'MAV_VTOL_STATE_TRANSITION_TO_MC',
    [3] = 'MAV_VTOL_STATE_MC',
    [4] = 'MAV_VTOL_STATE_FW',
}

-- MAVLink Landed state
mavlinkLandedState = {
    [0] = 'MAV_LANDED_STATE_UNDEFINED',
    [1] = 'MAV_LANDED_STATE_ON_GROUND',
    [2] = 'MAV_LANDED_STATE_IN_AIR',
    [3] = 'MAV_LANDED_STATE_TAKEOFF',
    [4] = 'MAV_LANDED_STATE_LANDING',
}

-- MAVLink param type
mavlinkParamType = {
    [1] = 'MAV_PARAM_TYPE_UINT8',
    [2] = 'MAV_PARAM_TYPE_INT8',
    [3] = 'MAV_PARAM_TYPE_UINT16',
    [4] = 'MAV_PARAM_TYPE_INT16',
    [5] = 'MAV_PARAM_TYPE_UINT32',
    [6] = 'MAV_PARAM_TYPE_INT32',
    [7] = 'MAV_PARAM_TYPE_UINT64',
    [8] = 'MAV_PARAM_TYPE_INT64',
    [9] = 'MAV_PARAM_TYPE_REAL32',
    [10] = 'MAV_PARAM_TYPE_REAL64',
}

-- MAVLink mission type
mavlinkMissionType = {
    [0] = 'MAV_MISSION_TYPE_MISSION',
    [1] = 'MAV_MISSION_TYPE_FENCE',
    [2] = 'MAV_MISSION_TYPE_RALLY',
    [255] = 'MAV_MISSION_TYPE_ALL',
}

-- MAVLink mission results
mavlinkMissionResult = {
    [0] = 'MAV_MISSION_ACCEPTED',
    [1] = 'MAV_MISSION_ERROR',
    [2] = 'MAV_MISSION_UNSUPPORTED_FRAME',
    [3] = 'MAV_MISSION_UNSUPPORTED',
    [4] = 'MAV_MISSION_NO_SPACE',
    [5] = 'MAV_MISSION_INVALID',
    [6] = 'MAV_MISSION_INVALID_PARAM1',
    [7] = 'MAV_MISSION_INVALID_PARAM2',
    [8] = 'MAV_MISSION_INVALID_PARAM3',
    [9] = 'MAV_MISSION_INVALID_PARAM4',
    [10] = 'MAV_MISSION_INVALID_PARAM5_X',
    [11] = 'MAV_MISSION_INVALID_PARAM6_Y',
    [12] = 'MAV_MISSION_INVALID_PARAM7',
    [13] = 'MAV_MISSION_INVALID_SEQUENCE',
    [14] = 'MAV_MISSION_DENIED',
}

-- MAVLink command results
mavlinkResults = {
    [0] = 'MAV_RESULT_ACCEPTED',
    [1] = 'MAV_RESULT_TEMPORARILY_REJECTED',
    [2] = 'MAV_RESULT_DENIED',
    [3] = 'MAV_RESULT_UNSUPPORTED',
    [4] = 'MAV_RESULT_FAILED',
    [5] = 'MAV_RESULT_IN_PROGRESS',
}

-- MAVLink protocol versions
mavlinkProtoVersions = {
    [0xfd] = 'MAVLink v2.0',
    [0xfe] = 'MAVLink v1.0',
    [0x55] = 'MAVLink v0.9'
}

-- Payload dissector functions
payload_fns = {}

-- Definition of each message
fields = mavlink_proto.fields
fields.magic = ProtoField.uint8("mavlink.magic", "Version", base.HEX, mavlinkProtoVersions)
fields.length = ProtoField.uint8("mavlink.length", "Payload length", base.DEC)
fields.sequence = ProtoField.uint8("mavlink.sequence", "Packet sequence", base.DEC)
fields.sysid = ProtoField.uint8("mavlink.sysid", "System ID", base.HEX)
fields.compid = ProtoField.uint8("mavlink.compid", "Component ID", base.HEX, mavlinkCompId)
fields.msgid = ProtoField.uint24("mavlink.msgid", "Message ID", base.DEC_HEX, messageName)
fields.payload = ProtoField.uint24("mavlink.payload", "Message", base.DEC, messageName)
fields.crc = ProtoField.uint16("mavlink.crc", "Message CRC", base.DEC_HEX)
-- MAVLink 2.0 fields
fields.incompat_flags = ProtoField.uint8("mavlink.incompat_flags", "Required flags", base.HEX)
fields.compat_flags = ProtoField.uint8("mavlink.compat_flags", "Optional flags", base.HEX)
-- Raw values on error
fields.rawheader = ProtoField.bytes("mavlink.rawheader", "Unparseable header")
fields.rawpayload = ProtoField.bytes("mavlink.rawpayload", "Unparseable payload")

-- Specific fields for each message type

-- Fields for message type 0: HEARTBEAT
fields.HEARTBEAT_type = ProtoField.uint8("mavlink.heartbeat.type", "Type", base.DEC, mavlinkType)
fields.HEARTBEAT_autopilot = ProtoField.uint8("mavlink.heartbeat.autopilot", "Autopilot", base.DEC, mavlinkAutopilot)
fields.HEARTBEAT_base_mode = ProtoField.uint8("mavlink.heartbeat.base_mode", "Base Mode", base.HEX)
fields.HEARTBEAT_base_mode_custom = ProtoField.uint8("mavlink.heartbeat.base_mode.custom", "Custom", base.DEC, nil, 0x01)
fields.HEARTBEAT_base_mode_test = ProtoField.uint8("mavlink.heartbeat.base_mode.test", "Test", base.DEC, nil, 0x02)
fields.HEARTBEAT_base_mode_auto = ProtoField.uint8("mavlink.heartbeat.base_mode.auto", "Auto", base.DEC, nil, 0x04)
fields.HEARTBEAT_base_mode_guided = ProtoField.uint8("mavlink.heartbeat.base_mode.guided", "Guided", base.DEC, nil, 0x08)
fields.HEARTBEAT_base_mode_stabilize = ProtoField.uint8("mavlink.heartbeat.base_mode.stabilize", "Stabilize", base.DEC, nil, 0x10)
fields.HEARTBEAT_base_mode_hil = ProtoField.uint8("mavlink.heartbeat.base_mode.hil", "HIL", base.DEC, nil, 0x20)
fields.HEARTBEAT_base_mode_manual_input = ProtoField.uint8("mavlink.heartbeat.base_mode.manual_input", "Manual input", base.DEC, nil, 0x40)
fields.HEARTBEAT_base_mode_safety_armed = ProtoField.uint8("mavlink.heartbeat.base_mode.safety_armed", "Safety armed", base.DEC, nil, 0x80)
fields.HEARTBEAT_custom_mode = ProtoField.uint32("mavlink.heartbeat.custom_mode", "Custom mode", base.DEC_HEX)
fields.HEARTBEAT_system_status = ProtoField.uint8("mavlink.heartbeat.system_status", "System status", base.DEC, mavlinkState)
fields.HEARTBEAT_mavlink_version = ProtoField.uint8("mavlink.heartbeat.mavlink_version", "MAVLink version", base.DEC)

-- Fields for message type 1: SYS_STATUS
fields.SYS_STATUS_control_sensors_present = ProtoField.uint32("mavlink.sys_status.control_sensors_present", "Sensors present", base.HEX, nil, 0xffffffff)
fields.SYS_STATUS_control_sensors_present_3d_gyro = ProtoField.uint32("mavlink.sys_status.control_sensors_present.3d_gyro", "3d_gyro", base.DEC, nil, 0x00000001)
fields.SYS_STATUS_control_sensors_present_3d_accel = ProtoField.uint32("mavlink.sys_status.control_sensors_present.3d_accel", "3d_accel", base.DEC, nil, 0x00000002)
fields.SYS_STATUS_control_sensors_present_3d_mag = ProtoField.uint32("mavlink.sys_status.control_sensors_present.3d_mag", "3d_mag", base.DEC, nil, 0x00000004)
fields.SYS_STATUS_control_sensors_present_absolute_pressure = ProtoField.uint32("mavlink.sys_status.control_sensors_present.absolute_pressure", "absolute_pressure", base.DEC, nil, 0x00000008)
fields.SYS_STATUS_control_sensors_present_differential_pressure = ProtoField.uint32("mavlink.sys_status.control_sensors_present.differential_pressure", "differential_pressure", base.DEC, nil, 0x00000010)
fields.SYS_STATUS_control_sensors_present_gps = ProtoField.uint32("mavlink.sys_status.control_sensors_present.gps", "gps", base.DEC, nil, 0x00000020)
fields.SYS_STATUS_control_sensors_present_optical_flow = ProtoField.uint32("mavlink.sys_status.control_sensors_present.optical_flow", "optical_flow", base.DEC, nil, 0x00000040)
fields.SYS_STATUS_control_sensors_present_vision_position = ProtoField.uint32("mavlink.sys_status.control_sensors_present.vision_position", "vision_position", base.DEC, nil, 0x00000080)
fields.SYS_STATUS_control_sensors_present_laser_position = ProtoField.uint32("mavlink.sys_status.control_sensors_present.laser_position", "laser_position", base.DEC, nil, 0x00000100)
fields.SYS_STATUS_control_sensors_present_external_ground_truth = ProtoField.uint32("mavlink.sys_status.control_sensors_present.external_ground_truth", "external_ground_truth", base.DEC, nil, 0x00000200)
fields.SYS_STATUS_control_sensors_present_angular_rate_control = ProtoField.uint32("mavlink.sys_status.control_sensors_present.angular_rate_control", "angular_rate_control", base.DEC, nil, 0x00000400)
fields.SYS_STATUS_control_sensors_present_attitude_stabilization = ProtoField.uint32("mavlink.sys_status.control_sensors_present.attitude_stabilization", "attitude_stabilization", base.DEC, nil, 0x00000800)
fields.SYS_STATUS_control_sensors_present_yaw_position = ProtoField.uint32("mavlink.sys_status.control_sensors_present.yaw_position", "yaw_position", base.DEC, nil, 0x00001000)
fields.SYS_STATUS_control_sensors_present_z_altitude_control = ProtoField.uint32("mavlink.sys_status.control_sensors_present.z_altitude_control", "z_altitude_control", base.DEC, nil, 0x00002000)
fields.SYS_STATUS_control_sensors_present_xy_position_control = ProtoField.uint32("mavlink.sys_status.control_sensors_present.xy_position_control", "xy_position_control", base.DEC, nil, 0x00004000)
fields.SYS_STATUS_control_sensors_present_motor_outputs = ProtoField.uint32("mavlink.sys_status.control_sensors_present.motor_outputs", "motor_outputs", base.DEC, nil, 0x00008000)
fields.SYS_STATUS_control_sensors_present_rc_receiver = ProtoField.uint32("mavlink.sys_status.control_sensors_present.rc_receiver", "rc_receiver", base.DEC, nil, 0x00010000)
fields.SYS_STATUS_control_sensors_present_3d_gyro2 = ProtoField.uint32("mavlink.sys_status.control_sensors_present.3d_gyro2", "3d_gyro2", base.DEC, nil, 0x00020000)
fields.SYS_STATUS_control_sensors_present_3d_accel2 = ProtoField.uint32("mavlink.sys_status.control_sensors_present.3d_accel2", "3d_accel2", base.DEC, nil, 0x00040000)
fields.SYS_STATUS_control_sensors_present_3d_mag2 = ProtoField.uint32("mavlink.sys_status.control_sensors_present.3d_mag2", "3d_mag2", base.DEC, nil, 0x00080000)
fields.SYS_STATUS_control_sensors_present_battery = ProtoField.uint32("mavlink.sys_status.control_sensors_present.battery", "battery", base.DEC, nil, 0x00100000)
fields.SYS_STATUS_control_sensors_enabled = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled", "Sensors enabled", base.HEX, nil, 0xffffffff)
fields.SYS_STATUS_control_sensors_enabled_3d_gyro = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.3d_gyro", "3d_gyro", base.DEC, nil, 0x00000001)
fields.SYS_STATUS_control_sensors_enabled_3d_accel = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.3d_accel", "3d_accel", base.DEC, nil, 0x00000002)
fields.SYS_STATUS_control_sensors_enabled_3d_mag = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.3d_mag", "3d_mag", base.DEC, nil, 0x00000004)
fields.SYS_STATUS_control_sensors_enabled_absolute_pressure = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.absolute_pressure", "absolute_pressure", base.DEC, nil, 0x00000008)
fields.SYS_STATUS_control_sensors_enabled_differential_pressure = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.differential_pressure", "differential_pressure", base.DEC, nil, 0x00000010)
fields.SYS_STATUS_control_sensors_enabled_gps = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.gps", "gps", base.DEC, nil, 0x00000020)
fields.SYS_STATUS_control_sensors_enabled_optical_flow = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.optical_flow", "optical_flow", base.DEC, nil, 0x00000040)
fields.SYS_STATUS_control_sensors_enabled_vision_position = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.vision_position", "vision_position", base.DEC, nil, 0x00000080)
fields.SYS_STATUS_control_sensors_enabled_laser_position = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.laser_position", "laser_position", base.DEC, nil, 0x00000100)
fields.SYS_STATUS_control_sensors_enabled_external_ground_truth = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.external_ground_truth", "external_ground_truth", base.DEC, nil, 0x00000200)
fields.SYS_STATUS_control_sensors_enabled_angular_rate_control = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.angular_rate_control", "angular_rate_control", base.DEC, nil, 0x00000400)
fields.SYS_STATUS_control_sensors_enabled_attitude_stabilization = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.attitude_stabilization", "attitude_stabilization", base.DEC, nil, 0x00000800)
fields.SYS_STATUS_control_sensors_enabled_yaw_position = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.yaw_position", "yaw_position", base.DEC, nil, 0x00001000)
fields.SYS_STATUS_control_sensors_enabled_z_altitude_control = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.z_altitude_control", "z_altitude_control", base.DEC, nil, 0x00002000)
fields.SYS_STATUS_control_sensors_enabled_xy_position_control = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.xy_position_control", "xy_position_control", base.DEC, nil, 0x00004000)
fields.SYS_STATUS_control_sensors_enabled_motor_outputs = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.motor_outputs", "motor_outputs", base.DEC, nil, 0x00008000)
fields.SYS_STATUS_control_sensors_enabled_rc_receiver = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.rc_receiver", "rc_receiver", base.DEC, nil, 0x00010000)
fields.SYS_STATUS_control_sensors_enabled_3d_gyro2 = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.3d_gyro2", "3d_gyro2", base.DEC, nil, 0x00020000)
fields.SYS_STATUS_control_sensors_enabled_3d_accel2 = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.3d_accel2", "3d_accel2", base.DEC, nil, 0x00040000)
fields.SYS_STATUS_control_sensors_enabled_3d_mag2 = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.3d_mag2", "3d_mag2", base.DEC, nil, 0x00080000)
fields.SYS_STATUS_control_sensors_enabled_battery = ProtoField.uint32("mavlink.sys_status.control_sensors_enabled.battery", "battery", base.DEC, nil, 0x00100000)
fields.SYS_STATUS_control_sensors_health = ProtoField.uint32("mavlink.sys_status.control_sensors_health", "Sensors health", base.HEX, nil, 0xffffffff)
fields.SYS_STATUS_control_sensors_health_3d_gyro = ProtoField.uint32("mavlink.sys_status.control_sensors_health.3d_gyro", "3d_gyro", base.DEC, nil, 0x00000001)
fields.SYS_STATUS_control_sensors_health_3d_accel = ProtoField.uint32("mavlink.sys_status.control_sensors_health.3d_accel", "3d_accel", base.DEC, nil, 0x00000002)
fields.SYS_STATUS_control_sensors_health_3d_mag = ProtoField.uint32("mavlink.sys_status.control_sensors_health.3d_mag", "3d_mag", base.DEC, nil, 0x00000004)
fields.SYS_STATUS_control_sensors_health_absolute_pressure = ProtoField.uint32("mavlink.sys_status.control_sensors_health.absolute_pressure", "absolute_pressure", base.DEC, nil, 0x00000008)
fields.SYS_STATUS_control_sensors_health_differential_pressure = ProtoField.uint32("mavlink.sys_status.control_sensors_health.differential_pressure", "differential_pressure", base.DEC, nil, 0x00000010)
fields.SYS_STATUS_control_sensors_health_gps = ProtoField.uint32("mavlink.sys_status.control_sensors_health.gps", "gps", base.DEC, nil, 0x00000020)
fields.SYS_STATUS_control_sensors_health_optical_flow = ProtoField.uint32("mavlink.sys_status.control_sensors_health.optical_flow", "optical_flow", base.DEC, nil, 0x00000040)
fields.SYS_STATUS_control_sensors_health_vision_position = ProtoField.uint32("mavlink.sys_status.control_sensors_health.vision_position", "vision_position", base.DEC, nil, 0x00000080)
fields.SYS_STATUS_control_sensors_health_laser_position = ProtoField.uint32("mavlink.sys_status.control_sensors_health.laser_position", "laser_position", base.DEC, nil, 0x00000100)
fields.SYS_STATUS_control_sensors_health_external_ground_truth = ProtoField.uint32("mavlink.sys_status.control_sensors_health.external_ground_truth", "external_ground_truth", base.DEC, nil, 0x00000200)
fields.SYS_STATUS_control_sensors_health_angular_rate_control = ProtoField.uint32("mavlink.sys_status.control_sensors_health.angular_rate_control", "angular_rate_control", base.DEC, nil, 0x00000400)
fields.SYS_STATUS_control_sensors_health_attitude_stabilization = ProtoField.uint32("mavlink.sys_status.control_sensors_health.attitude_stabilization", "attitude_stabilization", base.DEC, nil, 0x00000800)
fields.SYS_STATUS_control_sensors_health_yaw_position = ProtoField.uint32("mavlink.sys_status.control_sensors_health.yaw_position", "yaw_position", base.DEC, nil, 0x00001000)
fields.SYS_STATUS_control_sensors_health_z_altitude_control = ProtoField.uint32("mavlink.sys_status.control_sensors_health.z_altitude_control", "z_altitude_control", base.DEC, nil, 0x00002000)
fields.SYS_STATUS_control_sensors_health_xy_position_control = ProtoField.uint32("mavlink.sys_status.control_sensors_health.xy_position_control", "xy_position_control", base.DEC, nil, 0x00004000)
fields.SYS_STATUS_control_sensors_health_motor_outputs = ProtoField.uint32("mavlink.sys_status.control_sensors_health.motor_outputs", "motor_outputs", base.DEC, nil, 0x00008000)
fields.SYS_STATUS_control_sensors_health_rc_receiver = ProtoField.uint32("mavlink.sys_status.control_sensors_health.rc_receiver", "rc_receiver", base.DEC, nil, 0x00010000)
fields.SYS_STATUS_control_sensors_health_3d_gyro2 = ProtoField.uint32("mavlink.sys_status.control_sensors_health.3d_gyro2", "3d_gyro2", base.DEC, nil, 0x00020000)
fields.SYS_STATUS_control_sensors_health_3d_accel2 = ProtoField.uint32("mavlink.sys_status.control_sensors_health.3d_accel2", "3d_accel2", base.DEC, nil, 0x00040000)
fields.SYS_STATUS_control_sensors_health_3d_mag2 = ProtoField.uint32("mavlink.sys_status.control_sensors_health.3d_mag2", "3d_mag2", base.DEC, nil, 0x00080000)
fields.SYS_STATUS_control_sensors_health_battery = ProtoField.uint32("mavlink.sys_status.control_sensors_health.battery", "battery", base.DEC, nil, 0x00100000)
fields.SYS_STATUS_load = ProtoField.uint16("mavlink.sys_status.load", "Maximum usage in mainloop [% x10]", base.DEC)
fields.SYS_STATUS_voltage_battery = ProtoField.uint16("mavlink.sys_status.voltage_battery", "Battery voltage [mV]", base.DEC)
fields.SYS_STATUS_current_battery = ProtoField.int16("mavlink.sys_status.current_battery", "Battery current [mA x10]", base.DEC)
fields.SYS_STATUS_battery_remaining = ProtoField.int8("mavlink.sys_status.battery_remaining", "Remaining battery energy [%]", base.DEC)
fields.SYS_STATUS_drop_rate_comm = ProtoField.uint16("mavlink.sys_status.drop_rate_comm", "Communication drops [% x100]", base.DEC)
fields.SYS_STATUS_errors_comm = ProtoField.uint16("mavlink.sys_status.errors_comm", "Communication errors", base.DEC)
fields.SYS_STATUS_errors_count1 = ProtoField.uint16("mavlink.sys_status.errors_count1", "Autopilot-specific errors", base.DEC)
fields.SYS_STATUS_errors_count2 = ProtoField.uint16("mavlink.sys_status.errors_count2", "Autopilot-specific errors", base.DEC)
fields.SYS_STATUS_errors_count3 = ProtoField.uint16("mavlink.sys_status.errors_count3", "Autopilot-specific errors", base.DEC)
fields.SYS_STATUS_errors_count4 = ProtoField.uint16("mavlink.sys_status.errors_count4", "Autopilot-specific errors", base.DEC)

-- Fields for message type 2: SYSTEM_TIME
fields.SYSTEM_TIME_time_unix_usec = ProtoField.uint64("mavlink.system_time.time_unix_usec", "Epoch timestamp [us]", base.DEC)
fields.SYSTEM_TIME_boot_ms = ProtoField.uint32("mavlink.system_time.boot_ms", "Boot time [ms]", base.DEC)

-- Fields for message type 11: SET_MODE
fields.SET_MODE_target_system = ProtoField.uint8("mavlink.set_mode.target_system", "Target system", base.DEC_HEX)
fields.SET_MODE_base_mode = ProtoField.uint8("mavlink.set_mode.base_mode", "Base mode", base.DEC, mavlinkMode)
fields.SET_MODE_custom_mode = ProtoField.uint32("mavlink.set_mode.custom_mode", "Custom mode", base.DEC_HEX)

-- Fields for message type 20: PARAM_REQUEST_READ
fields.PARAM_REQUEST_READ_target_system = ProtoField.uint8("mavlink.param_request_read.target_system", "System ID", base.HEX)
fields.PARAM_REQUEST_READ_target_component = ProtoField.uint8("mavlink_proto.param_request_read.target_component", "Component ID", base.HEX, mavlinkCompId)
fields.PARAM_REQUEST_READ_param_id = ProtoField.string("mavlink.param_request_read.param_id", "Parameter ID", base.ASCII)
fields.PARAM_REQUEST_READ_param_index = ProtoField.int16("mavlink.param_request_read.param_index", "Parameter index", base.DEC)

-- Fields for message type 21: PARAM_REQUEST_LIST
fields.PARAM_REQUEST_LIST_target_system = ProtoField.uint8("mavlink_proto.param_request_list.target_system", "System ID", base.HEX)
fields.PARAM_REQUEST_LIST_target_component = ProtoField.uint8("mavlink_proto.param_request_list.target_component", "Component ID", base.HEX, mavlinkCompId)

-- Fields for message type 22: PARAM_VALUE
fields.PARAM_VALUE_param_id = ProtoField.string("mavlink.param_value.param_id", "Parameter ID", base.ASCII)
fields.PARAM_VALUE_param_value = ProtoField.float("mavlink.param_value.param_value", "Parameter value", base.DEC)
fields.PARAM_VALUE_param_type = ProtoField.uint8("mavlink.param_value.param_type", "Parameter type", base.DEC, mavlinkParamType)
fields.PARAM_VALUE_param_count = ProtoField.uint16("mavlink.param_value.param_count", "Total parameter count", base.DEC)
fields.PARAM_VALUE_param_index = ProtoField.uint16("mavlink.param_value.param_index", "Parameter index", base.DEC)

-- Fields for message type 23: PARAM_SET
fields.PARAM_SET_target_system = ProtoField.uint8("mavlink.param_set.target_system", "System ID", base.DEC_HEX)
fields.PARAM_SET_target_component = ProtoField.uint8("mavlink.param_set.target_component", "Component ID", base.DEC_HEX, mavlinkCompId)
fields.PARAM_SET_param_id = ProtoField.string("mavlink.param_set.param_id", "Parameter ID", base.ASCII)
fields.PARAM_SET_param_value = ProtoField.float("mavlink.param_set.param_value", "Parameter value", base.DEC)
fields.PARAM_SET_param_type = ProtoField.uint8("mavlink.param_set.param_type", "Param type", base.DEC, mavlinkParamType)

-- Fields for message type 28: RAW_PRESSURE
fields.RAW_PRESSURE_time_usec = ProtoField.uint64("mavlink.raw_pressure.time_usec", "Timestamp [us]", base.DEC)
fields.RAW_PRESSURE_press_abs = ProtoField.int16("mavlink.raw_pressure.press_abs", "Absolute pressure [raw ADC]", base.DEC)
fields.RAW_PRESSURE_press_diff1 = ProtoField.int16("mavlink.raw_pressure.press_diff1", "Differential pressure 1 [raw ADC]", base.DEC)
fields.RAW_PRESSURE_press_diff2 = ProtoField.int16("mavlink.raw_pressure.press_diff2", "Differential pressure 2 [raw ADC]", base.DEC)
fields.RAW_PRESSURE_temperature = ProtoField.int16("mavlink.raw_pressure.temperature", "Temperature [raw ADC]", base.DEC)

-- Fields for message type 30: ATTITUDE
fields.ATTITUDE_time_boot_ms = ProtoField.uint32("mavlink.attitude.time_boot_ms", "Uptime [ms]", base.DEC)
fields.ATTITUDE_roll = ProtoField.float("mavlink.attitude.roll", "Roll angle (-pi, pi) [rad]")
fields.ATTITUDE_pitch = ProtoField.float("mavlink.attitude.pitch", "Pitch angle (-pi, pi) [rad]")
fields.ATTITUDE_yaw = ProtoField.float("mavlink.attitude.yaw", "Yaw angle (-pi, pi) [rad]")
fields.ATTITUDE_rollspeed = ProtoField.float("mavlink.attitude.rollspeed", "Roll angular speed [rad/s]")
fields.ATTITUDE_pitchspeed = ProtoField.float("mavlink.attitude.pitchspeed", "Pitch angular speed [rad/s]")
fields.ATTITUDE_yawspeed = ProtoField.float("mavlink.attitude.yawspeed", "Yaw angular speed [rad/s]")

-- Fields for message type 31: ATTITUDE_QUATERNION
fields.ATTITUDE_QUATERNION_time_boot_ms = ProtoField.uint32("mavlink.attitude_quaternion.time_boot_ms", "Uptime [ms]", base.DEC)
fields.ATTITUDE_QUATERNION_q1 = ProtoField.float("mavlink.attitude_quaternion.q1", "W", base.DEC)
fields.ATTITUDE_QUATERNION_q2 = ProtoField.float("mavlink.attitude_quaternion.q2", "X", base.DEC)
fields.ATTITUDE_QUATERNION_q3 = ProtoField.float("mavlink.attitude_quaternion.q3", "Y", base.DEC)
fields.ATTITUDE_QUATERNION_q4 = ProtoField.float("mavlink.attitude_quaternion.q4", "Z", base.DEC)
fields.ATTITUDE_QUATERNION_rollspeed = ProtoField.float("mavlink.attitude_quaternion.rollspeed", "Roll angular speed [rad/s]", base.DEC)
fields.ATTITUDE_QUATERNION_pitchspeed = ProtoField.float("mavlink.attitude_quaternion.pitchspeed", "Pitch angular speed [rad/s]", base.DEC)
fields.ATTITUDE_QUATERNION_yawspeed = ProtoField.float("mavlink.attitude_quaternion.yawspeed", "Yaw angular speed [rad/s]", base.DEC)

-- Fields for message type 32: LOCAL_POSITION_NED
fields.LOCAL_POSITION_NED_time_boot_ms = ProtoField.uint32("mavlink.local_position_ned.time_boot_ms", "Uptime [ms]", base.DEC)
fields.LOCAL_POSITION_NED_x = ProtoField.float("mavlink.local_position_ned.x", "X Position [m]", base.DEC)
fields.LOCAL_POSITION_NED_y = ProtoField.float("mavlink.local_position_ned.y", "Y Position [m]", base.DEC)
fields.LOCAL_POSITION_NED_z = ProtoField.float("mavlink.local_position_ned.z", "Z Position [m]", base.DEC)
fields.LOCAL_POSITION_NED_vx = ProtoField.float("mavlink.local_position_ned.vx", "X Speed [m/s]", base.DEC)
fields.LOCAL_POSITION_NED_vy = ProtoField.float("mavlink.local_position_ned.vy", "Y Speed [m/s]", base.DEC)
fields.LOCAL_POSITION_NED_vz = ProtoField.float("mavlink.local_position_ned.vz", "Z Speed [m/s]", base.DEC)

-- Fields for message type 33: GLOBAL_POSITION_INT
fields.GLOBAL_POSITION_INT_time_boot_ms = ProtoField.uint32("mavlink.global_position_int.time_boot_ms", "Timestamp", base.DEC)
fields.GLOBAL_POSITION_INT_lat = ProtoField.int32("mavlink.global_position_int.lat", "Latitude [x10^7 deg]", base.DEC)
fields.GLOBAL_POSITION_INT_lon = ProtoField.int32("mavlink.global_position_int.lon", "Longitude [x10^7 deg]", base.DEC)
fields.GLOBAL_POSITION_INT_alt = ProtoField.int32("mavlink.global_position_int.alt", "Altitude [mm]", base.DEC)
fields.GLOBAL_POSITION_INT_relative_alt = ProtoField.int32("mavlink.global_position_int.relative_alt", "Relative altitude [mm]", base.DEC)
fields.GLOBAL_POSITION_INT_vx = ProtoField.int16("mavlink.global_position_int.vx", "Ground X speed [cm/s]", base.DEC)
fields.GLOBAL_POSITION_INT_vy = ProtoField.int16("mavlink.global_position_int.vy", "Ground Y speed [cm/s]", base.DEC)
fields.GLOBAL_POSITION_INT_vz = ProtoField.int16("mavlink.global_position_int.vz", "Ground Z speed [cm/s]", base.DEC)
fields.GLOBAL_POSITION_INT_hdg = ProtoField.uint16("mavlink.global_position_int.hdg", "Heading [x10^2 deg]", base.DEC)

-- Fields for message type 36: SERVO_OUTPUT_RAW
fields.SERVO_OUTPUT_RAW_time_usec = ProtoField.uint32("mavlink.servo_output_raw.time_usec", "Timestamp [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_port = ProtoField.uint8("mavlink.servo_output_raw.port", "Servo output port", base.DEC_HEX)
fields.SERVO_OUTPUT_RAW_servo1_raw = ProtoField.uint16("mavlink.servo_output_raw.servo1_raw", "Servo output 1 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo2_raw = ProtoField.uint16("mavlink.servo_output_raw.servo2_raw", "Servo output 2 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo3_raw = ProtoField.uint16("mavlink.servo_output_raw.servo3_raw", "Servo output 3 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo4_raw = ProtoField.uint16("mavlink.servo_output_raw.servo4_raw", "Servo output 4 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo5_raw = ProtoField.uint16("mavlink.servo_output_raw.servo5_raw", "Servo output 5 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo6_raw = ProtoField.uint16("mavlink.servo_output_raw.servo6_raw", "Servo output 6 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo7_raw = ProtoField.uint16("mavlink.servo_output_raw.servo7_raw", "Servo output 7 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo8_raw = ProtoField.uint16("mavlink.servo_output_raw.servo8_raw", "Servo output 8 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo9_raw = ProtoField.uint16("mavlink.servo_output_raw.servo9_raw", "Servo output 8 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo10_raw = ProtoField.uint16("mavlink.servo_output_raw.servo10_raw", "Servo output 8 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo11_raw = ProtoField.uint16("mavlink.servo_output_raw.servo11_raw", "Servo output 8 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo12_raw = ProtoField.uint16("mavlink.servo_output_raw.servo12_raw", "Servo output 8 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo13_raw = ProtoField.uint16("mavlink.servo_output_raw.servo13_raw", "Servo output 8 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo14_raw = ProtoField.uint16("mavlink.servo_output_raw.servo14_raw", "Servo output 8 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo15_raw = ProtoField.uint16("mavlink.servo_output_raw.servo15_raw", "Servo output 8 [us]", base.DEC)
fields.SERVO_OUTPUT_RAW_servo16_raw = ProtoField.uint16("mavlink.servo_output_raw.servo16_raw", "Servo output 8 [us]", base.DEC)

-- Fields for message type 39: MISSION_ITEM
fields.MISSION_ITEM_target_system = ProtoField.uint8("mavlink.mission_item.target_system", "System ID", base.DEC_HEX)
fields.MISSION_ITEM_target_component = ProtoField.uint8("mavlink.mission_item.target_component", "Component ID", base.DEC_HEX)
fields.MISSION_ITEM_seq = ProtoField.uint16("mavlink.mission_item.seq", "Sequence", base.DEC)
fields.MISSION_ITEM_frame = ProtoField.uint8("mavlink.mission_item.frame", "Coordinate system", base.DEC, mavlinkFrame)
fields.MISSION_ITEM_command = ProtoField.uint16("mavlink.mission_item.command", "Scheduled action", base.DEC, mavlinkCommand)
fields.MISSION_ITEM_current = ProtoField.uint8("mavlink.mission_item.current", "Current", base.DEC, nil, 0x01)
fields.MISSION_ITEM_autocontinue = ProtoField.uint8("mavlink.mission_item.autocontinue", "Autocontinue", base.DEC_HEX)
fields.MISSION_ITEM_param1 = ProtoField.float("mavlink.mission_item.param1", "PARAM1", base.DEC)
fields.MISSION_ITEM_param2 = ProtoField.float("mavlink.mission_item.param2", "PARAM2", base.DEC)
fields.MISSION_ITEM_param3 = ProtoField.float("mavlink.mission_item.param3", "PARAM3", base.DEC)
fields.MISSION_ITEM_param4 = ProtoField.float("mavlink.mission_item.param4", "PARAM4", base.DEC)
fields.MISSION_ITEM_x = ProtoField.float("mavlink.mission_item.x", "X position / Latitude", base.DEC)
fields.MISSION_ITEM_y = ProtoField.float("mavlink.mission_item.y", "Y position / Longitude", base.DEC)
fields.MISSION_ITEM_z = ProtoField.float("mavlink.mission_item.z", "Z position / Altitude", base.DEC)
fields.MISSION_ITEM_mission_type = ProtoField.uint8("mavlink.mission_item.mission_type", "Mission type", base.DEC, mavlinkMissionType)

-- Fields for message type 42: MISSION_CURRENT
fields.MISSION_CURRENT_seq = ProtoField.uint16("mavlink.mission_current.seq", "Sequence", base.DEC)

-- Fields for message type 43: MISSION_REQUEST_LIST
fields.MISSION_REQUEST_LIST_target_system = ProtoField.uint8("mavlink.mission_request_list.target_system", "System ID", base.DEC_HEX)
fields.MISSION_REQUEST_LIST_target_component = ProtoField.uint8("mavlink.mission_request_list.target_component", "Component ID", base.DEC_HEX, mavlinkCompId)
fields.MISSION_REQUEST_LIST_mission_type = ProtoField.uint8("mavlink.mission_request_list.mission_type", "Mission type", base.DEC, mavlinkMissionType)

-- Fields for message type 44: MISSION_COUNT
fields.MISSION_COUNT_target_system = ProtoField.uint8("mavlink.mission_count.target_system", "System ID", base.DEC_HEX)
fields.MISSION_COUNT_target_component = ProtoField.uint8("mavlink.mission_count.target_component", "Component ID", base.DEC_HEX)
fields.MISSION_COUNT_count = ProtoField.uint16("mavlink.mission_count.count", "Number of mission items", base.DEC)
fields.MISSION_COUNT_mission_type = ProtoField.uint8("mavlink.mission_count.mission_type", "Mission type", base.DEC, mavlinkMissionType)

-- fields for message type 45: MISSION_CLEAR_ALL
fields.MISSION_CLEAR_ALL_target_system = ProtoField.uint8("mavlink.mission_clear_all.target_system", "System ID", base.DEC_HEX)
fields.MISSION_CLEAR_ALL_target_component = ProtoField.uint8("mavlink.mission_clear_all.target_component", "Component ID", base.DEC_HEX)
fields.MISSION_CLEAR_ALL_mission_type = ProtoField.uint8("mavlink.mission_clear_all.mission_type", "Mission type", base.DEC, mavlinkMissionType)

-- Fields for message type 47: MISSION_ACK
fields.MISSION_ACK_target_system = ProtoField.uint8("mavlink.mission_ack.target_system", "System ID", base.DEC_HEX)
fields.MISSION_ACK_target_component = ProtoField.uint8("mavlink.mission_ack.target_component", "Component ID", base.DEC_HEX)
fields.MISSION_ACK_type = ProtoField.uint8("mavlink.mission_ack.type", "Mission result", base.DEC, mavlinkMissionResult)
fields.MISSION_ACK_mission_type = ProtoField.uint8("mavlink.mission_ack.mission_type", "Mission type", base.DEC, mavlinkMissionType)

-- Fields for message type 65: RC_CHANNELS
fields.RC_CHANNELS_time_boot_ms = ProtoField.uint32("mavlink.rc_channels.time_boot_ms", "Timestamp [ms]", base.DEC)
fields.RC_CHANNELS_chancount = ProtoField.uint8("mavlink.rc_channels.chancount", "Amount of RC channels", base.DEC)
fields.RC_CHANNELS_chan1_raw = ProtoField.uint16("mavlink.rc_channels.chan1_raw", "RC channel 1 [ms]", base.DEC)
fields.RC_CHANNELS_chan2_raw = ProtoField.uint16("mavlink.rc_channels.chan2_raw", "RC channel 2 [ms]", base.DEC)
fields.RC_CHANNELS_chan3_raw = ProtoField.uint16("mavlink.rc_channels.chan3_raw", "RC channel 3 [ms]", base.DEC)
fields.RC_CHANNELS_chan4_raw = ProtoField.uint16("mavlink.rc_channels.chan4_raw", "RC channel 4 [ms]", base.DEC)
fields.RC_CHANNELS_chan5_raw = ProtoField.uint16("mavlink.rc_channels.chan5_raw", "RC channel 5 [ms]", base.DEC)
fields.RC_CHANNELS_chan6_raw = ProtoField.uint16("mavlink.rc_channels.chan6_raw", "RC channel 6 [ms]", base.DEC)
fields.RC_CHANNELS_chan7_raw = ProtoField.uint16("mavlink.rc_channels.chan7_raw", "RC channel 7 [ms]", base.DEC)
fields.RC_CHANNELS_chan8_raw = ProtoField.uint16("mavlink.rc_channels.chan8_raw", "RC channel 8 [ms]", base.DEC)
fields.RC_CHANNELS_chan9_raw = ProtoField.uint16("mavlink.rc_channels.chan9_raw", "RC channel 9 [ms]", base.DEC)
fields.RC_CHANNELS_chan10_raw = ProtoField.uint16("mavlink.rc_channels.chan10_raw", "RC channel 10 [ms]", base.DEC)
fields.RC_CHANNELS_chan11_raw = ProtoField.uint16("mavlink.rc_channels.chan11_raw", "RC channel 11 [ms]", base.DEC)
fields.RC_CHANNELS_chan12_raw = ProtoField.uint16("mavlink.rc_channels.chan12_raw", "RC channel 12 [ms]", base.DEC)
fields.RC_CHANNELS_chan13_raw = ProtoField.uint16("mavlink.rc_channels.chan13_raw", "RC channel 13 [ms]", base.DEC)
fields.RC_CHANNELS_chan14_raw = ProtoField.uint16("mavlink.rc_channels.chan14_raw", "RC channel 14 [ms]", base.DEC)
fields.RC_CHANNELS_chan15_raw = ProtoField.uint16("mavlink.rc_channels.chan15_raw", "RC channel 15 [ms]", base.DEC)
fields.RC_CHANNELS_chan16_raw = ProtoField.uint16("mavlink.rc_channels.chan16_raw", "RC channel 16 [ms]", base.DEC)
fields.RC_CHANNELS_chan17_raw = ProtoField.uint16("mavlink.rc_channels.chan17_raw", "RC channel 17 [ms]", base.DEC)
fields.RC_CHANNELS_chan18_raw = ProtoField.uint16("mavlink.rc_channels.chan18_raw", "RC channel 18 [ms]", base.DEC)
fields.RC_CHANNELS_rssi = ProtoField.uint8("mavlink.rc_channels.rssi", "Receive signal strength indicator [%]", base.DEC)

-- Fields for message type 69: MANUAL_CONTROL
fields.MANUAL_CONTROL_target = ProtoField.uint8("mavlink.manual_control.target", "Target system", base.DEC)
fields.MANUAL_CONTROL_x = ProtoField.int16("mavlink.manual_control.x", "X-axis [-1000,1000]", base.DEC)
fields.MANUAL_CONTROL_y = ProtoField.int16("mavlink.manual_control.y", "Y-axis [-1000,1000]", base.DEC)
fields.MANUAL_CONTROL_z = ProtoField.int16("mavlink.manual_control.z", "Z-axis [-1000,1000]", base.DEC)
fields.MANUAL_CONTROL_r = ProtoField.int16("mavlink.manual_control.r", "R-axis [-1000,1000]", base.DEC)
fields.MANUAL_CONTROL_buttons = ProtoField.uint16("mavlink.manual_control.buttons", "Buttons bitfield", base.HEX)
fields.MANUAL_CONTROL_buttons_button_1 = ProtoField.uint16("mavlink.manual_control.buttons.button_1", "Button 1", base.DEC, nil, 0x0001)
fields.MANUAL_CONTROL_buttons_button_2 = ProtoField.uint16("mavlink.manual_control.buttons.button_2", "Button 2", base.DEC, nil, 0x0002)
fields.MANUAL_CONTROL_buttons_button_3 = ProtoField.uint16("mavlink.manual_control.buttons.button_3", "Button 3", base.DEC, nil, 0x0004)
fields.MANUAL_CONTROL_buttons_button_4 = ProtoField.uint16("mavlink.manual_control.buttons.button_4", "Button 4", base.DEC, nil, 0x0008)
fields.MANUAL_CONTROL_buttons_button_5 = ProtoField.uint16("mavlink.manual_control.buttons.button_5", "Button 5", base.DEC, nil, 0x0010)
fields.MANUAL_CONTROL_buttons_button_6 = ProtoField.uint16("mavlink.manual_control.buttons.button_6", "Button 6", base.DEC, nil, 0x0020)
fields.MANUAL_CONTROL_buttons_button_7 = ProtoField.uint16("mavlink.manual_control.buttons.button_7", "Button 7", base.DEC, nil, 0x0040)
fields.MANUAL_CONTROL_buttons_button_8 = ProtoField.uint16("mavlink.manual_control.buttons.button_8", "Button 8", base.DEC, nil, 0x0080)
fields.MANUAL_CONTROL_buttons_button_9 = ProtoField.uint16("mavlink.manual_control.buttons.button_9", "Button 9", base.DEC, nil, 0x0100)
fields.MANUAL_CONTROL_buttons_button_10 = ProtoField.uint16("mavlink.manual_control.buttons.button_10", "Button 10", base.DEC, nil, 0x0200)
fields.MANUAL_CONTROL_buttons_button_11 = ProtoField.uint16("mavlink.manual_control.buttons.button_11", "Button 11", base.DEC, nil, 0x0400)
fields.MANUAL_CONTROL_buttons_button_12 = ProtoField.uint16("mavlink.manual_control.buttons.button_12", "Button 12", base.DEC, nil, 0x0800)
fields.MANUAL_CONTROL_buttons_button_13 = ProtoField.uint16("mavlink.manual_control.buttons.button_13", "Button 13", base.DEC, nil, 0x1000)
fields.MANUAL_CONTROL_buttons_button_14 = ProtoField.uint16("mavlink.manual_control.buttons.button_14", "Button 14", base.DEC, nil, 0x2000)
fields.MANUAL_CONTROL_buttons_button_15 = ProtoField.uint16("mavlink.manual_control.buttons.button_15", "Button 15", base.DEC, nil, 0x4000)
fields.MANUAL_CONTROL_buttons_button_16 = ProtoField.uint16("mavlink.manual_control.buttons.button_16", "Button 16", base.DEC, nil, 0x8000)

-- Fields for message type 73: MISSION_ITEM_INT
fields.MISSION_ITEM_INT_target_system = ProtoField.uint8("mavlink.mission_item_int.target_system", "System ID", base.DEC_HEX)
fields.MISSION_ITEM_INT_target_component = ProtoField.uint8("mavlink.mission_item_int.target_component", "Component ID", base.DEC_HEX)
fields.MISSION_ITEM_INT_seq = ProtoField.uint16("mavlink.mission_item_int.seq", "Waypoint ID", base.DEC)
fields.MISSION_ITEM_INT_frame = ProtoField.uint8("mavlink.mission_item_int.frame", "Coordinate system", base.DEC, mavlinkFrame)
fields.MISSION_ITEM_INT_command = ProtoField.uint16("mavlink.mission_item_int.command", "Scheduled action", base.DEC, mavlinkCommand)
fields.MISSION_ITEM_INT_current = ProtoField.uint8("mavlink.mission_item_int.current", "Current", base.DEC, nil, 0x01)
fields.MISSION_ITEM_INT_autocontinue = ProtoField.uint8("mavlink.mission_item_int.autocontinue", "Autocontinue", base.DEC)
fields.MISSION_ITEM_INT_param1 = ProtoField.float("mavlink.mission_item_int.param1", "PARAM1", base.DEC)
fields.MISSION_ITEM_INT_param2 = ProtoField.float("mavlink.mission_item_int.param2", "PARAM2", base.DEC)
fields.MISSION_ITEM_INT_param3 = ProtoField.float("mavlink.mission_item_int.param3", "PARAM3", base.DEC)
fields.MISSION_ITEM_INT_param4 = ProtoField.float("mavlink.mission_item_int.param4", "PARAM4", base.DEC)
fields.MISSION_ITEM_INT_x = ProtoField.int32("mavlink.mission_item_int.x", "X position / Latitude", base.DEC)
fields.MISSION_ITEM_INT_y = ProtoField.int32("mavlink.mission_item_int.y", "Y position / Longitude", base.DEC)
fields.MISSION_ITEM_INT_z = ProtoField.float("mavlink.mission_item_int.z", "Z position / Altitude", base.DEC)
fields.MISSION_ITEM_INT_mission_type = ProtoField.uint8("mavlink.mission_item_int.mission_type", "Mission type", base.DEC, mavlinkMissionType)

-- Fields for message type 74: VFR_HUD
fields.VFR_HUD_airspeed = ProtoField.float("mavlink.vfr_hud.airspeed", "Airspeed [m/s]", base.DEC)
fields.VFR_HUD_groundspeed = ProtoField.float("mavlink.vfr_hud.groundspeed", "Ground speed [m/s]", base.DEC)
fields.VFR_HUD_heading = ProtoField.int16("mavlink.vfr_hud.heading", "Heading [deg]", base.DEC)
fields.VFR_HUD_throttle = ProtoField.uint16("mavlink.vfr_hud.throttle", "Throttle [%]", base.DEC)
fields.VFR_HUD_alt = ProtoField.float("mavlink.vfr_hud.alt", "Altitude [m]", base.DEC)
fields.VFR_HUD_climb = ProtoField.float("mavlink.vfr_hud.climb", "Climb rate [m/s]", base.DEC)

-- Fields for message type 76: COMMAND_LONG
fields.COMMAND_LONG_target_system = ProtoField.uint8("mavlink.command_long.target_system", "System", base.DEC_HEX)
fields.COMMAND_LONG_target_component = ProtoField.uint8("mavlink.command_long.target_component", "Component", base.DEC_HEX)
fields.COMMAND_LONG_command = ProtoField.uint16("mavlink.command_long.command", "Command", base.DEC, mavlinkCommand)
fields.COMMAND_LONG_confirmation = ProtoField.uint8("mavlink.command_long.confirmation", "Confirmation", base.DEC)
fields.COMMAND_LONG_param1 = ProtoField.float("mavlink.command_long.param1", "Parameter 1", base.DEC)
fields.COMMAND_LONG_param2 = ProtoField.float("mavlink.command_long.param2", "Parameter 2", base.DEC)
fields.COMMAND_LONG_param3 = ProtoField.float("mavlink.command_long.param3", "Parameter 3", base.DEC)
fields.COMMAND_LONG_param4 = ProtoField.float("mavlink.command_long.param4", "Parameter 4", base.DEC)
fields.COMMAND_LONG_param5 = ProtoField.float("mavlink.command_long.param5", "Parameter 5", base.DEC)
fields.COMMAND_LONG_param6 = ProtoField.float("mavlink.command_long.param6", "Parameter 6", base.DEC)
fields.COMMAND_LONG_param7 = ProtoField.float("mavlink.command_long.param7", "Parameter 7", base.DEC)

-- Fields for message type 77: COMMAND_ACK
fields.COMMAND_ACK_command = ProtoField.uint16("mavlink.command_ack.command", "Command", base.DEC, mavlinkCommand)
fields.COMMAND_ACK_result = ProtoField.uint8("mavlink.command_ack.result", "Result", base.DEC, mavlinkResults)
fields.COMMAND_ACK_progress = ProtoField.uint8("mavlink.command_ack.progress", "Progress / Result param 1", base.DEC)
fields.COMMAND_ACK_result_param2 = ProtoField.int32("mavlink.command_ack.result_param2", "Result param 2", base.DEC)
fields.COMMAND_ACK_target_system = ProtoField.uint8("mavlink.command_ack.target_system", "Target System", base.HEX)
fields.COMMAND_ACK_target_component = ProtoField.uint8("mavlink.command_ack.target_component", "Target Component", base.HEX, mavlinkCompId)

-- Fields for message type 83: ATTITUDE_TARGET
fields.ATTITUDE_TARGET_time_boot_ms = ProtoField.uint32("mavlink.attitude_target.time_boot_ms", "Uptime [ms]", base.DEC)
fields.ATTITUDE_TARGET_type_mask = ProtoField.uint8("mavlink.attitude_target.type_mask", "Type mask", base.HEX)
fields.ATTITUDE_TARGET_type_mask_body_roll = ProtoField.uint8("mavlink.attitude_target.type_mask.body_roll", "Body roll rate", base.DEC, nil, 0x01)
fields.ATTITUDE_TARGET_type_mask_body_pitch = ProtoField.uint8("mavlink.attitude_target.type_mask.body_pitch", "Body pitch rate", base.DEC, nil, 0x02)
fields.ATTITUDE_TARGET_type_mask_body_yaw = ProtoField.uint8("mavlink.attitude_target.type_mask.body_yaw", "Body yaw rate", base.DEC, nil, 0x04)
fields.ATTITUDE_TARGET_type_mask_reserved = ProtoField.uint8("mavlink.attitude_target.type_mask.reserved", "Reserved [4-7]", base.DEC, nil, 0x78)
fields.ATTITUDE_TARGET_type_mask_attitude = ProtoField.uint8("mavlink.attitude_target.type_mask.attitude", "Attitude", base.DEC, nil, 0x80)
fields.ATTITUDE_TARGET_q = ProtoField.bytes("mavlink.attitude_target.attitude_quaternion", "Attitude quaternion", base.HEX)
fields.ATTITUDE_TARGET_q0 = ProtoField.float("mavlink.attitude_target.attitude_quaternion.w", "Attitude quaternion W", base.DEC)
fields.ATTITUDE_TARGET_q1 = ProtoField.float("mavlink.attitude_target.attitude_quaternion.x", "Attitude quaternion X", base.DEC)
fields.ATTITUDE_TARGET_q2 = ProtoField.float("mavlink.attitude_target.attitude_quaternion.y", "Attitude quaternion Y", base.DEC)
fields.ATTITUDE_TARGET_q3 = ProtoField.float("mavlink.attitude_target.attitude_quaternion.z", "Attitude quaternion Z", base.DEC)
fields.ATTITUDE_TARGET_body_roll_rate = ProtoField.float("mavlink.attitude_target.body_roll_rate", "Body roll rate [rad/s]", base.DEC)
fields.ATTITUDE_TARGET_body_pitch_rate = ProtoField.float("mavlink.attitude_target.body_pitch_rate", "Body pitch rate [rad/s]", base.DEC)
fields.ATTITUDE_TARGET_body_yaw_rate = ProtoField.float("mavlink.attitude_target.body_yaw_rate", "Body yaw rate [rad/s]", base.DEC)
fields.ATTITUDE_TARGET_thrust = ProtoField.float("mavlink.attitude_target.thrust", "Collective thrust [normalized]", base.DEC)

-- Fields for message type 85: POSITION_TARGET_LOCAL_NED
fields.POSITION_TARGET_LOCAL_NED_time_boot_ms = ProtoField.uint32("mavlink.position_target_local_ned.time_boot_ms", "Timestamp [ms]", base.DEC)
fields.POSITION_TARGET_LOCAL_NED_coordinate_frame = ProtoField.uint8("mavlink.position_target_local_ned.coordinate_frame", "Coordinate frame", base.DEC, mavlinkFrame)
fields.POSITION_TARGET_LOCAL_NED_type_mask = ProtoField.uint16("mavlink.position_target_local_ned.type_mask", "Type mask", base.HEX)
fields.POSITION_TARGET_LOCAL_NED_type_mask_x = ProtoField.uint16("mavlink.position_target_local_ned.type_mask.x", "X position", base.HEX, nil, 0x0001)
fields.POSITION_TARGET_LOCAL_NED_type_mask_y = ProtoField.uint16("mavlink.position_target_local_ned.type_mask.y", "Y position", base.HEX, nil, 0x0002)
fields.POSITION_TARGET_LOCAL_NED_type_mask_z = ProtoField.uint16("mavlink.position_target_local_ned.type_mask.z", "Z position", base.HEX, nil, 0x0004)
fields.POSITION_TARGET_LOCAL_NED_type_mask_vx = ProtoField.uint16("mavlink.position_target_local_ned.type_mask.vx", "X velocity", base.HEX, nil, 0x0008)
fields.POSITION_TARGET_LOCAL_NED_type_mask_vy = ProtoField.uint16("mavlink.position_target_local_ned.type_mask.vy", "Y velocity", base.HEX, nil, 0x0010)
fields.POSITION_TARGET_LOCAL_NED_type_mask_vz = ProtoField.uint16("mavlink.position_target_local_ned.type_mask.vz", "Z velocity", base.HEX, nil, 0x0020)
fields.POSITION_TARGET_LOCAL_NED_type_mask_ax = ProtoField.uint16("mavlink.position_target_local_ned.type_mask.ax", "X acceleration", base.HEX, nil, 0x0040)
fields.POSITION_TARGET_LOCAL_NED_type_mask_ay = ProtoField.uint16("mavlink.position_target_local_ned.type_mask.ay", "Y acceleration", base.HEX, nil, 0x0080)
fields.POSITION_TARGET_LOCAL_NED_type_mask_az = ProtoField.uint16("mavlink.position_target_local_ned.type_mask.az", "Z acceleration", base.HEX, nil, 0x0100)
fields.POSITION_TARGET_LOCAL_NED_type_mask_fsp = ProtoField.uint16("mavlink.position_target_local_ned.type_mask.fsp", "Force setpoint", base.HEX, nil, 0x0200)
fields.POSITION_TARGET_LOCAL_NED_type_mask_yaw = ProtoField.uint16("mavlink.position_target_local_ned.type_mask.yaw", "Yaw", base.HEX, nil, 0x0400)
fields.POSITION_TARGET_LOCAL_NED_type_mask_yaw_rate = ProtoField.uint16("mavlink.position_target_local_ned.type_mask.yaw_rate", "yaw_rate", base.HEX, nil, 0x0800)
fields.POSITION_TARGET_LOCAL_NED_x = ProtoField.float("mavlink.position_target_local_ned.x", "X position [m]", base.DEC)
fields.POSITION_TARGET_LOCAL_NED_y = ProtoField.float("mavlink.position_target_local_ned.y", "Y position [m]", base.DEC)
fields.POSITION_TARGET_LOCAL_NED_z = ProtoField.float("mavlink.position_target_local_ned.z", "Z position [m]", base.DEC)
fields.POSITION_TARGET_LOCAL_NED_vx = ProtoField.float("mavlink.position_target_local_ned.vx", "X velocity [m/s]", base.DEC)
fields.POSITION_TARGET_LOCAL_NED_vy = ProtoField.float("mavlink.position_target_local_ned.vy", "Y velocity [m/s]", base.DEC)
fields.POSITION_TARGET_LOCAL_NED_vz = ProtoField.float("mavlink.position_target_local_ned.vz", "Z velocity [m/s]", base.DEC)
fields.POSITION_TARGET_LOCAL_NED_afx = ProtoField.float("mavlink.position_target_local_ned.afx", "X acceleration | force [m/s/s | N]", base.DEC)
fields.POSITION_TARGET_LOCAL_NED_afy = ProtoField.float("mavlink.position_target_local_ned.afy", "Y acceleration | force [m/s/s | N]", base.DEC)
fields.POSITION_TARGET_LOCAL_NED_afz = ProtoField.float("mavlink.position_target_local_ned.afz", "Z acceleration | force [m/s/s | N]", base.DEC)
fields.POSITION_TARGET_LOCAL_NED_yaw = ProtoField.float("mavlink.position_target_local_ned.yaw", "Yaw setpoint [rad]", base.DEC)
fields.POSITION_TARGET_LOCAL_NED_yaw_rate = ProtoField.float("mavlink.position_target_local_ned.yaw_rate", "Yaw rate setpoint [rad/s]", base.DEC)

-- Fields for message type 93: HIL_ACTUATOR_CONTROLS
fields.HIL_ACTUATOR_CONTROLS_time_usec = ProtoField.uint64("mavlink.time_usec", "Timestamp [us]", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls = ProtoField.bytes("mavlink.hil_actuator_controls.controls.control0", "Controls", base.HEX)
fields.HIL_ACTUATOR_CONTROLS_controls_0 = ProtoField.float("mavlink.hil_actuator_controls.controls.control0", "Control 0", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_1 = ProtoField.float("mavlink.hil_actuator_controls.controls.control1", "Control 1", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_2 = ProtoField.float("mavlink.hil_actuator_controls.controls.control2", "Control 2", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_3 = ProtoField.float("mavlink.hil_actuator_controls.controls.control3", "Control 3", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_4 = ProtoField.float("mavlink.hil_actuator_controls.controls.control4", "Control 4", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_5 = ProtoField.float("mavlink.hil_actuator_controls.controls.control5", "Control 5", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_6 = ProtoField.float("mavlink.hil_actuator_controls.controls.control6", "Control 6", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_7 = ProtoField.float("mavlink.hil_actuator_controls.controls.control7", "Control 7", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_8 = ProtoField.float("mavlink.hil_actuator_controls.controls.control8", "Control 8", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_9 = ProtoField.float("mavlink.hil_actuator_controls.controls.control9", "Control 9", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_10 = ProtoField.float("mavlink.hil_actuator_controls.controls.control10", "Control 10", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_11 = ProtoField.float("mavlink.hil_actuator_controls.controls.control11", "Control 11", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_12 = ProtoField.float("mavlink.hil_actuator_controls.controls.control12", "Control 12", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_13 = ProtoField.float("mavlink.hil_actuator_controls.controls.control13", "Control 13", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_14 = ProtoField.float("mavlink.hil_actuator_controls.controls.control14", "Control 14", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_controls_15 = ProtoField.float("mavlink.hil_actuator_controls.controls.control15", "Control 15", base.DEC)
fields.HIL_ACTUATOR_CONTROLS_mode = ProtoField.uint8("mavlink.hil_actuator_controls.mode", "System mode", base.DEC, mavlinkMode)
fields.HIL_ACTUATOR_CONTROLS_flags = ProtoField.uint64("mavlink.hil_actuator_controls.flags", "Flags", base.HEX)

-- Fields for message type 105: HIGHRES_IMU
fields.HIGHRES_IMU_time_usec = ProtoField.uint64("mavlink.highres_imu.time_usec", "Timestamp [us]", base.DEC)
fields.HIGHRES_IMU_xacc = ProtoField.float("mavlink.highres_imu.xacc", "X acceleration [m/s/s]", base.DEC)
fields.HIGHRES_IMU_yacc = ProtoField.float("mavlink.highres_imu.yacc", "Y acceleration [m/s/s]", base.DEC)
fields.HIGHRES_IMU_zacc = ProtoField.float("mavlink.highres_imu.zacc", "Z acceleration [m/s/s]", base.DEC)
fields.HIGHRES_IMU_xgyro = ProtoField.float("mavlink.highres_imu.xgyro", "Angular speed around X [rad/s]", base.DEC)
fields.HIGHRES_IMU_ygyro = ProtoField.float("mavlink.highres_imu.ygyro", "Angular speed around Y [rad/s]", base.DEC)
fields.HIGHRES_IMU_zgyro = ProtoField.float("mavlink.highres_imu.zgyro", "Angular speed around Z [rad/s]", base.DEC)
fields.HIGHRES_IMU_xmag = ProtoField.float("mavlink.highres_imu.xmag", "X magnetic field [G]", base.DEC)
fields.HIGHRES_IMU_ymag = ProtoField.float("mavlink.highres_imu.ymag", "Y magnetic field [G]", base.DEC)
fields.HIGHRES_IMU_zmag = ProtoField.float("mavlink.highres_imu.zmag", "Z magnetic field [G]", base.DEC)
fields.HIGHRES_IMU_abs_pressure = ProtoField.float("mavlink.highres_imu.abs_pressure", "Absolute pressure [mBar]", base.DEC)
fields.HIGHRES_IMU_diff_pressure = ProtoField.float("mavlink.highres_imu.diff_pressure", "Differential pressure [mBar]", base.DEC)
fields.HIGHRES_IMU_pressure_alt = ProtoField.float("mavlink.highres_imu.pressure_alt", "Altitude calculated from pressure", base.DEC)
fields.HIGHRES_IMU_temperature = ProtoField.float("mavlink.highres_imu.temperature", "Temperature [°C]", base.DEC)
fields.HIGHRES_IMU_fields_updated = ProtoField.uint16("mavlink.HIGHRES_IMU.fields_updated", "Fields updated", base.HEX)
fields.HIGHRES_IMU_fields_updated_xacc = ProtoField.uint16("mavlink.HIGHRES_IMU.fields_updated.xacc", "X acceleration", base.DEC, nil, 0x0001)
fields.HIGHRES_IMU_fields_updated_yacc = ProtoField.uint16("mavlink.HIGHRES_IMU.fields_updated.yacc", "Y acceleration", base.DEC, nil, 0x0002)
fields.HIGHRES_IMU_fields_updated_zacc = ProtoField.uint16("mavlink.HIGHRES_IMU.fields_updated.zacc", "Z acceleration", base.DEC, nil, 0x0004)
fields.HIGHRES_IMU_fields_updated_xgyro = ProtoField.uint16("mavlink.HIGHRES_IMU.fields_updated.xgyro", "Angular speed around X", base.DEC, nil, 0x0008)
fields.HIGHRES_IMU_fields_updated_ygyro = ProtoField.uint16("mavlink.HIGHRES_IMU.fields_updated.ygyro", "Angular speed around Y", base.DEC, nil, 0x0010)
fields.HIGHRES_IMU_fields_updated_zgyro = ProtoField.uint16("mavlink.HIGHRES_IMU.fields_updated.zgyro", "Angular speed around Z", base.DEC, nil, 0x0020)
fields.HIGHRES_IMU_fields_updated_xmag = ProtoField.uint16("mavlink.HIGHRES_IMU.fields_updated.xmag", "X magnetic field", base.DEC, nil, 0x0040)
fields.HIGHRES_IMU_fields_updated_ymag = ProtoField.uint16("mavlink.HIGHRES_IMU.fields_updated.ymag", "Y magnetic field", base.DEC, nil, 0x0080)
fields.HIGHRES_IMU_fields_updated_zmag = ProtoField.uint16("mavlink.HIGHRES_IMU.fields_updated.zmag", "Z magnetic field", base.DEC, nil, 0x0100)
fields.HIGHRES_IMU_fields_updated_abs_pressure = ProtoField.uint16("mavlink.HIGHRES_IMU.fields_updated.abs_pressure", "Absolute pressure", base.DEC, nil, 0x0200)
fields.HIGHRES_IMU_fields_updated_diff_pressure = ProtoField.uint16("mavlink.HIGHRES_IMU.fields_updated.dif_pressure", "Differential pressure", base.DEC, nil, 0x0400)
fields.HIGHRES_IMU_fields_updated_pressure_alt = ProtoField.uint16("mavlink.HIGHRES_IMU.fields_updated.pressure_alt", "Altitude calculated from pressure", base.DEC, nil, 0x0800)
fields.HIGHRES_IMU_fields_updated_temperature = ProtoField.uint16("mavlink.HIGHRES_IMU.fields_updated.temperature", "Temperature", base.DEC, nil, 0x1000)

-- Fields for message type 107: HIL_SENSOR
fields.HIL_SENSOR_time_usec = ProtoField.uint64("mavlink.hil_sensor.time_usec", "Timestamp [us]", base.DEC)
fields.HIL_SENSOR_xacc = ProtoField.float("mavlink.hil_sensor.xacc", "X acceleration [m/s/s]", base.DEC)
fields.HIL_SENSOR_yacc = ProtoField.float("mavlink.hil_sensor.yacc", "Y acceleration [m/s/s]", base.DEC)
fields.HIL_SENSOR_zacc = ProtoField.float("mavlink.hil_sensor.zacc", "Z acceleration [m/s/s]", base.DEC)
fields.HIL_SENSOR_xgyro = ProtoField.float("mavlink.hil_sensor.xgyro", "Angular speed around X [rad/s]", base.DEC)
fields.HIL_SENSOR_ygyro = ProtoField.float("mavlink.hil_sensor.ygyro", "Angular speed around Y [rad/s]", base.DEC)
fields.HIL_SENSOR_zgyro = ProtoField.float("mavlink.hil_sensor.zgyro", "Angular speed around Z [rad/s]", base.DEC)
fields.HIL_SENSOR_xmag = ProtoField.float("mavlink.hil_sensor.xmag", "X magnetic field [gauss]", base.DEC)
fields.HIL_SENSOR_ymag = ProtoField.float("mavlink.hil_sensor.ymag", "Y magnetic field [gauss]", base.DEC)
fields.HIL_SENSOR_zmag = ProtoField.float("mavlink.hil_sensor.zmag", "Z magnetic field [gauss]", base.DEC)
fields.HIL_SENSOR_abs_pressure = ProtoField.float("mavlink.hil_sensor.abs_pressure", "Absolute pressure [mbar]", base.DEC)
fields.HIL_SENSOR_diff_pressure = ProtoField.float("mavlink.hil_sensor.dif_pressure", "Differential pressure [mbar]", base.DEC)
fields.HIL_SENSOR_pressure_alt = ProtoField.float("mavlink.hil_sensor.pressure_alt", "Altitude calculated from pressure", base.DEC)
fields.HIL_SENSOR_temperature = ProtoField.float("mavlink.hil_sensor.temperature", "Temperature [°C]", base.DEC)
fields.HIL_SENSOR_fields_updated = ProtoField.uint32("mavlink.hil_sensor.fields_updated", "Fields updated", base.HEX)
fields.HIL_SENSOR_fields_updated_xacc = ProtoField.uint32("mavlink.hil_sensor.fields_updated.xacc", "X acceleration", base.DEC, nil, 0x00000001)
fields.HIL_SENSOR_fields_updated_yacc = ProtoField.uint32("mavlink.hil_sensor.fields_updated.yacc", "Y acceleration", base.DEC, nil, 0x00000002)
fields.HIL_SENSOR_fields_updated_zacc = ProtoField.uint32("mavlink.hil_sensor.fields_updated.zacc", "Z acceleration", base.DEC, nil, 0x00000004)
fields.HIL_SENSOR_fields_updated_xgyro = ProtoField.uint32("mavlink.hil_sensor.fields_updated.xgyro", "Angular speed around X", base.DEC, nil, 0x00000008)
fields.HIL_SENSOR_fields_updated_ygyro = ProtoField.uint32("mavlink.hil_sensor.fields_updated.ygyro", "Angular speed around Y", base.DEC, nil, 0x00000010)
fields.HIL_SENSOR_fields_updated_zgyro = ProtoField.uint32("mavlink.hil_sensor.fields_updated.zgyro", "Angular speed around Z", base.DEC, nil, 0x00000020)
fields.HIL_SENSOR_fields_updated_xmag = ProtoField.uint32("mavlink.hil_sensor.fields_updated.xmag", "X magnetic field", base.DEC, nil, 0x00000040)
fields.HIL_SENSOR_fields_updated_ymag = ProtoField.uint32("mavlink.hil_sensor.fields_updated.ymag", "Y magnetic field", base.DEC, nil, 0x00000080)
fields.HIL_SENSOR_fields_updated_zmag = ProtoField.uint32("mavlink.hil_sensor.fields_updated.zmag", "Z magnetic field", base.DEC, nil, 0x00000100)
fields.HIL_SENSOR_fields_updated_abs_pressure = ProtoField.uint32("mavlink.hil_sensor.fields_updated.abs_pressure", "Absolute pressure", base.DEC, nil, 0x00000200)
fields.HIL_SENSOR_fields_updated_diff_pressure = ProtoField.uint32("mavlink.hil_sensor.fields_updated.dif_pressure", "Differential pressure", base.DEC, nil, 0x00000400)
fields.HIL_SENSOR_fields_updated_pressure_alt = ProtoField.uint32("mavlink.hil_sensor.fields_updated.pressure_alt", "Altitude calculated from pressure", base.DEC, nil, 0x00000800)
fields.HIL_SENSOR_fields_updated_temperature = ProtoField.uint32("mavlink.hil_sensor.fields_updated.temperature", "Temperature", base.DEC, nil, 0x00001000)
fields.HIL_SENSOR_fields_updated_full_reset = ProtoField.uint32("mavlink.hil_sensor.fields_updated.full_reset", "Full reset", base.DEC, nil, 0x80000000)

-- Fields for message type 111: TIMESYNC
fields.TIMESYNC_tc1 = ProtoField.uint64("mavlink.timesync.tc1", "Time sync timestamp 1 [us]", base.DEC)
fields.TIMESYNC_ts1 = ProtoField.uint64("mavlink.timesync.ts1", "Time sync timestamp 2 [us]", base.DEC)

-- Fields for message type 113: HIL_GPS
fields.HIL_GPS_time_usec = ProtoField.uint64("mavlink.hil_gps.time_usec", "Timestamp [us]", base.DEC)
fields.HIL_GPS_fix_type = ProtoField.uint8("mavlink.hil_gps.fix_type", "GPS fix type", base.DEC, gpsFixType)
fields.HIL_GPS_lat = ProtoField.int32("mavlink.hil_gps.lat", "Latitude [°E7]", base.DEC)
fields.HIL_GPS_lon = ProtoField.int32("mavlink.hil_gps.lon", "Longitude [°E7]", base.DEC)
fields.HIL_GPS_alt = ProtoField.int32("mavlink.hil_gps.alt", "Altitude [mm]", base.DEC)
fields.HIL_GPS_eph = ProtoField.uint16("mavlink.hil_gps.eph", "GPS horizontal dilution [cm]", base.DEC)
fields.HIL_GPS_epv = ProtoField.uint16("mavlink.hil_gps.epv", "GPS vertical dilution [cm]", base.DEC)
fields.HIL_GPS_vel = ProtoField.uint16("mavlink.hil_gps.vel", "GPS ground speed [cm/s]", base.DEC)
fields.HIL_GPS_vn = ProtoField.int16("mavlink.hil_gps.vn", "GPS velocitcy NORTH [cm/s]", base.DEC)
fields.HIL_GPS_ve = ProtoField.int16("mavlink.hil_gps.ve", "GPS velocitcy EAST [cm/s]", base.DEC)
fields.HIL_GPS_vd = ProtoField.int16("mavlink.hil_gps.vd", "GPS velocitcy DOWN [cm/s]", base.DEC)
fields.HIL_GPS_cog = ProtoField.uint16("mavlink.hil_gps.cog", "Course over ground [°E2]", base.DEC)
fields.HIL_GPS_satellites_visible = ProtoField.uint16("mavlink.hil_gps.satellites_visible", "Satellites visible", base.DEC)

-- Fields for message type 115: HIL_STATE_QUATERNION
fields.HIL_STATE_QUATERNION_time_usec = ProtoField.uint64("mavlink.hil_state_quaternion.time_usec", "Timestamp [us]", base.DEC)
fields.HIL_STATE_QUATERNION_attitude_quaternion = ProtoField.bytes("mavlink.hil_state_quaternion.attitude_quaternion", "Attitude quaternion", base.HEX)
fields.HIL_STATE_QUATERNION_attitude_quaternion_0 = ProtoField.float("mavlink.hil_state_quaternion.attitude_quaternion.w", "Quaternion W", base.DEC)
fields.HIL_STATE_QUATERNION_attitude_quaternion_1 = ProtoField.float("mavlink.hil_state_quaternion.attitude_quaternion.x", "Quaternion X", base.DEC)
fields.HIL_STATE_QUATERNION_attitude_quaternion_2 = ProtoField.float("mavlink.hil_state_quaternion.attitude_quaternion.y", "Quaternion Y", base.DEC)
fields.HIL_STATE_QUATERNION_attitude_quaternion_3 = ProtoField.float("mavlink.hil_state_quaternion.attitude_quaternion.z", "Quaternion Z", base.DEC)
fields.HIL_STATE_QUATERNION_rollspeed = ProtoField.float("mavlink.hil_state_quaternion.rollspeed", "Roll angular speed [rad/s]", base.DEC)
fields.HIL_STATE_QUATERNION_pitchspeed = ProtoField.float("mavlink.hil_state_quaternion.pitchspeed", "Pitch angular speed [rad/s]", base.DEC)
fields.HIL_STATE_QUATERNION_yawspeed = ProtoField.float("mavlink.hil_state_quaternion.yawspeed", "Yaw angular speed [rad/s]", base.DEC)
fields.HIL_STATE_QUATERNION_lat = ProtoField.int32("mavlink.hil_state_quaternion.lat", "Latitude [°E7]", base.DEC)
fields.HIL_STATE_QUATERNION_lon = ProtoField.int32("mavlink.hil_state_quaternion.lon", "Longitude [°E7]", base.DEC)
fields.HIL_STATE_QUATERNION_alt = ProtoField.int32("mavlink.hil_state_quaternion.alt", "Altitude [mm]", base.DEC)
fields.HIL_STATE_QUATERNION_vx = ProtoField.int16("mavlink.hil_state_quaternion.vx", "Ground X speed [cm/s]", base.DEC)
fields.HIL_STATE_QUATERNION_vy = ProtoField.int16("mavlink.hil_state_quaternion.vy", "Ground Y speed [cm/s]", base.DEC)
fields.HIL_STATE_QUATERNION_vz = ProtoField.int16("mavlink.hil_state_quaternion.vz", "Ground Z speed [cm/s]", base.DEC)
fields.HIL_STATE_QUATERNION_ind_airspeed = ProtoField.uint16("mavlink.hil_state_quaternion.ind_airspeed", "Indicated airspeed [cm/s]", base.DEC)
fields.HIL_STATE_QUATERNION_true_airspeed = ProtoField.uint16("mavlink.hil_state_quaternion.true_airspeed", "True airspeed [cm/s]", base.DEC)
fields.HIL_STATE_QUATERNION_xacc = ProtoField.int16("mavlink.hil_state_quaternion.xacc", "x acceleration [mG]", base.DEC)
fields.HIL_STATE_QUATERNION_yacc = ProtoField.int16("mavlink.hil_state_quaternion.yacc", "y acceleration [mG]", base.DEC)
fields.HIL_STATE_QUATERNION_zacc = ProtoField.int16("mavlink.hil_state_quaternion.zacc", "z acceleration [mG]", base.DEC)

-- Fields for message type 140: ACTUATOR_CONTROL_TARGET
fields.ACTUATOR_CONTROL_TARGET_time_usec = ProtoField.uint64("mavlink.actuator_control_target.time_usec", "Timestamp [us]", base.DEC)
fields.ACTUATOR_CONTROL_TARGET_group_mlx = ProtoField.uint8("mavlink.actuator_control_target.group_mlx", "Actuator group", base.HEX)
fields.ACTUATOR_CONTROL_TARGET_controls_0 = ProtoField.float("mavlink.actuator_control_target.control_0", "Control 0 [-1..1]", base.DEC)
fields.ACTUATOR_CONTROL_TARGET_controls_1 = ProtoField.float("mavlink.actuator_control_target.control_1", "Control 1 [-1..1]", base.DEC)
fields.ACTUATOR_CONTROL_TARGET_controls_2 = ProtoField.float("mavlink.actuator_control_target.control_2", "Control 2 [-1..1]", base.DEC)
fields.ACTUATOR_CONTROL_TARGET_controls_3 = ProtoField.float("mavlink.actuator_control_target.control_3", "Control 3 [-1..1]", base.DEC)
fields.ACTUATOR_CONTROL_TARGET_controls_4 = ProtoField.float("mavlink.actuator_control_target.control_4", "Control 4 [-1..1]", base.DEC)
fields.ACTUATOR_CONTROL_TARGET_controls_5 = ProtoField.float("mavlink.actuator_control_target.control_5", "Control 5 [-1..1]", base.DEC)
fields.ACTUATOR_CONTROL_TARGET_controls_6 = ProtoField.float("mavlink.actuator_control_target.control_6", "Control 6 [-1..1]", base.DEC)
fields.ACTUATOR_CONTROL_TARGET_controls_7 = ProtoField.float("mavlink.actuator_control_target.control_7", "Control 7 [-1..1]", base.DEC)

-- Fields for message type 141: ALTITUDE
fields.ALTITUDE_time_usec = ProtoField.uint64("mavlink.altitude.time_usec", "Timestamp [usec]", base.DEC)
fields.ALTITUDE_altitude_monotonic = ProtoField.float("mavlink.altitude.altitude_monotonic", "Monotonic [m]", base.DEC)
fields.ALTITUDE_altitude_amsl = ProtoField.float("mavlink.altitude.altitude_amsl", "AMSL [m]", base.DEC)
fields.ALTITUDE_altitude_local = ProtoField.float("mavlink.altitude.altitude_local", "Local altitude [m]", base.DEC)
fields.ALTITUDE_altitude_relative = ProtoField.float("mavlink.altitude.altitude_relative", "Relative altitude [m]", base.DEC)
fields.ALTITUDE_altitude_terrain = ProtoField.float("mavlink.altitude.altitude_terrain", "Terrain altitude [m]", base.DEC)
fields.ALTITUDE_altitude_clearance = ProtoField.float("mavlink.altitude.altitude_clearance", "Clearance altitude [m]", base.DEC)

-- Fields for message type 147: BATTERY_STATUS
fields.BATTERY_STATUS_id = ProtoField.uint8("mavlink.battery_status.id", "Battery ID", base.HEX)
fields.BATTERY_STATUS_battery_function = ProtoField.uint8("mavlink.battery_status.battery_function", "Function of the battery", base.DEC, mavlinkBatteryFunction)
fields.BATTERY_STATUS_type = ProtoField.uint8("mavlink.battery_status.type", "Type (Chemistry)", base.DEC, mavlinkBatteryType)
fields.BATTERY_STATUS_temperature = ProtoField.int16("mavlink.battery_status.temperature", "Temperature [°C x100]", base.DEC)
fields.BATTERY_STATUS_voltages = ProtoField.bytes("mavlink.battery_status.voltages", "Battery voltage of cells", base.HEX)
fields.BATTERY_STATUS_voltages_0 = ProtoField.uint16("mavlink.battery_status.voltages.0", "Cell 0 [mV]", base.DEC)
fields.BATTERY_STATUS_voltages_1 = ProtoField.uint16("mavlink.battery_status.voltages.1", "Cell 1 [mV]", base.DEC)
fields.BATTERY_STATUS_voltages_2 = ProtoField.uint16("mavlink.battery_status.voltages.2", "Cell 2 [mV]", base.DEC)
fields.BATTERY_STATUS_voltages_3 = ProtoField.uint16("mavlink.battery_status.voltages.3", "Cell 3 [mV]", base.DEC)
fields.BATTERY_STATUS_voltages_4 = ProtoField.uint16("mavlink.battery_status.voltages.4", "Cell 4 [mV]", base.DEC)
fields.BATTERY_STATUS_voltages_5 = ProtoField.uint16("mavlink.battery_status.voltages.5", "Cell 5 [mV]", base.DEC)
fields.BATTERY_STATUS_voltages_6 = ProtoField.uint16("mavlink.battery_status.voltages.6", "Cell 6 [mV]", base.DEC)
fields.BATTERY_STATUS_voltages_7 = ProtoField.uint16("mavlink.battery_status.voltages.7", "Cell 7 [mV]", base.DEC)
fields.BATTERY_STATUS_voltages_8 = ProtoField.uint16("mavlink.battery_status.voltages.8", "Cell 8 [mV]", base.DEC)
fields.BATTERY_STATUS_voltages_9 = ProtoField.uint16("mavlink.battery_status.voltages.9", "Cell 9 [mV]", base.DEC)
fields.BATTERY_STATUS_current_battery = ProtoField.int16("mavlink.battery_status.current_battery", "Battery current [mA x100]", base.DEC)
fields.BATTERY_STATUS_current_consumed = ProtoField.int32("mavlink.battery_status.current_consumed", "Consumed charge [mAh]", base.DEC)
fields.BATTERY_STATUS_energy_consumed = ProtoField.int32("mavlink.battery_status.energy_consumed", "Consumed energy [hJ]", base.DEC)
fields.BATTERY_STATUS_battery_remaining = ProtoField.int8("mavlink.battery_status.battery_remaining", "Remaining battery energy [%]", base.DEC)
fields.BATTERY_STATUS_time_remaining = ProtoField.int32("mavlink.battery_status.time_remaining", "Remaining battery time [s]", base.DEC)
fields.BATTERY_STATUS_charge_state = ProtoField.uint8("mavlink.battery_state.charge_state", "Charge state", base.DEC, mavlinkBatteryChargeState)

-- Fields for message type 148: AUTOPILOT_VERSION
fields.AUTOPILOT_VERSION_capabilities = ProtoField.uint64("mavlink.autopilot_version.capabilities", "Capabilities", base.HEX, nil, 0xffffffffffffffff)
fields.AUTOPILOT_VERSION_capabilities_mission_float = ProtoField.uint64("mavlink.autopilot_version.capabilities.mission_float", "mission_float", base.DEC, nil, 0x0000000000000001)
fields.AUTOPILOT_VERSION_capabilities_param_float = ProtoField.uint64("mavlink.autopilot_version.capabilities.param_float", "param_float", base.DEC, nil, 0x0000000000000002)
fields.AUTOPILOT_VERSION_capabilities_mission_int = ProtoField.uint64("mavlink.autopilot_version.capabilities.mission_int", "mission_int", base.DEC, nil, 0x0000000000000004)
fields.AUTOPILOT_VERSION_capabilities_command_int = ProtoField.uint64("mavlink.autopilot_version.capabilities.command_int", "command_int", base.DEC, nil, 0x0000000000000008)
fields.AUTOPILOT_VERSION_capabilities_param_union = ProtoField.uint64("mavlink.autopilot_version.capabilities.param_union", "param_union", base.DEC, nil, 0x0000000000000010)
fields.AUTOPILOT_VERSION_capabilities_ftp = ProtoField.uint64("mavlink.autopilot_version.capabilities.ftp", "ftp", base.DEC, nil, 0x0000000000000020)
fields.AUTOPILOT_VERSION_capabilities_set_attitude_target = ProtoField.uint64("mavlink.autopilot_version.capabilities.set_attitude_target", "set_attitude_target", base.DEC, nil, 0x0000000000000040)
fields.AUTOPILOT_VERSION_capabilities_set_position_target_local_ned = ProtoField.uint64("mavlink.autopilot_version.capabilities.set_position_target_local_ned", "set_position_target_local_ned", base.DEC, nil, 0x0000000000000080)
fields.AUTOPILOT_VERSION_capabilities_set_position_target_global_int = ProtoField.uint64("mavlink.autopilot_version.capabilities.set_position_target_global_int", "set_position_target_global_int", base.DEC, nil, 0x0000000000000100)
fields.AUTOPILOT_VERSION_capabilities_terrain = ProtoField.uint64("mavlink.autopilot_version.capabilities.terrain", "terrain", base.DEC, nil, 0x0000000000000200)
fields.AUTOPILOT_VERSION_capabilities_set_actuator_target = ProtoField.uint64("mavlink.autopilot_version.capabilities.set_actuator_target", "set_actuator_target", base.DEC, nil, 0x0000000000000400)
fields.AUTOPILOT_VERSION_capabilities_flight_termination = ProtoField.uint64("mavlink.autopilot_version.capabilities.flight_termination", "flight_termination", base.DEC, nil, 0x0000000000000800)
fields.AUTOPILOT_VERSION_capabilities_compass_calibration = ProtoField.uint64("mavlink.autopilot_version.capabilities.compass_calibration", "compass_calibration", base.DEC, nil, 0x0000000000001000)
fields.AUTOPILOT_VERSION_capabilities_mavlink2 = ProtoField.uint64("mavlink.autopilot_version.capabilities.mavlink2", "mavlink2", base.DEC, nil, 0x0000000000002000)
fields.AUTOPILOT_VERSION_capabilities_mission_fence = ProtoField.uint64("mavlink.autopilot_version.capabilities.mission_fence", "mission_fence", base.DEC, nil, 0x0000000000004000)
fields.AUTOPILOT_VERSION_capabilities_mission_rally = ProtoField.uint64("mavlink.autopilot_version.capabilities.mission_rally", "mission_rally", base.DEC, nil, 0x0000000000008000)
fields.AUTOPILOT_VERSION_capabilities_flight_information = ProtoField.uint64("mavlink.autopilot_version.capabilities.flight_information", "flight_information", base.DEC, nil, 0x0000000000010000)
fields.AUTOPILOT_VERSION_flight_sw_version = ProtoField.uint32("mavlink.autopilot_version.flight_sw_version", "Firmware version", base.DEC)
fields.AUTOPILOT_VERSION_middleware_sw_version = ProtoField.uint32("mavlink.autopilot_version.middleware_sw_version", "Middleware version", base.DEC)
fields.AUTOPILOT_VERSION_os_sw_version = ProtoField.uint32("mavlink.autopilot_version.os_sw_version", "Operating System version", base.DEC)
fields.AUTOPILOT_VERSION_board_version = ProtoField.uint32("mavlink.autopilot_version.board_version", "HW/Board version", base.DEC)
fields.AUTOPILOT_VERSION_flight_custom_version = ProtoField.bytes("mavlink.autopilot_version.flight_custom_version", "Firmware custom version", base.HEX)
fields.AUTOPILOT_VERSION_flight_custom_version_0 = ProtoField.uint8("mavlink.autopilot_version.flight_custom_version.0", "Firmware custom version [0]", base.DEC)
fields.AUTOPILOT_VERSION_flight_custom_version_1 = ProtoField.uint8("mavlink.autopilot_version.flight_custom_version.1", "Firmware custom version [1]", base.DEC)
fields.AUTOPILOT_VERSION_flight_custom_version_2 = ProtoField.uint8("mavlink.autopilot_version.flight_custom_version.2", "Firmware custom version [2]", base.DEC)
fields.AUTOPILOT_VERSION_flight_custom_version_3 = ProtoField.uint8("mavlink.autopilot_version.flight_custom_version.3", "Firmware custom version [3]", base.DEC)
fields.AUTOPILOT_VERSION_flight_custom_version_4 = ProtoField.uint8("mavlink.autopilot_version.flight_custom_version.4", "Firmware custom version [4]", base.DEC)
fields.AUTOPILOT_VERSION_flight_custom_version_5 = ProtoField.uint8("mavlink.autopilot_version.flight_custom_version.5", "Firmware custom version [5]", base.DEC)
fields.AUTOPILOT_VERSION_flight_custom_version_6 = ProtoField.uint8("mavlink.autopilot_version.flight_custom_version.6", "Firmware custom version [6]", base.DEC)
fields.AUTOPILOT_VERSION_flight_custom_version_7 = ProtoField.uint8("mavlink.autopilot_version.flight_custom_version.7", "Firmware custom version [7]", base.DEC)
fields.AUTOPILOT_VERSION_middleware_custom_version = ProtoField.bytes("mavlink.autopilot_version.middleware_custom_version", "Middleware custom version", base.HEX)
fields.AUTOPILOT_VERSION_middleware_custom_version_0 = ProtoField.uint8("mavlink.autopilot_version.middleware_custom_version.0", "Middleware custom version [0]", base.DEC)
fields.AUTOPILOT_VERSION_middleware_custom_version_1 = ProtoField.uint8("mavlink.autopilot_version.middleware_custom_version.1", "Middleware custom version [1]", base.DEC)
fields.AUTOPILOT_VERSION_middleware_custom_version_2 = ProtoField.uint8("mavlink.autopilot_version.middleware_custom_version.2", "Middleware custom version [2]", base.DEC)
fields.AUTOPILOT_VERSION_middleware_custom_version_3 = ProtoField.uint8("mavlink.autopilot_version.middleware_custom_version.3", "Middleware custom version [3]", base.DEC)
fields.AUTOPILOT_VERSION_middleware_custom_version_4 = ProtoField.uint8("mavlink.autopilot_version.middleware_custom_version.4", "Middleware custom version [4]", base.DEC)
fields.AUTOPILOT_VERSION_middleware_custom_version_5 = ProtoField.uint8("mavlink.autopilot_version.middleware_custom_version.5", "Middleware custom version [5]", base.DEC)
fields.AUTOPILOT_VERSION_middleware_custom_version_6 = ProtoField.uint8("mavlink.autopilot_version.middleware_custom_version.6", "Middleware custom version [6]", base.DEC)
fields.AUTOPILOT_VERSION_middleware_custom_version_7 = ProtoField.uint8("mavlink.autopilot_version.middleware_custom_version.7", "Middleware custom version [7]", base.DEC)
fields.AUTOPILOT_VERSION_os_custom_version = ProtoField.bytes("mavlink.autopilot_version.os_custom_version", "Operating System custom version", base.HEX)
fields.AUTOPILOT_VERSION_os_custom_version_0 = ProtoField.uint8("mavlink.autopilot_version.os_custom_version.0", "Operating System custom version [0]", base.DEC)
fields.AUTOPILOT_VERSION_os_custom_version_1 = ProtoField.uint8("mavlink.autopilot_version.os_custom_version.1", "Operating System custom version [1]", base.DEC)
fields.AUTOPILOT_VERSION_os_custom_version_2 = ProtoField.uint8("mavlink.autopilot_version.os_custom_version.2", "Operating System custom version [2]", base.DEC)
fields.AUTOPILOT_VERSION_os_custom_version_3 = ProtoField.uint8("mavlink.autopilot_version.os_custom_version.3", "Operating System custom version [3]", base.DEC)
fields.AUTOPILOT_VERSION_os_custom_version_4 = ProtoField.uint8("mavlink.autopilot_version.os_custom_version.4", "Operating System custom version [4]", base.DEC)
fields.AUTOPILOT_VERSION_os_custom_version_5 = ProtoField.uint8("mavlink.autopilot_version.os_custom_version.5", "Operating System custom version [5]", base.DEC)
fields.AUTOPILOT_VERSION_os_custom_version_6 = ProtoField.uint8("mavlink.autopilot_version.os_custom_version.6", "Operating System custom version [6]", base.DEC)
fields.AUTOPILOT_VERSION_os_custom_version_7 = ProtoField.uint8("mavlink.autopilot_version.os_custom_version.7", "Operating System custom version [7]", base.DEC)
fields.AUTOPILOT_VERSION_vendor_id = ProtoField.uint16("mavlink.autopilot_version.vendor_id", "Vendor ID", base.HEX)
fields.AUTOPILOT_VERSION_product_id = ProtoField.uint16("mavlink.autopilot_version.product_id", "Product ID", base.HEX)
fields.AUTOPILOT_VERSION_uid = ProtoField.uint64("mavlink.autopilot_version.uid", "UID", base.DEC_HEX)
fields.AUTOPILOT_VERSION_uid2 = ProtoField.bytes("mavlink.autopilot_version.uid2", "UID 2", base.HEX)
fields.AUTOPILOT_VERSION_uid2_0 = ProtoField.uint8("mavlink.autopilot_version.uid2.0", "UID 2 [0]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_1 = ProtoField.uint8("mavlink.autopilot_version.uid2.1", "UID 2 [1]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_2 = ProtoField.uint8("mavlink.autopilot_version.uid2.2", "UID 2 [2]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_3 = ProtoField.uint8("mavlink.autopilot_version.uid2.3", "UID 2 [3]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_4 = ProtoField.uint8("mavlink.autopilot_version.uid2.4", "UID 2 [4]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_5 = ProtoField.uint8("mavlink.autopilot_version.uid2.5", "UID 2 [5]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_6 = ProtoField.uint8("mavlink.autopilot_version.uid2.6", "UID 2 [6]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_7 = ProtoField.uint8("mavlink.autopilot_version.uid2.7", "UID 2 [7]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_8 = ProtoField.uint8("mavlink.autopilot_version.uid2.8", "UID 2 [8]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_9 = ProtoField.uint8("mavlink.autopilot_version.uid2.9", "UID 2 [9]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_10 = ProtoField.uint8("mavlink.autopilot_version.uid2.10", "UID 2 [10]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_11 = ProtoField.uint8("mavlink.autopilot_version.uid2.11", "UID 2 [11]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_12 = ProtoField.uint8("mavlink.autopilot_version.uid2.12", "UID 2 [12]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_13 = ProtoField.uint8("mavlink.autopilot_version.uid2.13", "UID 2 [13]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_14 = ProtoField.uint8("mavlink.autopilot_version.uid2.14", "UID 2 [14]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_15 = ProtoField.uint8("mavlink.autopilot_version.uid2.15", "UID 2 [15]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_16 = ProtoField.uint8("mavlink.autopilot_version.uid2.16", "UID 2 [16]", base.DEC)
fields.AUTOPILOT_VERSION_uid2_17 = ProtoField.uint8("mavlink.autopilot_version.uid2.17", "UID 2 [17]", base.DEC)

-- Fields for message type 230: ESTIMATOR_STATUS
fields.ESTIMATOR_STATUS_time_usec = ProtoField.uint64("mavlink.estimator_status.time_usec", "Timestamp [us]", base.DEC)
fields.ESTIMATOR_STATUS_flags = ProtoField.uint16("mavlink.estimator_status.flags", "Flags", base.HEX, nil, 0xFFFF)
fields.ESTIMATOR_STATUS_flags_estimator_attitude = ProtoField.uint16("mavlink.estimator_status.flags.estimator_attitude", "estimator_attitude", base.DEC, nil, 0x0001)
fields.ESTIMATOR_STATUS_flags_estimator_velocity_horiz = ProtoField.uint16("mavlink.estimator_status.flags.estimator_velocity_horiz", "estimator_velocity_horiz", base.DEC, nil, 0x0002)
fields.ESTIMATOR_STATUS_flags_estimator_velocity_vert = ProtoField.uint16("mavlink.estimator_status.flags.estimator_velocity_vert", "estimator_velocity_vert", base.DEC, nil, 0x0004)
fields.ESTIMATOR_STATUS_flags_estimator_pos_horiz_rel = ProtoField.uint16("mavlink.estimator_status.flags.estimator_pos_horiz_rel", "estimator_pos_horiz_rel", base.DEC, nil, 0x0008)
fields.ESTIMATOR_STATUS_flags_estimator_pos_horiz_abs = ProtoField.uint16("mavlink.estimator_status.flags.estimator_pos_horiz_abs", "estimator_pos_horiz_abs", base.DEC, nil, 0x0010)
fields.ESTIMATOR_STATUS_flags_estimator_pos_vert_abs = ProtoField.uint16("mavlink.estimator_status.flags.estimator_pos_vert_abs", "estimator_pos_vert_abs", base.DEC, nil, 0x0020)
fields.ESTIMATOR_STATUS_flags_estimator_pos_vert_agl = ProtoField.uint16("mavlink.estimator_status.flags.estimator_pos_vert_agl", "estimator_pos_vert_agl", base.DEC, nil, 0x0040)
fields.ESTIMATOR_STATUS_flags_estimator_const_pos_mode = ProtoField.uint16("mavlink.estimator_status.flags.estimator_const_pos_mode", "estimator_const_pos_mode", base.DEC, nil, 0x0080)
fields.ESTIMATOR_STATUS_flags_estimator_pred_pos_horiz_rel = ProtoField.uint16("mavlink.estimator_status.flags.estimator_pred_pos_horiz_rel", "estimator_pred_pos_horiz_rel", base.DEC, nil, 0x0100)
fields.ESTIMATOR_STATUS_flags_estimator_pred_pos_horiz_abs = ProtoField.uint16("mavlink.estimator_status.flags.estimator_pred_pos_horiz_abs", "estimator_pred_pos_horiz_abs", base.DEC, nil, 0x0200)
fields.ESTIMATOR_STATUS_flags_estimator_gps_glitch = ProtoField.uint16("mavlink.estimator_status.flags.estimator_gps_glitch", "estimator_gps_glitch", base.DEC, nil, 0x0400)
fields.ESTIMATOR_STATUS_flags_estimator_accel_error = ProtoField.uint16("mavlink.estimator_status.flags.estimator_accel_error", "estimator_accel_error", base.DEC, nil, 0x0800)
fields.ESTIMATOR_STATUS_vel_ratio = ProtoField.float("mavlink.estimator_status.vel_ratio", "Velocity innovation test ratio", base.DEC)
fields.ESTIMATOR_STATUS_pos_horiz_ratio = ProtoField.float("mavlink.estimator_status.pos_horiz_ratio", "Horizontal position innovation test ratio", base.DEC)
fields.ESTIMATOR_STATUS_pos_vert_ratio = ProtoField.float("mavlink.estimator_status.pos_vert_ratio", "Vertical position innovation test ratio", base.DEC)
fields.ESTIMATOR_STATUS_mag_ratio = ProtoField.float("mavlink.estimator_status.mag_ratio", "Magnetometer innovation test ratio", base.DEC)
fields.ESTIMATOR_STATUS_hagl_ratio = ProtoField.float("mavlink.estimator_status.hagl_ratio", "Height above terrain innovation test ratio", base.DEC)
fields.ESTIMATOR_STATUS_tas_ratio = ProtoField.float("mavlink.estimator_status.tas_ratio", "True airspeed innovation test ratio", base.DEC)
fields.ESTIMATOR_STATUS_pos_horiz_accuracy = ProtoField.float("mavlink.estimator_status.pos_horiz_accuracy", "Horizontal position 1-STD accuracy [m]", base.DEC)
fields.ESTIMATOR_STATUS_pos_vert_accuracy = ProtoField.float("mavlink.estimator_status.pos_vert_accuracy", "Vertical position 1-STD accuracy [m]", base.DEC)

-- Fields for message type 231: WIND_COV
fields.WIND_COV_time_usec = ProtoField.uint64("mavlink.wind_cov.time_usec", "Timestamp [us]", base.DEC)
fields.WIND_COV_wind_x = ProtoField.float("mavlink.wind_cov.wind_x", "Wind in X [m/s]", base.DEC)
fields.WIND_COV_wind_y = ProtoField.float("mavlink.wind_cov.wind_y", "Wind in Y [m/s]", base.DEC)
fields.WIND_COV_wind_z = ProtoField.float("mavlink.wind_cov.wind_z", "Wind in Z [m/s]", base.DEC)
fields.WIND_COV_var_horiz = ProtoField.float("mavlink.wind_cov.var_horiz", "Variability of the wind XY [m/s]", base.DEC)
fields.WIND_COV_var_vert = ProtoField.float("mavlink.wind_cov.var_vert", "Variability of the wind Z [m/s]", base.DEC)
fields.WIND_COV_wind_alt = ProtoField.float("mavlink.wind_cov.wind_alt", "AMSL altitude [m]", base.DEC)
fields.WIND_COV_horiz_accuracy = ProtoField.float("mavlink.wind_cov.horiz_accuracy", "Horizontal speed accuracy [m]", base.DEC)
fields.WIND_COV_vert_accuracy = ProtoField.float("mavlink.wind_cov.vert_accuracy", "Vertical speed accuracy [m]", base.DEC)

-- Fields for message type 241: VIBRATION
fields.VIBRATION_time_usec = ProtoField.uint64("mavlink.vibration.time_usec", "Timestamp [us]", base.DEC)
fields.VIBRATION_vibration_x = ProtoField.float("mavlink.vibration.x", "Vibration X-axis", base.DEC)
fields.VIBRATION_vibration_y = ProtoField.float("mavlink.vibration.y", "Vibration Y-axis", base.DEC)
fields.VIBRATION_vibration_z = ProtoField.float("mavlink.vibration.z", "Vibration Z-axis", base.DEC)
fields.VIBRATION_clipping_0 = ProtoField.uint32("mavlink.clipping_0", "First accelerometer clipping count", base.DEC)
fields.VIBRATION_clipping_1 = ProtoField.uint32("mavlink.clipping_1", "Second accelerometer clipping count", base.DEC)
fields.VIBRATION_clipping_2 = ProtoField.uint32("mavlink.clipping_2", "Third accelerometer clipping count", base.DEC)

-- Fields for message type 245: EXTENDED_SYS_STATE
fields.EXTENDED_SYS_STATE_vtol_state = ProtoField.uint8("mavlink.extended_sys_state.vtol_state", "VTOL state", base.DEC, mavlinkVTOLState)
fields.EXTENDED_SYS_STATE_landed_state = ProtoField.uint8("mavlink.extended_sys_state.landed_state", "Landed state", base.DEC, mavlinkLandedState)

-- Fields for message type 253: STATUSTEXT
fields.STATUSTEXT_severity = ProtoField.uint8("mavlink.statustext.severity", "Severity", base.DEC, mavlinkSeverity)
fields.STATUSTEXT_text = ProtoField.string("mavlink.statustext.text", "Text", base.NONE)

-- Payload dissector functions

-- dissect payload of message type HEARTBEAT
function payload_fns.payload_0 (buffer, tree, offset, size)
    if (size == 9) then
        tree:add(fields.HEARTBEAT_type, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.HEARTBEAT_autopilot, buffer(offset, 1))
        offset = offset + 1
        local base_mode = tree:add(fields.HEARTBEAT_base_mode, buffer(offset, 1))
        base_mode:add(fields.HEARTBEAT_base_mode_custom, buffer(offset, 1))
        base_mode:add(fields.HEARTBEAT_base_mode_test, buffer(offset, 1))
        base_mode:add(fields.HEARTBEAT_base_mode_auto, buffer(offset, 1))
        base_mode:add(fields.HEARTBEAT_base_mode_guided, buffer(offset, 1))
        base_mode:add(fields.HEARTBEAT_base_mode_stabilize, buffer(offset, 1))
        base_mode:add(fields.HEARTBEAT_base_mode_hil, buffer(offset, 1))
        base_mode:add(fields.HEARTBEAT_base_mode_manual_input, buffer(offset, 1))
        base_mode:add(fields.HEARTBEAT_base_mode_safety_armed, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.HEARTBEAT_custom_mode, buffer(offset, 4))
        offset = offset + 4
        tree:add(fields.HEARTBEAT_system_status, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.HEARTBEAT_mavlink_version, buffer(offset, 1))
        offset = offset + 1
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed HEARTBEAT message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type SYS_STATUS
function payload_fns.payload_1 (buffer, tree, offset, size)
    if (size == 31) then
        local present = tree:add_le(fields.SYS_STATUS_control_sensors_present, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_3d_gyro, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_3d_accel, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_3d_mag, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_absolute_pressure, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_differential_pressure, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_gps, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_optical_flow, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_vision_position, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_laser_position, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_external_ground_truth, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_angular_rate_control, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_attitude_stabilization, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_yaw_position, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_z_altitude_control, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_xy_position_control, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_motor_outputs, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_rc_receiver, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_3d_gyro2, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_3d_accel2, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_3d_mag2, buffer(offset, 4))
        present:add_le(fields.SYS_STATUS_control_sensors_present_battery, buffer(offset, 4))
        offset = offset + 4
        local enabled = tree:add_le(fields.SYS_STATUS_control_sensors_enabled, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_3d_gyro, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_3d_accel, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_3d_mag, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_absolute_pressure, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_differential_pressure, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_gps, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_optical_flow, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_vision_position, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_laser_position, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_external_ground_truth, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_angular_rate_control, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_attitude_stabilization, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_yaw_position, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_z_altitude_control, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_xy_position_control, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_motor_outputs, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_rc_receiver, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_3d_gyro2, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_3d_accel2, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_3d_mag2, buffer(offset, 4))
        enabled:add_le(fields.SYS_STATUS_control_sensors_enabled_battery, buffer(offset, 4))
        offset = offset + 4
        local health = tree:add_le(fields.SYS_STATUS_control_sensors_health, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_3d_gyro, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_3d_accel, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_3d_mag, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_absolute_pressure, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_differential_pressure, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_gps, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_optical_flow, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_vision_position, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_laser_position, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_external_ground_truth, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_angular_rate_control, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_attitude_stabilization, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_yaw_position, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_z_altitude_control, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_xy_position_control, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_motor_outputs, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_rc_receiver, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_3d_gyro2, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_3d_accel2, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_3d_mag2, buffer(offset, 4))
        health:add_le(fields.SYS_STATUS_control_sensors_health_battery, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.SYS_STATUS_load, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SYS_STATUS_voltage_battery, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SYS_STATUS_current_battery, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SYS_STATUS_battery_remaining, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.SYS_STATUS_drop_rate_comm, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SYS_STATUS_errors_comm, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SYS_STATUS_errors_count1, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SYS_STATUS_errors_count2, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SYS_STATUS_errors_count3, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SYS_STATUS_errors_count4, buffer(offset, 2))
        offset = offset + 2
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed SYS_STATUS message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type SYSTEM_TIME
function payload_fns.payload_2 (buffer, tree, offset, size)
    if (size == 12) then
        tree:add_le(fields.SYSTEM_TIME_time_unix_usec, buffer(offset, 8))
        offset = offset + 8
        tree:add_le(fields.SYSTEM_TIME_boot_ms, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed SYSTEM_TIME message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type SET_MODE
function payload_fns.payload_11 (buffer, tree, offset, size)
    if (size == 6) then
        tree:add(fields.SET_MODE_target_system, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.SET_MODE_base_mode, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.SET_MODE_custom_mode, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed SET_MODE message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type PARAM_REQUEST_READ
function payload_fns.payload_20 (buffer, tree, offset, size)
    if (size == 20) then
        tree:add(fields.PARAM_REQUEST_READ_target_system, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.PARAM_REQUEST_READ_target_component, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.PARAM_REQUEST_READ_param_id, buffer(offset, 16))
        offset = offset + 16
        tree:add_le(fields.PARAM_REQUEST_READ_param_index, buffer(offset, 2))
        offset = offset + 2
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed PARAM_REQUEST_READ message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type PARAM_REQUEST_LIST
function payload_fns.payload_21(buffer, tree, offset, size)
    if (size == 2) then
        tree:add_le(fields.PARAM_REQUEST_LIST_target_system, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.PARAM_REQUEST_LIST_target_component, buffer(offset, 1))
        offset = offset + 1
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed PARAM_REQUEST_LIST message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type PARAM_VALUE
function payload_fns.payload_22 (buffer, tree, offset, size)
    if (size == 25) then
        tree:add(fields.PARAM_VALUE_param_id, buffer(offset, 16))
        offset = offset + 16
        tree:add_le(fields.PARAM_VALUE_param_value, buffer(offset, 4))
        offset = offset + 4
        tree:add(fields.PARAM_VALUE_param_type, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.PARAM_VALUE_param_count, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.PARAM_VALUE_param_index, buffer(offset, 2))
        offset = offset + 2
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed PARAM_VALUE message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type PARAM_SET
function payload_fns.payload_23 (buffer, tree, offset, size)
    if (size == 23) then
        tree:add(fields.PARAM_SET_target_system, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.PARAM_SET_target_component, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.PARAM_SET_param_id, buffer(offset, 16))
        offset = offset + 16
        tree:add(fields.PARAM_SET_param_value, buffer(offset, 4))
        offset = offset + 4
        tree:add(fields.PARAM_SET_param_type, buffer(offset, 1))
        offset = offset + 1
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed PARAM_SET message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type RAW_PRESSURE
function payload_fns.payload_28 (buffer, tree, offset, size)
    if (size == 16) then
        tree:add_le(fields.RAW_PRESSURE_time_usec, buffer(offset, 8))
        offset = offset + 8
        tree:add_le(fields.RAW_PRESSURE_press_abs, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RAW_PRESSURE_press_diff1, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RAW_PRESSURE_press_diff2, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RAW_PRESSURE_temperature, buffer(offset, 2))
        offset = offset + 2
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed RAW_PRESSURE message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type ATTITUDE
function payload_fns.payload_30 (buffer, tree, offset, size)
    if (size == 28) then
        tree:add_le(fields.ATTITUDE_time_boot_ms, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_roll, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_pitch, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_yaw, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_rollspeed, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_pitchspeed, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_yawspeed, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed ATTITUDE message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type ATTITUDE_QUATERION
function payload_fns.payload_31 (buffer, tree, offset, size)
    if (size == 32) then
        tree:add_le(fields.ATTITUDE_QUATERNION_time_boot_ms, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_QUATERNION_q1, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_QUATERNION_q2, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_QUATERNION_q3, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_QUATERNION_q4, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_QUATERNION_rollspeed, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_QUATERNION_pitchspeed, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_QUATERNION_yawspeed, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload, buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed ATTITUDE_QUATERNION message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type LOCAL_POSITION_NED
function payload_fns.payload_32 (buffer, tree, offset, size)
    if (size == 28) then
        tree:add_le(fields.LOCAL_POSITION_NED_time_boot_ms, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.LOCAL_POSITION_NED_x, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.LOCAL_POSITION_NED_y, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.LOCAL_POSITION_NED_z, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.LOCAL_POSITION_NED_vx, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.LOCAL_POSITION_NED_vy, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.LOCAL_POSITION_NED_vz, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed LOCAL_POSITION_NED message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type GLOBAL_POSITION_INT
function payload_fns.payload_33 (buffer, tree, offset, size)
    if (size == 28) then
        tree:add_le(fields.GLOBAL_POSITION_INT_time_boot_ms, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.GLOBAL_POSITION_INT_lat, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.GLOBAL_POSITION_INT_lon, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.GLOBAL_POSITION_INT_alt, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.GLOBAL_POSITION_INT_relative_alt, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.GLOBAL_POSITION_INT_vx, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.GLOBAL_POSITION_INT_vy, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.GLOBAL_POSITION_INT_vz, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.GLOBAL_POSITION_INT_hdg, buffer(offset, 2))
        offset = offset + 2
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed LOCAL_POSITION_NED message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type SERVO_OUTPUT_RAW
function payload_fns.payload_36 (buffer, tree, offset, size)
    if (size == 21 or size == 37) then
        tree:add_le(fields.SERVO_OUTPUT_RAW_time_usec, buffer(offset, 4))
        offset = offset + 4
        tree:add(fields.SERVO_OUTPUT_RAW_port, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.SERVO_OUTPUT_RAW_servo1_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SERVO_OUTPUT_RAW_servo2_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SERVO_OUTPUT_RAW_servo3_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SERVO_OUTPUT_RAW_servo4_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SERVO_OUTPUT_RAW_servo5_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SERVO_OUTPUT_RAW_servo6_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SERVO_OUTPUT_RAW_servo7_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.SERVO_OUTPUT_RAW_servo8_raw, buffer(offset, 2))
        offset = offset + 2
        if (size == 37) then
            tree:add_le(fields.SERVO_OUTPUT_RAW_servo9_raw, buffer(offset, 2))
            offset = offset + 2
            tree:add_le(fields.SERVO_OUTPUT_RAW_servo10_raw, buffer(offset, 2))
            offset = offset + 2
            tree:add_le(fields.SERVO_OUTPUT_RAW_servo11_raw, buffer(offset, 2))
            offset = offset + 2
            tree:add_le(fields.SERVO_OUTPUT_RAW_servo12_raw, buffer(offset, 2))
            offset = offset + 2
            tree:add_le(fields.SERVO_OUTPUT_RAW_servo13_raw, buffer(offset, 2))
            offset = offset + 2
            tree:add_le(fields.SERVO_OUTPUT_RAW_servo14_raw, buffer(offset, 2))
            offset = offset + 2
            tree:add_le(fields.SERVO_OUTPUT_RAW_servo15_raw, buffer(offset, 2))
            offset = offset + 2
            tree:add_le(fields.SERVO_OUTPUT_RAW_servo16_raw, buffer(offset, 2))
            offset = offset + 2
        end
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed SERVO_OUTPUT_RAW message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type MISSION_ITEM
function payload_fns.payload_39 (buffer, tree, offset, size)
    if (size == 37 or size == 38) then
        tree:add(fields.MISSION_ITEM_target_system, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.MISSION_ITEM_target_component, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.MISSION_ITEM_seq, buffer(offset, 2))
        offset = offset + 2
        tree:add(fields.MISSION_ITEM_frame, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.MISSION_ITEM_command, buffer(offset, 2))
        offset = offset + 2
        tree:add(fields.MISSION_ITEM_current, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.MISSION_ITEM_autocontinue, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.MISSION_ITEM_param1, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.MISSION_ITEM_param2, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.MISSION_ITEM_param3, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.MISSION_ITEM_param4, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.MISSION_ITEM_x, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.MISSION_ITEM_y, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.MISSION_ITEM_z, buffer(offset, 4))
        offset = offset + 4
        if (size == 38) then
            tree:add(fields.MISSION_ITEM_mission_type, buffer(offset, 1))
            offset = offset + 1
        end
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed MISSION_ITEM message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type MISSION_CURRENT
function payload_fns.payload_42 (buffer, tree, offset, size)
    if (size == 2) then
        tree:add_le(fields.MISSION_CURRENT_seq, buffer(offset, 2))
        offset = offset + 2
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed MISSION_CURRENT message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type MISSION_REQUEST_LIST
function payload_fns.payload_43 (buffer, tree, offset, size)
    if (size == 2 or size == 3) then
        tree:add(fields.MISSION_REQUEST_LIST_target_system, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.MISSION_REQUEST_LIST_target_component, buffer(offset, 1))
        offset = offset + 1
        if (size == 3) then
            tree:add(fields.MISSION_REQUEST_LIST_mission_type, buffer(offset, 1))
            offset = offset + 1
        end
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed MISSION_REQUEST_LIST message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type MISSION_COUNT
function payload_fns.payload_44 (buffer, tree, offset, size)
    if (size == 4 or size == 5) then
        tree:add(fields.MISSION_COUNT_target_system, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.MISSION_COUNT_target_component, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.MISSION_COUNT_count, buffer(offset, 2))
        offset = offset + 2
        if (size == 5) then
            tree:add(fields.MISSION_COUNT_mission_type, buffer(offset, 1))
            offset = offset + 1
        end
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed MISSION_COUNT message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type MISSION_CLEAR_ALL
function payload_fns.payload_45 (buffer, tree, offset, size)
    if (size == 2 or size == 3) then
        tree:add(fields.MISSION_CLEAR_ALL_target_system, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.MISSION_CLEAR_ALL_target_component, buffer(offset, 1))
        offset = offset + 1
        if (size == 3) then
            tree:add(fields.MISSION_CLEAR_ALL_mission_type, buffer(offset, 1))
            offset = offset + 1
        end
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed MISSION_CLEAR_ALL message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type MISSION_ACK
function payload_fns.payload_47 (buffer, tree, offset, size)
    if (size == 3 or size == 4) then
        tree:add(fields.MISSION_ACK_target_system, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.MISSION_ACK_target_component, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.MISSION_ACK_type, buffer(offset, 1))
        offset = offset + 1
        if (size == 4) then
            tree:add(fields.MISSION_ACK_mission_type, buffer(offset, 1))
            offset = offset + 1
        end
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed MISSION_ACK message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type RC_CHANNELS
function payload_fns.payload_65 (buffer, tree, offset, size)
    if (size == 42) then
        tree:add_le(fields.RC_CHANNELS_time_boot_ms, buffer(offset, 4))
        offset = offset + 4
        tree:add(fields.RC_CHANNELS_chancount, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.RC_CHANNELS_chan1_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan2_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan3_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan4_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan5_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan6_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan7_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan8_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan9_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan10_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan11_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan12_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan13_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan14_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan15_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan16_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan17_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.RC_CHANNELS_chan18_raw, buffer(offset, 2))
        offset = offset + 2
        tree:add(fields.RC_CHANNELS_rssi, buffer(offset, 1))
        offset = offset + 1
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed RC_CHANNELS message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type MANUAL_CONTROL
function payload_fns.payload_69 (buffer, tree, offset, size)
    if (size == 11) then
        tree:add(fields.MANUAL_CONTROL_target, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.MANUAL_CONTROL_x, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.MANUAL_CONTROL_y, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.MANUAL_CONTROL_z, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.MANUAL_CONTROL_r, buffer(offset, 2))
        offset = offset + 2
        local buttons = tree:add_le(fields.MANUAL_CONTROL_buttons, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_1, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_2, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_3, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_4, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_5, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_6, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_7, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_8, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_9, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_10, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_11, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_12, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_13, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_14, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_15, buffer(offset, 2))
        buttons:add_le(fields.MANUAL_CONTROL_buttons_button_16, buffer(offset, 2))
        offset = offset + 2
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed MANUAL_CONTROL message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type MISSION_ITEM_INT
function payload_fns.payload_73 (buffer, tree, offset, size)
    if (size == 37 or size == 38) then
        tree:add(fields.MISSION_ITEM_INT_target_system, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.MISSION_ITEM_INT_target_component, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.MISSION_ITEM_INT_seq, buffer(offset, 2))
        offset = offset + 2
        tree:add(fields.MISSION_ITEM_INT_frame, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.MISSION_ITEM_INT_command, buffer(offset, 2))
        offset = offset + 2
        tree:add(fields.MISSION_ITEM_INT_current, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.MISSION_ITEM_INT_autocontinue, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.MISSION_ITEM_INT_param1, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.MISSION_ITEM_INT_param2, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.MISSION_ITEM_INT_param3, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.MISSION_ITEM_INT_param4, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.MISSION_ITEM_INT_x, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.MISSION_ITEM_INT_y, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.MISSION_ITEM_INT_z, buffer(offset, 4))
        offset = offset + 4
        if (size == 38) then
            tree:add(fields.MISSION_ITEM_INT_mission_type, buffer(offset, 1))
            offset = offset + 1
        end
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed MISSION_ITEM_INT message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type VFR_HUD
function payload_fns.payload_74 (buffer, tree, offset, size)
    if (size == 20) then
        tree:add_le(fields.VFR_HUD_airspeed, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.VFR_HUD_groundspeed, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.VFR_HUD_heading, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.VFR_HUD_throttle, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.VFR_HUD_alt, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.VFR_HUD_climb, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed VFR_HUD message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type COMMAND_LONG
function payload_fns.payload_76 (buffer, tree, offset, size)
    if (size == 32 or size == 33) then
        tree:add(fields.COMMAND_LONG_target_system, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.COMMAND_LONG_target_component, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.COMMAND_LONG_command, buffer(offset, 2))
        offset = offset + 2
        if (size == 33) then
            tree:add(fields.COMMAND_LONG_confirmation, buffer(offset, 1))
            offset = offset + 1
        end
        tree:add_le(fields.COMMAND_LONG_param1, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.COMMAND_LONG_param2, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.COMMAND_LONG_param3, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.COMMAND_LONG_param4, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.COMMAND_LONG_param5, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.COMMAND_LONG_param6, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.COMMAND_LONG_param7, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed COMMAND_LONG message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type COMMAND_ACK
function payload_fns.payload_77 (buffer, tree, offset, size)
    if (size == 3 or size == 4 or size == 8 or size == 9 or size == 10) then
        tree:add_le(fields.COMMAND_ACK_command, buffer(offset, 2))
        offset = offset + 2
        tree:add(fields.COMMAND_ACK_result, buffer(offset, 1))
        offset = offset + 1
        if (size >= 4) then
            tree:add(fields.COMMAND_ACK_progress, buffer(offset, 1))
            offset = offset + 1
        end
        if (size >= 8) then
            tree:add_le(fields.COMMAND_ACK_result_param2, buffer(offset, 4))
            offset = offset + 4
        end
        if (size >= 9) then
            tree:add(fields.COMMAND_ACK_target_system, buffer(offset, 1))
            offset = offset + 1
        end
        if (size == 10) then
            tree:add(fields.COMMAND_ACK_target_component, buffer(offset, 1))
            offset = offset + 1
        end
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed COMMAND_ACK message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type ATTITUDE_TARGET
function payload_fns.payload_83 (buffer, tree, offset, size)
    if (size == 37) then
        tree:add_le(fields.ATTITUDE_TARGET_time_boot_ms, buffer(offset, 4))
        offset = offset + 4
        local type_mask = tree:add(fields.ATTITUDE_TARGET_type_mask, buffer(offset, 1))
        type_mask:add(fields.ATTITUDE_TARGET_type_mask_body_roll, buffer(offset, 1))
        type_mask:add(fields.ATTITUDE_TARGET_type_mask_body_pitch, buffer(offset, 1))
        type_mask:add(fields.ATTITUDE_TARGET_type_mask_body_yaw, buffer(offset, 1))
        type_mask:add(fields.ATTITUDE_TARGET_type_mask_reserved, buffer(offset, 1))
        type_mask:add(fields.ATTITUDE_TARGET_type_mask_attitude, buffer(offset, 1))
        offset = offset + 1
        local quaternion = tree:add(fields.ATTITUDE_TARGET_q, buffer(offset, 16))
        quaternion:add_le(fields.ATTITUDE_TARGET_q0, buffer(offset, 4))
        offset = offset + 4
        quaternion:add_le(fields.ATTITUDE_TARGET_q1, buffer(offset, 4))
        offset = offset + 4
        quaternion:add_le(fields.ATTITUDE_TARGET_q2, buffer(offset, 4))
        offset = offset + 4
        quaternion:add_le(fields.ATTITUDE_TARGET_q3, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_TARGET_body_roll_rate, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_TARGET_body_pitch_rate, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_TARGET_body_yaw_rate, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ATTITUDE_TARGET_thrust, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed ATTITUDE_TARGET message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type POSITION_TARGET_LOCAL_NED
function payload_fns.payload_85 (buffer, tree, offset, size)
    if (size == 51) then
        tree:add_le(fields.POSITION_TARGET_LOCAL_NED_time_boot_ms, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.POSITION_TARGET_LOCAL_NED_coordinate_frame, buffer(offset, 1))
        offset = offset + 1
        local tm = tree:add_le(fields.POSITION_TARGET_LOCAL_NED_type_mask, buffer(offset, 2))
        tm:add_le(fields.POSITION_TARGET_LOCAL_NED_type_mask_x, buffer(offset, 2))
        tm:add_le(fields.POSITION_TARGET_LOCAL_NED_type_mask_y, buffer(offset, 2))
        tm:add_le(fields.POSITION_TARGET_LOCAL_NED_type_mask_z, buffer(offset, 2))
        tm:add_le(fields.POSITION_TARGET_LOCAL_NED_type_mask_vx, buffer(offset, 2))
        tm:add_le(fields.POSITION_TARGET_LOCAL_NED_type_mask_vy, buffer(offset, 2))
        tm:add_le(fields.POSITION_TARGET_LOCAL_NED_type_mask_vz, buffer(offset, 2))
        tm:add_le(fields.POSITION_TARGET_LOCAL_NED_type_mask_ax, buffer(offset, 2))
        tm:add_le(fields.POSITION_TARGET_LOCAL_NED_type_mask_ay, buffer(offset, 2))
        tm:add_le(fields.POSITION_TARGET_LOCAL_NED_type_mask_az, buffer(offset, 2))
        tm:add_le(fields.POSITION_TARGET_LOCAL_NED_type_mask_fsp, buffer(offset, 2))
        tm:add_le(fields.POSITION_TARGET_LOCAL_NED_type_mask_yaw, buffer(offset, 2))
        tm:add_le(fields.POSITION_TARGET_LOCAL_NED_type_mask_yaw_rate, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.POSITION_TARGET_LOCAL_NED_x, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.POSITION_TARGET_LOCAL_NED_y, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.POSITION_TARGET_LOCAL_NED_z, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.POSITION_TARGET_LOCAL_NED_vx, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.POSITION_TARGET_LOCAL_NED_vy, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.POSITION_TARGET_LOCAL_NED_vz, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.POSITION_TARGET_LOCAL_NED_afx, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.POSITION_TARGET_LOCAL_NED_afy, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.POSITION_TARGET_LOCAL_NED_afz, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.POSITION_TARGET_LOCAL_NED_yaw, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.POSITION_TARGET_LOCAL_NED_yaw_rate, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed POSITION_TARGET_LOCAL_NED message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type HIL_ACTUATOR_CONTROLS
function payload_fns.payload_93 (buffer, tree, offset, size)
    if (size == 81) then
        tree:add_le(fields.HIL_ACTUATOR_CONTROLS_time_usec, buffer(offset, 8))
        offset = offset + 8
        local controls = tree:add(fields.HIL_ACTUATOR_CONTROLS_controls, buffer(offset, 64))
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_0, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_1, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_2, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_3, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_4, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_5, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_6, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_7, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_8, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_9, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_10, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_11, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_12, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_13, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_14, buffer(offset, 4))
        offset = offset + 4
        controls:add_le(fields.HIL_ACTUATOR_CONTROLS_controls_15, buffer(offset, 4))
        offset = offset + 4
        tree:add(fields.HIL_ACTUATOR_CONTROLS_mode, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.HIL_ACTUATOR_CONTROLS_flags, buffer(offset, 8))
        offset = offset + 8
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed HIL_ACTUATOR_CONTROLS message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type HIGHRES_IMU
function payload_fns.payload_105 (buffer, tree, offset, size)
    if (size == 62) then
        tree:add_le(fields.HIGHRES_IMU_time_usec, buffer(offset, 8))
        offset = offset + 8
        tree:add_le(fields.HIGHRES_IMU_xacc, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIGHRES_IMU_yacc, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIGHRES_IMU_zacc, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIGHRES_IMU_xgyro, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIGHRES_IMU_ygyro, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIGHRES_IMU_zgyro, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIGHRES_IMU_xmag, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIGHRES_IMU_ymag, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIGHRES_IMU_zmag, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIGHRES_IMU_abs_pressure, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIGHRES_IMU_diff_pressure, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIGHRES_IMU_pressure_alt, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIGHRES_IMU_temperature, buffer(offset, 4))
        offset = offset + 4
        local f_updated = tree:add_le(fields.HIGHRES_IMU_fields_updated, buffer(offset, 2))
        f_updated:add_le(fields.HIGHRES_IMU_fields_updated_xacc, buffer(offset, 2))
        f_updated:add_le(fields.HIGHRES_IMU_fields_updated_yacc, buffer(offset, 2))
        f_updated:add_le(fields.HIGHRES_IMU_fields_updated_zacc, buffer(offset, 2))
        f_updated:add_le(fields.HIGHRES_IMU_fields_updated_xgyro, buffer(offset, 2))
        f_updated:add_le(fields.HIGHRES_IMU_fields_updated_ygyro, buffer(offset, 2))
        f_updated:add_le(fields.HIGHRES_IMU_fields_updated_zgyro, buffer(offset, 2))
        f_updated:add_le(fields.HIGHRES_IMU_fields_updated_xmag, buffer(offset, 2))
        f_updated:add_le(fields.HIGHRES_IMU_fields_updated_ymag, buffer(offset, 2))
        f_updated:add_le(fields.HIGHRES_IMU_fields_updated_zmag, buffer(offset, 2))
        f_updated:add_le(fields.HIGHRES_IMU_fields_updated_abs_pressure, buffer(offset, 2))
        f_updated:add_le(fields.HIGHRES_IMU_fields_updated_diff_pressure, buffer(offset, 2))
        f_updated:add_le(fields.HIGHRES_IMU_fields_updated_pressure_alt, buffer(offset, 2))
        f_updated:add_le(fields.HIGHRES_IMU_fields_updated_temperature, buffer(offset, 2))
        offset = offset + 2
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed HIGHRES_IMU message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type HIL_SENSOR
function payload_fns.payload_107 (buffer, tree, offset, size)
    if (size == 64) then
        tree:add_le(fields.HIL_SENSOR_time_usec, buffer(offset, 8))
        offset = offset + 8
        tree:add_le(fields.HIL_SENSOR_xacc, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_SENSOR_yacc, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_SENSOR_zacc, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_SENSOR_xgyro, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_SENSOR_ygyro, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_SENSOR_zgyro, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_SENSOR_xmag, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_SENSOR_ymag, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_SENSOR_zmag, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_SENSOR_abs_pressure, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_SENSOR_diff_pressure, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_SENSOR_pressure_alt, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_SENSOR_temperature, buffer(offset, 4))
        offset = offset + 4
        local f_updated = tree:add_le(fields.HIL_SENSOR_fields_updated, buffer(offset, 4))
        f_updated:add_le(fields.HIL_SENSOR_fields_updated_xacc, buffer(offset, 4))
        f_updated:add_le(fields.HIL_SENSOR_fields_updated_yacc, buffer(offset, 4))
        f_updated:add_le(fields.HIL_SENSOR_fields_updated_zacc, buffer(offset, 4))
        f_updated:add_le(fields.HIL_SENSOR_fields_updated_xgyro, buffer(offset, 4))
        f_updated:add_le(fields.HIL_SENSOR_fields_updated_ygyro, buffer(offset, 4))
        f_updated:add_le(fields.HIL_SENSOR_fields_updated_zgyro, buffer(offset, 4))
        f_updated:add_le(fields.HIL_SENSOR_fields_updated_xmag, buffer(offset, 4))
        f_updated:add_le(fields.HIL_SENSOR_fields_updated_ymag, buffer(offset, 4))
        f_updated:add_le(fields.HIL_SENSOR_fields_updated_zmag, buffer(offset, 4))
        f_updated:add_le(fields.HIL_SENSOR_fields_updated_abs_pressure, buffer(offset, 4))
        f_updated:add_le(fields.HIL_SENSOR_fields_updated_diff_pressure, buffer(offset, 4))
        f_updated:add_le(fields.HIL_SENSOR_fields_updated_pressure_alt, buffer(offset, 4))
        f_updated:add_le(fields.HIL_SENSOR_fields_updated_temperature, buffer(offset, 4))
        f_updated:add_le(fields.HIL_SENSOR_fields_updated_full_reset, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed HIL_SENSOR message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type TIMESYNC
function payload_fns.payload_111 (buffer, tree, offset, size)
    if (size == 16) then
        tree:add_le(fields.TIMESYNC_tc1, buffer(offset, 8))
        offset = offset + 8
        tree:add_le(fields.TIMESYNC_ts1, buffer(offset, 8))
        offset = offset + 8
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed TIMESYNC message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type HIL_GPS
function payload_fns.payload_113 (buffer, tree, offset, size)
    if (size == 36) then
        tree:add_le(fields.HIL_GPS_time_usec, buffer(offset, 8))
        offset = offset + 8
        tree:add(fields.HIL_GPS_fix_type, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.HIL_GPS_lat, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_GPS_lon, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_GPS_alt, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_GPS_eph, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.HIL_GPS_epv, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.HIL_GPS_vel, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.HIL_GPS_vn, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.HIL_GPS_ve, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.HIL_GPS_vd, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.HIL_GPS_cog, buffer(offset, 2))
        offset = offset + 2
        tree:add(fields.HIL_GPS_satellites_visible, buffer(offset, 1))
        offset = offset + 1
    else
        tree:add(fields.rawpayload,buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed HIL_GPS message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type HIL_STATE_QUATERNION
function payload_fns.payload_115 (buffer, tree, offset, size)
    if (size == 64) then
        tree:add_le(fields.HIL_GPS_time_usec, buffer(offset, 8))
        offset = offset + 8
        local att_quat = tree:add(fields.HIL_STATE_QUATERNION_attitude_quaternion, buffer(offset, 16))
        att_quat:add_le(fields.HIL_STATE_QUATERNION_attitude_quaternion_0, buffer(offset, 4))
        offset = offset + 4
        att_quat:add_le(fields.HIL_STATE_QUATERNION_attitude_quaternion_1, buffer(offset, 4))
        offset = offset + 4
        att_quat:add_le(fields.HIL_STATE_QUATERNION_attitude_quaternion_2, buffer(offset, 4))
        offset = offset + 4
        att_quat:add_le(fields.HIL_STATE_QUATERNION_attitude_quaternion_3, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_STATE_QUATERNION_rollspeed, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_STATE_QUATERNION_pitchspeed, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_STATE_QUATERNION_yawspeed, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_STATE_QUATERNION_lat, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_STATE_QUATERNION_lon, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_STATE_QUATERNION_alt, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.HIL_STATE_QUATERNION_vx, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.HIL_STATE_QUATERNION_vy, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.HIL_STATE_QUATERNION_vz, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.HIL_STATE_QUATERNION_ind_airspeed, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.HIL_STATE_QUATERNION_true_airspeed, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.HIL_STATE_QUATERNION_xacc, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.HIL_STATE_QUATERNION_yacc, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.HIL_STATE_QUATERNION_zacc, buffer(offset, 2))
        offset = offset + 2
    else
        tree:add(fields.rawpayload, buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed HIL_STATE_QUATERNION message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type ACTUATOR_CONTROL_TARGET
function payload_fns.payload_140 (buffer, tree, offset, size)
    if (size == 41) then
        tree:add_le(fields.ACTUATOR_CONTROL_TARGET_time_usec, buffer(offset, 8))
        offset = offset + 8
        tree:add(fields.ACTUATOR_CONTROL_TARGET_group_mlx, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.ACTUATOR_CONTROL_TARGET_control_0, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ACTUATOR_CONTROL_TARGET_control_1, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ACTUATOR_CONTROL_TARGET_control_2, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ACTUATOR_CONTROL_TARGET_control_3, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ACTUATOR_CONTROL_TARGET_control_4, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ACTUATOR_CONTROL_TARGET_control_5, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ACTUATOR_CONTROL_TARGET_control_6, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ACTUATOR_CONTROL_TARGET_control_7, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload, buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed ACTUATOR_CONTROL_TARGET message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type ALTITUDE
function payload_fns.payload_141 (buffer, tree, offset, size)
    if (size == 32) then
        tree:add_le(fields.ALTITUDE_time_usec, buffer(offset, 8))
        offset = offset + 8
        tree:add_le(fields.ALTITUDE_altitude_monotonic, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ALTITUDE_altitude_amsl, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ALTITUDE_altitude_local, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ALTITUDE_altitude_relative, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ALTITUDE_altitude_terrain, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ALTITUDE_altitude_clearance, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload, buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed HIL_STATE_QUATERNION message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type BATTERY_STATUS
function payload_fns.payload_147 (buffer, tree, offset, size)
    if (size == 36 or size == 41) then
        tree:add(fields.BATTERY_STATUS_id, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.BATTERY_STATUS_battery_function, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.BATTERY_STATUS_type, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.BATTERY_STATUS_temperature, buffer(offset, 2))
        offset = offset + 2
        local voltages = tree:add(fields.BATTERY_STATUS_voltages, buffer(offset, 20))
        voltages:set_text("Battery voltage of cells")
        voltages:add_le(fields.BATTERY_STATUS_voltages_0, buffer(offset, 2))
        offset = offset + 2
        voltages:add_le(fields.BATTERY_STATUS_voltages_1, buffer(offset, 2))
        offset = offset + 2
        voltages:add_le(fields.BATTERY_STATUS_voltages_2, buffer(offset, 2))
        offset = offset + 2
        voltages:add_le(fields.BATTERY_STATUS_voltages_3, buffer(offset, 2))
        offset = offset + 2
        voltages:add_le(fields.BATTERY_STATUS_voltages_4, buffer(offset, 2))
        offset = offset + 2
        voltages:add_le(fields.BATTERY_STATUS_voltages_5, buffer(offset, 2))
        offset = offset + 2
        voltages:add_le(fields.BATTERY_STATUS_voltages_6, buffer(offset, 2))
        offset = offset + 2
        voltages:add_le(fields.BATTERY_STATUS_voltages_7, buffer(offset, 2))
        offset = offset + 2
        voltages:add_le(fields.BATTERY_STATUS_voltages_8, buffer(offset, 2))
        offset = offset + 2
        voltages:add_le(fields.BATTERY_STATUS_voltages_9, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.BATTERY_STATUS_current_battery, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.BATTERY_STATUS_current_consumed, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.BATTERY_STATUS_energy_consumed, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.BATTERY_STATUS_battery_remaining, buffer(offset, 1))
        offset = offset + 1
        if (size == 41) then
            tree:add_le(fields.BATTERY_STATUS_time_remaining, buffer(offset, 4))
            offset = offset + 4
            tree:add_le(fields.BATTERY_STATUS_charge_state, buffer(offset, 1))
            offset = offset + 1
        end
    else
        tree:add(fields.rawpayload, buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed BATTERY_STATUS message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type AUTOPILOT_VERSION
function payload_fns.payload_148 (buffer, tree, offset, size)
    if(size == 60 or size == 78) then
        local capabilities = tree:add_le(fields.AUTOPILOT_VERSION_capabilities, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_mission_float, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_param_float, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_mission_int, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_command_int, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_param_union, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_ftp, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_set_attitude_target, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_set_position_target_local_ned, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_set_position_target_global_int, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_terrain, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_set_actuator_target, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_flight_termination, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_compass_calibration, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_mavlink2, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_mission_fence, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_mission_rally, buffer(offset, 8))
        capabilities:add_le(fields.AUTOPILOT_VERSION_capabilities_flight_information, buffer(offset, 8))
        offset = offset + 8
        tree:add_le(fields.AUTOPILOT_VERSION_flight_sw_version, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.AUTOPILOT_VERSION_middleware_sw_version, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.AUTOPILOT_VERSION_os_sw_version, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.AUTOPILOT_VERSION_board_version, buffer(offset, 4))
        offset = offset + 4
        local flight = tree:add(fields.AUTOPILOT_VERSION_flight_custom_version, buffer(offset, 8))
        flight:add(fields.AUTOPILOT_VERSION_flight_custom_version_0, buffer(offset, 1))
        offset = offset + 1
        flight:add(fields.AUTOPILOT_VERSION_flight_custom_version_1, buffer(offset, 1))
        offset = offset + 1
        flight:add(fields.AUTOPILOT_VERSION_flight_custom_version_2, buffer(offset, 1))
        offset = offset + 1
        flight:add(fields.AUTOPILOT_VERSION_flight_custom_version_3, buffer(offset, 1))
        offset = offset + 1
        flight:add(fields.AUTOPILOT_VERSION_flight_custom_version_4, buffer(offset, 1))
        offset = offset + 1
        flight:add(fields.AUTOPILOT_VERSION_flight_custom_version_5, buffer(offset, 1))
        offset = offset + 1
        flight:add(fields.AUTOPILOT_VERSION_flight_custom_version_6, buffer(offset, 1))
        offset = offset + 1
        flight:add(fields.AUTOPILOT_VERSION_flight_custom_version_7, buffer(offset, 1))
        offset = offset + 1
        local middleware = tree:add(fields.AUTOPILOT_VERSION_middleware_custom_version, buffer(offset, 8))
        middleware:add(fields.AUTOPILOT_VERSION_middleware_custom_version_0, buffer(offset, 1))
        offset = offset + 1
        middleware:add(fields.AUTOPILOT_VERSION_middleware_custom_version_1, buffer(offset, 1))
        offset = offset + 1
        middleware:add(fields.AUTOPILOT_VERSION_middleware_custom_version_2, buffer(offset, 1))
        offset = offset + 1
        middleware:add(fields.AUTOPILOT_VERSION_middleware_custom_version_3, buffer(offset, 1))
        offset = offset + 1
        middleware:add(fields.AUTOPILOT_VERSION_middleware_custom_version_4, buffer(offset, 1))
        offset = offset + 1
        middleware:add(fields.AUTOPILOT_VERSION_middleware_custom_version_5, buffer(offset, 1))
        offset = offset + 1
        middleware:add(fields.AUTOPILOT_VERSION_middleware_custom_version_6, buffer(offset, 1))
        offset = offset + 1
        middleware:add(fields.AUTOPILOT_VERSION_middleware_custom_version_7, buffer(offset, 1))
        offset = offset + 1
        local os = tree:add(fields.AUTOPILOT_VERSION_os_custom_version, buffer(offset, 8))
        os:add(fields.AUTOPILOT_VERSION_os_custom_version_0, buffer(offset, 1))
        offset = offset + 1
        os:add(fields.AUTOPILOT_VERSION_os_custom_version_1, buffer(offset, 1))
        offset = offset + 1
        os:add(fields.AUTOPILOT_VERSION_os_custom_version_2, buffer(offset, 1))
        offset = offset + 1
        os:add(fields.AUTOPILOT_VERSION_os_custom_version_3, buffer(offset, 1))
        offset = offset + 1
        os:add(fields.AUTOPILOT_VERSION_os_custom_version_4, buffer(offset, 1))
        offset = offset + 1
        os:add(fields.AUTOPILOT_VERSION_os_custom_version_5, buffer(offset, 1))
        offset = offset + 1
        os:add(fields.AUTOPILOT_VERSION_os_custom_version_6, buffer(offset, 1))
        offset = offset + 1
        os:add(fields.AUTOPILOT_VERSION_os_custom_version_7, buffer(offset, 1))
        offset = offset + 1
        tree:add_le(fields.AUTOPILOT_VERSION_vendor_id, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.AUTOPILOT_VERSION_product_id, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.AUTOPILOT_VERSION_uid, buffer(offset, 8))
        offset = offset + 8
        if (size == 78) then
            local uid2 = tree:add(fields.AUTOPILOT_VERSION_uid2, buffer(offset, 18))
            uid2:add(fields.AUTOPILOT_VERSION_uid2_0, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_1, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_2, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_3, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_4, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_5, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_6, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_7, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_8, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_9, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_10, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_11, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_12, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_13, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_14, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_15, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_16, buffer(offset, 1))
            offset = offset + 1
            uid2:add(fields.AUTOPILOT_VERSION_uid2_17, buffer(offset, 1))
            offset = offset + 1
        end
    else
        tree:add(fields.rawpayload, buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed AUTOPILOT_VERSION message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type ESTIMATOR_STATUS
function payload_fns.payload_230 (buffer, tree, offset, size)
    if (size == 42) then
        tree:add_le(fields.ESTIMATOR_STATUS_time_usec, buffer(offset, 8))
        offset = offset + 8
        local flags = tree:add_le(fields.ESTIMATOR_STATUS_flags, buffer(offset,2))
        flags:add_le(fields.ESTIMATOR_STATUS_flags_estimator_attitude, buffer(offset, 2))
        flags:add_le(fields.ESTIMATOR_STATUS_flags_estimator_velocity_horiz, buffer(offset, 2))
        flags:add_le(fields.ESTIMATOR_STATUS_flags_estimator_velocity_vert, buffer(offset, 2))
        flags:add_le(fields.ESTIMATOR_STATUS_flags_estimator_pos_horiz_rel, buffer(offset, 2))
        flags:add_le(fields.ESTIMATOR_STATUS_flags_estimator_pos_horiz_abs, buffer(offset, 2))
        flags:add_le(fields.ESTIMATOR_STATUS_flags_estimator_pos_vert_abs, buffer(offset, 2))
        flags:add_le(fields.ESTIMATOR_STATUS_flags_estimator_pos_vert_agl, buffer(offset, 2))
        flags:add_le(fields.ESTIMATOR_STATUS_flags_estimator_const_pos_mode, buffer(offset, 2))
        flags:add_le(fields.ESTIMATOR_STATUS_flags_estimator_pred_pos_horiz_rel, buffer(offset, 2))
        flags:add_le(fields.ESTIMATOR_STATUS_flags_estimator_pred_pos_horiz_abs, buffer(offset, 2))
        flags:add_le(fields.ESTIMATOR_STATUS_flags_estimator_gps_glitch, buffer(offset, 2))
        flags:add_le(fields.ESTIMATOR_STATUS_flags_estimator_accel_error, buffer(offset, 2))
        offset = offset + 2
        tree:add_le(fields.ESTIMATOR_STATUS_vel_ratio, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ESTIMATOR_STATUS_pos_horiz_ratio, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ESTIMATOR_STATUS_pos_vert_ratio, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ESTIMATOR_STATUS_mag_ratio, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ESTIMATOR_STATUS_hagl_ratio, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ESTIMATOR_STATUS_tas_ratio, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ESTIMATOR_STATUS_pos_horiz_accuracy, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.ESTIMATOR_STATUS_pos_vert_accuracy, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload, buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed ESTIMATOR_STATUS message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type WIND_COV
function payload_fns.payload_231 (buffer, tree, offset, size)
    if (size == 40) then
        tree:add_le(fields.WIND_COV_time_usec, buffer(offset, 8))
        offset = offset + 8
        tree:add_le(fields.WIND_COV_wind_x, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.WIND_COV_wind_y, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.WIND_COV_wind_z, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.WIND_COV_var_horiz, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.WIND_COV_var_vert, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.WIND_COV_wind_alt, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.WIND_COV_horiz_accuracy, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.WIND_COV_vert_accuracy, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload, buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed WIND_COV message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type VIBRATION
function payload_fns.payload_241 (buffer, tree, offset, size)
    if (size == 32) then
        tree:add_le(fields.VIBRATION_time_usec, buffer(offset, 8))
        offset = offset + 8
        tree:add_le(fields.VIBRATION_vibration_x, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.VIBRATION_vibration_y, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.VIBRATION_vibration_z, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.VIBRATION_clipping_0, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.VIBRATION_clipping_1, buffer(offset, 4))
        offset = offset + 4
        tree:add_le(fields.VIBRATION_clipping_2, buffer(offset, 4))
        offset = offset + 4
    else
        tree:add(fields.rawpayload, buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed VIBRATION message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type EXTENDED_SYS_STATE
function payload_fns.payload_245 (buffer, tree, offset, size)
    if (size == 2) then
        tree:add(fields.EXTENDED_SYS_STATE_vtol_state, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.EXTENDED_SYS_STATE_landed_state, buffer(offset, 1))
        offset = offset + 1
    else
        tree:add(fields.rawpayload, buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed EXTENDED_SYS_STATE message")
        offset = offset + size
    end
    return offset
end

-- dissect payload of message type STATUSTEXT
function payload_fns.payload_253 (buffer, tree, offset, size)
    if (size == 51) then
        tree:add(fields.STATUSTEXT_severity, buffer(offset, 1))
        offset = offset + 1
        tree:add(fields.STATUSTEXT_text, buffer(offset, 50))
        offset = offset + 50
    else
        tree:add(fields.rawpayload, buffer(offset, size))
        tree:add_expert_info(PI_MALFORMED, PI_ERROR, "Malformed STATUSTEXT message")
        offset = offset + size
    end
    return offset
end

-- Dissector function
function mavlink_proto.dissector(buffer, pinfo, tree)
    local offset = 0
    local msgCount = 0
    local msgOffset

    -- loop through data
    while (offset < buffer:len())
    do
        msgCount = msgCount + 1
        local subtree

        -- decode version
        local version = buffer(offset, 1):uint()
        local unknownFrameBeginOffset = offset
        local unknownFrameEndOffset = buffer:len()
        local protocolString = nil
        local found = false

        while(not found)
        do
            if (version == 0xfd or version == 0xfe or version == 0x55) then
                protocolString = "MAVLink"
                found = true
            else
                -- some unknown data found, record the begin offset
                if (unknownFrameEndOffset < offset) then
                    unknownFrameBeginOffset = offset
                    unknownFrameEndOffset = buffer:len()
                end
                offset = offset + 1
                if (offset < buffer:len()) then
                    version = buffer(offset,1):uint()
                    if (version == 0xfd or version == 0xfe or version == 0x55) then
                        -- Found unknown data among the buffer followed by a MAVLink message
                        unknownFrameEndOffset = offset
                    end
                else
                    -- no magic value found in the whole buffer. print the raw data and exit
                    if (unknownFrameBeginOffset > 0) then
                        if (msgCount == 1) then
                            pinfo.cols.info:set("MAVLink messages: Unkown message")
                        else
                            pinfo.cols.info:append(" | Unkown message")
                        end
                        size = unknownFrameEndOffset - unknownFrameBeginOffset
                        tree:add(fields.rawpayload, buffer(unknownFrameBeginOffset,size))
                    end
                    --break
                end
            end
        end

        if (unknownFrameEndOffset < buffer:len() ) then
            if (msgCount == 1) then
                pinfo.cols.info:set("MAVLink messages: Unkown message 22")
            else
                pinfo.cols.info:append(" | Unkown message 22")
            end
            size = unknownFrameEndOffset - unknownFrameBeginOffset
            local unknowntree = tree:add(fields.rawpayload, buffer(unknownFrameBeginOffset, size))
            unknowntree:set_text("Unknown protocol ("..size..")")
            --break
        elseif (protocolString == nil and unknownFrameEndOffset == buffer:len()) then
            pinfo.cols.info:append(" | Unknown message 22")
            size = offset - unknownFrameBeginOffset
            local unknowntree = tree:add(fields.rawpayload, buffer(unknownFrameBeginOffset, size))
            unknowntree:set_text("Unknown protocol ("..size..")")
            -- jump to next loop
            --break
        else
            -- Create protocol subtree according to length
            local msgid
            msgOffset = offset
            offset = offset + 1
            local length = buffer(offset, 1)

            if( ( buffer:len() - 12 - msgOffset >= length:uint() and version == 0xfd ) or ( buffer:len() - 8 - msgOffset >= length:uint() and version ~= 0xfd ) ) then
                if (version == 0xfd) then
                    subtree = tree:add(mavlink_proto, buffer(msgOffset, length:uint() + 12), "MAVLink Protocol ("..length:uint()..")")
                else
                    subtree = tree:add(mavlink_proto, buffer(msgOffset, length:uint() + 8), "MAVLink Protocol ("..length:uint()..")")
                end
            else
                size = buffer:len() - msgOffset
                subtree = tree:add(fields.rawpayload, buffer(msgOffset, size))
                subtree:set_text("Unknown protocol ("..size..")")
                break
            end

            pinfo.cols.protocol = protocolString

            -- HEADER
            if (buffer:len() - 2 - offset > 6) then
                -- handle expected header
                local header
                if (version == 0xfd) then
                    header = subtree:add(buffer(msgOffset, 10), "Header")
                else
                    header = subtree:add(buffer(msgOffset, 6), "Header")
                end

                header:add(fields.magic, buffer(msgOffset, 1))
                header:add(fields.length, buffer(offset, 1))
                offset = offset + 1

                -- MAVLink 2.0 fields
                if (version == 0xfd) then
                    header:add(fields.incompat_flags, buffer(offset, 1))
                    offset = offset + 1

                    header:add(fields.compat_flags, buffer(offset, 1))
                    offset = offset + 1
                end

                header:add(fields.sequence, buffer(offset, 1))
                offset = offset + 1

                header:add(fields.sysid, buffer(offset, 1))
                offset = offset + 1

                header:add(fields.compid, buffer(offset, 1))
                offset = offset + 1

                -- Message ID
                if (version == 0xfd) then
                    msgid = buffer(offset, 3)
                    header:add_le(fields.msgid, buffer(offset, 3))
                    offset = offset + 3
                else
                    msgid = buffer(offset, 1)
                    header:add(fields.msgid, buffer(offset, 1))
                    offset = offset + 1
                end
            else
                -- handle truncated header
                local hsize = buffer:len() - 2 - offset
                subtree:add(fields.rawheader, buffer(msgOffset, hsize))
                offset = offset + hsize
            end

            -- BODY
            local msgnum
            if (version == 0xfd) then
                msgnum = msgid:le_uint()
            else
                msgnum = msgid:uint()
            end

            local dissector_payload_fn = "payload_"..tostring(msgnum)
            local fn = payload_fns[dissector_payload_fn]

            if (fn == nil) then
                if (msgCount == 1) then
                    pinfo.cols.info:set("MAVLink messages: Unknown message type")
                else
                    pinfo.cols.info:append(" | Unknown message type")
                end
                subtree:add_expert_info(PI_MALFORMED, PI_ERROR, "Unknown message type")
                local rawpayload = subtree:add(fields.rawpayload, buffer(offset, length:uint()))
                rawpayload:set_text("Unparseable payload ("..length:uint().." bytes)")
                offset = offset + length:uint()
            else
                local payload
                local mname
                -- Add payload according to version
                if (version == 0xfd) then
                    payload = subtree:add(fields.payload, msgid:le_uint())
                    mname = messageName[msgid:le_uint()]
                else
                    payload = subtree:add(fields.payload, msgid:uint())
                    mname = messageName[msgid:uint()]
                end
                if (msgCount == 1) then
                    pinfo.cols.info:set("MAVLink messages: "..mname)
                else
                    pinfo.cols.info:append(" | "..mname)
                end
                offset = fn(buffer, payload, offset, length:uint())
            end

            -- CRC
            subtree:add_le(fields.crc, buffer(offset, 2))
            offset = offset + 2
        end
    end
end

-- Bind protocol dissector to USER0 linktype
wtap_encap = DissectorTable.get("wtap_encap")
wtap_encap:add(wtap.USER0, mavlink_proto)

-- bind protocol dissector to ports
local udp_dissector_table = DissectorTable.get("udp.port")
udp_dissector_table:add(14550, mavlink_proto)
udp_dissector_table:add(14556, mavlink_proto)
udp_dissector_table:add(14557, mavlink_proto)
udp_dissector_table:add(14560, mavlink_proto)

local tcp_dissector_table = DissectorTable.get("tcp.port")
tcp_dissector_table:add(5760, mavlink_proto)