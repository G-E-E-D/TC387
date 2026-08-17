#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

/* Fixed vehicle geometry and mission limits (SI units). */
constexpr auto VEHICLE_WHEELBASE_M = 0.680f;
constexpr auto VEHICLE_FRONT_TRACK_M = 0.590f;
constexpr auto VEHICLE_REAR_TRACK_M = 0.585f;
constexpr auto RECORD_MAX_DISTANCE_M = 100.0f;
constexpr auto RECORD_MAX_TIME_S = 120.0f;
constexpr auto REPLAY_MAX_SPEED_MPS = 2.0f;
#define PATH_PERSISTENCE_ENABLE             (0)
#define GPS_ENABLE                          (0)
#define MAG_FUSION_ENABLE                   (0)

/* The camera DVP wiring is confirmed.  MT9V03X keeps the library configuration
 * of 188 x 120 pixels at 50 FPS (see zf_device_mt9v03x.h). */
#define VEHICLE_VISION_ENABLE               (1)
constexpr auto VEHICLE_VISION_TARGET_SPEED_MPS = 0.15f;
constexpr auto VEHICLE_VISION_MIN_CONFIDENCE = 650U;
constexpr auto VEHICLE_VISION_MAX_LOST_FRAMES = 0U;
constexpr auto VEHICLE_VISION_STEERING_RAD_LIMIT = 0.35f;

/* Static storage and path processing. */
constexpr auto PATH_MAX_POINTS = 4096U;
constexpr auto PATH_POINT_SIZE_BYTES = 24U;
constexpr auto PATH_STORAGE_BYTES = PATH_MAX_POINTS * PATH_POINT_SIZE_BYTES;
constexpr auto PATH_RECORD_SPACING_M = 0.030f;
constexpr auto PATH_RESAMPLE_SPACING_M = 0.030f;
constexpr auto PATH_MIN_POINT_DISTANCE_M = 0.002f;
constexpr auto PATH_YAW_SAMPLE_THRESHOLD_RAD = 0.035f;
constexpr auto PATH_MAX_INPUT_SEGMENT_M = 0.150f;
constexpr auto PATH_MAX_YAW_JUMP_RAD = 0.80f;
constexpr auto PATH_MIN_VALID_POINTS = 8U;
constexpr auto PATH_MIN_VALID_LENGTH_M = 0.20f;
constexpr auto PATH_SMOOTH_HALF_WINDOW = 2U;
constexpr auto PATH_CURVATURE_WINDOW = 4U;

/* Cooperative scheduler periods. */
constexpr auto VEHICLE_FAST_PERIOD_US = 1000ULL;
constexpr auto VEHICLE_CONTROL_PERIOD_US = 5000ULL;
constexpr auto VEHICLE_ESTIMATOR_PERIOD_US = 5000ULL;
constexpr auto VEHICLE_TRACKER_PERIOD_US = 10000ULL;
constexpr auto VEHICLE_LOG_PERIOD_US = 20000ULL;
constexpr auto VEHICLE_DISPLAY_PERIOD_US = 100000ULL;
constexpr auto VEHICLE_COMMAND_TIMEOUT_US = 150000ULL;
constexpr auto VEHICLE_CONTROL_TIMEOUT_US = 30000ULL;
constexpr auto VEHICLE_SENSOR_TIMEOUT_US = 50000ULL;

/* Sensor validation and estimator tuning. */
constexpr auto IMU_CALIBRATION_TIME_S = 3.0f;
constexpr auto IMU_CALIBRATION_MIN_SAMPLES = 400U;
constexpr auto IMU_CALIBRATION_MAX_GYRO_RADPS = 0.12f;
constexpr auto IMU_CALIBRATION_MAX_ACCEL_DELTA_MPS2 = 1.5f;
constexpr auto IMU_MAX_ACCEL_MPS2 = 100.0f;
constexpr auto IMU_MAX_GYRO_RADPS = 35.0f;
constexpr auto IMU_BIAS_STATIONARY_GAIN = 0.003f;
constexpr auto IMU_BIAS_MOVING_GAIN = 0.00015f;
constexpr auto LOCALIZATION_SPEED_FILTER_GAIN = 0.30f;
constexpr auto LOCALIZATION_WHEEL_OMEGA_GAIN = 0.10f;
constexpr auto LOCALIZATION_STEER_OMEGA_GAIN = 0.05f;
constexpr auto LOCALIZATION_STATIC_SPEED_MPS = 0.025f;
constexpr auto LOCALIZATION_STATIC_GYRO_RADPS = 0.04f;
constexpr auto LOCALIZATION_OMEGA_DISAGREE_RADPS = 0.80f;
constexpr auto LOCALIZATION_SLIP_SPEED_MPS = 0.80f;
constexpr auto LOCALIZATION_MIN_DT_S = 0.0005f;
constexpr auto LOCALIZATION_MAX_DT_S = 0.050f;
constexpr auto ENCODER_MAX_ABS_SPEED_MPS = 3.50f;
constexpr auto ENCODER_MAX_ACCEL_MPS2 = 20.0f;

/* Rear wheel speed control. */
constexpr auto WHEEL_SPEED_KP = 0.22f;
constexpr auto WHEEL_SPEED_KI = 0.75f;
constexpr auto WHEEL_SPEED_FEEDFORWARD = 0.18f;
constexpr auto WHEEL_SPEED_INTEGRAL_LIMIT = 0.35f;
constexpr auto WHEEL_SPEED_OUTPUT_LIMIT = 0.80f;
constexpr auto WHEEL_SPEED_OUTPUT_SLEW_PER_S = 3.0f;
constexpr auto WHEEL_SPEED_ZERO_BAND_MPS = 0.025f;
constexpr auto WHEEL_SPEED_DIRECTION_HOLD_S = 0.08f;

/* Steering position control. */
constexpr auto STEERING_KP = 1.80f;
constexpr auto STEERING_KI = 0.40f;
constexpr auto STEERING_KD = 0.050f;
constexpr auto STEERING_INTEGRAL_LIMIT = 0.25f;
constexpr auto STEERING_OUTPUT_LIMIT = 0.55f;
constexpr auto STEERING_OUTPUT_SLEW_PER_S = 2.5f;
constexpr auto STEERING_POSITION_DEADBAND_RAD = 0.006f;
constexpr auto STEERING_STALL_DUTY = 0.18f;
constexpr auto STEERING_STALL_COUNT_DELTA = 2LL;
constexpr auto STEERING_STALL_TIMEOUT_S = 0.80f;

/* Stage-one guarded command limits. */
constexpr auto STAGE1_MAX_SPEED_MPS = 0.80f;
constexpr auto STAGE1_MAX_STEERING_RAD = 0.70f;
constexpr auto STAGE1_DIAGNOSTIC_MAX_SPEED_MPS = 0.30f;
constexpr auto STAGE1_DIAGNOSTIC_MAX_STEERING_RAD = 0.35f;
constexpr auto STAGE1_STOP_DISTANCE_MARGIN_M = 0.15f;
constexpr auto STAGE1_STOP_TIME_MARGIN_S = 0.30f;
constexpr auto CALIBRATION_TEST_MAX_DUTY = 0.12f;
constexpr auto CALIBRATION_TEST_MAX_TIME_S = 2.0f;
constexpr auto CALIBRATION_TEST_MAX_SPEED_MPS = 0.30f;

/* Reverse path tracking. */
constexpr auto REVERSE_NOMINAL_SPEED_MPS = -0.80f;
constexpr auto REVERSE_LOOKAHEAD_MIN_M = 0.22f;
constexpr auto REVERSE_LOOKAHEAD_MAX_M = 0.90f;
constexpr auto REVERSE_LOOKAHEAD_TIME_S = 0.40f;
constexpr auto REVERSE_CURVATURE_SPEED_GAIN = 1.8f;
constexpr auto REVERSE_CROSSTRACK_SPEED_GAIN = 0.9f;
constexpr auto REVERSE_CROSSTRACK_GAIN = 0.80f;
constexpr auto REVERSE_HEADING_GAIN = 0.65f;
constexpr auto REVERSE_CURVATURE_FEEDFORWARD_GAIN = 1.0f;
constexpr auto REVERSE_MAX_STEERING_RAD = 0.70f;
constexpr auto REVERSE_ACCEL_LIMIT_MPS2 = 0.70f;
constexpr auto REVERSE_DECEL_LIMIT_MPS2 = 1.00f;
constexpr auto REVERSE_TERMINAL_DECEL_DISTANCE_M = 1.50f;
constexpr auto REVERSE_FINISH_DISTANCE_M = 0.12f;
constexpr auto REVERSE_FINISH_SPEED_MPS = 0.08f;
constexpr auto REVERSE_SEARCH_BACK_POINTS = 8U;
constexpr auto REVERSE_SEARCH_FORWARD_POINTS = 80U;
constexpr auto REVERSE_INDEX_LOSS_LIMIT = 20U;
constexpr auto REVERSE_MAX_CROSSTRACK_ERROR_M = 0.90f;
constexpr auto REVERSE_MAX_HEADING_ERROR_RAD = 1.20f;

/* Safety monitoring. */
constexpr auto MOTOR_STALL_DUTY = 0.25f;
constexpr auto MOTOR_STALL_SPEED_MPS = 0.025f;
constexpr auto MOTOR_STALL_TIMEOUT_S = 0.80f;
constexpr auto VEHICLE_CONTROLLED_STOP_TIMEOUT_US = 3000000ULL;
constexpr auto WHEEL_MISMATCH_MIN_SPEED_MPS = 0.25f;
constexpr auto WHEEL_MISMATCH_MAX_MPS = 1.20f;
constexpr auto WHEEL_MISMATCH_TIMEOUT_S = 0.60f;
constexpr auto STEERING_ENCODER_MAX_DELTA_COUNT_PER_SAMPLE = 1024;
constexpr auto STEERING_ENCODER_ERROR_LIMIT = 5U;
constexpr auto STEERING_ENCODER_SPI_TIMEOUT_US = 1000U;
/* Compatibility constants for the retained MT6701 host adapter. */
constexpr auto MT6701_MAX_DELTA_COUNT_PER_SAMPLE = 4096;
constexpr auto MT6701_ERROR_LIMIT = 5U;

/* Stage-one visual guidance. */
constexpr auto GUIDE_TARGET_TIMEOUT_US = 150000ULL;
constexpr auto GUIDE_TARGET_MIN_DISTANCE_M = 0.35f;
constexpr auto FORWARD_FOLLOW_DISTANCE_M = 1.00f;
constexpr auto FORWARD_LOOKAHEAD_MIN_M = 0.25f;
constexpr auto FORWARD_LOOKAHEAD_MAX_M = 1.20f;
constexpr auto FORWARD_DISTANCE_KP = 0.65f;
constexpr auto FORWARD_CURVATURE_SPEED_GAIN = 0.90f;
constexpr auto FORWARD_ACCEL_LIMIT_MPS2 = 0.60f;
constexpr auto FORWARD_DECEL_LIMIT_MPS2 = 0.90f;
constexpr auto FORWARD_MAX_SPEED_MPS = 0.80f;
constexpr auto FORWARD_MAX_STEERING_RAD = 0.70f;
constexpr auto FORWARD_MIN_TAG_WIDTH_M = 0.001f;
constexpr auto FORWARD_DEFAULT_FX_PX = 160.0f;
constexpr auto FORWARD_DEFAULT_FY_PX = 160.0f;
constexpr auto VISION_PROCESS_MAX_US = 18000U;

/* Fixed ring buffers. */
constexpr auto VEHICLE_LOG_BUFFER_SIZE = 16384U;
constexpr auto VEHICLE_COMMAND_BUFFER_SIZE = 192U;
constexpr auto VEHICLE_CSV_LINE_SIZE = 512U;

#endif
