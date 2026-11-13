#pragma once
// Frozen reference fixture for historical tracker scenarios. Never loaded by the node.
namespace exploration::test_reference {
inline constexpr auto CARLIKE_ALIGN_RESUME_DEG = 25.0;
inline constexpr auto CARLIKE_ALIGN_SETTLE_S = 0.06;
inline constexpr auto CARLIKE_ALIGN_STOP_SPEED = 0.06;
inline constexpr auto CARLIKE_ALIGN_STOP_YAW_RATE = 0.12;
inline constexpr auto CARLIKE_CORNER_STOP_DEG = 20.0;
inline constexpr auto CARLIKE_HEADING_GATE_DEG = 18.0;
inline constexpr auto CARLIKE_KD_YAW = 0.50;
inline constexpr auto CARLIKE_K_CURV = 1.00;
inline constexpr auto CARLIKE_LATERAL_PREDICTION_S = 0.55;
inline constexpr auto CARLIKE_LOOKAHEAD = 0.50;
inline constexpr auto CARLIKE_MAX_ACCEL = 0.30;
inline constexpr auto CARLIKE_MAX_LATERAL_ACCEL = 0.35;
inline constexpr auto CARLIKE_MAX_YAW_ACCEL = 1.60;
inline constexpr auto CARLIKE_MAX_YAW_RATE = 1.10;
inline constexpr auto CARLIKE_PREDICTION_S = 0.20;
inline constexpr auto CARLIKE_SMITH_S = 0.30;
inline constexpr auto CARLIKE_SMITH_YAW_S = 0.40;
inline constexpr auto CARLIKE_STOP_ALIGN_DEG = 75.0;
inline constexpr auto CARLIKE_YAW_FILTER_S = 0.12;
inline constexpr auto ENDPOINT_SLOW_R = 0.50;
inline constexpr auto KD_LAT = 0.04;
inline constexpr auto KP_LAT = 0.80;
inline constexpr auto KP_YAW = 1.60;
inline constexpr auto MAX_V_LAT = 0.20;
inline constexpr auto V_EST_ALPHA = 0.30;
inline constexpr auto V_MAX = 0.8;
inline constexpr auto V_MIN = 0.00;
} // namespace exploration::test_reference
