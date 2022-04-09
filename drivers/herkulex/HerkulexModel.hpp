/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Herkulex model types.
 **/

#pragma once

#include <stdint.h>
#include <string>

namespace momentum
{

/// Herkulex motor control gains
struct HerkulexControlGains {
	/// Set of control gains for herkulex motors
	uint16_t position_gain_Kp;	///< Position control loop proportional gain
	uint16_t position_gain_Kd;	///< Position control loop derivative gain
	uint16_t position_gain_Ki;	///< Position control loop integral gain
	uint16_t position_gain_Kpff1; ///< Position control loop feedforward 1st gain
	uint16_t position_gain_Kpff2; ///< Position control loop feedforward 2nd gain
	uint16_t velocity_gain_Kp;	///< Velocity control loop proportional gain
	uint16_t velocity_gain_Ki;	///< Velocity control loop integral gain

	/// From section 4-2. Register Map of Herkulex user's manual.
	static constexpr uint16_t kGainMax =
		0x7FFF; ///< Control loop gain (proportional, integral, derivative) maximum value.
};

/// Container class for storing model-specific parameters, constants and conversion methods.
struct HerkulexModel {
	/// Raw value for maximum torque (PWM duty cycle).
	static constexpr uint16_t torque_max_raw = 0x03FF;

public:
	/// Copy constructor.
	/// @param [in] src The source model to copy.
	/// @return Copy of src.
	HerkulexModel(const HerkulexModel &src) = default;

	/// Compare two models.
	/// @param [in] model The model to compare.
	/// @return true if the model numbers are equal.
	bool operator==(const HerkulexModel &model) const;

	/// Compare two models.
	/// @param [in] model The model to compare.
	/// @return true if the model numbers differ.
	bool operator!=(const HerkulexModel &model) const;

	/// Wrap an angle to (-180, 180].
	/// @param [in] angle_deg The angle, in deg.
	/// @return angle, in deg, wrapped to (-180, 180].
	static double wrap_angle_deg(const double angle_deg);

	/// Translate absolute raw position to degrees.
	/// @note By convention, this driver maps 0 degrees to the middle
	/// of the range of motion, such that when looking directly at the
	/// motor, 0 degrees is pointing north (up towards the LED).
	/// @param [in] raw_position Raw position to convert.
	/// @return position in degrees.
	double raw_to_deg(const uint16_t raw_position) const;

	/// Translate absolute 2nd position raw value to degrees
	/// @note No offset is being taken into account. Method
	/// will just scale raw value
	/// @param [in] raw_position Raw position to convert
	/// @return position in degrees.
	double raw_pos_2nd_to_deg(const uint16_t raw_position) const;

	/// Translate differential raw value to degrees / second
	/// @param [in] raw_differential Raw differential value to convert
	/// @return speed in deg/s.
	double raw_to_deg_s(const int16_t raw_differential) const;

	/// Translate position tolerance from degrees to raw.
	/// @param [in] tolerance_deg Position tolerance, in deg.
	/// @return tolerance, in raw encoder values.
	uint8_t tolerance_deg_to_raw(const double tolerance_deg) const;

	/// Translate absolute degrees to jog format, a raw 15 bit encoder value.
	/// @note By convention, this driver maps 0 degrees to the middle
	/// of the range of motion, such that when looking directly at the
	/// motor, 0 degrees is pointing north (up towards the LED).
	/// @param [in] deg Degrees. Signed value.
	/// @return Nearest raw encoder value.
	uint16_t deg_to_jog(double deg) const;

	/// Translate deg / s to jog format, a 14-bit + 1 sign bit raw value.
	/// No calibration is applied.
	/// @param [in] deg_s Degrees per second. Signed value.
	/// @return Nearest raw velocity.
	uint16_t deg_s_to_jog(double deg_s) const;

	/// Translate torque percentage (0-100) to jog format, a 14-bit + 1 sign bit raw value.
	/// @param [in] torque_percent Torque, in units of percent (0-100). Signed value.
	/// @return Nearest raw torque.
	uint16_t torque_percent_to_jog(double torque_percent) const;

	/// Translate raw torque to percent.
	/// @param [in] torque_raw Raw torque value.
	/// @return torque, in percent. Signed value.
	double torque_raw_to_percent(const int16_t torque_raw) const;

	/// Translate torque from percentage to raw.
	/// @param [in] torque_percent Torque, in percent.
	/// @return torque, in signed raw value.
	int16_t torque_percent_to_raw(const double torque_percent) const;

	/// Translate raw volts to V.
	/// @param [in] voltage_raw Raw voltage value.
	/// @return voltage, in V.
	double voltage_raw_to_V(const uint16_t voltage_raw) const;

	/// Translate milliseconds to raw time value.
	/// @param [in] time_ms Time, in ms.
	/// @return The raw value nearest this time.
	uint8_t time_ms_to_raw(const uint16_t time_ms) const;

	/// Position is in deadband?
	/// @param [in] position, in deg.
	/// @return true if position is in deadband.
	bool is_in_deadband(const double position_deg) const;

	/// Model ID of the motor. DRS-101 = 0x0101
	uint16_t model_id;

	/// String version of the model.
	std::string model_name;

	/// Minimum allowed position, in raw encoder units.
	uint16_t position_min_raw;

	/// Maximum allows position, in raw encoder units.
	uint16_t position_max_raw;

	/// Default value of in position tolerance register
	uint8_t tolerance_default_raw;

	/// Maximum velocity, in deg / s.
	/// Maximum velocity is specified for the rated voltage of the motor.
	double velocity_max_deg_s;

	/// Model supports velocity control.
	bool velocity_control_supported;

	/// Model has velocity control gains exposed
	bool velocity_gains_exposed;

	/// Model supports absolute 2nd position reading
	bool abs_position_2nd_supported;

	// For converting temperature register values to degree Celsius
	// some models return the actual temperature and others return
	// the raw ADC value that must be converted.
	// TODO: not handling case for ADC conversion since not able to
	// use a equation

	/// Model returns temperature registers values in degree Celsius?
	bool temp_value_in_C;

	/// Default value for max temperature setting (degree Celsius)
	static constexpr double max_temp_default_C = 80.0;

	/// Default value for "is_moving" detection threshold, in degrees
	double is_moving_threshold_deg;

	/// Default time for "is_moving" detection period, in milli-seconds
	/// Herkulex user manual see default value for register name Stop Detection Period
	static constexpr uint16_t is_moving_detect_period_ms = 302;

	/// Default control loop gains for motor
	HerkulexControlGains control_gains_default;

	/// Clock tick period, in ms. Used for most time-based values.
	static constexpr double tick_period_ms = 11.2;

	/// Maximum play time, in ms.
	/// = 254 * 11.2ms
	static constexpr uint32_t playtime_max_ms = static_cast<double>(0xFE * tick_period_ms);

	/// Maximum rotation away from center before hitting deadband, in deg.
	static constexpr double deadband_boundary_deg = 159;

	/// Minimum torque offset (PWM duty cycle offset), in percent.
	static constexpr double torque_offset_min_percent = (-128 / static_cast<double>(torque_max_raw)) * 100.;

	/// Maximum torque offset (PWM duty cycle offset), in percent.
	static constexpr double torque_offset_max_percent = (127 / static_cast<double>(torque_max_raw)) * 100.;

	/// Default trajectory planning acceleration ratio, in percent.
	static constexpr double accel_ratio_percent_default = 25.0;

	/// Default trajectory planning maximum acceleration time, in ms.
	static constexpr double accel_time_max_ms_default = 504.0;

	/// Maximum allowed value for trajectory planning maximum acceleration time, in ms.
	static constexpr double accel_time_max_ms_max = 2844.0;

	/// Default herkulex dead zone
	static constexpr double dead_zone_degree_default = 0.0;

	/// Scale factor from raw position register to degrees rotation.
	/// scale = (360' - [deadband]) / [encoder resolution]
	/// e.g. for DRS-101 = (360' - 26.7') / 1024 = 0.325
	double scale_raw_to_deg;

	/// Scale factor for absolute 2nd position raw value to degree
	double scale_raw_to_pos_2nd_deg;

	// Herkulex models

	/// Unknown model.
	static const HerkulexModel kUnknown;

	/// Herkulex DRS-101
	static const HerkulexModel kDrs0101;

	/// Herkulex DRS-201
	static const HerkulexModel kDrs0201;

	/// Herkulex DRS-401
	static const HerkulexModel kDrs0401;

	/// Herkulex DRS-402
	static const HerkulexModel kDrs0402;

	/// Herkulex DRS-601
	static const HerkulexModel kDrs0601;

	/// Herkulex DRS-602
	static const HerkulexModel kDrs0602;

private:
	HerkulexModel(const uint16_t i_model_id,
				  const std::string i_model_name,
				  const uint16_t i_encoder_resolution,
				  const uint16_t i_encoder_offset,
				  const bool i_encoder_has_deadband,
				  const uint16_t i_position_min_raw,
				  const uint16_t i_position_max_raw,
				  const uint8_t i_tolerance_default_raw,
				  const double i_velocity_max_deg_s,
				  const bool i_velocity_control_supported,
				  const bool i_velocity_gains_exposed,
				  const bool i_abs_position_2nd_supported,
				  const bool i_temp_value_in_C,
				  const double i_is_moving_threshold_deg,
				  const double i_scale_raw_to_deg,
				  const double i_scale_raw_to_pos_2nd_deg,
				  const double i_scale_diff_raw_to_deg_s,
				  const double i_scale_raw_to_volt,
				  const HerkulexControlGains &i_control_gains_default);

	/// Resolution of the encoder, in steps.
	uint16_t encoder_resolution;

	/// Encoder offset at 180' - applies to 0x02 models only
	uint16_t encoder_offset;

	/// Model rescales encoder resolution over live regions only?
	/// Models with continuous rotation (0x02) use all 360'.
	/// Other models (0x01) use 360 - 26.7
	bool encoder_has_deadband;

	/// Scale factor from raw differential register to velocity, in deg/sec.
	double scale_diff_raw_to_deg_s;

	/// Scale factor from raw voltage register to volts.
	double scale_raw_to_volt;

	// parameters consistent to all models

	/// Degrees of rotation over which the encoder is in its deadband.
	static constexpr double deadband_size_deg = 26.7;

	/// Scale factor from desired velocity in deg/sec to raw velocity.
	/// Due to differences in register and packet formats, this is not
	/// necessarily the inverse of scale_diff_position_raw_to_deg_s.
	/// For all models this is found in the manual page 38 footnote.
	static constexpr double scale_deg_s_to_jog = 1.0 / 0.62;
};

} // namespace momentum
