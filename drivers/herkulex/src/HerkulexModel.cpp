/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/herkulex/HerkulexModel.hpp>

#include <cmath> // for std::abs()
#include <map>

namespace momentum
{

HerkulexModel::HerkulexModel(const uint16_t i_model_id,
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
							 const HerkulexControlGains &i_control_gains_default)
  : model_id(i_model_id),
	model_name(i_model_name),
	position_min_raw(i_position_min_raw),
	position_max_raw(i_position_max_raw),
	tolerance_default_raw(i_tolerance_default_raw),
	velocity_max_deg_s(i_velocity_max_deg_s),
	velocity_control_supported(i_velocity_control_supported),
	velocity_gains_exposed(i_velocity_gains_exposed),
	abs_position_2nd_supported(i_abs_position_2nd_supported),
	temp_value_in_C(i_temp_value_in_C),
	is_moving_threshold_deg(i_is_moving_threshold_deg),
	control_gains_default(i_control_gains_default),
	scale_raw_to_deg(i_scale_raw_to_deg),
	scale_raw_to_pos_2nd_deg(i_scale_raw_to_pos_2nd_deg),
	encoder_resolution(i_encoder_resolution),
	encoder_offset(i_encoder_offset),
	encoder_has_deadband(i_encoder_has_deadband),
	scale_diff_raw_to_deg_s(i_scale_diff_raw_to_deg_s),
	scale_raw_to_volt(i_scale_raw_to_volt)
{
}

bool HerkulexModel::operator==(const HerkulexModel &model) const
{
	return model_id == model.model_id;
}

bool HerkulexModel::operator!=(const HerkulexModel &model) const
{
	return !operator==(model);
}

double HerkulexModel::wrap_angle_deg(const double angle_deg)
{
	// Map angle to value [-360, 360]
	const double angle_mod = fmod(angle_deg, 360);

	// If the absolute value of angle is less than 180 deg, return that value.
	// If the angle is greater than 180 degrees, subtract 360.
	// If the angle is less than -180 degrees, add 360.
	if (fabs(angle_mod) <= 180) {
		return angle_mod;
	} else if (angle_mod > 180.0) {
		return angle_mod - 360.0;
	} else {
		return angle_mod + 360.0;
	}
}

double HerkulexModel::raw_to_deg(const uint16_t raw_position) const
{
	double position = static_cast<double>(raw_position);
	// bias for encoders that do not start at zero due to deadband
	position -= encoder_offset;
	// rescale to [0, 360]
	position *= scale_raw_to_deg;
	// if encoder has deadband, account for it by offsetting half the deadband
	// since the full encoder range covers only outside the deadband, this
	// is a bias to center the encoder range about zero.
	if (encoder_has_deadband) {
		position += deadband_size_deg / 2.0;
	}
	// translate by 180' so that by convention, 0 degrees is in the center of
	// the reachable workspace.
	position -= 180.0;

	return position;
}

double HerkulexModel::raw_pos_2nd_to_deg(const uint16_t raw_position) const
{
	return static_cast<double>(raw_position) * scale_raw_to_pos_2nd_deg;
}

double HerkulexModel::raw_to_deg_s(const int16_t raw_differential) const
{
	return static_cast<double>(raw_differential) * scale_diff_raw_to_deg_s;
}

uint8_t HerkulexModel::tolerance_deg_to_raw(const double tolerance_deg) const
{
	return tolerance_deg / scale_raw_to_deg;
}

// "JOG" field
//  0 - 13: goal position
//      14: unused
//      15: reserved = 0
uint16_t HerkulexModel::deg_to_jog(double deg) const
{
	// by convention, zero degrees is the center of range of motion,
	// but absolute positions begin at the range of motion, so translate here
	deg += 180.0;
	// if encoder has deadband, account for it by offsetting half the deadband
	// since the full encoder range covers only outside the deadband, this
	// is a bias to center the encoder range about zero.
	if (encoder_has_deadband) {
		deg -= deadband_size_deg / 2.0;
	}
	// rescale and offset - mind your bits here...
	// use a larger value for raw position to allow bounding (instead of rollover)
	uint32_t raw = static_cast<int32_t>(deg / scale_raw_to_deg);
	// bias for encoders that do not start at zero due to deadband
	raw += static_cast<int32_t>(encoder_offset);
	return raw;
}

// "JOG" field.
//  0 - 13: goal velocity
//      14: turn direction, 0 positive, 1 negative
//      15: reserved = 0
uint16_t HerkulexModel::deg_s_to_jog(double deg_s) const
{
	// bound
	if (std::abs(deg_s) > velocity_max_deg_s) {
		deg_s = velocity_max_deg_s * (deg_s > 0 ? 1.0 : -1.0);
	}
	// capture bits 0-13 bit unsigned velocity
	uint16_t raw = static_cast<uint16_t>(std::abs(deg_s) * scale_deg_s_to_jog);
	raw &= 0x3FFF;
	// sign is bit 14
	raw |= (deg_s < 0) << 14;
	return raw;
}

// "JOG" field.
//  0 - 13: goal PWM
//      14: torque direction, 0 positive, 1 negative
//      15: reserved = 0
uint16_t HerkulexModel::torque_percent_to_jog(double torque_percent) const
{
	// bound
	if (std::abs(torque_percent) > 100) {
		torque_percent = torque_percent > 0 ? 100.0 : -100.0;
	}
	// capture bits 0-13 bit unsigned velocity
	uint16_t raw = static_cast<uint16_t>(std::abs(torque_percent) / 100.0 * torque_max_raw);
	raw &= 0x3FFF;
	// sign is bit 14
	raw |= (torque_percent < 0) << 14;
	return raw;
}

double HerkulexModel::torque_raw_to_percent(const int16_t torque_raw) const
{
	// note: torque is apparently a signed value
	return (static_cast<double>(torque_raw) / static_cast<double>(torque_max_raw)) * 100.0;
}

int16_t HerkulexModel::torque_percent_to_raw(const double torque_percent) const
{
	// note: torque is apparently a signed value
	return static_cast<int16_t>((torque_percent / 100.0) * static_cast<double>(torque_max_raw));
}

double HerkulexModel::voltage_raw_to_V(const uint16_t voltage_raw) const
{
	return static_cast<double>(voltage_raw) * scale_raw_to_volt;
}

uint8_t HerkulexModel::time_ms_to_raw(const uint16_t time_ms) const
{
	return static_cast<uint8_t>(static_cast<double>(time_ms) / tick_period_ms);
}

bool HerkulexModel::is_in_deadband(const double position_deg) const
{
	// note: regardless of whether or not the motor model has a deadband
	// in its encoder resolution, all models have a deadband region.
	// do not check _model.encoder_has_deadband
	return std::abs(wrap_angle_deg(position_deg)) >= deadband_boundary_deg;
}

// clang-format off
// use Drs101 values so at least sane values are set.
const HerkulexModel HerkulexModel::kUnknown = {
	.model_id					= 0x0000,
	.model_name					= "Unknown",
	.encoder_resolution 		= 1024,
	.encoder_offset				= 0,
	.encoder_has_deadband		= true,
	.position_min_raw			= 1,
	.position_max_raw			= 1023,
	.tolerance_default_raw 		= 0x00,
	.velocity_max_deg_s			= 361.4,
	.velocity_control_supported	= false,
	.velocity_gains_exposed		= false,
	.abs_position_2nd_supported = false,
	.temp_value_in_C            = false,
	.is_moving_threshold_deg    = 0.0,
	.scale_raw_to_deg			= 0.325,
	.scale_raw_to_pos_2nd_deg	= 0.0,
	.scale_diff_raw_to_deg_s	= 29.09,
	.scale_raw_to_volt			= 0.074,
	.control_gains_default		= HerkulexControlGains{	.position_gain_Kp = 0x0000,
														.position_gain_Kd = 0x0000,
														.position_gain_Ki = 0x0000,
														.position_gain_Kpff1 = 0x0000,
														.position_gain_Kpff2 = 0x0000,
														.velocity_gain_Kp = 0x0000,
														.velocity_gain_Ki = 0x0000 },
};

const HerkulexModel HerkulexModel::kDrs0101 = {
	.model_id 					= 0x0101,
	.model_name 				= "DRS-0101",
	.encoder_resolution 		= 1024,		// Herkulex DRS-101/201 pg 9 Resolution
	.encoder_offset				= 0,		// Herkluex DRS-101/201 pg 25 Absolute Position
	.encoder_has_deadband		= true,		// Herkulex DRS-101/201 pg 25 Absolute Position
	.position_min_raw			= 21,		// Herkulex DRS-101/201 pg 22 Min. Position
	.position_max_raw			= 1002,		// Herkulex DRS-101/201 pg 22 Max. Position
	.tolerance_default_raw 		= 0x03,		// Herkulex DRS-101/201 pg 32 Inposition Margin
	.velocity_max_deg_s			= 361.4,	// Herkulex DRS-101/201 pg 9 Maximum Speed (60 / 0.166)
	.velocity_control_supported	= false,
	.velocity_gains_exposed		= false,
	.abs_position_2nd_supported = false,
	.temp_value_in_C            = false,
	.is_moving_threshold_deg    = 0.975,    // Herkulex DRS-101/201 pg 23 Stop Threshold
	.scale_raw_to_deg			= 0.325,	// Herkulex DRS-101/201 pg 32 Absolute Position
											// = (360' - 26.7') / 1024
	.scale_raw_to_pos_2nd_deg	= 0.0,
	.scale_diff_raw_to_deg_s	= 29.09,	// Herkulex DRS-101/201 pg 32 Diff Position
											// ~= scale_raw_to_deg / 11.2ms
	.scale_raw_to_volt			= 0.074,	// Herkulex DRS-101/201 pg 32 Voltage
	.control_gains_default		= HerkulexControlGains{	.position_gain_Kp = 0x00FE, 	//Found by testing
														.position_gain_Kd = 0x1964, 	//Found by testing
														.position_gain_Ki = 0x0000, 	//Found by testing
														.position_gain_Kpff1 = 0x0000,	//Found by testing
														.position_gain_Kpff2 = 0x0000,	//Found by testing
														.velocity_gain_Kp = 0x0000,
														.velocity_gain_Ki = 0x0000 },
};

const HerkulexModel HerkulexModel::kDrs0201 = {
	.model_id 					= 0x0201,
	.model_name 				= "DRS-0201",
	.encoder_resolution 		= 1024,		// Herkulex DRS-101/201 pg 9 Resolution
	.encoder_offset				= 0,		// Herkluex DRS-101/201 pg 25 Absolute Position
	.encoder_has_deadband		= true,		// Herkulex DRS-101/201 pg 25 Absolute Position
	.position_min_raw			= 21,		// Herkulex DRS-101/201 pg 22 Min. Position
	.position_max_raw			= 1002,		// Herkulex DRS-101/201 pg 22 Max. Position
	.tolerance_default_raw		= 0x03,		// Herkulex DRS-101/201 pg 32 Inposition Margin
	.velocity_max_deg_s			= 408.1,	// Herkulex DRS-101/201 pg 9 Maximum Speed (60 / 0.147)
	.velocity_control_supported	= false,
	.velocity_gains_exposed		= false,
	.abs_position_2nd_supported = false,
	.temp_value_in_C            = false,
	.is_moving_threshold_deg    = 0.975,    // Herkulex DRS-101/201 pg 23 Stop Threshold
	.scale_raw_to_deg			= 0.325,	// Herkulex DRS-101/201 pg 32 Absolute Position
											// = (360' - 26.7') / 1024
	.scale_diff_raw_to_deg_s	= 29.09,	// Herkulex DRS-101/201 pg 32 Diff Position
											// ~= scale_raw_to_deg / 11.2ms
	.scale_raw_to_pos_2nd_deg	= 0.0,
	.scale_raw_to_volt			= 0.074,	// Herkulex DRS-101/201 pg 31 Voltage
	.control_gains_default		= HerkulexControlGains{	.position_gain_Kp = 0x01B8, 	//Herkulex DRS-602 pg 22 Position Kp
														.position_gain_Kd = 0x1F40, 	//Herkulex DRS-602 pg 22 Position Kd
														.position_gain_Ki = 0x0000, 	//Herkulex DRS-602 pg 22 Position Ki
														.position_gain_Kpff1 = 0x0000,	//Herkulex DRS-602 pg 22 Position Feedfoward 1st Gain
														.position_gain_Kpff2 = 0x0000,	//Herkulex DRS-602 pg 22 Position Feedfoward 2nd Gain
														.velocity_gain_Kp = 0x0000,
														.velocity_gain_Ki = 0x0000 },
};

const HerkulexModel HerkulexModel::kDrs0401 = {
	.model_id 					= 0x0401,
	.model_name 				= "DRS-0401",
	.encoder_resolution 		= 2048,		// Herkulex DRS-401 pg 8 Resolution
	.encoder_offset				= 0,		// Herkluex DRS-101/201 pg 25 Absolute Position
	.encoder_has_deadband		= true,		// Herkulex DRS-401 pg 25 Absolute Position
	.position_min_raw			= 42,		// Herkulex DRS-401 pg 20 Min. Position
	.position_max_raw			= 2004,		// Herkulex DRS-401 pg 20 Max. Position
	.tolerance_default_raw 		= 0x06,		// Herkulex DRS-401 pg 21 Inposition Margin
	.velocity_max_deg_s			= 370.3,	// Herkulex DRS-401 pg 9 Maximum Speed (60 / 0.162)
	.velocity_control_supported	= true,
	.velocity_gains_exposed		= false,
	.abs_position_2nd_supported = false,
	.temp_value_in_C            = false,
	.is_moving_threshold_deg    = 0.978,    // Herkulex DRS-401 pg 21 Stop Threshold
	.scale_raw_to_deg			= 0.163,	// Herkulex DRS-401 pg 25 Absolute Position
											// = (360' - 26.7') / 2048
											// Note: this is incorrect in the register table on pg. 22
	.scale_raw_to_pos_2nd_deg	= 0.0,
	.scale_diff_raw_to_deg_s	= 3.634,	// Herkuelx DRS-401 pg 25 Differential Position
											// Unknown how this is calculated.
	.scale_raw_to_volt			= 0.1,		// Herkulex DRS-401 p 24 Voltage
	.control_gains_default		= HerkulexControlGains{	.position_gain_Kp = 0x0046, 	//Herkulex DRS-602 pg 21 Position Kp
														.position_gain_Kd = 0x0000, 	//Herkulex DRS-602 pg 21 Position Kd
														.position_gain_Ki = 0x0000, 	//Herkulex DRS-602 pg 21 Position Ki
														.position_gain_Kpff1 = 0x0000,	//Herkulex DRS-602 pg 21 Position Feedfoward 1st Gain
														.position_gain_Kpff2 = 0x0000,	//Herkulex DRS-602 pg 21 Position Feedfoward 2nd Gain
														.velocity_gain_Kp = 0x0000,
														.velocity_gain_Ki = 0x0000 },
};

const HerkulexModel HerkulexModel::kDrs0402 = {
	.model_id 					= 0x0402,
	.model_name 				= "DRS-0402",
	.encoder_resolution 		= 12962,	// Herkulex DRS-402 pg 8 Resolution
	.encoder_offset				= 9903,		// Herkulex DRS-402 pg 25 Absolute Position
	.encoder_has_deadband		= false,	// Herkulex DRS-402 pg 25 Absolute Position
	.position_min_raw			= 0,		// Herkulex DRS-402 pg 20 Min. Position defaults to 10627 (beginning of deadband)
											//	but actual range is -450, 450. -450 corresponds to raw position 0
	.position_max_raw			= 32767,	// Herkulex DRS-402 pg 20 Min. Position defaults to 22129 (end of deadband)
											//	but actual range is -450, 450. 450 corresponds to raw position 32767
	.tolerance_default_raw 		= 0x06,		// Herkulex DRS-402 pg 21 Inposition Margin
	.velocity_max_deg_s			= 370.3,	// Herkulex DRS-402 pg 9 Maximum Speed (60 / 0.162)
	.velocity_control_supported	= true,
	.velocity_gains_exposed		= true,
	.abs_position_2nd_supported = true,
	.temp_value_in_C            = true,
	.is_moving_threshold_deg    = 0.1668,   // Herkulex DRS-402 pg 21 Stop Threshold
	.scale_raw_to_deg			= 0.02778,	// Herkulex DRS-402 pg 25 Absolute Position
											// = 360' / 12962
	.scale_diff_raw_to_deg_s	= 0.62,		// Herkulex DRS-402 pg 25 Differential Position
											// Unknown how this is calculated.
	.scale_raw_to_pos_2nd_deg	= 0.163,	// Herkulex DRS-402 pg 25 Absolute 2nd Position
	.scale_raw_to_volt			= 0.1,		// Herkulex DRS-402 p 24 Voltage
	.control_gains_default		= HerkulexControlGains{	.position_gain_Kp = 0x0046, 	//Herkulex DRS-602 pg 21 Position Kp
														.position_gain_Kd = 0x0000, 	//Herkulex DRS-602 pg 21 Position Kd
														.position_gain_Ki = 0x0000, 	//Herkulex DRS-602 pg 21 Position Ki
														.position_gain_Kpff1 = 0x0000,	//Herkulex DRS-602 pg 21 Position Feedfoward 1st Gain
														.position_gain_Kpff2 = 0x0000,	//Herkulex DRS-602 pg 21 Position Feedfoward 2nd Gain
														.velocity_gain_Kp = 0x0064,		//Herkulex DRS-602 pg 21 Velocity Kp
														.velocity_gain_Ki = 0x2EE0 },	//Herkulex DRS-602 pg 21 Velocity Ki
};

const HerkulexModel HerkulexModel::kDrs0601 = {
	.model_id 					= 0x0601,
	.model_name 				= "DRS-0601",
	.encoder_resolution		 	= 2048,		// Herkulex DRS-601 pg 8 Resolution
	.encoder_offset				= 0,		// Herkluex DRS-601 pg 25 Absolute Position
	.encoder_has_deadband		= true,		// Herkulex DRS-601 pg 25 Absolute Position
	.position_min_raw			= 42,		// Herkulex DRS-601 pg 20 Min. Position
	.position_max_raw			= 2004,		// Herkulex DRS-601 pg 20 Max. Position
	.tolerance_default_raw 		= 0x06,		// Herkulex DRS-601 pg 21 Inposition Margin
	.velocity_max_deg_s			= 365.8,	// Herkulex DRS-601 pg 9 Maximum Speed (60 / 0.164)
	.velocity_control_supported	= true,
	.velocity_gains_exposed		= false,
	.abs_position_2nd_supported = false,
	.temp_value_in_C            = true,
	.is_moving_threshold_deg    = 0.978,    // Herkulex DRS-601 pg 21 Stop Threshold
	.scale_raw_to_deg			= 0.163,	// Herkulex DRS-601 pg 25 Absolute Position
											// = (360' - 26.7') / 2048
	.scale_diff_raw_to_deg_s	= 3.634,	// Herkulex DRS-601 pg 25 Differential Position
											// Unknown how this is calculated.
	.scale_raw_to_pos_2nd_deg	= 0.0,
	.scale_raw_to_volt			= 0.1,		// Herkulex DRS-601 p 24 Voltage
	.control_gains_default		= HerkulexControlGains{	.position_gain_Kp = 0x00A0, 	//Herkulex DRS-602 pg 21 Position Kp
														.position_gain_Kd = 0x0000,		//Herkulex DRS-602 pg 21 Position Kd
														.position_gain_Ki = 0x0000,		//Herkulex DRS-602 pg 21 Position Ki
														.position_gain_Kpff1 = 0x0000,	//Herkulex DRS-602 pg 21 Position Feedfoward 1st Gain
														.position_gain_Kpff2 = 0x0000,	//Herkulex DRS-602 pg 21 Position Feedfoward 2nd Gain
														.velocity_gain_Kp = 0x0000,
														.velocity_gain_Ki = 0x0000 },
};

const HerkulexModel HerkulexModel::kDrs0602 = {
	.model_id 					= 0x0602,
	.model_name 				= "DRS-0602",
	.encoder_resolution 		= 12962,	// Herkulex DRS-602 pg 8 Resolution
	.encoder_offset				= 9903,		// Herkluex DRS-602 pg 25 Absolute Position
	.encoder_has_deadband		= false,	// Herkulex DRS-401 pg 25 Absolute Position
	.position_min_raw			= 0,		// Herkulex DRS-602 pg 20 Min. Position defaults to 10627 (beginning of deadband)
											//	but actual range is -450, 450. -450 corresponds to raw position 0
	.position_max_raw			= 32767,	// Herkulex DRS-602 pg 20 Min. Position defaults to 22129 (end of deadband)
											//	but actual range is -450, 450. 450 corresponds to raw position 32767
	.tolerance_default_raw 		= 0x06,		// Herkulex DRS-602 pg 21 Inposition Margin
	.velocity_max_deg_s			= 365.8,	// Herkulex DRS-602 pg 9 Maximum Speed (60 / 0.164)
	.velocity_control_supported	= true,
	.velocity_gains_exposed		= true,
	.abs_position_2nd_supported = true,
	.temp_value_in_C            = true,
	.is_moving_threshold_deg    = 0.1668,   // Herkulex DRS-602 pg 21 Stop Threshold
	.scale_raw_to_deg			= 0.02778,	// Herkulex DRS-602 pg 25 Absolute Position
											// = 360' / 12962
	.scale_raw_to_pos_2nd_deg	= 0.163,	// Herkulex DRS-602 pg 25 Absolute 2nd Position
	.scale_diff_raw_to_deg_s	= 0.62,		// Herkulex DRS-602 pg 25 Differential Position
											// Unknown how this is calculated.
	.scale_raw_to_volt			= 0.1,		// Herkulex DRS-602 pg 24 Voltage
	.control_gains_default 		= HerkulexControlGains{ .position_gain_Kp = 0x0046, 	//Herkulex DRS-602 pg 21 Position Kp
														.position_gain_Kd = 0x0000,		//Herkulex DRS-602 pg 21 Position Kd
														.position_gain_Ki = 0x0000,		//Herkulex DRS-602 pg 21 Position Ki
														.position_gain_Kpff1 = 0x0000,	//Herkulex DRS-602 pg 21 Position Feedfoward 1st Gain
														.position_gain_Kpff2 = 0x0000,	//Herkulex DRS-602 pg 21 Position Feedfoward 2nd Gain
														.velocity_gain_Kp = 0x0064,		//Herkulex DRS-602 pg 21 Velocity Kp
														.velocity_gain_Ki = 0x2EE0 },	//Herkulex DRS-602 pg 21 Velocity Ki
};
// clang-format on

} // namespace momentum
