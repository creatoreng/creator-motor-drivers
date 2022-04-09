/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/herkulex/Herkulex.hpp>

#include <math.h>

namespace momentum
{

Herkulex::Herkulex(const std::shared_ptr<HerkulexBus> bus)
  : _is_connected(false),
	_zero_offset_deg(0.0),
	_gear_ratio(1.0),
	_default_playtime_ms(HerkulexModel::playtime_max_ms),
	// Set motor ID to an invalid motor ID until initialized to prevent communicating to another motor.
	_motor_id(HerkulexPacket::kMotorIdMax),
	_ack_policy(HerkulexAckPolicy::kReplyRead),
	_model(HerkulexModel::kUnknown),
	_cached_state(),
	_bus(bus),
	_status(),
	_motor_status()
{
	_cached_state.motor_event.clear();
	_cached_state.torque_mode		= TorqueControlMode::kUnknown;
	_cached_state.previous_velocity = 0.0;
}

Herkulex::~Herkulex(void)
{
	// stop the motor on destruction (as long as the bus is valid)
	disconnect();
}

const HerkulexStatus &Herkulex::status(void) const
{
	// order: bus -> driver -> motor
	return HerkulexStatus::max(_bus->status(), _status, _motor_status);
}

std::string Herkulex::status_string(void) const
{
	return status().to_string();
}

bool Herkulex::status_ok(void) const
{
	return status().ok();
}

void Herkulex::status_clear(void)
{
	_motor_status.clear();
	_status.clear();
}

const HerkulexStatus &Herkulex::driver_status(void) const
{
	return Status<HerkulexContext>::max(_bus->status(), _status);
}

uint8_t Herkulex::get_motor_id(void) const
{
	return _motor_id;
}

double Herkulex::get_max_velocity_deg_s(void) const
{
	return _model.velocity_max_deg_s / _gear_ratio;
}

void Herkulex::set_zero_offset_deg(const double offset_deg)
{
	_zero_offset_deg = offset_deg;
}

double Herkulex::get_zero_offset_deg(void) const
{
	return _zero_offset_deg;
}

void Herkulex::set_gear_ratio(const double gear_ratio)
{
	if (gear_ratio > 0.0) {
		_gear_ratio = gear_ratio;
			} else {
		_status.raise_warning(HerkulexStatus::Code::kMotorParameterInvalid,
							  "Gear ratio must be a positive, non-zero value. Value was not changed.");
	}
}

void Herkulex::set_default_playtime_ms(const uint32_t playtime_ms)
{
	if (playtime_ms <= _model.playtime_max_ms) {
		_default_playtime_ms = playtime_ms;
			} else {
		_status.raise_warning(HerkulexStatus::Code::kMotorParameterInvalid, "Default playtime must be less than " +
																				std::to_string(_model.playtime_max_ms) +
																				". Default playtime unchanged.");
	}
}

bool Herkulex::connect(const uint8_t motor_id)
{
		if (connect_weak(motor_id)) {
				// reset the motor to clear any previous errors
		reboot();

		if (_is_connected && driver_status().ok()) {
						// place into free torque mode
			write_torque_control(TorqueControlMode::kFree);
			// set garbage collection rate
			write_register(HerkulexRegister::kEepPacketGarbageCheckPeriod,
						   HerkulexBus::kDeviceGarbageCollectionTickPeriods);
			write_register(HerkulexRegister::kRamPacketGarbageCheckPeriod,
						   HerkulexBus::kDeviceGarbageCollectionTickPeriods);
			// set range of motion - some motors have no deadband, but factory settings limit motion
			write_software_limits(false);
			// ensure that calibration (what appears to be a position bias on the motor) is set to zero.
			verify_calibration_difference();
			// set control loop gains to default values (not all models expose velocity gains)
			write_default_position_gains();
			if (_model.velocity_gains_exposed)
				write_default_velocity_gains();
			// set position tolerance to default value
			write_position_tolerance_default();
			// set torque limit to some initial default
			write_torque_limit_percent(kDefaultTorqueLimitPercent);
			// set torque offset to zero
			write_torque_offset_percent(0.0);
			write_acceleration_ratio_default();
			write_acceleration_time_default();

			// set default max temp limit
			if (_model.temp_value_in_C)
				write_temperature_limit_C(_model.max_temp_default_C);

			// set torque release policy
			write_register(HerkulexRegister::kEepTorquePolicy,
						   static_cast<uint8_t>(HerkulexStatusRegister::StatusFieldMask::kExceedInputVoltageLimit) |
							   static_cast<uint8_t>(HerkulexStatusRegister::StatusFieldMask::kExceedTemperatureLimit) |
							   static_cast<uint8_t>(HerkulexStatusRegister::StatusFieldMask::kOverloadDetected));
			write_register(HerkulexRegister::kRamTorquePolicy,
						   static_cast<uint8_t>(HerkulexStatusRegister::StatusFieldMask::kExceedInputVoltageLimit) |
							   static_cast<uint8_t>(HerkulexStatusRegister::StatusFieldMask::kExceedTemperatureLimit) |
							   static_cast<uint8_t>(HerkulexStatusRegister::StatusFieldMask::kOverloadDetected));
		}
	}

	_is_connected = driver_status().ok();
	return _is_connected;
}

bool Herkulex::connect_weak(const uint8_t motor_id)
{
	// already connected?
	if (is_connected()) {
		_status.raise_warning(HerkulexStatus::Code::kAlreadyOpen);
	}
	// not yet connected
	else if (driver_status().ok()) {
		_motor_id = motor_id;

		// bus is connected?
		if (_bus->is_connected()) {
			// set initialized true to enable initial communication to the motor; reset if an error occurs.
			_is_connected = true;
			// suppress timeouts while configuring ack policy
			_status.suppress_push(HerkulexStatus::Code::kTimeout);
			// first read acknowledgement policy to know when to expect a response
			read_ack_policy();
			// set acknowledgement policy - and push back to flash so it is retained after reset
			write_ack_policy(HerkulexAckPolicy::kReplyAll);
			_status.suppress_pop(HerkulexStatus::Code::kTimeout);
			_status.clear(HerkulexStatus::Code::kTimeout);

			// update status register to verify connectivity
			read_status_register();

			if (driver_status().ok()) {
				// read model
				_model = read_model_number();

				// driver faults indicate the motor can communicate, but it ain't gonna work
				if (_motor_status == HerkulexStatus::Code::kMotorDriverFault) {
					_status.raise_error(HerkulexStatus::Code::kMotorDriverFault,
										"Motor status register indicates a driver fault.");
				}
			}
		} else {
			_status.raise_error(HerkulexStatus::Code::kNotOpen, "attempted to connect before bus connected.");
		}
	}
	_is_connected = driver_status().ok();

	return _is_connected;
}

bool Herkulex::is_connected(void) const
{
	return _bus->is_connected() && _is_connected;
}

void Herkulex::disconnect(void)
{
	if (is_connected()) {
		torque_release();

		// Raise a warning if the motor stopped in its deadband region
		if (_model.is_in_deadband(_model.raw_to_deg(read_register(HerkulexRegister::kRamAbsolutePosition)))) {
			_status.raise_warning(HerkulexStatus::Code::kMotorStoppedInDeadband);
		}
		// turn off the damn flashing red LEDs
		clear_deadband_error();

				disconnect_weak();
	}
	_is_connected = false;
}

void Herkulex::disconnect_weak(void)
{
	_is_connected = false;
}

void Herkulex::verify_calibration_difference(void)
{
	if (driver_status().ok()) {
		uint8_t calibration_difference = read_register(HerkulexRegister::kRamCalibrationDifferenceLower);
		if (calibration_difference != 0) {
			_status.raise_warning(HerkulexStatus::Code::kInvalidConfiguration,
								  "Calibration difference is nonzero. Lower calibration difference is set to " +
									  util::int_to_hex(calibration_difference) + ".");
		}
		calibration_difference = read_register(HerkulexRegister::kRamCalibrationDifferenceUpper);
		if (calibration_difference != 0) {
			_status.raise_warning(HerkulexStatus::Code::kInvalidConfiguration,
								  "Calibration difference is nonzero. Upper calibration difference is set to " +
									  util::int_to_hex(calibration_difference) + ".");
		}
	}
}

HerkulexAckPolicy Herkulex::read_ack_policy(void)
{
	if (driver_status().ok()) {
		_motor_status.suppress_push(HerkulexStatus::Code::kTimeout);
		_ack_policy = static_cast<HerkulexAckPolicy>(read_register(HerkulexRegister::kRamAckPolicy));
		if (_status == HerkulexStatus::Code::kTimeout) {
			_status.clear();
			// assume no acknowledgement means ack policy is none
			// if the assumption is incorrect, the next communication will fail
			_ack_policy = HerkulexAckPolicy::kNoReply;
		}
		_motor_status.suppress_pop(HerkulexStatus::Code::kTimeout);
	}
	return _ack_policy;
}

void Herkulex::read_status_register(void)
{
	if (driver_status().ok()) {
		// status register is automatically updated on write_command due to ACK policy.
		write_command(HerkulexCommand::STAT);
	}
}

void Herkulex::write_clear_status_register(void)
{
	if (driver_status().ok()) {
		// temporarily suppress motor errors while pushing status register
		auto scoped_suppression = _motor_status.suppress_all();
		write_register(HerkulexRegister::kRamStatusError, 0x00);
		write_register(HerkulexRegister::kRamStatusDetail, 0x00);
		_motor_status.clear();
	}
}

HerkulexModel Herkulex::read_model_number(void)
{
	HerkulexModel model = _model; // in case of error, return model number in case it was previously cached
	if (driver_status().ok()) {
		// first model number register is the series, i.e. 0x06 = 6 series
		// second model number register is the model, i.e. 0x02 = 2
		uint16_t model_id;
		model_id =
			(read_register(HerkulexRegister::kEepModelNo1) << 0x08) | read_register(HerkulexRegister::kEepModelNo2);
		switch (model_id) {
		case 0x0101:
			model = HerkulexModel::kDrs0101;
			break;
		case 0x0201:
			model = HerkulexModel::kDrs0201;
			break;
		case 0x0401:
			model = HerkulexModel::kDrs0401;
			break;
		case 0x0402:
			model = HerkulexModel::kDrs0402;
			break;
		case 0x0601:
			model = HerkulexModel::kDrs0601;
			break;
		case 0x0602:
			model = HerkulexModel::kDrs0602;
			break;
		default:
			_status.raise_error(HerkulexStatus::Code::kMotorModelUnknown);
			break;
		}
	}
	return model;
}

void Herkulex::write_ack_policy(const HerkulexAckPolicy policy)
{
	if (driver_status().ok()) {
		// push first to flash so policy is not changed online, then change in RAM.
		write_register(HerkulexRegister::kRamAckPolicy, static_cast<uint8_t>(policy));
		_ack_policy = policy;
		write_register(HerkulexRegister::kEepAckPolicy, static_cast<uint8_t>(policy));
		// Here's a nice mofo hack... for some reason, the ACK policy seems to take
		// a few pings to take hold. Time delay does not seem to work here, but oddly,
		// requesting three status packets does the trick.
		// Without these lines, subsequent reads will timeout. No clue. WTF?
		read_status_register();
		read_status_register();
		read_status_register();
	}
}

void Herkulex::write_position_reset(const HerkulexPositionResetCommand command)
{
	if (driver_status().ok())
		write_register(HerkulexRegister::kRamAux1, static_cast<uint8_t>(command));
}

void Herkulex::write_torque_control(const TorqueControlMode mode)
{
	if (driver_status().ok() && mode != _cached_state.torque_mode) {
		std::string mode_name;
		switch (mode) {
		case TorqueControlMode::kBrake:
			mode_name = "brake";
			break;
		case TorqueControlMode::kFree:
			mode_name = "free";
			break;
		case TorqueControlMode::kOn:
			mode_name = "on";
			break;
		case TorqueControlMode::kUnknown:
			mode_name = "unknown";
			break;
		}
		// Cache torque mode before writing LEDs, otherwise the change is not seen by torque_mode_to_leds()
		_cached_state.torque_mode = mode;

		// Write new torque mode.
		write_register(HerkulexRegister::kRamTorqueControl, static_cast<uint8_t>(mode));
		write_leds(torque_mode_to_leds());
	}
}

void Herkulex::write_position_gains(const Herkulex::PositionGains gains)
{
	write_position_gains(gains.Kp, gains.Kd, gains.Ki, gains.Kpff1, gains.Kpff2);
}

void Herkulex::write_position_gains(
	const uint16_t Kp, const uint16_t Kd, const uint16_t Ki, const uint16_t Kpff1, const uint16_t Kpff2)
{
	if (driver_status().ok() && is_connected()) {
		if (Kp > HerkulexControlGains::kGainMax) {
			_status.raise_warning(HerkulexStatus::Code::kMotorParameterInvalid,
								  "Desired position proportional gain is outside allowed range of [0, " +
									  std::to_string(HerkulexControlGains::kGainMax) + "].");
		} else {
			write_register(HerkulexRegister::kRamPositionKp, Kp);
			write_register(HerkulexRegister::kEepPositionKp, Kp);
		}

		if (Kd > HerkulexControlGains::kGainMax) {
			_status.raise_warning(HerkulexStatus::Code::kMotorParameterInvalid,
								  "Desired position derivative gain is outside allowed range of [0, " +
									  std::to_string(HerkulexControlGains::kGainMax) + "].");
		} else {
			write_register(HerkulexRegister::kRamPositionKd, Kd);
			write_register(HerkulexRegister::kEepPositionKd, Kd);
		}

		if (Ki > HerkulexControlGains::kGainMax) {
			_status.raise_warning(HerkulexStatus::Code::kMotorParameterInvalid,
								  "Desired position integral gain is outside allowed range of [0, " +
									  std::to_string(HerkulexControlGains::kGainMax) + "].");
		} else {
			write_register(HerkulexRegister::kRamPositionKi, Ki);
			write_register(HerkulexRegister::kEepPositionKi, Ki);
		}

		if (Kpff1 > HerkulexControlGains::kGainMax) {
			_status.raise_warning(HerkulexStatus::Code::kMotorParameterInvalid,
								  "Desired position feedforward 1st gain is outside allowed range of [0, " +
									  std::to_string(HerkulexControlGains::kGainMax) + "].");
		} else {
			write_register(HerkulexRegister::kRamPositionFeedFwdGain1, Kpff1);
			write_register(HerkulexRegister::kEepPositionFeedFwdGain1, Kpff1);
		}

		if (Kpff2 > HerkulexControlGains::kGainMax) {
			_status.raise_warning(HerkulexStatus::Code::kMotorParameterInvalid,
								  "Desired position feedforward 2nd gain is outside allowed range of [0, " +
									  std::to_string(HerkulexControlGains::kGainMax) + "].");
		} else {
			write_register(HerkulexRegister::kRamPositionFeedFwdGain2, Kpff2);
			write_register(HerkulexRegister::kEepPositionFeedFwdGain2, Kpff2);
		}
	}
}

void Herkulex::write_default_position_gains(void)
{
	write_position_gains(_model.control_gains_default.position_gain_Kp, _model.control_gains_default.position_gain_Kd,
						 _model.control_gains_default.position_gain_Ki,
						 _model.control_gains_default.position_gain_Kpff1,
						 _model.control_gains_default.position_gain_Kpff2);
}

Herkulex::PositionGains Herkulex::read_position_gains(void)
{
	Herkulex::PositionGains gains = {0, 0, 0, 0, 0};

	if (_status.ok() && is_connected()) {
		// Read position control gains currently on motor
		gains.Kp	= read_register(HerkulexRegister::kRamPositionKp);
		gains.Kd	= read_register(HerkulexRegister::kRamPositionKd);
		gains.Ki	= read_register(HerkulexRegister::kRamPositionKi);
		gains.Kpff1 = read_register(HerkulexRegister::kRamPositionFeedFwdGain1);
		gains.Kpff2 = read_register(HerkulexRegister::kRamPositionFeedFwdGain2);
	}

	return gains;
}

void Herkulex::write_velocity_gains(const uint16_t Kp, const uint16_t Ki)
{
	if (!driver_status().ok()) {
	} else if (!_model.velocity_gains_exposed) {
		_status.raise_warning(HerkulexStatus::Code::kMotorVelocityUnsupported, "Cannot set velocity control gains.");
	} else {
		if (Kp > HerkulexControlGains::kGainMax) {
			_status.raise_warning(HerkulexStatus::Code::kMotorParameterInvalid,
								  "Desired velocity proportional gain is outside allowed range of [0, " +
									  std::to_string(HerkulexControlGains::kGainMax) + "].");
		} else {
			write_register(HerkulexRegister::kRamVelocityKp, Kp);
			write_register(HerkulexRegister::kEepVelocityKp, Kp);
		}

		if (Kp > HerkulexControlGains::kGainMax) {
			_status.raise_warning(HerkulexStatus::Code::kMotorParameterInvalid,
								  "Desired velocity integral gain is outside allowed range of [0, " +
									  std::to_string(HerkulexControlGains::kGainMax) + "].");
		} else {
			write_register(HerkulexRegister::kRamVelocityKi, Ki);
			write_register(HerkulexRegister::kEepVelocityKi, Ki);
		}
	}
}

void Herkulex::write_velocity_gains(const Herkulex::VelocityGains gains)
{
	write_velocity_gains(gains.Kp, gains.Ki);
}

void Herkulex::write_default_velocity_gains(void)
{
	write_velocity_gains(_model.control_gains_default.velocity_gain_Kp, _model.control_gains_default.velocity_gain_Ki);
}

Herkulex::VelocityGains Herkulex::read_velocity_gains(void)
{
	Herkulex::VelocityGains gains = {0, 0};

	if (_status.ok() && is_connected()) {
		// Read velocity control gains currently on motor
		gains.Kp = read_register(HerkulexRegister::kRamVelocityKp);
		gains.Ki = read_register(HerkulexRegister::kRamVelocityKi);
	}

	return gains;
}

void Herkulex::write_position_tolerance_deg(const double tolerance_deg)
{
	// Status ok?
	if (driver_status().ok()) {
		// convert degree to raw count
		uint8_t tolerance_raw = _model.tolerance_deg_to_raw(tolerance_deg);

		// check if raw count is in valid range
		if (tolerance_raw > HerkulexRegister::kEepInpositionMargin.range.max) {
			_status.raise_warning(HerkulexStatus::Code::kMotorParameterInvalid, "In position tolerance out of range");
		}
		// write to Inposition register
		else {
						write_register(HerkulexRegister::kEepInpositionMargin, tolerance_raw);
			write_register(HerkulexRegister::kRamInpositionMargin, tolerance_raw);
		}
	}
}

void Herkulex::write_position_tolerance_default(void)
{
	// Status ok?
	if (driver_status().ok()) {
		// note: do not call _model.raw_to_deg() as it is designed for absolute, not relative
		// positions and will offset the value by 180'.
		write_position_tolerance_deg(_model.tolerance_default_raw * _model.scale_raw_to_deg);
	}
}

double Herkulex::read_position_tolerance_deg(void)
{
	double tolerance_deg = 0;
	// Status ok?
	if (driver_status().ok()) {
		tolerance_deg = read_register(HerkulexRegister::kRamInpositionMargin) * _model.scale_raw_to_deg;
			}
	return tolerance_deg;
}

void Herkulex::write_moving_detection_params(const double threshold_deg, const uint16_t period_ms)
{
	// Error already occurs
	if (!driver_status().ok()) {
	}
	// Threshold should be positive
	else if (threshold_deg < 0) {
		_status.raise_warning(HerkulexStatus::Code::kMotorParameterInvalid,
							  "Stop detection threshold cannont be a negative value");
	}
	// Okay to write stop detection parameters
	else {
		uint8_t threshold_raw = static_cast<uint8_t>(threshold_deg / _model.scale_raw_to_deg);
		uint8_t period_raw	= _model.time_ms_to_raw(period_ms);
		if (period_raw == 255 || threshold_raw == 255) {
			_status.raise_warning(HerkulexStatus::Code::kMotorParameterInvalid,
								  "A moving detection parameter raw value is too high. No changes made.");
		} else {
			write_register(HerkulexRegister::kRamStopThreshold, threshold_raw);
			write_register(HerkulexRegister::kRamStopDetectPeriod, period_raw);
		}
	}
}

void Herkulex::write_moving_detection_defaults(void)
{
	write_moving_detection_params(_model.is_moving_threshold_deg, _model.is_moving_detect_period_ms);
}

double Herkulex::read_moving_theshold_deg(void)
{
	double threshold_deg = 0.0;

	if (driver_status().ok()) {
		uint16_t raw  = read_register(HerkulexRegister::kRamStopThreshold);
		threshold_deg = static_cast<double>(raw * _model.scale_raw_to_deg);
	}

	return threshold_deg;
}

uint16_t Herkulex::read_moving_period_ms(void)
{
	uint16_t period_ms = 0;

	if (driver_status().ok()) {
		double raw = static_cast<double>(read_register(HerkulexRegister::kRamStopDetectPeriod));
		period_ms  = static_cast<uint16_t>(raw * _model.tick_period_ms);
	}

	return period_ms;
}

void Herkulex::write_acceleration_ratio_percent(const double acceleration_ratio_percent)
{
	if (driver_status().ok()) {
		if (acceleration_ratio_percent < 0.0 || acceleration_ratio_percent > 50.0) {
			_status.raise_warning(HerkulexStatus::Code::kMotorParameterInvalid,
								  "Acceleration ratio must be within 0 and 50%. Change ignored.");
		} else {
			write_register(HerkulexRegister::kRamAccelRatio, static_cast<uint8_t>(acceleration_ratio_percent));
					}
	}
}

void Herkulex::write_acceleration_ratio_default(void)
{
	write_acceleration_ratio_percent(_model.accel_ratio_percent_default);
}

double Herkulex::read_acceleration_ratio_percent(void)
{
	double acceleration_ratio_percent = 0.0;
	if (driver_status().ok()) {
		acceleration_ratio_percent = static_cast<double>(read_register(HerkulexRegister::kRamAccelRatio));
			}
	return acceleration_ratio_percent;
}

void Herkulex::write_acceleration_time_ms(const double acceleration_time_ms)
{
	if (driver_status().ok()) {
		if (acceleration_time_ms < 0.0 || acceleration_time_ms > _model.accel_time_max_ms_max) {
			_status.raise_warning(HerkulexStatus::Code::kMotorParameterInvalid,
								  "Max Acceleration time must be within 0 and 2844 ms. Change ignored.");
		} else {
			write_register(HerkulexRegister::kRamMaxAccelTime,
						   static_cast<uint8_t>(acceleration_time_ms / _model.tick_period_ms));
		}
	}
}

void Herkulex::write_acceleration_time_default(void)
{
	write_acceleration_time_ms(_model.accel_time_max_ms_default);
}

double Herkulex::read_accleration_time_ms(void)
{
	double acceleration_time_ms = 0.0;
	if (driver_status().ok()) {
		acceleration_time_ms =
			static_cast<double>(read_register(HerkulexRegister::kRamMaxAccelTime)) * _model.tick_period_ms;
	}
	return acceleration_time_ms;
}

double Herkulex::read_position_error_deg(void)
{
	double error_deg = 0.0;

	if (driver_status().ok()) {
		double curr_pos_deg   = _model.raw_to_deg(read_register(HerkulexRegister::kRamAbsolutePosition));
		double target_pos_deg = _model.raw_to_deg(read_register(HerkulexRegister::kRamAbsoluteGoalPosition));

		error_deg = (curr_pos_deg - target_pos_deg) / _gear_ratio;
	}

	return error_deg;
}

void Herkulex::write_leds(const Herkulex::Led leds)
{
	// status OK?
	if (driver_status().ok()) {
		write_register(HerkulexRegister::kRamLedControl, static_cast<uint8_t>(leds));
	}
}

Herkulex::Led Herkulex::torque_mode_to_leds(void) const
{
	switch (_cached_state.torque_mode) {
	case TorqueControlMode::kBrake:
		return Herkulex::Led::kRed;
		break;
	case TorqueControlMode::kFree:
		return Herkulex::Led::kBlue;
		break;
	case TorqueControlMode::kOn:
		return Herkulex::Led::kGreen;
		break;
	default:
		return Herkulex::Led::kGreen;
		break;
	}
}

double Herkulex::read_position_deg(const bool restrict_range)
{
	double position = 0;
	if (driver_status().ok()) {
		// request raw position
		position = _model.raw_to_deg(read_register(HerkulexRegister::kRamAbsolutePosition));

		// account for gear train
		position /= _gear_ratio;

		// account for calibration bias (if any)
		position -= _zero_offset_deg;

		// wrap to [-180, 180]
		if (restrict_range) {
			position = _model.wrap_angle_deg(position);
		}
	}
	return position;
}

double Herkulex::read_goal_position_deg(const bool restrict_range)
{
	double position = 0;
	if (driver_status().ok()) {
		// request goal position
		position = _model.raw_to_deg(read_register(HerkulexRegister::kRamAbsoluteGoalPosition));

		// account for gear train
		position /= _gear_ratio;

		// account for calibration bias (if any)
		position -= _zero_offset_deg;

		// wrap to [-180, 180]
		if (restrict_range) {
			position = _model.wrap_angle_deg(position);
		}
	}
	return position;
}

double Herkulex::read_position_2nd_deg(void)
{
	double position = 0.0;

	if (driver_status().ok()) {
		if (_model.abs_position_2nd_supported) {
			position = _model.raw_pos_2nd_to_deg(read_register(HerkulexRegister::kRamAbsoluteSecondPosition));
		} else {
			_status.raise_warning(HerkulexContext::Code::kMotor2ndPositionUnsupported);
		}
	}

	return position;
}

void Herkulex::drive_absolute_deg(double position_deg)
{
	drive_absolute_deg_playtime_ms(position_deg, _default_playtime_ms);
}

void Herkulex::drive_absolute_deg_velocity_deg_s(double position_deg, double velocity_deg_s)
{
	if (status().ok()) {
		double distance		 = fabs(position_deg - read_position_deg());
		uint32_t playtime_ms = static_cast<uint32_t>(ceil(1000 * distance / velocity_deg_s));

		if (velocity_deg_s * _gear_ratio > _model.velocity_max_deg_s) {
			_status.raise_warning(HerkulexStatus::Code::kMotorVelocityOutOfBound,
								  "Requested velocity of " + std::to_string(velocity_deg_s) +
									  " was greater than the motor max velocity.");
		} else {
			write_absolute_drive(position_deg, playtime_ms);
		}
	}
}

void Herkulex::drive_absolute_deg_playtime_ms(double position_deg, uint32_t playtime_ms)
{
	if (status().ok()) {
		// Is average velocity greater than model max? (x1000 since playtime is in ms)
		// Note, the actual peak velocity will exceed the average since the velocity
		// profile is trapezoidal not rectangular.
		double move_distance	  = position_deg - read_position_deg();
		double avg_velocity_deg_s = fabs(move_distance / static_cast<double>(playtime_ms)) * 1000.0;

		if (avg_velocity_deg_s * _gear_ratio > _model.velocity_max_deg_s) {
			_status.raise_warning(HerkulexStatus::Code::kMotorVelocityOutOfBound,
								  "Requested velocity of " + std::to_string(avg_velocity_deg_s) +
									  " was greater than the motor max velocity.");
		} else {
			write_absolute_drive(position_deg, playtime_ms);
		}
	}
}

void Herkulex::write_absolute_drive(double position_deg, uint32_t playtime_ms)
{
	if (status().ok()) {
		// calibrate and bound position
		position_deg += _zero_offset_deg;
		position_deg *= _gear_ratio;

		// validate playtime
		if (playtime_ms > _model.playtime_max_ms) {
			_status.raise_warning(HerkulexStatus::Code::kPacketPlaytimeBound);
			playtime_ms = _model.playtime_max_ms;
		}

		uint16_t jog = _model.deg_to_jog(position_deg);
		if (jog < _model.position_min_raw || jog > _model.position_max_raw) {
			_status.raise_error(HerkulexStatus::Code::kMotorPositionOutOfBound,
								"Requested position of " + std::to_string(position_deg) +
									" was outside the bounds of this motor.");
		} else {
			write_jog(jog, JogMode::kPositionControl, playtime_ms);
			_cached_state.drive_command_was_position_control = true;
			_cached_state.previous_velocity					 = 0.0;
		}
	}
}

void Herkulex::drive_velocity_deg_s(double velocity_deg_s)
{
	if (driver_status().ok()) {
		// verify this model supports velocity control
		if (_model.velocity_control_supported) {
			// Take into account gear train
			velocity_deg_s *= _gear_ratio;

			// bound velocity
			if (fabs(velocity_deg_s) > _model.velocity_max_deg_s) {
				_status.raise_error(HerkulexStatus::Code::kMotorVelocityOutOfBound,
									"Requested velocity of " + std::to_string(velocity_deg_s) +
										" was greater than the motor max velocity.");
			} else {
				write_jog(_model.deg_s_to_jog(velocity_deg_s), JogMode::kTurnMode);
				_cached_state.drive_command_was_position_control = false;
				_cached_state.previous_velocity					 = velocity_deg_s;
			}
		} else {
			// if the motor does not support velocity control, throw a warning
			_status.raise_warning(HerkulexStatus::Code::kMotorVelocityUnsupported, "No changes will be made to motor.");
		}
	}
}

double Herkulex::read_velocity_deg_s(void)
{
	double speed_deg_s = 0.0;
	if (driver_status().ok()) {
		int16_t raw_value = static_cast<int16_t>(read_register(HerkulexRegister::kRamDifferentialPosition));
		speed_deg_s		  = _model.raw_to_deg_s(raw_value) / _gear_ratio;
	}

	return speed_deg_s;
}

double Herkulex::read_torque_limit_percent(void)
{
	double torque_percent = 0.0;
	if (driver_status().ok()) {
		torque_percent = _model.torque_raw_to_percent(read_register(HerkulexRegister::kRamMaxPwm));
	}
	return torque_percent;
}

void Herkulex::write_torque_limit_percent(const double torque_limit_percent)
{
	if (!driver_status().ok()) {
	} else if (torque_limit_percent > 100.0 || torque_limit_percent < 0.0) {
		_status.raise_warning(HerkulexStatus::Code::kMotorTorquePercentOutOfBound);
	} else {
		write_register(HerkulexRegister::kRamMaxPwm, _model.torque_percent_to_raw(torque_limit_percent));
	}
}

double Herkulex::read_torque_percent(void)
{
	double torque_percent = 0.0;
	if (driver_status().ok()) {
		torque_percent = _model.torque_raw_to_percent(read_register(HerkulexRegister::kRamPwm));
	}
	return torque_percent;
}

void Herkulex::drive_torque_percent(const double torque_percent)
{
	if (driver_status().ok()) {
		// Jog can only support either torque or velocity control, depending on the model.
		if (_model.velocity_control_supported) {
			// if the motor does not support torque control, throw a warning
			_status.raise_warning(HerkulexStatus::Code::kMotorTorqueUnsupported, "No changes will be made to motor.");
		} else {
			// Read max torque percent limit to check if user is requesting valid number
			double torque_limit = read_torque_limit_percent();

			if (abs(torque_percent) > torque_limit) {
				_status.raise_warning(HerkulexStatus::Code::kMotorTorqueOutOfBound,
									  "Desired drive torque is greater than set limit of " +
										  std::to_string(torque_limit) + "%.");
			} else {
								write_jog(_model.torque_percent_to_jog(torque_percent), JogMode::kTurnMode);
				_cached_state.drive_command_was_position_control = false;
				_cached_state.previous_velocity					 = torque_percent;
			}
		}
	}
}

void Herkulex::write_torque_offset_percent(double torque_offset_percent)
{
	if (driver_status().ok()) {
		if (torque_offset_percent < HerkulexModel::torque_offset_min_percent) {
			_status.raise_warning(HerkulexStatus::Code::kMotorTorquePercentOutOfBound,
								  "Lowest torque offset is " +
									  std::to_string(HerkulexModel::torque_offset_min_percent) +
									  "%. Offset change ignored.");
			torque_offset_percent = HerkulexModel::torque_offset_min_percent;
		} else if (torque_offset_percent > HerkulexModel::torque_offset_max_percent) {
			_status.raise_warning(HerkulexStatus::Code::kMotorTorquePercentOutOfBound,
								  "Highest torque offset is " +
									  std::to_string(HerkulexModel::torque_offset_max_percent) +
									  "%. Offset change ignored.");
			torque_offset_percent = HerkulexModel::torque_offset_max_percent;
		} else {
			write_register(HerkulexRegister::kRamPwmOffset,
						   static_cast<int8_t>(_model.torque_percent_to_raw(torque_offset_percent)));
		}
	}
}

double Herkulex::read_torque_offset_percent(void)
{
	double torque_offset_percent = 0.0;
	if (driver_status().ok()) {
		torque_offset_percent =
			_model.torque_raw_to_percent(static_cast<int8_t>(read_register(HerkulexRegister::kRamPwmOffset)));
	}
	return torque_offset_percent;
}

double Herkulex::read_temperature_C(void)
{
	double temp_C = 0.0;

	if (driver_status().ok()) {
		if (_model.temp_value_in_C) {
			temp_C = static_cast<double>(read_register(HerkulexRegister::kRamTemp));
		} else {
			_status.raise_warning(HerkulexStatus::Code::kMotorTemperatureReadWriteUnsupported);
		}
	}

	return temp_C;
}

int Herkulex::read_temperature_raw(void)
{
	double temp_raw = 0;

	if (driver_status().ok()) {
		if (!_model.temp_value_in_C) {
			temp_raw = read_register(HerkulexRegister::kRamTemp);
		} else {
			_status.raise_warning(HerkulexStatus::Code::kMotorTemperatureReadWriteUnsupported);
		}
	}

	return temp_raw;
}

void Herkulex::write_temperature_limit_C(double temp_C)
{
	if (driver_status().ok()) {
		if (!_model.temp_value_in_C) {
			_status.raise_warning(HerkulexStatus::Code::kMotorTemperatureReadWriteUnsupported);
		} else if (temp_C < static_cast<double>(HerkulexRegister::kRamMaxTemp.range.min) ||
				   temp_C > static_cast<double>(HerkulexRegister::kRamMaxTemp.range.max)) {
			_status.raise_warning(HerkulexStatus::Code::kMotorMaxTemperatureSettingInvalid);
		} else {
			write_register(HerkulexRegister::kRamMaxTemp, static_cast<uint8_t>(temp_C));
		}
	}
}

double Herkulex::read_temperature_limit_C(void)
{
	double temp_limit_C = 0.0;
	if (driver_status().ok()) {
		if (_model.temp_value_in_C) {
			temp_limit_C = static_cast<double>(read_register(HerkulexRegister::kRamMaxTemp));
		} else {
			_status.raise_warning(HerkulexStatus::Code::kMotorTemperatureReadWriteUnsupported);
		}
	}
	return temp_limit_C;
}

void Herkulex::brake(void)
{
	if (is_connected()) {
		// always attempt to stop, even if an error occurred
		HerkulexStatus previous_status;
		_status.cache(previous_status);

		write_torque_control(TorqueControlMode::kBrake);

		// restore previous status
		_status.restore(previous_status);
	}
}

void Herkulex::torque_release(void)
{
	if (is_connected()) {
		// always attempt to stop, even if an error occurred
		HerkulexStatus previous_status;
		_status.cache(previous_status);

		write_torque_control(TorqueControlMode::kFree);
		_cached_state.previous_velocity = 0.0;

		// restore previous status
		_status.restore(previous_status);
	}
}

void Herkulex::reboot(void)
{
	if (is_connected()) {
		_motor_status.clear();
		_status.clear();

				write_clear_status_register();

		// for some reason, sometimes on reboot a motor will
		// transpose checksum2 and optional data[1], resulting
		// in a checksum fail; ignore this error here.
		//
		// also sometimes the acknowledgement is not received, so again, ignore here.
		_status.suppress_push(HerkulexStatus::Code::kChecksumFailAtHost);
		_status.suppress_push(HerkulexStatus::Code::kTimeout);
		write_command(HerkulexCommand::REBOOT);
		_status.suppress_pop(HerkulexStatus::Code::kTimeout);
		_status.suppress_pop(HerkulexStatus::Code::kChecksumFailAtHost);
		_status.clear(HerkulexStatus::Code::kTimeout);
		_status.clear(HerkulexStatus::Code::kChecksumFailAtHost);

		// wait for the motor to reset
		usleep(kRebootDelayMs * 1000);

		// reset cached state
		_cached_state.motor_event.clear();
		_cached_state.torque_mode = TorqueControlMode::kUnknown;

		// set range of motion - some motors have no deadband, but factory settings limit motion
		write_register(HerkulexRegister::kRamMinPosition, _model.position_min_raw);
		write_register(HerkulexRegister::kRamMaxPosition, _model.position_max_raw);

		// read status register to update status and confirm connection
		read_status_register();
		_is_connected = _status.ok();
	}
}

void Herkulex::reset(void)
{
	status_clear();

	_default_playtime_ms = HerkulexModel::playtime_max_ms;
	_cached_state.motor_event.clear();
	_cached_state.torque_mode = TorqueControlMode::kUnknown;
	// omitted: _cached_state.drive_command_was_position_control
	// omitted: _cached_state.previous_velocity
	// these should be retained across resets

	if (is_connected()) {
		write_clear_status_register();
		disconnect();
		connect(_motor_id);
	}
}

double Herkulex::read_voltage(void)
{
	double voltage = 0.0;
	if (driver_status().ok()) {
		voltage = _model.voltage_raw_to_V(read_register(HerkulexRegister::kRamVoltage));
	}
	return voltage;
}

bool Herkulex::is_on(void)
{
	if (driver_status().ok()) {
		read_status_register();
		return _cached_state.motor_event.torque_on();
	}
	return false;
}

bool Herkulex::is_moving(void)
{
	if (driver_status().ok()) {
		read_status_register();
		return _cached_state.motor_event.moving();
	}
	return false;
}

bool Herkulex::is_in_position(void)
{
	if (driver_status().ok()) {
		read_status_register();
		return _cached_state.motor_event.in_position();
	}
	return false;
}

bool Herkulex::is_stuck(void)
{
	bool is_stuck = false;

	if (driver_status().ok()) {
		read_status_register();
		if (!_cached_state.motor_event.torque_on())
			return false;
		else if (_cached_state.drive_command_was_position_control)
			is_stuck = !_cached_state.motor_event.moving() && !_cached_state.motor_event.in_position();
		else
			is_stuck = !_cached_state.motor_event.moving() && _cached_state.previous_velocity != 0.0;
	}

	return is_stuck;
}

void Herkulex::write_register(const HerkulexRegister &reg, uint16_t value)
{
	if (driver_status().ok()) {
		if (value >= reg.range.min && value <= reg.range.max) {
			HerkulexCommand command = HerkulexCommand::RAM_WRITE;
			std::deque<uint8_t> payload;

			// command
			if (reg.location == HerkulexRegister::MemoryLocation::kNonVolatile) {
				command = HerkulexCommand::EEP_WRITE;
			} else {
				command = HerkulexCommand::RAM_WRITE;
			}

			// payload: address (1), len (1), data(len)
			payload.push_back(reg.address);
			payload.push_back(reg.bytes);
			// byte order: little endian
			payload.push_back(static_cast<uint8_t>(value & 0xFF));
			if (reg.bytes == 2) {
				payload.push_back(static_cast<uint8_t>((value >> 0x08) & 0xFF));
			}

			write_command(command, payload);
		} else {
			std::string detail = std::to_string(value) + " outside of range [" + std::to_string(reg.range.min) + " ," +
								 std::to_string(reg.range.max) + "]";
			_status.raise_warning(HerkulexStatus::Code::kInvalidRegisterValue, detail);
		}
	}
}

uint16_t Herkulex::read_register(const HerkulexRegister &reg)
{
	uint16_t value			   = 0x00;
	uint32_t transmit_attempts = 0;
	while (driver_status().ok() && transmit_attempts < _bus->get_transmit_attempts()) {
		value					= 0x00;
		HerkulexCommand command = HerkulexCommand::RAM_READ;
		std::deque<uint8_t> payload;

		// command
		if (reg.location == HerkulexRegister::MemoryLocation::kNonVolatile) {
			command = HerkulexCommand::EEP_READ;
		} else {
			command = HerkulexCommand::RAM_READ;
		}

		// payload: address (1), length (1)
		payload.push_back(reg.address);
		payload.push_back(reg.bytes);

		// transmit packet
		HerkulexPacket response = write_command(command, payload);

		// read response
		if (driver_status().ok()) {
			transmit_attempts += response.get_transmission_attempts();

			// verify correct response
			// payload = addr (1) + length(1) + [data]
			payload = response.get_payload();
			if (payload.size() == static_cast<uint8_t>(reg.bytes + 2) && payload.at(0) == reg.address &&
				payload.at(1) == reg.bytes) {
				// pop address and size
				payload.pop_front();
				payload.pop_front();

				// byte order: little endian
				value = payload.at(0);
				if (reg.bytes == 2) {
					value |= payload.at(1) << 0x08;
				}
			}
			// valid, well-formed packet but incorrect payload (either
			// not the correct number of bytes, or register address returned
			// did not match the register requested).
			else {
				std::string detail = "Register read returned malformed address or size.";
				// can retransmit?
				if (transmit_attempts < _bus->get_transmit_attempts()) {
					_status.raise_warning(HerkulexStatus::Code::kPacketPayloadOutOfRange,
										  detail + " Command will be retried.");
				} else {
					_status.raise_error(HerkulexStatus::Code::kPacketPayloadOutOfRange,
										detail + " Maximum retransmit attempts reached.");
				}
				value = 0x00;
			}
		}
	}

	return value;
}

void Herkulex::write_software_limits(bool limits_on, double pos_limit, double neg_limit)
{
	uint16_t max_pos_raw = _model.position_max_raw;
	uint16_t min_pos_raw = _model.position_min_raw;

	if (limits_on) {
		if (neg_limit >= pos_limit) {
			_status.raise_warning(HerkulexStatus::Code::kMotorInvalidSoftwareLimits,
								  "Positive software limit (" + std::to_string(pos_limit) +
									  ") must be greater than negative limit (" + std::to_string(neg_limit) + ")");
		} else {
      // Convert positions to raw values
			pos_limit += _zero_offset_deg;
			pos_limit *= _gear_ratio;
			neg_limit += _zero_offset_deg;
			neg_limit *= _gear_ratio;

			max_pos_raw = _model.deg_to_jog(pos_limit);
			min_pos_raw = _model.deg_to_jog(neg_limit);
		}
	}

	write_register(HerkulexRegister::kRamMaxPosition, max_pos_raw);
	write_register(HerkulexRegister::kRamMinPosition, min_pos_raw);
}

double Herkulex::read_positive_software_limit_deg(void)
{
	double pos_limit_deg = 0.0;

	if (driver_status().ok()) {
		pos_limit_deg = _model.raw_to_deg(read_register(HerkulexRegister::kRamMaxPosition));

		// account for gear train and zero offset
		pos_limit_deg /= _gear_ratio;
		pos_limit_deg -= _zero_offset_deg;
	}

	return pos_limit_deg;
}

double Herkulex::read_negative_software_limit_deg(void)
{
	double neg_limit_deg = 0.0;

	if (driver_status().ok()) {
		neg_limit_deg = _model.raw_to_deg(read_register(HerkulexRegister::kRamMinPosition));

		// account for gear train and zero offset
		neg_limit_deg /= _gear_ratio;
		neg_limit_deg -= _zero_offset_deg;
	}

	return neg_limit_deg;
}

void Herkulex::write_dead_zone_deg(const double dead_zone_deg)
{
	if (driver_status().ok()) {
		if (dead_zone_deg >= 0.0) {
			uint16_t dead_zone_raw = static_cast<uint16_t>(dead_zone_deg * _gear_ratio / _model.scale_raw_to_deg);
			write_register(HerkulexRegister::kRamDeadZone, dead_zone_raw);
		} else {
			_status.raise_warning(HerkulexStatus::Code::kMotorInvalidDeadZone);
		}
	}
}

void Herkulex::write_dead_zone_default(void)
{
	write_dead_zone_deg(_model.dead_zone_degree_default);
}

double Herkulex::read_dead_zone_deg(void)
{
	double dead_zone_deg = 0.0;
	if (driver_status().ok()) {
		dead_zone_deg = _model.scale_raw_to_deg * read_register(HerkulexRegister::kRamDeadZone);
		dead_zone_deg /= _gear_ratio;
	}
	return dead_zone_deg;
}

void Herkulex::clear_deadband_error(void)
{
	if (driver_status().ok()) {
		// Clear position errors - these are ignored by this driver.
		if (_cached_state.motor_event.position_limit()) {
			uint8_t raw_status_register = _cached_state.motor_event.get_raw_status();
			raw_status_register ^= static_cast<uint8_t>(HerkulexStatusRegister::StatusFieldMask::kExceedPositionLimit);
			write_register(HerkulexRegister::kRamStatusError, raw_status_register);
			// led state will be off after resetting this error
			write_leds(torque_mode_to_leds());
		}
	}
}

void Herkulex::write_jog(const uint16_t raw_data, const JogMode mode, const uint32_t playtime_ms)
{
	// "SET" field, 1 byte
	//		 0: stop flag
	//		 1: mode
	//	 2 - 4: LEDs
	// 		 5: JOG invalid (no action)
	//		 6: Disable VOR (velocity override)
	//		 7: reserved = 0
	enum JogSetFieldBit : uint8_t {
		kStop		= 0,
		kMode		= 1,
		kLed		= 2, // three bits
		kInvalid	= 5,
		kDisableVor = 6,
		kReserved2  = 7
	};

	if (driver_status().ok()) {
		std::deque<uint8_t> payload;

		// "JOG" field, 2 bytes, little endien.
		payload.push_back(raw_data & 0xFF);  // LSB
		payload.push_back(raw_data >> 0x08); // MSB

		// "SET" field.
		payload.push_back(static_cast<uint8_t>(false) << JogSetFieldBit::kStop |
						  (static_cast<uint8_t>(mode) << JogSetFieldBit::kMode) |
						  (static_cast<uint8_t>(torque_mode_to_leds()) << JogSetFieldBit::kLed) // 3 bits
						  | (static_cast<uint8_t>(false) << JogSetFieldBit::kInvalid) |
						  (static_cast<uint8_t>(false) << JogSetFieldBit::kDisableVor));

		// ID.
		payload.push_back(_motor_id);

		// raw play time = desired playtime in ms / tick period.
		payload.push_back(_model.time_ms_to_raw(playtime_ms));

		write_torque_control(TorqueControlMode::kOn);
		write_command(HerkulexCommand::I_JOG, payload);

		// When moving, it's possible that a deadband position error may be cleared.
		clear_deadband_error();
	}
}

HerkulexPacket Herkulex::write_command(const HerkulexCommand command, const std::deque<uint8_t> &payload)
{
	HerkulexPacket response;
	// error already occurred?
	if (!driver_status().ok()) {
	}
	// not connected?
	else if (!is_connected()) {
		_status.raise_error(HerkulexStatus::Code::kNotConnected);
	}
	// ready to communicate
	else {
		bool previous_motor_status_ok = _motor_status.ok();
		response = _bus->write_packet(HerkulexPacket(_motor_id, command, payload), _ack_policy, _status);

		// if an acknowledgement is received, update status from motor status registers
		if (response.is_ack()) {
			_cached_state.motor_event.update(response.get_status_register(), _motor_status);

			// torque free on error; don't call torque_release() as it calls write_command() resulting in stack overflow
			if (previous_motor_status_ok && !_motor_status.ok())
				_bus->write_packet(HerkulexPacket(_motor_id, HerkulexCommand::RAM_WRITE,
												 std::deque<uint8_t>{HerkulexRegister::kRamTorqueControl.address,
																	 HerkulexRegister::kRamTorqueControl.bytes,
																	 static_cast<uint8_t>(TorqueControlMode::kFree)}),
								  _ack_policy, _status);
		}
	}

	return response;
}

} // namespace momentum
