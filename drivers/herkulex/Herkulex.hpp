/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Driver for Herkulex motors.
 **/

#pragma once

#include <drivers/herkulex/HerkulexBus.hpp>
#include <drivers/herkulex/HerkulexModel.hpp>
#include <drivers/herkulex/HerkulexRegister.hpp>
#include <drivers/herkulex/HerkulexStatusRegister.hpp>
#include <drivers/herkulex/HerkulexStatus.hpp>

#include <libs/util/util.hpp>

namespace momentum
{

/// Herkulex motor driver. This is the top-level driver for a Herkulex motor
class Herkulex
{
public:
	/// Torque control mode.
	enum TorqueControlMode : uint8_t {
		kFree = 0x00, ///< Torque released, joints may be manually manipulated.
		kBrake = 0x40, ///< Brake on. Command functions have no effect. This is misspelled "break" in Herkulex documentation.
		kOn		 = 0x60, ///< Torque is enabled. Command functions will control motor.
		kUnknown = 0xFF  ///< Torque mode unknown.
	};

	/// Jog command modes.
	enum class JogMode : uint8_t {
		kPositionControl = 0x00, ///< Position control mode.
		kTurnMode		 = 0x01  ///< Velocity or torque control mode.
	};

	/// On-motor LEDs
	enum Led : uint8_t {
		kNone  = 0x00, ///< No LED.
		kGreen = 0x01, ///< Green LED.
		kBlue  = 0x02, ///< Blue LED.
		kRed   = 0x04  ///< Red LED.
	};

	/// Gains for position control loop
	struct PositionGains {
		uint16_t Kp;	///< Proportional
		uint16_t Kd;	///< Derivative
		uint16_t Ki;	///< Integral
		uint16_t Kpff1; ///< Feedforward term 1
		uint16_t Kpff2; ///< Feedforward term 2
	};

	/// Gains for velocity control loop
	struct VelocityGains {
		uint16_t Kp; ///< Proportional
		uint16_t Ki; ///< Integral
	};

	/// Default constructor.
	/// @param [in,out] bus The Herkulex on which this motor resides.
	explicit Herkulex(const std::shared_ptr<HerkulexBus> bus);

	/// Destructor. Will attempt to stop the motor.
	virtual ~Herkulex(void);

	/// Status of this driver. Does not poll the motor. Nonblocking.
	/// @return Status.
	const HerkulexStatus &status(void) const;

	/// Status string of this driver. Does not poll the motor. Nonblocking.
	/// @return formatted status string.
	std::string status_string(void) const;

	/// Is the status of this driver OK?
	/// @return true if status is OK.
	bool status_ok(void) const;

	/// Clear the status of this driver.
	/// @warning Are you sure it's okay to clear this status?
	void status_clear(void);

	/// Connect to the motor and configure for operation.
	/// @note The communication bus must be connected before calling this method.
	/// @param [in] motor_id The unique ID of this motor.
	/// @return true if connected.
	virtual bool connect(const uint8_t motor_id);

	/// This motor is connected?
	/// @return true if the motor is connected, false otherwise.
	bool is_connected(void) const;

	/// Disconnect the motor. Disables torque control mode.
	/// @note This is a safe method to call even if an error has occurred.
	virtual void disconnect(void);

	/// Read the calibration difference and verify it is set to zero.
	void verify_calibration_difference(void);

	/// ID of this motor.
	/// @return motor ID.
	uint8_t get_motor_id(void) const;

	/// Get the max velocity of the motor from the _model object.
	/// @return the max velocity in deg/s, accounting for any custom gear ratio.
	double get_max_velocity_deg_s(void) const;

	/// Calibrate the zero position of the motor.
	/// The default zero position noted in documentation.
	/// @note Calibration is handled by this driver and is not transmitted to the motor.
	/// @param [in] offset_deg The calibration offset.
	void set_zero_offset_deg(const double offset_deg);

	/// Get the current zero position of the motor.
	/// @note Calibration is handled by this driver and is not a register on the motor.
	/// @return Calibration offset currently used by driver
	double get_zero_offset_deg(void) const;

	/// Set the gear ratio if a gear train exists for control of output. Defaults to 1 : 1
	/// @note Calibration is handled by this driver and is not transmitted to the motor.
	/// @param [in] gear_ratio Gear train ratio as double of output teeth / input teeth
	void set_gear_ratio(const double gear_ratio);

	/// Set the default playtime, in ms.
	/// @param [in] playtime_ms The playtime, in ms.
	void set_default_playtime_ms(const uint32_t playtime_ms);

	/// Get the current position of the motor, in deg.
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, 0 is returned.
	/// @param [in] restrict_range: Flag to determine if reported range should be wrapped to [-180, 180] deg.
	/// @return position in deg.
	double read_position_deg(const bool restrict_range = false);

	/// Get the target position of the motor, in deg.
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, 0 is returned.
	/// @param [in] restrict_range: Flag to determine if reported range should be wrapped to [-180, 180] deg.
	/// @return target position in deg.
	double read_goal_position_deg(const bool restrict_range = false);

	/// Read the absolute 2nd position angle based on reading from a potentiometer.
	/// @warning Not all Herkulex models support this read
	/// @return Result read form absolute 2nd position register
	double read_position_2nd_deg(void);

	/// Drive the motor to a goal position, in degrees.
	/// This method accounts for a previously set zero offset.
	/// Default playtime is used.
	/// @param [in] position_deg The desired goal position (absolute), in degrees.
	void drive_absolute_deg(const double position_deg);

	/// Drive the motor to a goal position, in degrees.
	/// This method accounts for a previously set zero offset.
	/// @param [in] position_deg The desired goal position (absolute), in degrees.
	/// @param [in] velocity_deg_s The velocity, in deg_s
	void drive_absolute_deg_velocity_deg_s(double position_deg, double velocity_deg_s);

	/// Drive the motor to a goal position, in degrees.
	/// This method accounts for a previously set zero offset.
	/// @param [in] position_deg The desired goal position (absolute), in degrees.
	/// @param [in] playtime_ms The play (action) time, in ms.
	void drive_absolute_deg_playtime_ms(double position_deg, uint32_t playtime_ms);

	/// Drive the motor in a constant velocity.
	/// @warning Not all Herkluex models support velocity control.
	/// @param [in] velocity_deg_s The desired velocity, in deg/s.
	void drive_velocity_deg_s(double velocity_deg_s);

	/// Get the current velocity of the motor, in deg/s.
	/// @warning Not all Herkulex models support velocity control.
	/// @return velocity, in deg/s
	double read_velocity_deg_s(void);

	/// Get the current allowed torque limit of the motor, in percent.
	/// @return torque limit, in percent.
	double read_torque_limit_percent(void);

	/// Limits output motor torque allowed by percent [0, 100%] of maximum possible.
	/// @param [in] torque_limit_percent Allowed percentage of maximum torque of motor output
	void write_torque_limit_percent(const double torque_limit_percent);

	/// Get the current torque output of the motor, in percent.
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, 0 is returned.
	/// @return torque, in percent.
	double read_torque_percent(void);

	/// Drive the motor with a constant torque.
	/// @warning Not all Herkulex motors support torque control.
	/// @param [in] torque_percent The desired torque, in percent (0-100).
	void drive_torque_percent(double torque_percent);

	/// Set the motor torque offset (bias).
	/// Use when motor load is exposed to a constant force (such as gravity).
	/// @param [in] torque_offset_percent Torque offset, in percent of maximum output. Range [-12.4,12.4].
	void write_torque_offset_percent(const double torque_offset_percent);

	/// Get the motor torque offset (bias).
	/// @return torque offset, in percent of maximum output.
	double read_torque_offset_percent(void);

	/// Read the motor temperature
	/// @return Temperature (degree Celsius) read from motor
	double read_temperature_C(void);

	/// Read the raw temperature ADC value from motor (Model 101 or 201)
	/// See Herkulex 101/201 User Manual for lookup table
	/// @return ADC value correspoding to a measured temperature.
	int read_temperature_raw(void);

	/// Write a new value to the max temperature limit for motor error
	/// @param [in] temp_c Max temperature (degree Celsius) allowed before motor error
	void write_temperature_limit_C(double temp_C);

	/// Read the current value for max temperature limit of motor
	/// @return Temperature (degree Celsius) setting for maximum temperature error
	double read_temperature_limit_C(void);

	/// Stops the motor. Will attempt to stop regardless of current status.
	/// Motor shaft may not be freely movable.
	void brake(void);

	/// Halts the motor and places in free torque mode. Will attempt to halt regardless of current status.
	/// Use for safe stop.
	void torque_release(void);

	/// Reboots the motor. Resets the absolute position if it is > 360 or  < 0
	/// Blocks for kResetDelayMs milliseconds.
	void reboot(void);

	/// Resets the motor, disconnects it, resets the state of member variables
	/// resets all cached state (motor_event & torque control mode),
	/// and re-connects.
	void reset(void);

	/// Returns the supply voltage of the motor.
	/// @return supply voltage, in V.
	double read_voltage(void);

	/// Is the motor on?
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, false is returned.
	/// @return true if the motor is on.
	bool is_on(void);

	/// Is the motor moving?
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, false is returned.
	/// @return true if the motor is moving.
	bool is_moving(void);

	/// Is the motor in position?
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, false is returned.
	/// @return true if the motor is in the desired position.
	bool is_in_position(void);

	/// Is the motor stuck?
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, false is returned.
	/// @return true if (the last move was a velocity move AND the velocity was
	/// not 0 AND the motor is NOT moving) OR (the last move was a position move
	/// AND the motor is NOT moving AND the motor is NOT in position)
	bool is_stuck(void);

	/// Writes a command to the herkulex 'aux' register to reset the encoder
	/// to one of several values
	/// @param [in] command the command to write (see HerkulexCommand.hpp)
	void write_position_reset(const HerkulexPositionResetCommand command);

	/// Commit a torque control mode to the motor.
	/// @param [in] mode The torque control mode to commit.
	void write_torque_control(const TorqueControlMode mode);

	/// Write position control loop gains using single gains struct
	/// @param [in] gains Struct with all control gains (Kp, Kd, Ki, Kpff1 and Kpff2)
	void write_position_gains(const PositionGains gains);

	/// Write position control loop gains to motor
	/// @param [in] Kp Position proportional gain
	/// @param [in] Kd Position derivative gain
	/// @param [in] Ki Position integral gain
	/// @param [in] Kpff1 Position feedforward 1st gain
	/// @param [in] Kpff2 Position feedforward 2nd gain
	void write_position_gains(
		const uint16_t Kp, const uint16_t Kd, const uint16_t Ki, const uint16_t Kpff1 = 0, const uint16_t Kpff2 = 0);

	/// Write the default position gains onto motor
	void write_default_position_gains(void);

	/// Read the current set of gains for position control loop on motor
	/// @return Struct containing position loop gains Kp, Kd, Ki, Kpff1, and Kpff2 from motor
	PositionGains read_position_gains(void);

	/// Set velocity control loop gains. Method will check if motor model will
	/// allow velocity control.
	/// @param [in] Kp Velocity proportional gain
	/// @param [in] Ki Velocity integral gain
	void write_velocity_gains(const uint16_t Kp, const uint16_t Ki);

	/// Write velocity control gains using single gains struct
	/// @param [in] gains Struct with velocity gains (Kp and Ki)
	void write_velocity_gains(const VelocityGains gains);

	/// Sets the control loop gains to default values based on the motor model.
	void write_default_velocity_gains(void);

	/// Read the current set of gains for velocity control loop on motor
	/// @warning Not all motors support velocity control. If not, this function will retrun struct
	/// contain all zeros.
	/// @return Struct containing velocity loop gains Kp and Ki from motor
	VelocityGains read_velocity_gains(void);

	/// Set the "in position" tolerance for motor.
	/// @param [in] tolerance_deg Degree tolerance desired for "in position" setting.
	void write_position_tolerance_deg(double tolerance_deg);

	/// Set default values of the "in position" tolerance of motor.
	void write_position_tolerance_default(void);

	/// Read the "in position" tolerance.
	/// @return tolerance, in deg.
	double read_position_tolerance_deg(void);

	/// Write parameters used for motor "is_moving" check
	/// @note threshold should be less than your normal in position tolerance
	/// @param [in] threshold_deg Not moving if change is less than threshold (in degrees)
	/// @param [in] period_ms Not moving if stop lasts longer than period (in ms)
	void write_moving_detection_params(const double threshold_deg, const uint16_t period_ms);

	/// Write default values for "is_moving" check parameters on motor
	void write_moving_detection_defaults(void);

	/// Read threshold for "is_moving" check on motor
	/// @return Stop detection threshold (in degrees) read from motor
	double read_moving_theshold_deg(void);

	/// Read period for "is_moving" check on motor
	/// @return Stop detection threshold (in milliseconds) read from motor
	uint16_t read_moving_period_ms(void);

	/// Set the acceleration ratio of the motor. This is the percent
	/// of time during a trajectory move that the path is accelerating.
	/// @example Acceleration ratio of 0% is a square move (very jerky).
	/// @example Acceleration ratio of 50% is triangular.
	/// @example 0% < acceleration ratio < 50% is trapezoidal.
	/// @param [in] acceleration_ratio_percent. Must be between 0% and 50% inclusive.
	void write_acceleration_ratio_percent(const double acceleration_ratio_percent);

	/// Write the acceleration ratio default.
	void write_acceleration_ratio_default(void);

	/// Read the acceleration ratio of the motor trajectory planner.
	/// @return acceleration ratio, in percent. Range is 0-50%.
	double read_acceleration_ratio_percent(void);

	/// Set the acceleration time of the motor. This is the maximum
	/// allowable time allowed for acceleration. Range for this setting
	/// is [0.0 ms - 2800 ms]
	/// @param [in] acceleration_time_ms Max time in ms for acceleration
	void write_acceleration_time_ms(const double acceleration_time_ms);

	/// Write default value for acceleration time register.
	void write_acceleration_time_default(void);

	/// Read the accleration time set to motor
	/// @return Time in ms for the maximum allowable acceleration
	double read_accleration_time_ms(void);

	/// Read difference in target position and current actual position of motor
	/// @return Difference (in degrees) of target and current postition
	double read_position_error_deg(void);

	/// Write a register.
	/// @param [in] reg The register to write.
	/// @param [in] value The value to write to the register. Depending on the
	///		register, 1-2 bytes will be written.
	void write_register(const HerkulexRegister &reg, uint16_t value);

	/// Read a register.
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, 0 is returned.
	/// @param [in] reg The register to read.
	/// @return The value to read from the register. Depending on the register, 1-2 bytes will be read.
	uint16_t read_register(const HerkulexRegister &reg);

	/// Writes the soft limits and turns them on or off
	/// @warning if limits_on is false, the limit values are ignored
	/// @warning Software limit positions will not shift if zero offset is adjusted
	/// @param [in] limits_on whether to turn the limits on or off
	/// @param [in] pos_limit_deg Absolute degree position for positive software limit
	/// @param [in] neg_limit_deg Absolute degree position for negative software limit
	void write_software_limits(bool limits_on, double pos_limit = 0.0, double neg_limit = 0.0);

	/// Read current software positive limit currently set on device
	/// @return Absolute position degree of positive software limit
	double read_positive_software_limit_deg(void);

	/// Read current software negative limit currently set on device
	/// @return Absolute negative degree of positive software limit
	double read_negative_software_limit_deg(void);

	/// Write value to the dead zone register on motor
	/// to set region around goal position where PWM is
	/// always zero.
	/// @param [in] dead_zone_deg Distance from goal (+/-) where PWM is zero
	void write_dead_zone_deg(const double dead_zone_deg);

	/// Write the default value for dead zone to motor
	void write_dead_zone_default(void);

	/// Read value of dead zone register currently on motor
	double read_dead_zone_deg(void);

protected:
	/// Weakly connect to a motor without configuring. Use only to establish communication.
	/// This method is protected to allow a scanner object to weakly connect; otherwise
	/// all application code should use the public connect() method.
	/// @return true if communication established.
	bool connect_weak(const uint8_t motor_id);

	/// Weakly disconnect from a motor without issuing any "safe state" commands.
	/// Use only to terminate communication. This method is protected to allow a scanner
	/// object to weakly disconnect; otherwise all application code should use the public connect() method.
	void disconnect_weak(void);

private:
	/// Status of just this driver, without querying the motor itself.
	/// @return Driver status.
	const HerkulexStatus &driver_status(void) const;

	/// Query the motor for its current acknowledgement policy.
	/// If no acknowledgement is received, HerkulexAckPolicy::kNoReply is returned.
	/// @return acknowledgement policy.
	HerkulexAckPolicy read_ack_policy(void);

	/// Read the status register from the motor.
	/// Updates cached value.
	void read_status_register(void);

	/// Request the model number from the motor.
	/// @return the model number of this motor.
	HerkulexModel read_model_number(void);

	/// Commits the acknowledgement policy to the motor.
	/// @param [in] policy The acknowledgement policy.
	void write_ack_policy(const HerkulexAckPolicy policy);

	/// Clear the internal status registers of the motor.
	void write_clear_status_register(void);

	/// Commit LED states to the motor.
	/// @param [in] leds Bitwise OR of LEDs to turn on; others will remain off.
	void write_leds(const Herkulex::Led leds);

	/// Convert the current torque state to LED state.
	/// @return LED state corresponding to the torque mode.
	Led torque_mode_to_leds(void) const;

	/// Clear position error flag on the motor. This error is ignored by this driver.
	void clear_deadband_error(void);

	/// Write an absolute move command and performs basic parameter checks.
	/// More specific checks are done by specific drive methods (e.g drive_absolute_deg_velocity_deg_s)
	/// @param [in] position_deg The desired goal position (absolute), in degrees.
	/// @param [in] velocity_deg_s The velocity, in deg_s
	void write_absolute_drive(double position_deg, uint32_t playtime_ms);

	/// Write a jog command to the bus.
	/// @param [in] raw_data The raw data to write for position or velocity control.
	/// @param [in] mode Jog mode.
	/// @param [in] playtime_ms The playtime of the motion, in ms.
	void
	write_jog(const uint16_t raw_data, const JogMode mode, const uint32_t playtime_ms = HerkulexModel::playtime_max_ms);

	/// Write a command to the bus. Handles acknowledgement, if any.
	/// @warning Check the status of this object before using the response.
	/// If an error has occurred, the response may be invalid.
	/// @param [in] command The command to send.
	/// @param [in] payload The payload of the command.
	/// @return Acknowledgement or response; used only if ack policy dictates.
	HerkulexPacket write_command(const HerkulexCommand command, const std::deque<uint8_t> &payload = {});

	/// Cached state of the motor. Use to detect changes in
	/// state and to allow reporting of multiple errors.
	/// @note This should be kept as small as possible in
	/// deference to frequent polling of the amplifier.
	struct CachedState {
		/// Last known motor status.
		HerkulexStatusRegister motor_event;
		/// Last known torque mode.
		TorqueControlMode torque_mode;
		/// Was the last move a position move (as opposed to velocity)
		bool drive_command_was_position_control;
		/// The requested velocity of the last velocity move
		double previous_velocity;

		CachedState()
		  : motor_event(),
			torque_mode(TorqueControlMode::kUnknown),
			drive_command_was_position_control(),
			previous_velocity(){};
	};

	/// Amount of time to wait before reconnecting to a motor after it has been reset, in ms.
	/// This value determined empirically.
	static constexpr uint32_t kRebootDelayMs = 500;

	/// Default torque limit (as percent of maximum) at motor start-up
	static constexpr double kDefaultTorqueLimitPercent = 80.0;

	/// The motor has been initialized and connected.
	bool _is_connected;

	/// The calibration offset, in degrees, of the motor.
	/// This adjusts the zero position from that defined by the motor.
	double _zero_offset_deg;

	/// Gear ratio for Herkulex
	double _gear_ratio;

	/// The default play time, in ms.
	uint32_t _default_playtime_ms;

	/// The unique ID of this motor.
	uint8_t _motor_id;

	/// Acknowledgement policy
	HerkulexAckPolicy _ack_policy;

	/// The model of this motor. Set on connect().
	HerkulexModel _model;

	/// Cached state.
	CachedState _cached_state;

	/// Reference to the communication bus.
	std::shared_ptr<HerkulexBus> _bus;

	/// Status of this driver.
	HerkulexStatus _status;

	/// Status of the Herkulex motor.
	HerkulexStatus _motor_status;
};

} // namespace momentum
