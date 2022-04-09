/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Driver for Copley Amplifiers.
 **/

#pragma once

#include <drivers/copley/CopleyBus.hpp>
#include <drivers/copley/CopleyCommandBuilder.hpp>
#include <drivers/copley/CopleyEventRegister.hpp>
#include <drivers/copley/CopleyModel.hpp>
#include <drivers/copley/CopleyParameter.hpp>
#include <drivers/copley/CopleyStatus.hpp>

namespace momentum
{

/// Copley amplifier driver. This is the top-level driver for a Copley amplifier.
class Copley
{
public:
	/// Home methods.
	/// Not all home methods are supported by this driver.
	struct HomeMethod {
		/// Home function. Bits 0-3 and 5 of HomingMethod parameter.
		/// Not all homing functions are supported by this driver.
		enum class Function : CopleyParameter::value_t {
			kThisIsHome   = 0,  ///< Mark the current position as home.
			kMoveToLimit  = 1,  ///< Move to a limit switch and mark final position as home.
			kMoveToHome   = 2,  ///< Move to a homing switch and mark final position as home.
			KHardstop	 = 4,  ///< Home to a hard stop. Move in desired direction until home current limit is reached.
			kHomeAbsolute = 15, ///< Immediate home. This value causes the amp to be referenced immediately on power-up.
								/// Once the encoder is initialized, the home offset value is added to the encoder
								/// position and the result is set as the current referenced position.
			kMoveToIndex = 32,  ///< Move to an index pulse and mark the position as home.
		};

		/// Move direction. Bit 4 of HomingMethod parameter.
		enum class Direction : CopleyParameter::value_t {
			kPositive = 0, ///< Positive direction.
			kNegative = 1, ///< Negative direction.
		};

		// Bit offsets for homing method configuration parameter.
		static constexpr uint8_t kFunctionOffset  = 0; ///< Home Function
		static constexpr uint8_t kDirectionOffset = 4; ///< Direction of initial move
	};

	/// Construct and add to a Logger.
	/// @param [in,out] bus The Copley bus to which this device is connected.
	/// @param [in] node_id The network node ID of the amplifier.
	/// @param [in] hardware The specification of the hardware attatched to this amplifier.
	explicit Copley(const std::shared_ptr<CopleyBus> bus,
					const CopleyHardwareSpecification &hardware,
					const uint8_t node_id,
					const uint8_t axis = 0);

	/// Non-copyable.
	Copley(const Copley &) = delete;

	/// Destructor. Will attempt to stop the attached motor.
	virtual ~Copley(void);

	/// Status of this driver. Does not poll the motor status register. Nonblocking.
	/// @return Status.
	const CopleyStatus &status(void) const;

	/// Boolean status of this driver. Wrapper method. Equivallent to status().ok().
	/// @return Boolean status.
	bool status_ok(void) const;

	/// Status string of this driver. Wrapper method. Equivallent to status().to_string().
	/// @return status string.
	std::string status_string(void) const;

	/// Clear status for motor object
	/// @warning Are you sure you want to do this?
	void clear_status(void);

	/// Clear latched event registers on Copley amplifier
	void clear_latched_registers(void);

	/// Check the load encoder status register.
	/// Raise warning/error in status if necessary.
	/// @note method is currently on supports check if encoder is BiSS encoder type
	void load_encoder_status_check(void);

	/// Set the communication protocol type.
	/// @note Default on object construction is binary.
	/// @param [in] protocol Which protocol to use going forward
	void set_communication_protocol(const CopleyCommand::Protocol protocol);

	/// Connect to the amplifier.
	/// @note The communication bus must be connected before calling this method.
	/// @return True if connect successful.
	virtual bool connect(void);

	/// This amplifier is connected?
	/// @return true if the amplifier is connected, false otherwise.
	bool is_connected(void) const;

	/// Disconnect the amplifier. Disables torque control mode.
	/// @note This is a safe method to call even if an error has occurred.
	virtual void disconnect(void);

	/// Reset driver with status cleared.
	void reset(void);

	/// Is this amplifier directly connected via serial port?
	/// @return true if amplifier is directly connected via serial port.
	bool is_serial_node(void) const;

	/// Network node ID of this amplifier. This is the true network node ID
	/// stored on the device, regardless of whether or not the device is
	/// connected directly over serial.
	/// @return network ID.
	uint8_t read_node_id(void);

	/// Set the network node ID of this amplifier.
	/// @warning This is a blocking method.
	/// @param [in] node_id The network node ID of this amplifier.
	/// @return true if change was successfull
	bool write_node_id(const uint8_t node_id);

	/// Read unique serial number
	/// @return serial number
	uint32_t read_serial_number(void);

	/// Read the current position of the motor, in degrees. Accounts
	/// for zero offset calibration and gear train.
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, 0 is returned.
	/// @return position of output in deg.
	double read_position_deg(void);

	/// Read the position of the load encoder if configured.
	/// This is also passive load position when used in passive mode.
	/// @return Value of load encoder position register.
	double read_load_position_deg(void);

	/// Read the current position loop error of the motor, in degrees. Accounts
	/// for zero offset calibration and gear train.
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, 0 is returned.
	/// @return position loop error in deg.
	double read_position_error_deg(void);

	/// Read the current commanded trajectory position of the motor, in degrees.
	/// The position loop error is the difference between this and the actual
	/// position. Accounts for zero offset calibration and gear train.
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, 0 is returned.
	/// @return commanded position in deg.
	double read_commanded_position_deg(void);

	/// Read the current trajectory destination position of the motor, in degrees.
	/// This is the position the trajectory generator is using as its destination.
	/// Accounts for zero offset calibration and gear train.
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, 0 is returned.
	/// @return destination position in deg.
	double read_trajectory_destination_position_deg(void);

	/// Read back the trajectory position command of the motor, in degrees.
	/// This is the register written by drive_absolute_deg and drive_relative_deg.
	/// Accounts for zero offset calibration and gear train.
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, 0 is returned.
	/// @return trajectory generator command position in deg.
	double read_back_trajectory_generator_position_command_deg(void);

	/// Drive the motor to an absolute goal position, in degrees.
	/// This method accounts for a previously set zero offset and gear train.
	/// @param [in] position_deg The desired goal absolute position, in degrees.
	void drive_absolute_deg(double position_deg);

	/// Drive the motor by a relative amount, in degrees. Driver takes
	/// into account a gear train if any exists.
	/// @param [in] move_deg The desired relative move, in degrees.
	void drive_relative_deg(double move_deg);

	/// Read the current velocity of the motor, in deg per s.
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, 0 is returned.
	/// @return velocity of the output shaft in deg/s.
	double read_velocity_deg_s(void);

	/// Drive the motor in a constant velocity.
	/// @param [in] velocity_deg_s The desired velocity of output, in deg/s. Must not exceed motor specifications.
	void drive_velocity_deg_s(double velocity_deg_s);

	/// Start a homing algorithm.
	/// @note Homing will start motor motion.
	/// @param [in] function The homing function to execute.
	/// @param [in] direction The direction to move.
	/// @param [in] load_offset_deg After homing, the zero position is equal to home position plus load_offset_deg
	/// @note load_offset_deg refers to load/output shaft degrees & takes gear ratio into account
	void home_start(const HomeMethod::Function function,
					const HomeMethod::Direction direction = HomeMethod::Direction::kPositive,
					const double load_offset_deg		  = 0.0);

	/// Set current position as home.
	void home_set_here(const double load_offset_deg = 0.0);

	/// Is homing algorithm running?
	/// @return true if homing algorithm is running.
	bool home_is_running(void);

	/// Method to start search for middle of homing area when the homing switch is active over
	/// a range. The method will end by moving the motor to the middle of the active homing area.
	/// @note Homing window operation will potentially move output 3 complete cycles.
	/// @note Motor may reverse direction to move to new home position
	/// @note home_window_complete() needs to be polled until it returns true
	/// @param [in] direction Direction to run motor while finding the homing switch
	void home_window_start(HomeMethod::Direction direction = HomeMethod::Direction::kPositive);

	/// Method to cancel search for middle of homing area and stop the motor.
	/// @note Motor will abort any active trajectory and torque free.
	void home_window_cancel(void);

	/// Checks current state of the homing window algorithm.
	/// @param [out] window_size_cts Size of slice with homing switch active (in degrees).
	/// @return true if homing window has been found and new position set as home.
	bool home_window_complete(double &home_size_deg);

	/// Set the current limit for homing hardstop operations.
	/// Hard stop home is reached when the amplifier outputs the homing
	/// Current Limit continuously for the time specified in the Delay Time.
	/// @param [in] current_limit_A Current limit (amps)
	/// @param [in] delay_time_ms Delay time (milli-seconds)
	void home_hardstop_current_limit_write(double current_limit_A, uint16_t delay_time_ms);

	/// Aborts any active trajectory.
	/// Use for safety stop.
	void torque_free(void);

	/// Read if the STO circuit is active (drive disabled).
	/// @param [in] false to read STO 0, true to read STO 1.
	/// @return false if STO inactive, true if STO active
	bool is_sto_active(bool sto_1);

	/// Is the motor moving?
	/// @warning Always check status() after this method to ensure
	///	value returned is accurate. If an error has occurred, false is returned.
	/// @return true if the motor is moving.
	bool is_moving(void);

	/// Read if the brake is active.
	/// @return true if the brake is active.
	bool brake_is_active(void);

	/// Read if the positive limit switch is active.
	/// @return true if the switch is active.
	bool limit_pos_is_active(void);

	/// Read if the negative limit switch is active.
	/// @return true if the switch is active.
	bool limit_neg_is_active(void);

	/// Read if the home switch is active.
	/// @return true if the switch is active.
	bool home_switch_is_active(void);

	/// Read if the positive software limit is active.
	/// @return true if positive software limit condition is active
	bool software_limit_pos_is_active(void);

	/// Read if the negative software limit is active.
	/// @return true if negative software limit condition is active
	bool software_limit_neg_is_active(void);

	/// Read motor event register. Updates _motor_status.
	/// @note If an error is detected, the amplifier will be disabled.
	/// @return event register.
	CopleyEventRegister read_event_register(void);

	/// Read the raw value of the event register.
	/// @note If an error is detected, the amplifier will be disabled.
	/// @return event register.
	int read_event_register_raw(void);

	/// Reads the voltage of the high-voltage bus.
	/// @return Voltage, in V.
	double read_voltage_V(void);

	/// Reads the motor current.
	/// @return Current, in A.
	double read_current_A(void);

	/// Reads the motor torque after gearing. Calculated from motor current.
	/// @return Torque after gearing, in Nm.
	double read_torque_Nm(void);

	/// Reads the motor mechanical power after gearing. Calculated from torque and velocity.
	/// @return Mechanical power after gearing, in W.
	double read_mechanical_power_W(void);

	/// Reads the temperature of the drive.
	/// @return temperature, in C.
	double read_temperature_C(void);

	/// Set current control loop gains.
	/// @param [in] Cp The proportional gain.
	/// @param [in] Ci The integral gain.
	/// @param [in] offset_A The current loop offset, in A. Defaults to 0.
	void write_current_gains(const double Cp, const double Ci, const double offset_A = 0.0);

	/// Set default control loop gains.
	void write_current_gains_defaults(void);

	/// Set continuous and peak current limits, in A.
	/// @param [in] continuous_A The maximum continuous current limit, in A.
	/// @param [in] peak_A The maximum peak current limit, in A.
	/// @param [in] I2t_ms The I^2*t time window, in ms.
	void write_current_limits_A(const double continuous_A, const double peak_A, const double I2t_ms);

	/// Set continuous and peak current limit defaults.
	void write_current_limits_defaults(void);

	/// Set velocity control loop gains.
	/// @param [in] Vp The proportional gain.
	/// @param [in] Vi The integral gain.
	/// @param [in] Vi_drain The integral drain. Defaults to 0.0.
	void write_velocity_gains(const double Vp, const double Vi, const double Vi_drain = 0.0);

	/// Read the velocity gains currently set
	/// @return String of velocity gains (Vp, Vi, and Vi_drain)
	std::string read_velocity_gains(void);

	/// Set velocity control loop gains defaults.
	void write_velocity_gains_defaults(void);

	/// Set the velocity loop shift.
	/// After the velocity loop is calculated, the result is right
	/// shifted this many times to arrive at the commanded current value.
	void write_velocity_loop_shift(const int16_t shift);

	/// Set the velocity loop shift to default value.
	void write_velocity_loop_shift_defaults(void);

	/// Set velocity control loop limits, in deg.
	/// @note Limits reflect motion after gearing.
	/// @param [in] velocity_deg_s Velocity limit, in deg/s.
	/// @param [in] accel_deg_s2 Acceleration limit, in deg/s^2.
	void write_velocity_limits_deg(const double velocity_deg_s, const double accel_deg_s2);

	/// Set velocity control loop limits defaults.
	void write_velocity_limits_defaults(void);

	/// Set velocity control loop window.
	/// @note Limits reflect motion before gearing.
	/// @param [in] tracking_window_deg_s If velocity error exceeds this value, a warning is generated, in deg/s.
	/// @param [in] tracking_window_time_ms Duration velocity error must be within window to clear a previous window
	/// warning, in ms.
	void write_ungeared_velocity_window_deg_s(const double tracking_window_deg_s, const double tracking_window_time_ms);

	/// Set velocity control loop window.
	/// @note Limits reflect motion after gearing.
	/// @param [in] tracking_window_deg_s If velocity error exceeds this value, a warning is generated, in deg/s.
	/// @param [in] tracking_window_time_ms Duration velocity error must be within window to clear a previous window
	/// warning, in ms.
	void write_velocity_window_deg_s(const double tracking_window_deg_s, const double tracking_window_time_ms);

	/// Read the velocity control loop window settings.
	/// Prints values to the console.
	/// @return string representation of velocity window.
	std::string read_velocity_window_deg_s(void);

	/// Set velocity control loop tracking window defaults.
	void write_velocity_window_defaults(void);

	/// Read the value of the velocity loop error register.
	double read_velocity_loop_error_deg_s(void);

	/// Set position control loop gains.
	/// @param [in] Pp The proportional gain.
	/// @param [in] Vff_percent The velocity feed-forward gain, in percent. Maximum value is 200%.
	/// @param [in] Aff The acceleration feed-forward gain.
	/// @param [in] gains_multiplier_percent The gains multiplier, in percent. Applied to position control loop output.
	/// Default is 100%. May be larger than 100%.
	void write_position_gains(const double Pp,
							  const double Vff_percent,
							  const double Aff,
							  const double gains_multiplier_percent = 100.0);

	/// Set position control loop gains defaults.
	void write_position_gains_defaults(void);

	/// Read all gains associated with position control
	/// @return string displaying all position control parameters
	std::string read_position_gains(void);

	/// Set position tracking warning and error limits for motor shaft; accepts position values for motor shaft in
	/// degrees.
	/// @param [in] position_warning_deg If position error exceeds this value, a tracking warning is raised.
	/// @param [in] position_error_deg If position error exceeds this value, a tracking error is raised.
	void write_ungeared_position_tracking_limits_deg(const double position_warning_deg,
													 const double position_error_deg);

	/// Set position tracking warning and error limits for load shaft; accepts position values for load shaft in
	/// degrees.
	/// @param [in] position_warning_deg If position error exceeds this value, a tracking warning is raised.
	/// @param [in] position_error_deg If position error exceeds this value, a tracking error is raised.
	void write_position_tracking_limits_deg(const double position_warning_deg, const double position_error_deg);

	/// Set position tracking warning and error limit defaults.
	void write_position_tracking_limits_defaults(void);

	/// Read current values of position tracking warning and error limits
	/// @return String with all position control tracking values
	std::string read_position_tracking_limits_deg(void);

	/// Set trajectory planning limits, in deg.
	/// @note Limits reflect motion after gearing.
	/// @param [in] velocity_deg_s The maximum velocity, in deg/s.
	/// @param [in] accel_deg_s2 The maximum acceleration & deceleration, in deg/s^2.
	void write_trajectory_limits_deg(const double velocity_deg_s, const double accel_deg_s2);

	/// Set trajectory planning limit defaults.
	void write_trajectory_limits_defaults(void);

	/// Set trajectory motion limits for homing, in degrees.
	/// @note Limits reflect moion after gearing
	/// @param [in] fast_velocity_rpm The velocity (deg/sec) used to find limit or home switch
	/// 			also used when moving to an offset position, or a resolcer or Servo Tube index position.
	/// @param [in] slow_velocity_rpm The velocity (deg/sec) used to find switch edge, incremental/analog
	/// 			encoder	index pulse, or hard stop.
	/// @param [in] accel_rps2 The acceleration & deceleration used during homing, in deg/s^2.
	void write_home_trajectory_limits_deg(const double fast_velocity_deg_s,
										  const double slow_velocity_deg_s,
										  const double accel_deg_s2);

	/// Set the default motion limits for homing.
	void write_home_trajectory_defaults(void);

	/// Write an input pin configuration.
	/// @param [in] pin The pin index to write. Must be in [0,7].
	/// @param [in] mode The pin mode.
	/// @param [in] enable_pullup If true, enables the pull up for this pin.
	/// @param [in] location The memory location to write. Defaults to volatile.
	void write_input_pin_config(const uint8_t pin,
								const CopleyHardwareSpecification::InputPin mode,
								bool enable_pullup						 = false,
								CopleyParameter::MemoryLocation location = CopleyParameter::MemoryLocation::kVolatile);

	/// Checks if the input pin index is valid and raises an error if not.
	/// @param [in] index The pin index to check.
	/// @return True if the input pin index is in [0, 7]
	bool is_input_index_valid(uint16_t index);

	/// Read the value of a digital input.
	/// @param [in] index The pin index to read. Must be in [0,2].
	/// @return True if the specified input pin is high.
	bool input_is_high(uint8_t index);

	/// Read all of the digital inputs.
	/// @return The value of the digital inputs as an 8 bit integer.
	uint8_t input_read_raw(void);

	/// Write an input pin configuration.
	/// @param [in] pin The pin index to write. Must be in [0,2].
	/// @param [in] config The pin mode.
	/// @param [in] location The memory location to write. Defaults to volatile.
	void write_output_pin_config(const uint8_t pin,
								 const CopleyHardwareSpecification::OutputPin mode,
								 CopleyParameter::MemoryLocation location = CopleyParameter::MemoryLocation::kVolatile);

	/// Checks if the output pin index is valid and raises an error if not.
	/// @param [in] index The pin index to check.
	/// @return True if the optput pin index is in [0, 2]
	bool is_output_index_valid(uint16_t index);

	/// Read the value of a digital output.
	/// @param [in] index The pin index to read. Must be in [0,2].
	/// @return True if the specified output pin is high.
	bool output_is_high(uint8_t index);

	/// Set the value of a digital output high.
	/// @param [in] index The pin index to write. Must be in [0,2].
	void output_set_high(uint8_t index);

	/// Set the value of a digital output low.
	/// @param [in] index The pin index to write. Must be in [0,2].
	void output_set_low(uint8_t index);

	/// Set all of the digital outputs to a bit pattern.
	/// @param [in] bitmask the lower three bits of this value will be set to outputs 0-2.
	void output_set_raw(uint8_t bitmask);

	/// Read all of the digital outputs.
	/// @return The value of the digital outputs as an 8 bit integer.
	uint8_t output_read_raw(void);

	/// Use Copley encoder command to reset/clear faults from encoder
	/// @param motor_encoder True if encoder is configure as motor encoder, false for load encoder
	void encoder_reset(bool motor_encoder);

	/// Zero the internal position of attached encoder.
	/// @note Only the binary communication protocol supports this feature
	/// @param motor_encoder True if encoder is configure as motor encoder, false for load encoder
	void encoder_zero(bool motor_encoder);

	/// Set the value of a program register on attached encoder
	/// @note Only the binary communication protocol supports this feature
	/// @param reg Register to write
	/// @param value Value to write to register
	/// @param motor_encoder True if encoder is configure as motor encoder, false for load encoder
	void encoder_set_register(const uint16_t reg, const uint16_t value, bool motor_encoder);

	/// Read value of program register on attached encoder
	/// @note Only the binary communication protocol supports this feature
	/// @param reg Register to read
	/// @param motor_encoder True if encoder is configure as motor encoder, false for load encoder
	/// @return Value of register returned from encoder
	uint16_t encoder_read_register(const uint16_t reg, bool motor_encoder);

	/// Set the positive software limit.
	/// @note Software limits are only in effect after homing has completed
	/// @param [in] pos_limit_deg Desired position, in degrees, to set as the
	/// positive software enforced limit
	void write_pos_software_limit(const double pos_limit_deg);

	/// Read the positive software limit.
	/// @note Software limits are only in effect after homing has completed
	/// @return The current positive software limit, in degrees
	double read_pos_software_limit(void);

	/// Set the negative software limit.
	/// @note Software limits are only in effect after homing has completed
	/// @param [in] neg_limit_deg Desired position, in degrees, to set as the
	/// negative software enforced limit
	void write_neg_software_limit(const double neg_limit_deg);

	/// Read the negative software limit.
	/// @note Software limits are only in effect after homing has completed
	/// @return The current negative software limit, in degrees
	double read_neg_software_limit(void);

	/// Disables the software limits by setting positive/negative limits to
	/// default values of zero.
	void write_disable_software_limits(void);

	/// Set the deceleration rate for Copley when approaching software limit
	/// @param [in] deceleration_deg_s2 The deceleration rate in deg/s^2
	void write_software_limit_deceleration_deg_s2(const double deceleration_deg_s2);

	/// Get the software limit deceleration rate currently on the Copley.
	/// @return The deceleration rate (deg/s^2) when motor approaches a software limit
	double read_software_limit_deceleration_deg_s2(void);

	/// Write new velocity loop output filter values to Copley using filter struct
	/// @param [in] filter Velocity output filter values to write
	void write_velocity_output_filter(const CopleyOutputFilter filter);

	/// Write default velocity loop output filter values to Copley using filter struct
	void write_velocity_ouput_filter_default(void);

	/// Configure the position capture register for home switch capture
	/// @note Assumes your capturing with the home switch
	/// @param [in] activeRise True if you want to capture switch inactive to switch active transition
	/// @param [in] overwrite  True if you want positions to be overwritten if new transition before read
	void write_position_capture_configuration(bool activeRise, bool overwrite);

	/// Change the wiring configuration on the amplifier for a motor
	/// @param [in] motor_wiring Desired motor wiring configuration
	/// @param [in] hall_wiring Desired hall sensor wiring configuration
	/// @param [in] hall_inversion Desired hall inversion configuration to write
	/// @param [in] hall_offset_deg Hall sensor offset (degrees)
	void write_wiring_and_hall_cofiguration(const CopleyMotor::Wiring motor_wiring,
											const CopleyMotor::HallWiring hall_wiring,
											const CopleyMotor::HallInversion hall_inversion,
											const int32_t hall_offset_deg = 0);

	/// Check if the position capture is ready to be read
	/// @return True if position capture is ready
	bool position_capture_ready(void);

	/// Read the position capture position
	/// @return Position read from the position capture register
	double read_position_capture_deg(void);

	/// Estimates travel time for a trapezoidal motor move
	/// @param [in] distance_deg Distance to travel (degrees)
	/// @param [in] max_velocity_deg_s Trajectory velocity limit (deg/s)
	/// @param [in]acceleration_deg_s2 Trajectory acceleration (deg/s^2)
	/// @return Calculated time of travel in ms
	static int trapezoidal_move_calculate_time_ms(const double distance_deg,
												  const double max_velocity_deg_s,
												  const double acceleration_deg_s2);

	/// Read the value from a amplifier register with integer type
	/// @param [in] Integer identifier for register addres to read
	/// @param [in] True to read from volatile memory, false for flash
	/// @return Value read from register (if able)
	int32_t read_parameter(int parameter_id, bool is_volatile);

	/// Amount of time to wait before reconnecting to a amplifier after it has been reset, in ms.
	/// This value determined empirically.
	/// Serial nodes reset quickly, but downtream nodes need longer to complete the reset process.
	static constexpr uint32_t kSerialNodeResetDelayMs	 = 1500;
	static constexpr uint32_t kDownstreamNodeResetDelayMs = 2500;

	/// Largest addressible node id.
	static constexpr uint8_t kNodeIdMax = 127;

protected:
	/// Weakly connect to an amplifier without configuring. Use only to establish communication.
	/// This method is protected to allow a scanner object to weakly connect; otherwise
	/// all application code should use the public connect() method.
	/// @param [in] node_id The network node ID.
	/// @return true if communication established.
	bool connect_weak(const uint8_t node_id);

	/// Weakly disconnect from an amplifier without issuing any "safe state" commands.
	/// Use only to terminate communication. This method is protected to allow a scanner
	/// object to weakly disconnect; otherwise all application code should use the public connect() method.
	void disconnect_weak(void);

	/// Write a get command.
	/// @param [in] parameter The parameter to read.
	/// @param [in] location The memory location to read. Defaults to RAM.
	/// @return The value read.
	CopleyParameter::value_t
	write_get(const CopleyParameter &parameter,
			  const CopleyParameter::MemoryLocation location = CopleyParameter::MemoryLocation::kVolatile);

	/// Write a set numeric value command.
	/// @param [in] parameter The parameter to write.
	/// @param [in] value The value to write. Caller must verify the value will fit in the parameter.
	/// @param [in] location The memory location to write. Defaults to RAM.
	void write_set(const CopleyParameter &parameter,
				   const CopleyParameter::value_t value,
				   const CopleyParameter::MemoryLocation location = CopleyParameter::MemoryLocation::kVolatile);

	/// Write a set string value command.
	/// @param [in] parameter The parameter to write.
	/// @param [in] value The value to write. Caller must verify the value will fit in the parameter.
	/// @param [in] location The memory location to write. Defaults to RAM.
	virtual void write_set(const CopleyParameter &parameter,
						   const std::string &value,
						   const CopleyParameter::MemoryLocation location = CopleyParameter::MemoryLocation::kVolatile);

	/// Write a list of numeric values. Required for the few parameters
	/// that are multi-word. (e.g. Velocity Loop Output Filter)
	/// @param [in] parameter The parameter to write.
	/// @param [in] values The list of words to write to amplifier.
	/// @param [in] location The memory location to write. Defaults to RAM.
	void write_set(const CopleyParameter &parameter,
				   const std::vector<int16_t> values,
				   const CopleyParameter::MemoryLocation location = CopleyParameter::MemoryLocation::kVolatile);

private:
	/// Amplifier drive modes.
	/// Only a subset of drive modes are supported here.
	/// See Copley Parameter Dictionary pg. 14 0x24.
	enum class DriveMode : CopleyParameter::value_t {
		kDisabled = 0,		///< Drive disabled.
		kVelocity = 11,		///< The velocity loop is driven by the programmed velocity value.
		kPosition = 21,		///< The position loop is driven by the trajectory generator.
		kUnknown  = 0xFFFF, ///< Unknown drive mode.
	};

	/// Trajectory command types.
	/// See Copley ASCII Programmer's Guide page 14.
	enum class TrajectoryCommand : CopleyParameter::value_t {
		kAbort		= 0x0000, ///< Abort the move in progress.
		kInitUpdate = 0x0001, ///< Initiate or update a move.
		kHome		= 0x0002, ///< Initiate home sequence.
	};

	/// Trajectory move type - absolute or relative.
	enum class TrajectoryMove : CopleyParameter::value_t {
		kAbsolute = 0x000, ///< Absolute move.
		kRelative = 0x100, ///< Relative move.
	};

	/// Trajectory profile type.
	/// Only trapezoidal is supported.
	enum class TrajectoryProfile : CopleyParameter::value_t {
		kTrapezoidal = 0x00, ///< Trapezoidal profile.
		// omitted: S-Curve
		kVelocityMode = 0x02, ///< Velocity control profile.
	};

	/// Trajectory status register.
	enum class TrajectoryStatusRegister : CopleyParameter::value_t {
		// unused: bit 9, CAM table underflow
		kHomingError = 1 << 11, ///< Homing error.
		kReferenced  = 1 << 12, ///< Homing successful, position referenced.
		kHoming		 = 1 << 13, ///< Currently homing.
		kAborted	 = 1 << 14, ///< Move aborted.
		kMoving		 = 1 << 15, ///< Drive is currently moving.
	};

	/// Struct to group relevant interfaces to the parameters
	/// used for position capture function on the Copley
	struct PositionCapture {
		/// Struct for Position Capture Control register.
		/// @note not all functionality of the position capture is exposed for user.
		struct Control {
			/// Select edge of home switch to capture position.
			enum class HomeCaptureEdge : CopleyParameter::value_t {
				kActiveRise = 0, ///< Capture on inactive to active edge.
				kActiveFall = 1, ///< Capture on active to inactive edge.
			};

			/// Overwrite new position captures until it has been read
			enum class HomeCaptureRewrite : CopleyParameter::value_t {
				kOverwrite = 0, ///< Overwrite new values.
				kKeep	  = 1, ///< Keep capture until read.
			};

			// Bit offsets for capture control register
			/// Home capture edge offset in the capture control parameter.
			static constexpr uint8_t kHomeCaptureEdgeOffset = 5;
			/// Rewrite home offset in the capture control parameter.
			static constexpr uint8_t kRewriteHomeOffset = 6;
		};

		/// Struct for Position Capture Status register.
		/// @note not all functionality of the status is exposed for user.
		struct Status {
			/// Has a home position been captured and not read yet
			enum class HomeCaptured : CopleyParameter::value_t {
				kWaiting = 0, ///< Last captured home position already read
				kReady   = 1, ///< Unread captured home position ready
			};

			/// New home switch transition captured when an unread captured home
			/// position is currently stored.
			/// @note New value may or may not be written depending on control register configuration
			enum class MultipleHomeCaptures : CopleyParameter::value_t {
				kNoExtraTransition = 0, ///< No extra home transition has occurred
				kExtraTransition   = 1, ///< New transition when one already stored
			};

			// Bit offsets for position capture status register.
			/// Home capture offset for position capture status parameter.
			static constexpr uint8_t kHomeCapturedOffset = 4;
			/// Multiple home capture offset for position capture status parameter.
			static constexpr uint8_t kMultipleHomeCaptureOffset = 5;
		};
	};

	/// Structure to group all relevant information used
	/// during the homing window search operation.
	struct HomingWindow {
		/// State of homing algorithm during the homing window operation
		enum class Mode {
			kIdle,
			kFirstEdgeSearch,
			kSecondEdgeSearch,
			kMovingToHome,
		};

		/// Saved state for homing window algorithm
		Mode mode = Mode::kIdle;

		/// Direction for homing window direction
		HomeMethod::Direction direction = HomeMethod::Direction::kPositive;

		/// Encoder positions for first and second homing switch transitions found during homing window search
		std::pair<int32_t, int32_t> edges = {0, 0};

		/// During search for home window, the motor shaft has to move 360 degrees
		static constexpr double kHomeWindowCyle = 360.0;
	};

	/// Cached state of the amplifier. Use to detect changes in
	/// state and to reduce redundant configuration messages.
	/// For example, caching the drive mode allows drive mode
	/// change messages to be sent only when the drive mode
	/// is changing from its current state.
	/// @note This should be kept as small as possible in
	/// deference to frequent polling of the amplifier.
	struct CachedState {
		/// Last known drive mode.
		DriveMode drive_mode = DriveMode::kUnknown;
		/// Last known motor status.
		CopleyEventRegister motor_event;
		/// Homing window results.
		HomingWindow homing_window;

		/// Constructor
		CachedState() : drive_mode(DriveMode::kUnknown), motor_event(), homing_window(){};
	};

	/// Status of just this driver and its bus, without querying the amplifier itself.
	/// @return Driver status.
	const CopleyStatus &driver_status(void) const;

	/// Method for homing window search. Sets the position
	/// capture configuration and starts moving motor output 360 degrees.
	/// to find desired homing switch edge
	void home_window_cycle(void);

	/// Reset all cached values to default or unknown states.
	/// Use when resetting the amplifier.
	void clear_cached_state(void);

	/// Sets the drive mode of the amplifier.
	/// @param [in] mode The desired drive mode of the amplifier.
	/// /// @param [in] location The memory location to write. Defaults to volatile.
	void write_drive_mode(const DriveMode mode,
						  CopleyParameter::MemoryLocation location = CopleyParameter::MemoryLocation::kVolatile);

	/// Sets the trajectory of the motor.
	/// @param [in] trajectory The trajectory command.
	void write_trajectory(const TrajectoryCommand trajectory);

	/// Set velocity control loop limits, in deg.
	/// @note Limits reflect motion before gearing.
	/// @param [in] velocity_deg_s Velocity limit, in deg/s.
	/// @param [in] accel_deg_s2 Acceleration limit, in deg/s^2.
	void write_ungeared_velocity_limits_deg(const double velocity_deg_s, const double accel_deg_s2);

	/// Sets the ungeared (motor shaft) trajectory limits.
	/// @param [in] velocity_deg_s The maximum velocity, in deg/s.
	/// @param [in] accel_deg_s2 The maximum acceleration & deceleration, in deg/s^2.
	void write_ungeared_trajectory_limits_deg(const double velocity_deg_s, const double accel_deg_s2);

	/// Set trajectory (ungeared) motion limits for homing, in degrees.
	/// @param [in] fast_velocity_rpm The velocity (deg/sec) used to find limit or home switch
	/// 			also used when moving to an offset position, or a resolcer or Servo Tube index position.
	/// @param [in] slow_velocity_rpm The velocity (deg/sec) used to find switch edge, incremental/analog
	/// 			encoder	index pulse, or hard stop.
	/// @param [in] accel_rps2 The acceleration & deceleration used during homing, in deg/s^2.
	void write_ungeared_home_trajectory_limits_deg(const double fast_velocity_deg_s,
												   const double slow_velocity_deg_s,
												   const double accel_deg_s2);

	/// Program input pin pull-up resistor
	/// @param [in] pin The pin index to write. Must be in [0,7].
	/// @param [in] enable_pullup If true, enables the pull up for this pin.
	/// @param [in] location The memory location to write. Defaults to volatile.
	void write_input_pin_pullup_enable(
		const uint8_t pin,
		bool enable_pullup,
		CopleyParameter::MemoryLocation location = CopleyParameter::MemoryLocation::kVolatile);

	/// Write the configuration parameters (algorithm, settings, etc.) to the amplifier.
	void write_configuration(void);

	/// Write hardware specification to the amplifier.
	void write_hardware_specification(void);

	/// Write a reset command.
	/// Waits for reset and reconnects.
	/// @warning This is a blocking method.
	void write_reset(void);

	/// Write a command on the bus.
	/// @warning Check the status of this object before using the response.
	/// If an error has occurred, the response may be invalid.
	/// @param [in] command The command to transmit.
	/// @return Response;
	std::unique_ptr<CopleyResponse> write_command(const CopleyCommand &command);

	/// Checks if the system has a motor encoder (pre gearbox).
	/// The motor must either have a motor encoder or use hall feedback for phasing.
	/// @return True if the system has a motor encoder.
	bool has_motor_encoder(void) const;

	/// Checks if the system has a load encoder (post gearbox).
	/// The motor may have a load encoder if it is using hall feedback for phasing.
	/// @return True if the system has a load encoder.
	bool has_load_encoder_for_feedback(void) const;

	/// Does the motor have an encoder (load or primary)
	/// that is used for trajectory feedback?
	bool has_encoder_feedback(void) const;

	/// Convert deg to encoder counts.
	/// Gear ratio is not taken into account.
	/// @param [in] deg Degrees.
	/// @param [in] use_hall_cpr True to use halls counts/rev for scaling
	/// @return counts.
	int32_t deg_to_count(const double deg, const bool use_gear_ratio, const bool use_hall_cpr = false) const;

	/// Convert encoder counts to deg.
	/// Gear ratio is not taken into account.
	/// @param [in] count Encoder counts.
	/// @return deg.
	double count_to_deg(const int32_t count, const bool use_gear_ratio, const bool use_hall_cpr = false) const;

	/// Convert a drive mode to a string representation.
	/// @param [in] mode The drive mode.
	/// @return formatted string.
	static std::string to_string(const DriveMode mode);

	/// Convert a home mode to a string representation.
	/// @param [in] mode The home mode.
	/// @return formatted string.
	static std::string to_string(const HomeMethod::Function mode);

	/// Convert a home window mode to a string representation.
	/// @param [in] mode The home window mode.
	/// @return formatted string.
	static std::string to_string(const HomingWindow::Mode mode);

	/// Convenience method. Checks if a particular bit in a value is set.
	/// @return True if bit is set in value
	static bool bit_set(const uint32_t value, const int8_t bit);

	/// Specification of the hardware connected to this amplifier.
	const CopleyHardwareSpecification _hardware;

	/// Network node ID of this amplifier.
	/// @note Copley overloads this value in the case of serial
	/// communication, where the serial device always responds to ID 0x00
	/// even though it's CAN network ID may differ.
	uint8_t _node_id;

	/// Axis of the motor on the Amplifier.
	/// @note Axis numbers start at 0
	uint8_t _axis;

	/// The amplifier has been initialized and connected.
	bool _is_connected;

	/// Builder object for commands to send via bus.
	std::unique_ptr<CopleyCommandBuilder> _command_builder;

	/// Communication bus.
	std::shared_ptr<CopleyBus> _bus;

	/// Cached state of the amplifier.
	CachedState _cached_state;

	/// Status of this driver.
	CopleyStatus _status;

	/// Status of the motor.
	CopleyStatus _motor_status;
};

} // namespace momentum
