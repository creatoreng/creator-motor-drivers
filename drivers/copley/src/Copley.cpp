/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/copley/Copley.hpp>
#include <drivers/copley/CopleyCommandBuilderAscii.hpp>
#include <drivers/copley/CopleyCommandBuilderBinary.hpp>
#include <drivers/copley/CopleyConversion.hpp>
#include <drivers/copley/CopleyResponseAscii.hpp>
#include <drivers/copley/CopleyResponseBinary.hpp>

namespace momentum
{

// put CopleyConversion values into this namespace for convenience
using namespace CopleyConversion;

Copley::Copley(const std::shared_ptr<CopleyBus> bus,
			   const CopleyHardwareSpecification &hardware,
			   const uint8_t node_id,
			   const uint8_t axis)
  : _hardware(hardware),
	_node_id(node_id),
	_axis(axis),
	_is_connected(false),
	_command_builder(nullptr),
	_bus(bus),
	_cached_state(),
	_status(),
	_motor_status()
{
	_cached_state.drive_mode = DriveMode::kUnknown;

	if (node_id > kNodeIdMax) {
		_status.raise_error(CopleyStatus::Code::kPacketInvalidMotorId,
							"Attempted to construct motor object with invalid node ID.");
	}

	// Default to binary. Can be changed at runtime.
	set_communication_protocol(CopleyCommand::Protocol::kBinary);
}

Copley::~Copley(void)
{
	disconnect();
}

const CopleyStatus &Copley::status(void) const
{
	// order: bus -> driver -> motor
	return Status<CopleyContext>::max(_bus->status(), _status, _motor_status);
}

std::string Copley::status_string(void) const
{
	return status().to_string();
}

bool Copley::status_ok(void) const
{
	return status().ok();
}

void Copley::clear_status(void)
{
	_motor_status.clear();
	_status.clear();
}

const CopleyStatus &Copley::driver_status(void) const
{
	return Status<CopleyContext>::max(_bus->status(), _status);
}

void Copley::clear_latched_registers(void)
{
	// clear latched event registers
	write_set(CopleyParameter::kDriveLatchingFaultStatus, 0xFFFFFFFF);
	write_set(CopleyParameter::kDriveLatchedEventStatus, 0xFFFFFFFF);
	write_reset();
	clear_cached_state();
	read_event_register();
}

void Copley::load_encoder_status_check(void)
{
	CopleyParameter::value_t encdr_status = write_get(CopleyParameter::kLoadEncoderStatus);

	// Currently only implementing check for load encoder of BiSS type.
	// See Copley Controls Parameter Dictionay (Register 0x12E) for documentation.
	if (_hardware.load_encoder.type == CopleyLoadEncoder::Type::kBiSS) {
		if (bit_set(encdr_status, 0))
			_motor_status.raise_warning(CopleyStatus::Code::kLoadEncoderStatus, "CRC error on data received.");
		if (bit_set(encdr_status, 1))
			_motor_status.raise_warning(CopleyStatus::Code::kLoadEncoderStatus,
										"Encoder failed to transmit data to drive.");
		if (bit_set(encdr_status, 2))
			_motor_status.raise_warning(CopleyStatus::Code::kLoadEncoderStatus,
										"Error bit on encoder stream is active.");
		if (bit_set(encdr_status, 3))
			_motor_status.raise_warning(CopleyStatus::Code::kLoadEncoderStatus,
										"Warning bit on encoder stream is active.");
		if (bit_set(encdr_status, 4))
			_motor_status.raise_warning(CopleyStatus::Code::kLoadEncoderStatus,
										"Encoder transmission delay is too long.");
	}
}

int32_t Copley::deg_to_count(const double deg, const bool use_gear_ratio, const bool use_hall_cpr) const
{
	// Because all calculations take into account gear ratio and assume the counts are measured at the
	// motor shaft, convert load encoder cpr into motor encoder cpr (and in count_to_deg)
	double motor_cpr = 0.0;

	if (use_hall_cpr)
		motor_cpr = _hardware.motor.pole_pairs * hall_sensor_cpr;
	else if (has_motor_encoder() || _hardware.load_encoder.passive_mode)
		motor_cpr = _hardware.motor_encoder.resolution_cpr;
	else
		motor_cpr = _hardware.load_encoder.resolution_cpr;

	if (use_gear_ratio)
		motor_cpr *= _hardware.gear_ratio;

	return static_cast<int32_t>(motor_cpr * deg / 360.0);
}

double Copley::count_to_deg(const int32_t count, const bool use_gear_ratio, const bool use_hall_cpr) const
{
	// See above comment
	double motor_cpr = 0.0;

	if (use_hall_cpr)
		motor_cpr = _hardware.motor.pole_pairs * hall_sensor_cpr;
	else if (has_motor_encoder() || _hardware.load_encoder.passive_mode)
		motor_cpr = _hardware.motor_encoder.resolution_cpr;
	else
		motor_cpr = _hardware.load_encoder.resolution_cpr;

	if (use_gear_ratio)
		motor_cpr *= _hardware.gear_ratio;

	return 360.0 * static_cast<double>(count) / motor_cpr;
}

bool Copley::has_motor_encoder(void) const
{
	return _hardware.motor_encoder.type != CopleyMotorEncoder::Type::kDigitalHall;
}

bool Copley::has_load_encoder_for_feedback(void) const
{
	return _hardware.load_encoder.type != CopleyLoadEncoder::Type::kNone && !_hardware.load_encoder.passive_mode;
}

bool Copley::has_encoder_feedback(void) const
{
	return has_motor_encoder() || has_load_encoder_for_feedback();
}

void Copley::set_communication_protocol(const CopleyCommand::Protocol protocol)
{
	if (protocol == CopleyCommand::Protocol::kAscii) {
		_command_builder = static_cast<std::unique_ptr<CopleyCommandBuilder>>(
			std::unique_ptr<CopleyCommandBuilderAscii>(new CopleyCommandBuilderAscii(_node_id, _axis)));
	} else {
		_command_builder = static_cast<std::unique_ptr<CopleyCommandBuilder>>(
			std::unique_ptr<CopleyCommandBuilderBinary>(new CopleyCommandBuilderBinary(_node_id, _axis)));
	}
}

bool Copley::connect(void)
{
	// already connected?
	if (is_connected()) {
		_status.raise_warning(CopleyStatus::Code::kComAlreadyOpen);
	} else {
		// set state to connected to attempt to communicate
		_is_connected = true;

		{
			// temporarily suppress reporting of all motor errors.
			// clear latched errors, issue stop command, and then query status.
			auto scoped_suppression = _motor_status.suppress_all();

			// clear latched event registers
			write_set(CopleyParameter::kDriveLatchingFaultStatus, 0xFFFFFFFF);
			write_set(CopleyParameter::kDriveLatchedEventStatus, 0xFFFFFFFF);

			write_hardware_specification();

			// latch only on:
			//	:0  Data flash CRC failure
			//	:1  Drive internal error.
			//  :11 FPGA error 1
			//  :13 FPGA error 2
			write_set(CopleyParameter::kDriveLatchingFaultMask,
					  static_cast<uint32_t>(CopleyEventRegister::LatchingFaultRegisterBit::kDriveFlashCrcFail |
											CopleyEventRegister::LatchingFaultRegisterBit::kDriveInternalError |
											CopleyEventRegister::LatchingFaultRegisterBit::kDriveFpgaErrorType1 |
											CopleyEventRegister::LatchingFaultRegisterBit::kDriveFpgaErrorType2),
					  CopleyParameter::MemoryLocation::kBoth);
			// clear latched event registers
			write_set(CopleyParameter::kDriveLatchingFaultStatus, 0xFFFFFFFF);
			write_set(CopleyParameter::kDriveLatchedEventStatus, 0xFFFFFFFF);

			write_configuration();

			// reset amplifier and update the motor status.
			// this clears any latched errors.
			clear_latched_registers();

			// Found that a Warning: "velocity window - velocity loop error outside limits" will
			// persist in the event register after clearning latched regsiters. Could potentially be an
			// issue of a the longer time required to write to flash. Also, seems to only occur on the motors
			// connected as part of the CAN network. Not the first motor connected via UART.
			// Empirically found that delaying a short amount of time (~25 ms) will allow read
			// of read event register with velocity window error cleared.  -psherman 4/11/17
			usleep(CopleyBus::kNonVolatileWriteTimeoutMs * 1000);

			// Alaways end by disabling drive
			write_drive_mode(DriveMode::kDisabled, CopleyParameter::MemoryLocation::kBoth);
		}

		// resume motor error status reporting
		_cached_state.motor_event.reset();
		_motor_status.clear();

		// read status register to report any new errors
		read_event_register();
	}

	_is_connected = driver_status().ok();
	return _is_connected;
}

bool Copley::connect_weak(const uint8_t node_id)
{
	_node_id = node_id;

	// already connected?
	if (is_connected()) {
		_status.raise_warning(CopleyStatus::Code::kComAlreadyOpen);
	} else {
		_is_connected = _bus->ping(node_id);
	}
	return _is_connected;
}

bool Copley::is_connected(void) const
{
	return _bus->is_connected() && _is_connected;
}

void Copley::disconnect(void)
{
	if (is_connected()) {
		torque_free();
	}
	_is_connected = false;
}

void Copley::disconnect_weak(void)
{
	_is_connected = false;
}

uint32_t Copley::read_serial_number(void)
{
	uint32_t serial_number = static_cast<uint32_t>(
		write_get(CopleyParameter::kDriveSerialNumber, CopleyParameter::MemoryLocation::kNonVolatile));
	return serial_number;
}

bool Copley::is_serial_node(void) const
{
	return _node_id == CopleyBus::kSerialNodeId;
}

uint8_t Copley::read_node_id(void)
{
	uint8_t node_id = static_cast<uint8_t>(write_get(CopleyParameter::kNetworkNodeId));
	return node_id;
}

bool Copley::write_node_id(const uint8_t node_id)
{
	if (driver_status().ok()) {
		// node ID is serial node, or is not serial node and is available?
		if (node_id == CopleyBus::kSerialNodeId || !_bus->ping(node_id)) {
			write_set(CopleyParameter::kNetworkNodeIdConfiguration, node_id,
					  CopleyParameter::MemoryLocation::kNonVolatile);
			write_reset();
			disconnect_weak();
			// if communicating to the serial bus master, do not change bus ID in this driver.
			// otherwise, update to the new value.
			if (is_serial_node()) {
				connect_weak(CopleyBus::kSerialNodeId);
			} else {
				connect_weak(node_id);
			}

			if (driver_status().ok()) {
				if (read_node_id() != node_id) {
					_status.raise_error(CopleyStatus::Code::kConfigFail, "Failed to set node ID.");
				}
			}
		} else {
			_status.raise_error(CopleyStatus::Code::kComNodeIdInUse);
		}
	}
	return driver_status().ok();
}

void Copley::clear_cached_state(void)
{
	_motor_status.clear();
	_cached_state.drive_mode = DriveMode::kUnknown;
	_cached_state.motor_event.reset();
}

CopleyEventRegister Copley::read_event_register(void)
{
	read_event_register_raw(); // updates _cached_state.motor_event
	return _cached_state.motor_event;
}

int Copley::read_event_register_raw(void)
{
	const CopleyParameter::value_t event_register_value = 0;
	if (driver_status().ok()) {
		const bool previous_motor_status_ok					= _motor_status.ok();
		const CopleyParameter::value_t event_register_value = write_get(CopleyParameter::kDriveEventStatus);
		// check for success, otherwise the cached state is set with uninitialized values
		if (driver_status().ok()) {
			// Use a temporary status so that we can ignore a velocity window-loop error warning if
			// not in velocity drive mode
			CopleyStatus temp_status;
			bool latching_fault = _cached_state.motor_event.update(event_register_value, temp_status);
			if (temp_status.severity() >= CopleyStatus::Severity::kWarning) {
				// If status raised was the motor velocity window and the drive mode
				// isn't velocity mode, don't raise an error
				if (temp_status != CopleyStatus::Code::kMotorVelocityWindow ||
					_cached_state.drive_mode == DriveMode::kVelocity) {
					_motor_status.raise(temp_status, temp_status.severity(), temp_status.detail());
				}
			}

			if (latching_fault) {
				_cached_state.motor_event.raise_latching(write_get(CopleyParameter::kDriveLatchingFaultStatus),
														 _motor_status);
			}
			// rising edge of an error? if so, disable the motor
			if (previous_motor_status_ok && !_motor_status.ok())
				torque_free();
		}
	}
	return static_cast<int>(event_register_value);
}

double Copley::read_position_deg(void)
{
	double position = 0;
	if (_bus->status().ok()) {
		// Position actual contains position from device used for trajectory feedback.
		// Could be motor encoder, load encoder, or halls depending on setup
		position = count_to_deg(write_get(CopleyParameter::kPositionActual) * scale_raw_to_count,
								!has_load_encoder_for_feedback(), !has_encoder_feedback());

		read_event_register();
	}
	return position;
}

double Copley::read_load_position_deg(void)
{
	double position = 0;
	if (_bus->status().ok()) {
		position = static_cast<double>(write_get(CopleyParameter::kLoadEncoderPosition)) * 360.0 /
				   _hardware.load_encoder.resolution_cpr;

		read_event_register();
	}
	return position;
}

double Copley::read_position_error_deg(void)
{
	double position_error = 0;
	if (driver_status().ok()) {
		position_error = count_to_deg(write_get(CopleyParameter::kPositionLoopError) * scale_raw_to_count,
									  !has_load_encoder_for_feedback(), !has_encoder_feedback());

				read_event_register();
	}
	return position_error;
}

double Copley::read_commanded_position_deg(void)
{
	double commanded_position = 0;
	if (driver_status().ok()) {
		commanded_position = count_to_deg(write_get(CopleyParameter::kPositionCommanded) * scale_raw_to_count,
										  !has_load_encoder_for_feedback(), !has_encoder_feedback());

				read_event_register();
	}
	return commanded_position;
}

double Copley::read_trajectory_destination_position_deg(void)
{
	double destination_position = 0;
	if (driver_status().ok()) {
		destination_position =
			count_to_deg(write_get(CopleyParameter::kTrajectoryPositionDestination) * scale_raw_to_count,
						 !has_load_encoder_for_feedback(), !has_encoder_feedback());

				read_event_register();
	}
	return destination_position;
}

double Copley::read_back_trajectory_generator_position_command_deg(void)
{
	double command_position = 0;
	if (driver_status().ok()) {
		command_position = count_to_deg(write_get(CopleyParameter::kTrajectoryPosition) * scale_raw_to_count,
										!has_load_encoder_for_feedback(), !has_encoder_feedback());

				read_event_register();
	}
	return command_position;
}

void Copley::drive_absolute_deg(double position_deg)
{
	if (status().ok()) {

		// configure trajectory
		write_set(CopleyParameter::kTrajectoryProfile,
				  static_cast<CopleyParameter::value_t>(TrajectoryProfile::kTrapezoidal) |
					  static_cast<CopleyParameter::value_t>(TrajectoryMove::kAbsolute),
				  CopleyParameter::MemoryLocation::kVolatile);
		write_set(CopleyParameter::kTrajectoryPosition,
				  deg_to_count(position_deg, !has_load_encoder_for_feedback(), !has_encoder_feedback()),
				  CopleyParameter::MemoryLocation::kVolatile);
		write_drive_mode(DriveMode::kPosition);
		write_trajectory(TrajectoryCommand::kInitUpdate);

		read_event_register();
	}
}

void Copley::drive_relative_deg(double move_deg)
{
	if (status().ok()) {

		// configure trajectory
		write_set(CopleyParameter::kTrajectoryProfile,
				  static_cast<CopleyParameter::value_t>(TrajectoryProfile::kTrapezoidal) |
					  static_cast<CopleyParameter::value_t>(TrajectoryMove::kRelative),
				  CopleyParameter::MemoryLocation::kVolatile);
		write_set(CopleyParameter::kTrajectoryPosition,
				  deg_to_count(move_deg, !has_load_encoder_for_feedback(), !has_encoder_feedback()),
				  CopleyParameter::MemoryLocation::kVolatile);
		write_drive_mode(DriveMode::kPosition);
		write_trajectory(TrajectoryCommand::kInitUpdate);

		read_event_register();
	}
}

double Copley::read_velocity_deg_s(void)
{
	double velocity_deg_s = 0;
	if (driver_status().ok()) {
		velocity_deg_s = count_to_deg(write_get(CopleyParameter::kVelocityActual) * scale_raw_to_cps,
									  !has_load_encoder_for_feedback(), !has_motor_encoder());


		read_event_register();
	}
	return velocity_deg_s;
}

void Copley::drive_velocity_deg_s(double velocity_deg_s)
{
	if (status().ok()) {
		// verify velocity is in range
		if (abs(velocity_deg_s * _hardware.gear_ratio) > _hardware.motor.velocity_max_rpm * scale_rpm_to_deg_s) {
			_status.raise_warning(CopleyStatus::Code::kMotorVelocityLimit,
								  "Commanded velocity of " + std::to_string(velocity_deg_s * _hardware.gear_ratio) +
									  " deg/s exceeds motor specified limit of " +
									  std::to_string(_hardware.motor.velocity_max_rpm * scale_rpm_to_deg_s) +
									  " deg/s.");
		} else {
			write_set(CopleyParameter::kVelocityProgrammed,
					  deg_to_count(velocity_deg_s * _hardware.gear_ratio, has_load_encoder_for_feedback(),
								   !has_motor_encoder()) *
						  scale_cps_to_raw,
					  CopleyParameter::MemoryLocation::kVolatile);
			write_drive_mode(DriveMode::kVelocity);

			read_event_register();
		}
	}
}

void Copley::write_drive_mode(const DriveMode mode, CopleyParameter::MemoryLocation location)
{
	if (driver_status().ok()) {
		if (_cached_state.drive_mode != mode) {

			write_set(CopleyParameter::kDriveMode, static_cast<CopleyParameter::value_t>(mode), location);
			_cached_state.drive_mode = mode;

			read_event_register();
		}
	}
}

void Copley::home_start(const HomeMethod::Function function,
						const HomeMethod::Direction direction,
						const double load_offset_deg)
{
	if (status().ok()) {

		CopleyParameter::value_t value = 0;
		value |= static_cast<CopleyParameter::value_t>(function) << HomeMethod::kFunctionOffset;
		value |= static_cast<CopleyParameter::value_t>(direction) << HomeMethod::kDirectionOffset;

		// set configuration
		write_set(CopleyParameter::kHomeMethodConfiguration, value, CopleyParameter::MemoryLocation::kVolatile);

		// set offset degrees
		const double motor_offset_deg = load_offset_deg;
		write_set(CopleyParameter::kHomeOffset,
				  static_cast<CopleyParameter::value_t>(
					  deg_to_count(motor_offset_deg, !has_load_encoder_for_feedback(), !has_encoder_feedback())));

		// start motion
		write_drive_mode(DriveMode::kPosition);
		write_trajectory(TrajectoryCommand::kHome);

		read_event_register();
	}
}

bool Copley::home_is_running(void)
{
	bool is_running = false;
	if (status().ok()) {
		// update status register to catch general motor errors
		read_event_register();

		if (status().ok()) {
			CopleyParameter::value_t trajectory_status = write_get(CopleyParameter::kTrajectoryStatus);
			if (trajectory_status & static_cast<CopleyParameter::value_t>(TrajectoryStatusRegister::kHomingError)) {
				_motor_status.raise_error(CopleyStatus::Code::kMotorHomeFail);
			} else {
				is_running =
					trajectory_status & static_cast<CopleyParameter::value_t>(TrajectoryStatusRegister::kHoming);
			}
		}
	}
	return is_running;
}

void Copley::home_set_here(const double load_offset_deg)
{
	// The direction argument is just to be able to pass the second optional argument, offset_deg
	home_start(HomeMethod::Function::kThisIsHome, HomeMethod::Direction::kPositive, load_offset_deg);
}

void Copley::home_window_cancel(void)
{
	_cached_state.homing_window.mode = HomingWindow::Mode::kIdle;
	torque_free();
}

void Copley::home_window_start(HomeMethod::Direction direction)
{
	// Errors already occurred?
	if (!status().ok()) {
	}
	// Make sure motor isn't already trying to find homing window
	else if (_cached_state.homing_window.mode != HomingWindow::Mode::kIdle) {
		_status.raise_warning(CopleyContext::Code::kHomingWindowFail, "homing window search is already running.");
	}
	// Ok to start homing
	else {
		_cached_state.homing_window.mode = HomingWindow::Mode::kFirstEdgeSearch;

		// If home capture is already stored. Read value to clear flag in event register
		CopleyParameter::value_t capture_status =
			static_cast<CopleyParameter::value_t>(write_get(CopleyParameter::kPositionCaptureStatus));
		if (capture_status & (static_cast<CopleyParameter::value_t>(PositionCapture::Status::HomeCaptured::kReady)
							  << PositionCapture::Status::kHomeCapturedOffset)) {
			write_get(CopleyParameter::kHomePositionCapture);
		}

		_cached_state.homing_window.direction = direction;

		// Start looking for inactive->active edge
		home_window_cycle();
	}
}

bool Copley::home_window_complete(double &home_size_deg)
{
	bool slice_found = false;

	// Errors already occurred
	if (!status().ok()) {
	}
	// If homing is still running. Do nothing.
	else if (is_moving()) {
	}
	// Ok to check homing
	else {
		CopleyParameter::value_t capture_status = write_get(CopleyParameter::kPositionCaptureStatus);

		switch (_cached_state.homing_window.mode) {
		case (HomingWindow::Mode::kIdle):
			_status.raise_warning(CopleyContext::Code::kHomingWindowFail, "homing window search isn't active.");
			break;

		case (HomingWindow::Mode::kFirstEdgeSearch):
			// If a home position capture has been made, but only 1 switch transition has occurred
			if ((capture_status & (static_cast<CopleyParameter::value_t>(PositionCapture::Status::HomeCaptured::kReady)
								   << PositionCapture::Status::kHomeCapturedOffset)) &&
				!(capture_status & (static_cast<CopleyParameter::value_t>(
										PositionCapture::Status::MultipleHomeCaptures::kExtraTransition)
									<< PositionCapture::Status::kMultipleHomeCaptureOffset))) {
				_cached_state.homing_window.mode = HomingWindow::Mode::kSecondEdgeSearch;

				// Read home capture position register
				_cached_state.homing_window.edges.first = write_get(CopleyParameter::kHomePositionCapture);

				// Start looking for active->inactive edge.
				home_window_cycle();
			} else {
				_cached_state.homing_window.mode = HomingWindow::Mode::kIdle;


				_status.raise_error(CopleyContext::Code::kHomingWindowFail,
									"invalid home position capture reading active rise.");
			}

			break;

		case (HomingWindow::Mode::kSecondEdgeSearch):
			if ((capture_status & (static_cast<CopleyParameter::value_t>(PositionCapture::Status::HomeCaptured::kReady)
								   << PositionCapture::Status::kHomeCapturedOffset)) &&
				!(capture_status & (static_cast<CopleyParameter::value_t>(
										PositionCapture::Status::MultipleHomeCaptures::kExtraTransition)
									<< PositionCapture::Status::kMultipleHomeCaptureOffset))) {
				_cached_state.homing_window.mode = HomingWindow::Mode::kMovingToHome;

				// Read home capture position register
				_cached_state.homing_window.edges.second = write_get(CopleyParameter::kHomePositionCapture);

				// Determine positive and negative edges
				int32_t edge_pos, edge_neg;
				if (_cached_state.homing_window.edges.first > _cached_state.homing_window.edges.second) {
					edge_pos = _cached_state.homing_window.edges.first;
					edge_neg = _cached_state.homing_window.edges.second;
				} else {
					edge_pos = _cached_state.homing_window.edges.second;
					edge_neg = _cached_state.homing_window.edges.first;
				}

				// Get the size of the homing window in degrees.
				// If motor started with the home switch inactive, an extra revolution had to be
				// done to find the second edge and 360 of the output should be subtracted.
				home_size_deg =
					count_to_deg(edge_pos - edge_neg, !has_load_encoder_for_feedback(), !has_motor_encoder());
				if (home_size_deg > 360.0) {
					home_size_deg -= 360.0;
				}

				// Calculate (and start moving to) new home position
				drive_absolute_deg(count_to_deg(edge_pos, !has_load_encoder_for_feedback(), !has_motor_encoder()) -
								   (home_size_deg / 2.0));
			} else {
				_cached_state.homing_window.mode = HomingWindow::Mode::kIdle;
				_status.raise_error(CopleyContext::Code::kHomingWindowFail,
									"invalid home position capture reading active fall.");
			}

			break;

		case (HomingWindow::Mode::kMovingToHome):
			_cached_state.homing_window.mode = HomingWindow::Mode::kIdle;

			// Notify user homing window search is done
			slice_found = true;
			break;
		}
	}

	return slice_found;
}

void Copley::home_hardstop_current_limit_write(double current_limit_A, uint16_t delay_time_ms)
{
	if (status().ok() && !is_moving()) {
		if (current_limit_A > _hardware.amplifier.current_continuous_limit_A) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested home current limit of " + std::to_string(current_limit_A) +
									  " A exceeds amplifier limit of " +
									  std::to_string(_hardware.amplifier.current_continuous_limit_A) +
									  "A. Change ignored.");
		} else if (current_limit_A > _hardware.motor.current_continuous_A) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested home current limit of " + std::to_string(current_limit_A) +
									  "A exceeds motor specified limit of " +
									  std::to_string(_hardware.motor.current_continuous_A) + " A. Change ignored.");
		} else if (current_limit_A > _hardware.algorithm_default.current_continuous_max_A) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested home current limit of " + std::to_string(current_limit_A) +
									  "A exceeds default control algorithm limit of " +
									  std::to_string(_hardware.algorithm_default.current_continuous_max_A) +
									  " A. Change ignored.");
		} else {
						write_set(CopleyParameter::kHomeHardStopCurrentLimit,
					  static_cast<CopleyParameter::value_t>(current_limit_A * scale_A_to_raw),
					  CopleyParameter::kHomeHardStopCurrentLimit.location);

			write_set(CopleyParameter::kHomeHardStopCurrentDelayTime,
					  static_cast<CopleyParameter::value_t>(delay_time_ms),
					  CopleyParameter::kHomeHardStopCurrentDelayTime.location);
		}
	}
}

void Copley::home_window_cycle(void)
{
	if (!status().ok()) {
	} else if (_cached_state.homing_window.mode != HomingWindow::Mode::kFirstEdgeSearch &&
			   _cached_state.homing_window.mode != HomingWindow::Mode::kSecondEdgeSearch) {
		_status.raise_warning(CopleyContext::Code::kHomingWindowFail,
							  "homing window operation in wrong state to cycle.");
	} else {
		// Configure position capture method to find inactive to active edge.
		CopleyParameter::value_t capture_config = 0;
		capture_config |= static_cast<CopleyParameter::value_t>(PositionCapture::Control::HomeCaptureRewrite::kKeep)
						  << PositionCapture::Control::kRewriteHomeOffset;
		if (_cached_state.homing_window.mode == HomingWindow::Mode::kFirstEdgeSearch)
			capture_config |=
				static_cast<CopleyParameter::value_t>(PositionCapture::Control::HomeCaptureEdge::kActiveRise)
				<< PositionCapture::Control::kHomeCaptureEdgeOffset;
		else
			capture_config |=
				static_cast<CopleyParameter::value_t>(PositionCapture::Control::HomeCaptureEdge::kActiveFall)
				<< PositionCapture::Control::kHomeCaptureEdgeOffset;

		write_set(CopleyParameter::kPositionCaptureControl, capture_config,
				  CopleyParameter::kPositionCaptureControl.location);

		// Move one revolution in direction specified. Looking for homing switch transition.
		if (_cached_state.homing_window.direction == HomeMethod::Direction::kPositive) {
			drive_relative_deg(_cached_state.homing_window.kHomeWindowCyle);
		} else {
			drive_relative_deg(-_cached_state.homing_window.kHomeWindowCyle);
		}

		read_event_register();
	}
}

void Copley::torque_free(void)
{
	if (is_connected()) {
		// always attempt to stop, even if an error has occurred
		CopleyStatus previous_status;
		_status.cache(previous_status);

		// abort trajectory so it does not resume when re-enabled.
		write_trajectory(TrajectoryCommand::kAbort);
		write_drive_mode(DriveMode::kDisabled);

		_status.restore(previous_status);
		read_event_register();
	}
}

bool Copley::is_sto_active(bool sto_1)
{
	if (driver_status().ok()) {
		int32_t sto_register = write_get(CopleyParameter::kDriveSafetyCircuitStatus) & 0x0003;

		bool sto_0_status = sto_register & 0x1;
		bool sto_1_status = (sto_register & 0x2) >> 1;

		read_event_register();

		if (sto_1) {
			return sto_1_status;
		} else {
			return sto_0_status;
		}
	}

	return false;
}

bool Copley::is_moving(void)
{
	if (driver_status().ok()) {
		read_event_register();
		return write_get(CopleyParameter::kTrajectoryStatus) &
			   static_cast<CopleyParameter::value_t>(TrajectoryStatusRegister::kMoving);
	}
	return false;
}

bool Copley::brake_is_active(void)
{
	if (driver_status().ok()) {
		return read_event_register().brake_is_active();
	}
	return false;
}

bool Copley::limit_pos_is_active(void)
{
	if (driver_status().ok()) {
		return read_event_register().limit_pos_is_active();
	}
	return false;
}

bool Copley::limit_neg_is_active(void)
{
	if (driver_status().ok()) {
		return read_event_register().limit_neg_is_active();
	}
	return false;
}

bool Copley::home_switch_is_active(void)
{
	if (driver_status().ok()) {
		return read_event_register().home_switch_is_active();
	}
	return false;
}

bool Copley::software_limit_pos_is_active(void)
{
	if (driver_status().ok()) {
		return read_event_register().software_limit_pos_is_active();
	}
	return false;
}

bool Copley::software_limit_neg_is_active(void)
{
	if (driver_status().ok()) {
		return read_event_register().software_limit_neg_is_active();
	}
	return false;
}

double Copley::read_voltage_V(void)
{
	double voltage_V = 0;
	if (driver_status().ok()) {
		read_event_register();
		voltage_V = write_get(CopleyParameter::kHighVoltage) * scale_raw_to_V;
			}
	return voltage_V;
}

double Copley::read_current_A(void)
{
	double current_A = 0;
	if (_bus->status().ok()) {
		read_event_register();
		current_A = write_get(CopleyParameter::kCurrentActual) * scale_raw_to_A;
	}
	return current_A;
}

double Copley::read_torque_Nm(void)
{
	return read_current_A() * _hardware.motor.torque_constant_Nm_A * _hardware.gear_ratio;
}

double Copley::read_mechanical_power_W(void)
{
	double power_W = 0;
	if (driver_status().ok()) {
		power_W = read_torque_Nm() * read_velocity_deg_s() * scale_deg_to_rad;
			}
	return power_W;
}

double Copley::read_temperature_C(void)
{
	double temperature_C = 0;
	if (driver_status().ok()) {
		read_event_register();
		temperature_C = static_cast<double>(write_get(CopleyParameter::kDriveTemperature));
			}
	return temperature_C;
}

void Copley::reset(void)
{
		clear_status();

	if (is_connected()) {
		clear_latched_registers();
		disconnect();
	}

	if (_node_id != kNodeIdMax + 1) {
		connect(); // connect method will clear status
	}
}

void Copley::write_current_gains(const double Cp, const double Ci, const double offset_A)
{
	if (driver_status().ok()) {
		// Cp
		if (Cp < CopleyAlgorithm::kGainMin || Cp > CopleyAlgorithm::kGainMax) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested current loop proportional gain (Cp) " + std::to_string(Cp) +
									  "is outside the amplifer range of [" + std::to_string(CopleyAlgorithm::kGainMin) +
									  ", " + std::to_string(CopleyAlgorithm::kGainMax) + "]. Gain change ignored.");
		} else {
			write_set(CopleyParameter::kCurrentLoopCp, static_cast<CopleyParameter::value_t>(Cp),
					  CopleyParameter::kCurrentLoopCp.location);
		}

		// Ci
		if (Ci < CopleyAlgorithm::kGainMin || Ci > CopleyAlgorithm::kGainMax) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested current loop integral gain (Ci) " + std::to_string(Ci) +
									  "is outside the amplifer range of [" + std::to_string(CopleyAlgorithm::kGainMin) +
									  ", " + std::to_string(CopleyAlgorithm::kGainMax) + "]. Gain change ignored.");
		} else {
			write_set(CopleyParameter::kCurrentLoopCi, static_cast<CopleyParameter::value_t>(Ci),
					  CopleyParameter::kCurrentLoopCi.location);
		}

		// Current loop offset
		if (offset_A > _hardware.algorithm_default.current_continuous_max_A) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested current loop offset " + std::to_string(offset_A) +
									  "exceeds the default algorithm maximum continuous current of " +
									  std::to_string(_hardware.algorithm_default.current_continuous_max_A) +
									  ". Gain change ignored.");
		} else {
			write_set(CopleyParameter::kCurrentLoopOffset,
					  static_cast<CopleyParameter::value_t>(offset_A * CopleyConversion::scale_A_to_raw),
					  CopleyParameter::kCurrentLoopOffset.location);
		}
	}
}

void Copley::write_current_gains_defaults(void)
{
	if (driver_status().ok()) {
		write_current_gains(_hardware.algorithm_default.current_gain_Cp, _hardware.algorithm_default.current_gain_Ci,
							_hardware.algorithm_default.current_offset_A);
	}
}

void Copley::write_current_limits_A(const double continuous_A, const double peak_A, const double I2t_ms)
{
	if (driver_status().ok()) {
		// continuous current
		if (continuous_A > _hardware.amplifier.current_continuous_limit_A) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested continuous current limit of " + std::to_string(continuous_A) +
									  "A exceeds amplifier limit of " +
									  std::to_string(_hardware.amplifier.current_continuous_limit_A) + "A." +
									  " Continuous current limit change ignored.");
		} else if (continuous_A > _hardware.motor.current_continuous_A) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested continuous current limit of " + std::to_string(continuous_A) +
									  "A exceeds motor specified limit of " +
									  std::to_string(_hardware.motor.current_continuous_A) + "A." +
									  " Continuous current limit change ignored.");
		} else if (continuous_A > _hardware.algorithm_default.current_continuous_max_A) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested continuous current limit of " + std::to_string(continuous_A) +
									  "A exceeds default control algorithm limit of " +
									  std::to_string(_hardware.algorithm_default.current_continuous_max_A) + "A." +
									  " Continuous current limit change ignored.");
		} else if (continuous_A > peak_A) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested continuous current limit of " + std::to_string(continuous_A) +
									  "A exceeds requested peak current limit of " + std::to_string(peak_A) + "A." +
									  " Continuous current limit change ignored.");
		} else {
						write_set(CopleyParameter::kCurrentLimitContinuous,
					  static_cast<CopleyParameter::value_t>(continuous_A * scale_A_to_raw),
					  CopleyParameter::kCurrentLimitContinuous.location);
		}

		// peak current
		if (peak_A > _hardware.amplifier.current_peak_limit_A) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested peak current limit of " + std::to_string(peak_A) +
									  "A exceeds amplifier limit of " +
									  std::to_string(_hardware.amplifier.current_peak_limit_A) + "A." +
									  " Peak current limit change ignored.");
		} else if (peak_A > _hardware.motor.current_peak_A) {
			_status.raise_warning(
				CopleyStatus::Code::kInvalidInputData,
				"requested peak current limit of " + std::to_string(peak_A) + "A exceeds motor specified limit of " +
					std::to_string(_hardware.motor.current_peak_A) + "A." + " Peak current limit change ignored.");
		} else if (peak_A < continuous_A) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested peak current limit of " + std::to_string(peak_A) +
									  "A is less than requsted continuous current limit of " +
									  std::to_string(continuous_A) + "A." + " Peak current limit change ignored.");
		} else {
						write_set(CopleyParameter::kCurrentLimitPeak,
					  static_cast<CopleyParameter::value_t>(peak_A * scale_A_to_raw),
					  CopleyParameter::kCurrentLimitPeak.location);
		}

		// I^2*t time ms
		if (I2t_ms < 0) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested invalid current I^2*t time limit of " + std::to_string(I2t_ms) +
									  "ms. Current limit changes ignored.");
		} else {
						write_set(CopleyParameter::kCurrentLimitPeakTime, static_cast<CopleyParameter::value_t>(I2t_ms),
					  CopleyParameter::kCurrentLimitPeakTime.location);
		}
	}
}

void Copley::write_current_limits_defaults(void)
{
	if (driver_status().ok()) {
		double continuous_current_default =
			std::min(_hardware.algorithm_default.current_continuous_max_A,
					 std::min(_hardware.motor.current_continuous_A, _hardware.amplifier.current_continuous_limit_A));

		double peak_current_default =
			std::min(_hardware.algorithm_default.current_peak_max_A,
					 std::min(_hardware.motor.current_peak_A, _hardware.amplifier.current_peak_limit_A));

		write_current_limits_A(continuous_current_default, peak_current_default,
							   _hardware.algorithm_default.current_I2t_ms);
	}
}

void Copley::write_velocity_gains(const double Vp, const double Vi, const double Vi_drain)
{
	if (driver_status().ok()) {
		// Vp
		if (Vp < CopleyAlgorithm::kGainMin || Vp > CopleyAlgorithm::kGainMax) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested velocity loop proportional gain (Vp) " + std::to_string(Vp) +
									  "is outside the amplifer range of [" + std::to_string(CopleyAlgorithm::kGainMin) +
									  ", " + std::to_string(CopleyAlgorithm::kGainMax) + "]. Gain change ignored.");
		} else {
			write_set(CopleyParameter::kVelocityLoopVp, static_cast<CopleyParameter::value_t>(Vp),
					  CopleyParameter::kVelocityLoopVp.location);
		}

		// Vi
		if (Vi < CopleyAlgorithm::kGainMin || Vi > CopleyAlgorithm::kGainMax) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested velocity loop proportional gain (Vi) " + std::to_string(Vi) +
									  "is outside the amplifer range of [" + std::to_string(CopleyAlgorithm::kGainMin) +
									  ", " + std::to_string(CopleyAlgorithm::kGainMax) + "]. Gain change ignored.");
		} else {
			write_set(CopleyParameter::kVelocityLoopVi, static_cast<CopleyParameter::value_t>(Vi),
					  CopleyParameter::kVelocityLoopVi.location);
		}

		// Vi_drain
		if (Vi_drain < CopleyAlgorithm::kGainMin || Vi_drain > CopleyAlgorithm::kGainMax) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested velocity loop integral drain (Vi drain) " + std::to_string(Vi_drain) +
									  "is outside the amplifer range of [" + std::to_string(CopleyAlgorithm::kGainMin) +
									  ", " + std::to_string(CopleyAlgorithm::kGainMax) + "]. Gain change ignored.");
		} else {
			write_set(CopleyParameter::kVelocityLoopViDrain, static_cast<CopleyParameter::value_t>(Vi_drain),
					  CopleyParameter::kVelocityLoopViDrain.location);
		}
	}
}

std::string Copley::read_velocity_gains(void)
{
	std::string gains_str = "";
	if (driver_status().ok()) {
		double Vp =
			static_cast<double>(write_get(CopleyParameter::kVelocityLoopVp, CopleyParameter::kVelocityLoopVp.location));
		double Vi =
			static_cast<double>(write_get(CopleyParameter::kVelocityLoopVi, CopleyParameter::kVelocityLoopVi.location));
		double Vi_drain = static_cast<double>(
			write_get(CopleyParameter::kVelocityLoopViDrain, CopleyParameter::kVelocityLoopViDrain.location));

		gains_str += "Vp: " + std::to_string(Vp) + '\n';
		gains_str += "Vi: " + std::to_string(Vi) + '\n';
		gains_str += "Vi_drain: " + std::to_string(Vi_drain) + '\n';
	}

	return gains_str;
}

void Copley::write_velocity_gains_defaults(void)
{
	if (driver_status().ok()) {
		write_velocity_gains(_hardware.algorithm_default.velocity_gain_Vp, _hardware.algorithm_default.velocity_gain_Vi,
							 _hardware.algorithm_default.velocity_gain_Vi_drain);
	}
}

void Copley::write_velocity_loop_shift(const int16_t shift)
{
	if (driver_status().ok()) {
		write_set(CopleyParameter::kVelocityLoopShift, static_cast<CopleyParameter::value_t>(shift),
				  CopleyParameter::kVelocityLoopShift.location);
			}
}

void Copley::write_velocity_loop_shift_defaults(void)
{
	write_velocity_loop_shift(_hardware.algorithm_default.velocity_loop_shift);
}

void Copley::write_ungeared_velocity_window_deg_s(const double tracking_window_deg_s,
												  const double tracking_window_time_ms)
{
	if (driver_status().ok()) {
		write_set(CopleyParameter::kVelocityWindow,
				  static_cast<CopleyParameter::value_t>(
					  deg_to_count(tracking_window_deg_s, false, !has_motor_encoder()) * scale_cps_to_raw),
				  CopleyParameter::kVelocityWindow.location);
		write_set(CopleyParameter::kVelocityWindowTime, static_cast<CopleyParameter::value_t>(tracking_window_time_ms),
				  CopleyParameter::kVelocityWindowTime.location);
	}
}

std::string Copley::read_velocity_window_deg_s(void)
{
	std::string msg;
	if (driver_status().ok()) {
		double window_deg_s = (count_to_deg(write_get(CopleyParameter::kVelocityWindow) * scale_raw_to_cps, false,
											!has_motor_encoder())) /
							  _hardware.gear_ratio;
		double window_time_ms = write_get(CopleyParameter::kVelocityWindowTime);

		msg += "Velocity window: " + std::to_string(window_deg_s) + " deg/s.";
		msg += "\nVelocity window period: " + std::to_string(window_time_ms) + " ms.";
			}
	return msg;
}

void Copley::write_velocity_window_deg_s(const double tracking_window_deg_s, const double tracking_window_time_ms)
{
	write_ungeared_velocity_window_deg_s(tracking_window_deg_s * _hardware.gear_ratio, tracking_window_time_ms);
}

void Copley::write_velocity_window_defaults(void)
{

	write_ungeared_velocity_window_deg_s(_hardware.algorithm_default.velocity_tracking_window_rpm * scale_rpm_to_deg_s,
										 _hardware.algorithm_default.velocity_tracking_time_ms);
}

double Copley::read_velocity_loop_error_deg_s(void)
{
	return driver_status().ok() ? (count_to_deg(write_get(CopleyParameter::kVelocityLoopError) * scale_raw_to_cps,
												false, !has_motor_encoder())) /
									  _hardware.gear_ratio
								: 0.0;
}

void Copley::write_ungeared_velocity_limits_deg(const double velocity_deg_s, const double accel_deg_s2)
{
	if (driver_status().ok()) {
		// velocity limit
		if (velocity_deg_s > _hardware.motor.velocity_max_rpm * scale_rpm_to_deg_s) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested velocity limit of " + std::to_string(velocity_deg_s) +
									  "deg/s exceeds motor limit of " +
									  std::to_string(_hardware.motor.velocity_max_rpm * scale_rpm_to_deg_s) +
									  " deg/s." + " Velocity limit change ignored.");
		} else {
			// For motors with a load encoder and no primary encoder, the velocity loop limits should be
			// scaled from degrees to counts using the hall sensor switching for counts/rev.
			write_set(CopleyParameter::kVelocityLoopVelocityLimit,
					  static_cast<CopleyParameter::value_t>(deg_to_count(velocity_deg_s, false, !has_motor_encoder()) *
															scale_cps_to_raw),
					  CopleyParameter::kVelocityLoopVelocityLimit.location);
		}

		// acceleration limits
		write_set(CopleyParameter::kVelocityLoopAccelLimit,
				  static_cast<CopleyParameter::value_t>(deg_to_count(accel_deg_s2, false, !has_motor_encoder()) *
														scale_accel_limit_cps2_to_raw),
				  CopleyParameter::kVelocityLoopAccelLimit.location);
		// this driver assumes accel, decel, emergency limits are the same
		write_set(CopleyParameter::kVelocityLoopDecelLimit,
				  static_cast<CopleyParameter::value_t>(deg_to_count(accel_deg_s2, false, !has_motor_encoder()) *
														scale_accel_limit_cps2_to_raw),
				  CopleyParameter::kVelocityLoopDecelLimit.location);
		write_set(CopleyParameter::kVelocityLoopEmergencyAccelLimit,
				  static_cast<CopleyParameter::value_t>(deg_to_count(accel_deg_s2, false, !has_motor_encoder()) *
														scale_accel_limit_cps2_to_raw),
				  CopleyParameter::kVelocityLoopEmergencyAccelLimit.location);
	}
}

void Copley::write_velocity_limits_deg(const double velocity_deg_s, const double accel_deg_s2)
{
	write_ungeared_velocity_limits_deg(_hardware.gear_ratio * velocity_deg_s, _hardware.gear_ratio * accel_deg_s2);
}

void Copley::write_velocity_limits_defaults(void)
{
	write_ungeared_velocity_limits_deg(_hardware.algorithm_default.velocity_max_rpm * scale_rpm_to_deg_s,
									   _hardware.algorithm_default.velocity_accel_max_rps2 * scale_rps_to_deg_s);
}

void Copley::write_position_gains(const double Pp,
								  const double Vff_percent,
								  const double Aff,
								  const double gains_multiplier_percent)
{
	if (driver_status().ok()) {
		// Pp
		if (Pp < CopleyAlgorithm::kGainMin || Pp > CopleyAlgorithm::kGainMax) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested position loop proportional gain (Pp) " + std::to_string(Pp) +
									  "is outside the amplifier range of [" +
									  std::to_string(CopleyAlgorithm::kGainMin) + ", " +
									  std::to_string(CopleyAlgorithm::kGainMax) + "]. Gain change ignored.");
		} else {
			write_set(CopleyParameter::kPositionLoopPp, static_cast<CopleyParameter::value_t>(Pp),
					  CopleyParameter::kPositionLoopPp.location);
		}

		// Vff
		if (Vff_percent * scale_vff_percent_to_raw < CopleyAlgorithm::kGainMin ||
			Vff_percent * scale_vff_percent_to_raw > CopleyAlgorithm::kGainMax) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested position loop velocity feed-forward gain (Vff) " +
									  std::to_string(Vff_percent) + "is outside the amplifer range of [" +
									  std::to_string(CopleyAlgorithm::kGainMin / scale_vff_percent_to_raw) + "%, " +
									  std::to_string(CopleyAlgorithm::kGainMax / scale_vff_percent_to_raw) +
									  "%]. Gain change ignored.");
		} else {
			write_set(CopleyParameter::kPositionLoopVff,
					  static_cast<CopleyParameter::value_t>(Vff_percent * scale_vff_percent_to_raw),
					  CopleyParameter::kPositionLoopVff.location);
		}

		// Aff
		if (Aff < CopleyAlgorithm::kGainMin || Aff > CopleyAlgorithm::kGainMax) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "Requested position loop proportional gain (Pp) " + std::to_string(Pp) +
									  "is outside the amplifer range of [" + std::to_string(CopleyAlgorithm::kGainMin) +
									  ", " + std::to_string(CopleyAlgorithm::kGainMax) + "]. Gain change ignored.");
		} else {
			write_set(CopleyParameter::kPositionLoopAff, static_cast<CopleyParameter::value_t>(Aff),
					  CopleyParameter::kPositionLoopAff.location);
		}

		write_set(CopleyParameter::kPositionLoopGainsMultiplier,
				  static_cast<CopleyParameter::value_t>(gains_multiplier_percent),
				  CopleyParameter::kPositionLoopGainsMultiplier.location);
	}
}

void Copley::write_position_gains_defaults(void)
{
	if (driver_status().ok()) {
		write_position_gains(_hardware.algorithm_default.position_gain_Pp,
							 _hardware.algorithm_default.position_gain_Vff / scale_vff_percent_to_raw,
							 _hardware.algorithm_default.position_gain_Aff,
							 _hardware.algorithm_default.position_gains_multiplier_percent);
	}
}

std::string Copley::read_position_gains(void)
{
	std::string gains_str = "";
	if (driver_status().ok()) {
		gains_str += "Position Control Gains:\n";

		double Pp =
			static_cast<double>(write_get(CopleyParameter::kPositionLoopPp, CopleyParameter::kPositionLoopPp.location));
		gains_str += "Pp: " + std::to_string(Pp) + '\n';

		// While Pi and Pd registers exist, not all models support them.
		// The model used with this driver does not, hence they are removed.
		// If the model does not support these values, an error is returned
		// of invalid parameter.
		if (_hardware.amplifier.supports_Pi) {
			double Pi = static_cast<double>(
				write_get(CopleyParameter::kPositionLoopPi, CopleyParameter::kPositionLoopPi.location));
			gains_str += "Pi: " + std::to_string(Pi) + '\n';
		}
		if (_hardware.amplifier.supports_Pd) {
			double Pd = static_cast<double>(
				write_get(CopleyParameter::kPositionLoopPd, CopleyParameter::kPositionLoopPd.location));
			gains_str += "Pd: " + std::to_string(Pd) + '\n';
			;
		}

		double Vff = static_cast<double>(
			write_get(CopleyParameter::kPositionLoopVff, CopleyParameter::kPositionLoopVff.location) /
			scale_vff_percent_to_raw);
		gains_str += "Vff: " + std::to_string(Vff) + "% \n";

		double Aff = static_cast<double>(
			write_get(CopleyParameter::kPositionLoopAff, CopleyParameter::kPositionLoopAff.location));
		gains_str += "Aff: " + std::to_string(Aff) + '\n';
	}
	return gains_str;
}

void Copley::write_ungeared_position_tracking_limits_deg(const double position_warning_deg,
														 const double position_error_deg)
{
	if (driver_status().ok()) {
		write_set(
			CopleyParameter::kPositionFollowingWarningLimit,
			static_cast<CopleyParameter::value_t>(deg_to_count(position_warning_deg, false, !has_encoder_feedback())),
			CopleyParameter::kPositionFollowingWarningLimit.location);
		write_set(CopleyParameter::kPositionFollowingErrorLimit,
				  static_cast<CopleyParameter::value_t>(deg_to_count(position_error_deg, false, !has_encoder_feedback())),
				  CopleyParameter::kPositionFollowingErrorLimit.location);
	}
}

void Copley::write_position_tracking_limits_deg(const double position_warning_deg, const double position_error_deg)
{
	if (driver_status().ok()) {
		write_ungeared_position_tracking_limits_deg(position_warning_deg * _hardware.gear_ratio,
													position_error_deg * _hardware.gear_ratio);
	}
}

void Copley::write_position_tracking_limits_defaults(void)
{
	if (driver_status().ok()) {
		write_ungeared_position_tracking_limits_deg(
			count_to_deg(_hardware.algorithm_default.position_following_warning_cts, false, !has_encoder_feedback()),
			count_to_deg(_hardware.algorithm_default.position_following_error_cts, false, !has_encoder_feedback()));
	}
}

std::string Copley::read_position_tracking_limits_deg(void)
{
	std::string window_values = "";
	if (driver_status().ok()) {
		double warning_deg = count_to_deg(write_get(CopleyParameter::kPositionFollowingWarningLimit,
													CopleyParameter::kPositionFollowingWarningLimit.location),
										  !has_load_encoder_for_feedback(), !has_encoder_feedback());
		double error_deg   = count_to_deg(write_get(CopleyParameter::kPositionFollowingErrorLimit,
													CopleyParameter::kPositionFollowingErrorLimit.location),
										  !has_load_encoder_for_feedback(), !has_encoder_feedback());

		window_values += "Warning Limit: " + std::to_string(warning_deg) + "deg\n";
		window_values += "Error Limit: " + std::to_string(error_deg) + "deg\n";
	}

	return window_values;
}

void Copley::write_trajectory_limits_deg(const double velocity_deg_s, const double accel_deg_s2)
{
	if (!has_load_encoder_for_feedback())
		write_ungeared_trajectory_limits_deg(_hardware.gear_ratio * velocity_deg_s,
											 _hardware.gear_ratio * accel_deg_s2);
	else
		write_ungeared_trajectory_limits_deg(velocity_deg_s, accel_deg_s2);
}

void Copley::write_trajectory_limits_defaults(void)
{
	write_ungeared_trajectory_limits_deg(_hardware.algorithm_default.trajectory_velocity_max_rpm * scale_rpm_to_deg_s,
										 _hardware.algorithm_default.trajectory_accel_max_rps2 * scale_rps_to_deg_s);
}

void Copley::write_ungeared_trajectory_limits_deg(const double velocity_deg_s, const double accel_deg_s2)
{
	if (driver_status().ok()) {
		if (velocity_deg_s > _hardware.motor.velocity_max_rpm * scale_rpm_to_deg_s) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested ungeared velocity limit of " + std::to_string(velocity_deg_s) +
									  " deg/s exceeds motor specified limit of " +
									  std::to_string(_hardware.motor.velocity_max_rpm * scale_rpm_to_deg_s) +
									  " deg/s. Parameter change ignored.");
		} else if (velocity_deg_s > _hardware.algorithm_default.trajectory_velocity_max_rpm * scale_rpm_to_deg_s) {
			_status.raise_warning(
				CopleyStatus::Code::kInvalidInputData,
				"requested ungeared velocity limit of " + std::to_string(velocity_deg_s) +
					" deg/s exceeds default trajectory control algorithm limit of " +
					std::to_string(_hardware.algorithm_default.trajectory_velocity_max_rpm * scale_rpm_to_deg_s) +
					" deg/s. Parameter change ignored.");
		} else {
			write_set(CopleyParameter::kTrajectoryMaxVelocity,
					  static_cast<CopleyParameter::value_t>(deg_to_count(velocity_deg_s, false, !has_encoder_feedback()) *
															scale_cps_to_raw),
					  CopleyParameter::kTrajectoryMaxVelocity.location);
		}

		if (accel_deg_s2 > _hardware.algorithm_default.trajectory_accel_max_rps2 * scale_rps_to_deg_s) {
			_status.raise_warning(
				CopleyStatus::Code::kInvalidInputData,
				"requested ungeared acceleration limit of " + std::to_string(accel_deg_s2) +
					" deg/s^2 exceeds calculated trajectory acceleration limit of " +
					std::to_string(_hardware.algorithm_default.trajectory_accel_max_rps2 * scale_rps_to_deg_s) +
					" deg/s^2. Parameter change ignored.");
		} else {
			write_set(CopleyParameter::kTrajectoryMaxAccel,
					  static_cast<CopleyParameter::value_t>(deg_to_count(accel_deg_s2, false, !has_encoder_feedback()) *
															scale_cps2_to_raw),
					  CopleyParameter::kTrajectoryMaxAccel.location);
			// this driver assumes accel, decel, emergency limits are the same
			write_set(CopleyParameter::kTrajectoryMaxDecel,
					  static_cast<CopleyParameter::value_t>(deg_to_count(accel_deg_s2, false, !has_encoder_feedback()) *
															scale_cps2_to_raw),
					  CopleyParameter::kTrajectoryMaxDecel.location);
			write_set(CopleyParameter::kTrajectoryAbortDecel,
					  static_cast<CopleyParameter::value_t>(deg_to_count(accel_deg_s2, false, !has_encoder_feedback()) *
															scale_cps2_to_raw),
					  CopleyParameter::kTrajectoryAbortDecel.location);

			// jerk is not thought to need to be configured, so use motor specification
			write_set(CopleyParameter::kTrajectoryMaxJerk,
					  static_cast<CopleyParameter::value_t>(
						  deg_to_count(_hardware.algorithm_default.trajectory_jerk_max_rps3 * scale_rps_to_deg_s, false,
									   !has_encoder_feedback()) *
						  scale_cps3_to_raw),
					  CopleyParameter::kTrajectoryMaxJerk.location);
		}
	}
}

void Copley::write_input_pin_config(const uint8_t pin,
									const CopleyHardwareSpecification::InputPin mode,
									bool enable_pullup,
									CopleyParameter::MemoryLocation location)
{
	if (driver_status().ok() && is_input_index_valid(pin)) {
		CopleyParameter::value_t value = static_cast<CopleyParameter::value_t>(mode);
		value |= _axis << CopleyHardwareSpecification::kInputPinConfigAxisOffset;

		switch (pin) {
		case 0:
			write_set(CopleyParameter::kDigitalInput0Configuration, value, location);
			break;
		case 1:
			write_set(CopleyParameter::kDigitalInput1Configuration, value, location);
			break;
		case 2:
			write_set(CopleyParameter::kDigitalInput2Configuration, value, location);
			break;
		case 3:
			write_set(CopleyParameter::kDigitalInput3Configuration, value, location);
			break;
		case 4:
			write_set(CopleyParameter::kDigitalInput4Configuration, value, location);
			break;
		case 5:
			write_set(CopleyParameter::kDigitalInput5Configuration, value, location);
			break;
		case 6:
			write_set(CopleyParameter::kDigitalInput6Configuration, value, location);
			break;
		case 7:
			write_set(CopleyParameter::kDigitalInput7Configuration, value, location);
			break;
		case 8:
			write_set(CopleyParameter::kDigitalInput8Configuration, value, location);
			break;
		case 9:
			write_set(CopleyParameter::kDigitalInput9Configuration, value, location);
			break;
		case 10:
			write_set(CopleyParameter::kDigitalInput10Configuration, value, location);
			break;
		case 11:
			write_set(CopleyParameter::kDigitalInput11Configuration, value, location);
			break;
		case 12:
			write_set(CopleyParameter::kDigitalInput12Configuration, value, location);
			break;
		case 13:
			write_set(CopleyParameter::kDigitalInput13Configuration, value, location);
			break;
		case 14:
			write_set(CopleyParameter::kDigitalInput14Configuration, value, location);
			break;
		case 15:
			write_set(CopleyParameter::kDigitalInput15Configuration, value, location);
			break;
		case 16:
			write_set(CopleyParameter::kDigitalInput16Configuration, value, location);
			break;
		case 17:
			write_set(CopleyParameter::kDigitalInput17Configuration, value, location);
			break;
		default:
			std::string detail = "Invalid input pin index " + std::to_string(static_cast<int>(pin)) +
								 ". Must by between 0 and " + std::to_string(_hardware.amplifier.num_inputs - 1) +
								 " for this amp.";
			_status.raise_warning(CopleyStatus::Code::kAmplifierParameterDoesNotExist, detail);
			// return here so the pull-up register does not get set
			return;
		}

		if (std::find(_hardware.amplifier.inputs_with_pullup_option.begin(),
					  _hardware.amplifier.inputs_with_pullup_option.end(),
					  pin) != _hardware.amplifier.inputs_with_pullup_option.end()) {
			write_input_pin_pullup_enable(pin, enable_pullup, location);
		}
	}
}

void Copley::write_input_pin_pullup_enable(const uint8_t pin,
										   bool enable_pullup,
										   CopleyParameter::MemoryLocation location)
{

	// First, check if pin is able to support pull-up programming
	auto it = std::find(_hardware.amplifier.inputs_with_pullup_option.begin(),
						_hardware.amplifier.inputs_with_pullup_option.end(), pin);
	if (it == _hardware.amplifier.inputs_with_pullup_option.end()) {
		std::string detail = "Amplifier model doesn't support programming pull-up for pin " + std::to_string(pin);
		_status.raise_warning(CopleyStatus::Code::kAmplifierParameterDoesNotExist, detail);
		return;
	}

	// If the enable pull up option is set, write to either the extended (for more than 16 inputs) or regular
	// pull up config register
	CopleyParameter::value_t pullup_state =
		write_get(_hardware.amplifier.num_inputs > 16 ? CopleyParameter::kDigitalInputPullupConfigurationExtended
													  : CopleyParameter::kDigitalInputPullupConfiguration,
				  location);

	// Because not all inputs can programmatically enable the pull-up resistors, the bit to set
	// in the pull-state register is NOT the same as the pin number.
	int pos = it - _hardware.amplifier.inputs_with_pullup_option.begin();
	if (enable_pullup)
		pullup_state |= 1 << pos;
	else
		pullup_state &= ~(1 << pos);
	write_set(_hardware.amplifier.num_inputs > 16 ? CopleyParameter::kDigitalInputPullupConfigurationExtended
												  : CopleyParameter::kDigitalInputPullupConfiguration,
			  pullup_state, location);
}

bool Copley::is_input_index_valid(uint16_t index)
{
	if (index >= _hardware.amplifier.num_inputs) {
		_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
							  "Input pin index " + std::to_string(index) + " was not in the range [0, " +
								  std::to_string(_hardware.amplifier.num_inputs - 1) + "].");
		return false;
	} else {
		return true;
	}
}

bool Copley::input_is_high(uint8_t index)
{
	return is_input_index_valid(index) ? (input_read_raw() & (1 << index)) : false;
}

uint8_t Copley::input_read_raw(void)
{
	return write_get(CopleyParameter::kDigitalInputState);
}

void Copley::write_output_pin_config(const uint8_t pin,
									 const CopleyHardwareSpecification::OutputPin mode,
									 CopleyParameter::MemoryLocation location)
{
	// Copley's documentation on how to configure output pins is very poor and often contradictory.
	// According to the parameter dictionary, this parameter requires 1 to 5 words.
	// According to the ASCII programmer's guide, this paramter requires 3 to 5 words.
	// Neither documents the format of the 2nd through 5th words. The ASCII programmer's guide also
	// indicates that the value "258 0" sets the pin to manual control, active high, despite previously
	// indicating the parameter required 3 words. Empirically, sending two words - the first following
	// the documentation in the paramter dictionary, and the second equal to 0, has been found to work.

	if (driver_status().ok() && is_output_index_valid(pin)) {
		std::vector<int16_t> config_params{
			static_cast<int16_t>(util::to_underlying(mode) |
								 _axis << CopleyHardwareSpecification::kOutputPinConfigAxisOffset),
			0, 0};

		switch (pin) {
		case 0:
			write_set(CopleyParameter::kDigitalOutput0Configuration, config_params, location);
			break;
		case 1:
			write_set(CopleyParameter::kDigitalOutput1Configuration, config_params, location);
			break;
		case 2:
			write_set(CopleyParameter::kDigitalOutput2Configuration, config_params, location);
			break;
		case 3:
			write_set(CopleyParameter::kDigitalOutput3Configuration, config_params, location);
			break;
		case 4:
			write_set(CopleyParameter::kDigitalOutput4Configuration, config_params, location);
			break;
		case 5:
			write_set(CopleyParameter::kDigitalOutput5Configuration, config_params, location);
			break;
		case 6:
			write_set(CopleyParameter::kDigitalOutput6Configuration, config_params, location);
			break;
		default:
			std::string detail = "Invalid output pin index " + std::to_string(static_cast<int>(pin)) +
								 ". Must by between 0 and " + std::to_string(_hardware.amplifier.num_outputs - 1) +
								 " for this amp.";
			_status.raise_warning(CopleyStatus::Code::kAmplifierParameterDoesNotExist, detail);
			break;
		}
	}
}

bool Copley::is_output_index_valid(uint16_t index)
{
	if (index >= _hardware.amplifier.num_outputs) {
		_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
							  "Output pin index " + std::to_string(index) + " was not in the range [0, " +
								  std::to_string(_hardware.amplifier.num_outputs - 1) + "].");
		return false;
	} else {
		return true;
	}
}

bool Copley::output_is_high(uint8_t index)
{
	return is_output_index_valid(index) ? output_read_raw() & (1 << index) : false;
}

void Copley::output_set_high(uint8_t index)
{
	if (is_output_index_valid(index)) {
		uint16_t output_state = output_read_raw();
		output_set_raw(output_state | (1 << index));
	}
}

void Copley::output_set_low(uint8_t index)
{
	if (is_output_index_valid(index)) {
		uint16_t output_state = output_read_raw();
		output_set_raw(output_state & ~(1 << index));
	}
}

void Copley::output_set_raw(uint8_t bitmask)
{
	write_set(CopleyParameter::kDigitalOutputState, bitmask & 0b00000111);
}

uint8_t Copley::output_read_raw(void)
{
	return write_get(CopleyParameter::kDigitalOutputState);
}

void Copley::encoder_reset(bool motor_encoder)
{
	if (!is_connected()) {
		_status.raise_warning(CopleyStatus::Code::kComNotConnected);
	} else if (driver_status().ok() && _command_builder) {
		CopleyCommand command =
			_command_builder->build_encoder_command(CopleyEncoderSubCommand::kResetErrors, motor_encoder);
		write_command(command);
	}
}
void Copley::encoder_zero(bool motor_encoder)
{
	if (!is_connected()) {
		_status.raise_warning(CopleyStatus::Code::kComNotConnected);
	} else if (driver_status().ok() && _command_builder) {
		if (_command_builder->get_protocol_type() == CopleyCommand::Protocol::kBinary) {
			CopleyCommand command =
				_command_builder->build_encoder_command(CopleyEncoderSubCommand::kZeroInternalPosition, motor_encoder);
			write_command(command);
		} else {
			_status.raise_warning(CopleyStatus::Code::kInvalidComProtocol, "Zero encoder command not supported.");
		}
	}
}
void Copley::encoder_set_register(const uint16_t reg, const uint16_t value, bool motor_encoder)
{
	if (!is_connected()) {
		_status.raise_warning(CopleyStatus::Code::kComNotConnected);
	} else if (driver_status().ok() && _command_builder) {
		if (_command_builder->get_protocol_type() == CopleyCommand::Protocol::kBinary) {
			CopleyCommand command = _command_builder->build_encoder_command(CopleyEncoderSubCommand::kSetRegister,
																			motor_encoder, {reg, value});
			write_command(command);
		} else {
			_status.raise_warning(CopleyStatus::Code::kInvalidComProtocol, "Set encoder register not supported.");
		}
	}
}
uint16_t Copley::encoder_read_register(const uint16_t reg, bool motor_encoder)
{
	uint16_t value = 0;

	if (!is_connected()) {
		_status.raise_warning(CopleyStatus::Code::kComNotConnected);
	} else if (driver_status().ok() && _command_builder) {
		if (_command_builder->get_protocol_type() == CopleyCommand::Protocol::kBinary) {
			CopleyCommand command =
				_command_builder->build_encoder_command(CopleyEncoderSubCommand::kReadRegister, motor_encoder, {reg});
			std::unique_ptr<CopleyResponse> resp = write_command(command);
			if (resp->has_numeric_value())
				value = resp->get_value();
		} else {
			_status.raise_warning(CopleyStatus::Code::kInvalidComProtocol, "Read encoder register not supported.");
		}
	}
	return value;
}

void Copley::write_home_trajectory_limits_deg(const double fast_velocity_deg_s,
											  const double slow_velocity_deg_s,
											  const double accel_deg_s2)
{

	if (has_load_encoder_for_feedback()) {
		write_ungeared_home_trajectory_limits_deg(fast_velocity_deg_s, slow_velocity_deg_s, accel_deg_s2);
	} else {
		write_ungeared_home_trajectory_limits_deg(_hardware.gear_ratio * fast_velocity_deg_s,
												  _hardware.gear_ratio * slow_velocity_deg_s,
												  _hardware.gear_ratio * accel_deg_s2);
	}
}

void Copley::write_home_trajectory_defaults(void)
{

	write_ungeared_home_trajectory_limits_deg(
		_hardware.algorithm_default.home_fast_velocity_max_rpm * scale_rpm_to_deg_s,
		_hardware.algorithm_default.home_slow_velocity_max_rpm * scale_rpm_to_deg_s,
		_hardware.algorithm_default.home_accel_max_rps2 * scale_rps_to_deg_s);
}

void Copley::write_ungeared_home_trajectory_limits_deg(const double fast_velocity_deg_s,
													   const double slow_velocity_deg_s,
													   const double accel_deg_s2)
{
	if (driver_status().ok()) {
		// Check and set homing velocity
		if (fast_velocity_deg_s > _hardware.motor.velocity_max_rpm * scale_rpm_to_deg_s ||
			slow_velocity_deg_s > _hardware.motor.velocity_max_rpm * scale_rpm_to_deg_s) {
			_status.raise_warning(CopleyStatus::Code::kInvalidInputData,
								  "requested ungeared homing velocity exceeds motor specified limit of " +
									  std::to_string(_hardware.motor.velocity_max_rpm * scale_rpm_to_deg_s) +
									  " deg/s. Parameter change ignored.");
		} else if (fast_velocity_deg_s > _hardware.algorithm_default.trajectory_velocity_max_rpm * scale_rpm_to_deg_s ||
				   slow_velocity_deg_s > _hardware.algorithm_default.trajectory_velocity_max_rpm * scale_rpm_to_deg_s) {
			_status.raise_warning(
				CopleyStatus::Code::kInvalidInputData,
				"requested ungeared homing velocity exceeds default trajectory control algorithm limit of " +
					std::to_string(_hardware.algorithm_default.trajectory_velocity_max_rpm * scale_rpm_to_deg_s) +
					" deg/s. Parameter change ignored.");
		} else {
			write_set(CopleyParameter::kHomeVelocityFast,
					  static_cast<CopleyParameter::value_t>(
						  deg_to_count(fast_velocity_deg_s, false, !has_encoder_feedback()) * scale_cps_to_raw),
					  CopleyParameter::kHomeVelocityFast.location);
			write_set(CopleyParameter::kHomeVelocitySlow,
					  static_cast<CopleyParameter::value_t>(
						  deg_to_count(slow_velocity_deg_s, false, !has_encoder_feedback()) * scale_cps_to_raw),
					  CopleyParameter::kHomeVelocitySlow.location);
		}

		// Check and set homing acceleration
		if (accel_deg_s2 > _hardware.algorithm_default.trajectory_accel_max_rps2 * scale_rps_to_deg_s) {
			_status.raise_warning(
				CopleyStatus::Code::kInvalidInputData,
				"requested ungeared home acceleration value exceeds calculated trajectory acceleration limit of " +
					std::to_string(_hardware.algorithm_default.trajectory_accel_max_rps2 * scale_rps_to_deg_s) +
					" deg/s^2. Parameter change ignored.");
		} else {
			write_set(CopleyParameter::kHomeAcceleration,
					  static_cast<CopleyParameter::value_t>(deg_to_count(accel_deg_s2, false, !has_encoder_feedback()) *
															scale_cps2_to_raw),
					  CopleyParameter::kHomeAcceleration.location);
		}
	}
}

void Copley::write_pos_software_limit(const double pos_limit_deg)
{
	if (driver_status().ok()) {
		write_set(CopleyParameter::kPositionLimitPositive,
				  static_cast<CopleyParameter::value_t>(
					  deg_to_count(pos_limit_deg, !has_load_encoder_for_feedback(), !has_encoder_feedback())),
				  CopleyParameter::kPositionLimitPositive.location);
	}
}

double Copley::read_pos_software_limit(void)
{
	double limit = 0.0;
	if (driver_status().ok()) {
		limit = count_to_deg(
			write_get(CopleyParameter::kPositionLimitPositive, CopleyParameter::kPositionLimitPositive.location),
			!has_load_encoder_for_feedback(), !has_encoder_feedback());
	}
	return limit;
}

void Copley::write_neg_software_limit(const double neg_limit_deg)
{
	if (driver_status().ok()) {
				write_set(CopleyParameter::kPositionLimitNegative,
				  static_cast<CopleyParameter::value_t>(
					  deg_to_count(neg_limit_deg, !has_load_encoder_for_feedback(), !has_encoder_feedback())),
				  CopleyParameter::kPositionLimitNegative.location);
	}
}

double Copley::read_neg_software_limit(void)
{
	double limit = 0.0;
	if (driver_status().ok()) {
		limit = count_to_deg(
			write_get(CopleyParameter::kPositionLimitNegative, CopleyParameter::kPositionLimitNegative.location),
			!has_load_encoder_for_feedback(), !has_encoder_feedback());
	}
	return limit;
}

void Copley::write_disable_software_limits(void)
{
	if (driver_status().ok()) {
		write_pos_software_limit(0.0);
		write_neg_software_limit(0.0);
	}
}

void Copley::write_software_limit_deceleration_deg_s2(const double deceleration_deg_s2)
{
	if (driver_status().ok()) {
		double decel = deceleration_deg_s2;
		if (!has_load_encoder_for_feedback())
			decel *= _hardware.gear_ratio;

		if (decel > _hardware.algorithm_default.trajectory_accel_max_rps2 * scale_rps_to_deg_s) {
			_status.raise_warning(
				CopleyStatus::Code::kInvalidInputData,
				"requested ungeared acceleration limit of " + std::to_string(decel) +
					" deg/s^2 exceeds calculated trajectory acceleration limit of " +
					std::to_string(_hardware.algorithm_default.trajectory_accel_max_rps2 * scale_rps_to_deg_s) +
					" deg/s^2. Parameter change ignored.");
		} else {
			write_set(CopleyParameter::kPositionLimitDeceleration,
					  static_cast<CopleyParameter::value_t>(deg_to_count(decel, false, !has_encoder_feedback()) *
															scale_cps2_to_raw),
					  CopleyParameter::kPositionLimitDeceleration.location);
		}
	}
}

double Copley::read_software_limit_deceleration_deg_s2(void)
{
	double decel = 0.0;

	if (driver_status().ok()) {
		// Value returned is 10 counts/s^2. See CME Parameter Dictionary.
		decel = scale_raw_to_cps2 * count_to_deg(write_get(CopleyParameter::kPositionLimitDeceleration,
														   CopleyParameter::kPositionLimitDeceleration.location),
												 !has_load_encoder_for_feedback(), !has_encoder_feedback());
	}

	return decel;
}

void Copley::write_velocity_output_filter(const CopleyOutputFilter filter)
{
	if (driver_status().ok()) {
		// If Copley amplifier model AccelNet ACK-090-020
		// is part of a copley chain, writing Velocity Loop Output Filter
		// results in Command Parsing Error. For now, suppress error
		// so other subsystems without model varient can still write output
		// filter.
		_status.suppress_push(CopleyStatus::Code::kAmplifierAsciiParsingError);

		if (_hardware.amplifier.model != CopleyAmplifier::Model::AccelNet_ACK_090_20) {
			write_set(CopleyParameter::kVelocityLoopOutputFilter, filter.filter_params,
					  CopleyParameter::kVelocityLoopOutputFilter.location);
		}

		_status.clear(CopleyStatus::Code::kAmplifierAsciiParsingError);
		_status.suppress_pop(CopleyStatus::Code::kAmplifierAsciiParsingError);
	}
}

void Copley::write_velocity_ouput_filter_default(void)
{
	if (driver_status().ok()) {
		write_velocity_output_filter(_hardware.filter_default);
	}
}

void Copley::write_position_capture_configuration(bool activeRise, bool overwrite)
{
	if (driver_status().ok()) {
		CopleyParameter::value_t capture_config = 0;

		if (overwrite)
			capture_config |=
				static_cast<CopleyParameter::value_t>(PositionCapture::Control::HomeCaptureRewrite::kOverwrite)
				<< PositionCapture::Control::kRewriteHomeOffset;
		else
			capture_config |= static_cast<CopleyParameter::value_t>(PositionCapture::Control::HomeCaptureRewrite::kKeep)
							  << PositionCapture::Control::kRewriteHomeOffset;

		if (activeRise)
			capture_config |=
				static_cast<CopleyParameter::value_t>(PositionCapture::Control::HomeCaptureEdge::kActiveRise)
				<< PositionCapture::Control::kHomeCaptureEdgeOffset;
		else
			capture_config |=
				static_cast<CopleyParameter::value_t>(PositionCapture::Control::HomeCaptureEdge::kActiveFall)
				<< PositionCapture::Control::kHomeCaptureEdgeOffset;

		write_set(CopleyParameter::kPositionCaptureControl, capture_config,
				  CopleyParameter::kPositionCaptureControl.location);

		// To reset the position capture "ready" bit
		read_position_capture_deg();
	}
}

void Copley::write_wiring_and_hall_cofiguration(const CopleyMotor::Wiring motor_wiring,
												const CopleyMotor::HallWiring hall_wiring,
												const CopleyMotor::HallInversion hall_inversion,
												const int32_t hall_offset_deg)
{
	if (driver_status().ok()) {
		write_set(CopleyParameter::kMotorWiring, static_cast<CopleyParameter::value_t>(motor_wiring),
				  CopleyParameter::kMotorWiring.location);
		write_set(CopleyParameter::kMotorHallWiring,
				  util::to_underlying(hall_wiring) | util::to_underlying(hall_inversion),
				  CopleyParameter::kMotorHallWiring.location);
		write_set(CopleyParameter::kMotorHallOffset, static_cast<CopleyParameter::value_t>(hall_offset_deg));
	}
}

bool Copley::position_capture_ready(void)
{
	bool ready = false;

	if (driver_status().ok()) {
		CopleyParameter::value_t capture_status = write_get(CopleyParameter::kPositionCaptureStatus);
		ready = capture_status & (static_cast<CopleyParameter::value_t>(PositionCapture::Status::HomeCaptured::kReady)
								  << PositionCapture::Status::kHomeCapturedOffset);
	}

	return ready;
}

double Copley::read_position_capture_deg(void)
{
	double pos_deg = 0.0;

	if (driver_status().ok()) {
		pos_deg = count_to_deg(write_get(CopleyParameter::kHomePositionCapture), !has_load_encoder_for_feedback(),
							   !has_encoder_feedback());
			}

	return pos_deg;
}

int Copley::trapezoidal_move_calculate_time_ms(const double distance_deg,
											   const double max_velocity_deg_s,
											   const double acceleration_deg_s2)
{
	return abs(1000 * (distance_deg < max_velocity_deg_s * max_velocity_deg_s / acceleration_deg_s2
						   ? 2 * sqrt(distance_deg / acceleration_deg_s2)
						   : distance_deg / max_velocity_deg_s + max_velocity_deg_s / acceleration_deg_s2));
}

void Copley::write_configuration(void)
{
	if (driver_status().ok()) {
		// current control loop gains
		write_current_gains_defaults();
		// current control loop limits
		write_current_limits_defaults();

		// velocity control loop gains
		write_velocity_gains_defaults();
		// velocity control loop limits
		write_velocity_limits_defaults();
		// velocity control loop tracking limits
		write_velocity_window_defaults();
		// velocity loop shift
		write_velocity_loop_shift_defaults();

		// velocity loop output filter
		write_velocity_ouput_filter_default();

		// position control loop gains
		write_position_gains_defaults();

		// position control loop window and tracking limits
		write_position_tracking_limits_defaults();

		// trajectory planning limits
		write_trajectory_limits_defaults();

		// disable software limits (default behavior)
		write_disable_software_limits();

		// homing
		write_home_trajectory_defaults();
	}
}

void Copley::write_hardware_specification(void)
{
	if (driver_status().ok()) {
		// motor configuration
		write_set(CopleyParameter::kMotorType,
				  static_cast<CopleyParameter::value_t>(_hardware.motor.type) |
					  static_cast<CopleyParameter::value_t>(_hardware.motor.architecture),
				  CopleyParameter::kMotorType.location);
		write_set(CopleyParameter::kMotorManufacturer, _hardware.motor.manufacturer,
				  CopleyParameter::kMotorManufacturer.location);
		write_set(CopleyParameter::kMotorModel, _hardware.motor.model_name, CopleyParameter::kMotorModel.location);
		// omitted: motor units (CME2 display only)
		write_set(CopleyParameter::kMotorInertia,
				  static_cast<CopleyParameter::value_t>(_hardware.motor.inertia_Kg_cm2 * scale_kg_cm2_to_raw),
				  CopleyParameter::kMotorInertia.location);
		write_set(CopleyParameter::kMotorPolePairs, static_cast<CopleyParameter::value_t>(_hardware.motor.pole_pairs),
				  CopleyParameter::kMotorPolePairs.location);
		write_set(CopleyParameter::kMotorTorqueConstant,
				  static_cast<CopleyParameter::value_t>(_hardware.motor.torque_constant_Nm_A * scale_Nm_A_to_raw),
				  CopleyParameter::kMotorTorqueConstant.location);
		write_set(CopleyParameter::kMotorResistance,
				  static_cast<CopleyParameter::value_t>(_hardware.motor.resistance_ohm * scale_Ohm_to_raw),
				  CopleyParameter::kMotorResistance.location);
		write_set(CopleyParameter::kMotorInductance,
				  static_cast<CopleyParameter::value_t>(_hardware.motor.inductance_mH * scale_mH_to_raw),
				  CopleyParameter::kMotorInductance.location);
		write_set(CopleyParameter::kMotorPeakTorque,
				  static_cast<CopleyParameter::value_t>(_hardware.motor.torque_peak_Nm * scale_Nm_to_raw),
				  CopleyParameter::kMotorPeakTorque.location);
		write_set(CopleyParameter::kMotorContinuousTorque,
				  static_cast<CopleyParameter::value_t>(_hardware.motor.torque_continuous_Nm * scale_Nm_to_raw),
				  CopleyParameter::kMotorContinuousTorque.location);
		// motor max velocity: convert rpm -> cps -> raw
		write_set(CopleyParameter::kMotorMaxVelocity,
				  static_cast<CopleyParameter::value_t>(
					  deg_to_count(_hardware.motor.velocity_max_rpm * scale_rpm_to_deg_s, false, !has_motor_encoder()) *
					  scale_cps_to_raw),
				  CopleyParameter::kMotorMaxVelocity.location);
		write_set(CopleyParameter::kMotorWiring, static_cast<CopleyParameter::value_t>(_hardware.motor.wiring),
				  CopleyParameter::kMotorWiring.location);
		write_set(CopleyParameter::kMotorHallOffset,
				  static_cast<CopleyParameter::value_t>(_hardware.motor.hall_offset_deg),
				  CopleyParameter::kMotorHallOffset.location);
		write_set(CopleyParameter::kMotorHallType, static_cast<CopleyParameter::value_t>(_hardware.motor.hall_sensor),
				  CopleyParameter::kMotorHallType.location);
		write_set(CopleyParameter::kMotorHallWiring,
				  static_cast<CopleyParameter::value_t>(_hardware.motor.hall_wiring) |
					  static_cast<CopleyParameter::value_t>(_hardware.motor.hall_inversion),
				  CopleyParameter::kMotorHallWiring.location);
		write_set(CopleyParameter::kMotorBackEmfConstant,
				  static_cast<CopleyParameter::value_t>(_hardware.motor.back_emf_V_Krpm * scale_V_krpm_to_raw),
				  CopleyParameter::kMotorBackEmfConstant.location);
		write_set(CopleyParameter::kPhaseMode, static_cast<CopleyParameter::value_t>(_hardware.motor.commutation),
				  CopleyParameter::kPhaseMode.location);

		// motor encoder configuration
		write_set(CopleyParameter::kMotorEncoderType,
				  static_cast<CopleyParameter::value_t>(_hardware.motor_encoder.type),
				  CopleyParameter::kMotorEncoderType.location);
		if (_hardware.motor_encoder.identifier == CopleyMotorEncoder::Identifier::Hall_Feedback) {
			write_set(CopleyParameter::kMotorEncoderCountsPerRevolution,
					  static_cast<CopleyParameter::value_t>(_hardware.motor.pole_pairs) * 6,
					  CopleyParameter::kMotorEncoderCountsPerRevolution.location);
		} else {
			write_set(CopleyParameter::kMotorEncoderCountsPerRevolution,
					  static_cast<CopleyParameter::value_t>(_hardware.motor_encoder.resolution_cpr),
					  CopleyParameter::kMotorEncoderCountsPerRevolution.location);
		}
		write_set(CopleyParameter::kEncoderDirection,
				  static_cast<CopleyParameter::value_t>(_hardware.motor_encoder.direction),
				  CopleyParameter::kEncoderDirection.location);

		// load encoder configuration
		write_set(CopleyParameter::kLoadEncoderType,
				  static_cast<CopleyParameter::value_t>(_hardware.load_encoder.type) +
					  ((_hardware.load_encoder.passive_mode ? 1 : 0) << CopleyLoadEncoder::kPositionFeedbackBitOffset),
				  CopleyParameter::kLoadEncoderType.location);
		write_set(CopleyParameter::kLoadEncoderDirection,
				  static_cast<CopleyParameter::value_t>(_hardware.load_encoder_direction),
				  CopleyParameter::kLoadEncoderDirection.location);

		// From experimentation, when trying to read/write the load encoder options register, the
		// micro module will throw an "Unknown Parameter" error. Could be that because the micro
		// module won't allow non-incremental encoders, they don't allow user to write to register
		if (_hardware.amplifier.identifier != CopleyAmplifier::Identifier::AccelNet_Micro_Module) {
			write_set(CopleyParameter::kLoadEncoderOptions,
					  static_cast<CopleyParameter::value_t>(_hardware.load_encoder.options),
					  CopleyParameter::kLoadEncoderOptions.location);
		}

		// Write a default function to all input pins. User is responsible for writing all input functions.
		for (uint8_t pin = 0; pin < _hardware.amplifier.num_inputs; ++pin)
			write_input_pin_config(pin, CopleyHardwareSpecification::InputPin::kNoFunction, false,
				CopleyParameter::MemoryLocation::kBoth);

		// Write a default function to all output pins. User is response for writing all output functions.
		for (uint8_t pin = 0; pin < _hardware.amplifier.num_outputs; ++pin)
			write_output_pin_config(pin, CopleyHardwareSpecification::OutputPin::kTrackEventStatus,
				CopleyParameter::MemoryLocation::kBoth);
	}
}

int32_t Copley::read_parameter(int parameter_id, bool is_volatile)
{
	const CopleyParameter::MemoryLocation location =
		is_volatile ? CopleyParameter::MemoryLocation::kVolatile : CopleyParameter::MemoryLocation::kNonVolatile;
	const CopleyParameter *const parameter = CopleyParameter::lookup(parameter_id, location);

	return write_get(*parameter, location);
}

CopleyParameter::value_t Copley::write_get(const CopleyParameter &parameter,
										   const CopleyParameter::MemoryLocation location)
{
	CopleyParameter::value_t value = 0;

	if (!is_connected()) {
		_status.raise_warning(CopleyStatus::Code::kComNotConnected);
	} else if (driver_status().ok() && _command_builder) {
		// memory location valid?
		if (static_cast<int>(location) & static_cast<int>(parameter.location)) {

			CopleyCommand command					 = _command_builder->build_get_command(parameter, location);
			std::unique_ptr<CopleyResponse> response = write_command(command);

			if (driver_status().ok() && response) {
				// value returned?
				if (response->get_type() == CopleyResponse::ResponseType::kValue && !parameter.is_string) {
					if (response->has_numeric_value()) {
						value = response->get_value();

						if (command.protocol == CopleyCommand::Protocol::kBinary && parameter.bytes == 2 &&
							parameter.is_signed) {
							// Need to handle the case where binary protocol returns a 16-bit signed integer. The
							// response object doesn't know the details of the paramter so it will always pack the
							// value into 32 bits. This can result in a negative number (in 2s complement form)
							// incorrectly being cast to a large positive value.
							value = static_cast<int16_t>(value);
						}
					}
				}
				// incorrect reponse type
				else if (response->get_type() != CopleyResponse::ResponseType::kError) {
					_status.raise_error(CopleyStatus::Code::kPacketUnknownResponse,
										"unexpected response \"" + response->to_string() + "\".");
				}
			}
		}
		// invalid memory location
		else {
			_status.raise_error(CopleyStatus::Code::kPacketRegisterBankInvalid,
								"attempted to set a parameter in the incorrect memory bank.");
		}
	}

	return value;
}

void Copley::write_set(const CopleyParameter &parameter,
					   const CopleyParameter::value_t value,
					   const CopleyParameter::MemoryLocation location)
{
	if (!is_connected()) {
		_status.raise_warning(CopleyStatus::Code::kComNotConnected);
	} else if (driver_status().ok() && _command_builder) {
		if (parameter.is_string) {
			_status.raise_error(CopleyStatus::Code::kPacketParameterNonNumeric,
								"attempted to write a numeric value to a string parameter.");
		} else if (parameter.access != CopleyParameter::AccessMode::kReadWrite) {
			// Not writable
			_status.raise_error(CopleyStatus::Code::kPacketRegisterNotWritable,
								"attempted to set a read-only parameter.");
		} else {
			if (location == CopleyParameter::MemoryLocation::kBoth) {
				// write to both non-volatile and volatile?
				write_set(parameter, value, CopleyParameter::MemoryLocation::kNonVolatile);
				write_set(parameter, value, CopleyParameter::MemoryLocation::kVolatile);
			} else if (static_cast<int>(location) & static_cast<int>(parameter.location)) {
				// memory location valid
				CopleyCommand command = _command_builder->build_set_command(parameter, location, value);
				write_command(command);
			} else {
				// invalid memory location
				_status.raise_error(CopleyStatus::Code::kPacketRegisterBankInvalid,
									"attempted to set a parameter in the incorrect memory bank.");
			}
		}
	}
}

void Copley::write_set(const CopleyParameter &parameter,
					   const std::string &value,
					   const CopleyParameter::MemoryLocation location)
{
	if (!is_connected()) {
		_status.raise_warning(CopleyStatus::Code::kComNotConnected);
	} else if (driver_status().ok() && _command_builder) {
		// write to both non-volatile and volatile?
		if (location == CopleyParameter::MemoryLocation::kBoth) {
			write_set(parameter, value, CopleyParameter::MemoryLocation::kNonVolatile);
			write_set(parameter, value, CopleyParameter::MemoryLocation::kVolatile);
		}
		// memory location valid?
		else if (static_cast<int>(location) & static_cast<int>(parameter.location)) {
			// writable?
			if (parameter.access == CopleyParameter::AccessMode::kReadWrite) {
				CopleyCommand command = _command_builder->build_set_command(parameter, location, value);
				write_command(command);
			}
			// not writable
			else {
				_status.raise_error(CopleyStatus::Code::kPacketRegisterNotWritable,
									"attempted to set a read-only parameter.");
			}
		}
		// invalid memory location
		else {
			_status.raise_error(CopleyStatus::Code::kPacketRegisterBankInvalid,
								"attempted to set a parameter in the incorrect memory bank.");
		}
	}
}

void Copley::write_set(const CopleyParameter &parameter,
					   const std::vector<int16_t> values,
					   const CopleyParameter::MemoryLocation location)
{
	if (!is_connected()) {
		_status.raise_warning(CopleyStatus::Code::kComNotConnected);
	} else if (driver_status().ok() && _command_builder) {
		// write to both non-volatile and volatile?
		if (location == CopleyParameter::MemoryLocation::kBoth) {
			write_set(parameter, values, CopleyParameter::MemoryLocation::kNonVolatile);
			write_set(parameter, values, CopleyParameter::MemoryLocation::kVolatile);
		}
		// memory location valid?
		else if (static_cast<int>(location) & static_cast<int>(parameter.location)) {
			// writable?
			if (parameter.access == CopleyParameter::AccessMode::kReadWrite) {
				CopleyCommand command = _command_builder->build_set_command(parameter, location, values);
				write_command(command);
			}
			// not writable
			else {
				_status.raise_error(CopleyStatus::Code::kPacketRegisterNotWritable,
									"attempted to set a read-only parameter.");
			}
		}
		// invalid memory location
		else {
			_status.raise_error(CopleyStatus::Code::kPacketRegisterBankInvalid,
								"attempted to set a parameter in the incorrect memory bank.");
		}
	}
}

void Copley::write_trajectory(const TrajectoryCommand trajectory)
{
	if (!is_connected()) {
		_status.raise_warning(CopleyStatus::Code::kComNotConnected);
	} else if (driver_status().ok() && _command_builder) {
		CopleyCommand command = _command_builder->build_trajectory_command(util::to_underlying(trajectory));
		write_command(command);
	}
}

void Copley::write_reset(void)
{
	if (!is_connected()) {
		_status.raise_warning(CopleyStatus::Code::kComNotConnected);
	} else if (driver_status().ok() && _command_builder) {
		// if resetting the serial node, change baud rate back to power-on baud rate
		if (is_serial_node()) {
			_bus->set_baud_rate(CopleyBus::kPowerOnBaudRate);
		}

		// as noted in the Copley ASCII Programming guide, nodes on CAN multi-drop will send
		// a CAN communication error on reset, an error which may be ignored.
		_status.suppress_push(CopleyStatus::Code::kAmplifierCanCommunicationFailure);

		// With Copley model AccelNet ACK-090-020 as part of amplifier chain, the amp
		// is returning undocumented error code 50. Representative from copley has said
		// that error 50 is a timeout but that it is safe to ignore and the amplifier
		// should still successfully reset.
		_status.suppress_push(CopleyStatus::Code::kAmplifierUndocumentedTimeout);

		write_command(_command_builder->build_reset_command());

		// wait for drive to reset
		usleep((is_serial_node() ? kSerialNodeResetDelayMs : kDownstreamNodeResetDelayMs) * 1000);

		// if resetting the master, bus has to be reset as well
		if (is_serial_node()) {
			_bus->reset();
		}

		_status.clear(CopleyStatus::Code::kAmplifierUndocumentedTimeout);
		_status.clear(CopleyStatus::Code::kAmplifierCanCommunicationFailure);
		_status.suppress_pop(CopleyStatus::Code::kAmplifierUndocumentedTimeout);
		_status.suppress_pop(CopleyStatus::Code::kAmplifierCanCommunicationFailure);
	}
}

std::unique_ptr<CopleyResponse> Copley::write_command(const CopleyCommand &command)
{
	std::unique_ptr<CopleyResponse> response;
	if (command.protocol == CopleyCommand::Protocol::kAscii) {
		response = static_cast<std::unique_ptr<CopleyResponse>>(
			std::unique_ptr<CopleyResponseAscii>(new CopleyResponseAscii(_status)));
	} else {
		response = static_cast<std::unique_ptr<CopleyResponse>>(
			std::unique_ptr<CopleyResponseBinary>(new CopleyResponseBinary(_status)));
	}

	// error already occurred?
	if (!driver_status().ok()) {
	}
	// ready to communicate
	else {
		response = _bus->write_command(command, _status);
	}

	return response;
}

std::string Copley::to_string(const DriveMode mode)
{
	switch (mode) {
	case DriveMode::kDisabled:
		return "disabled";
		break;
	case DriveMode::kVelocity:
		return "velocity control";
		break;
	case DriveMode::kPosition:
		return "position control";
		break;
	case DriveMode::kUnknown:
		return "unknown";
		break;
	default:
		return "?";
		break;
	}
}

std::string Copley::to_string(const HomeMethod::Function mode)
{
	switch (mode) {
	case HomeMethod::Function::kHomeAbsolute:
		return "absolute";
		break;
	case HomeMethod::Function::kMoveToHome:
		return "move to home";
		break;
	case HomeMethod::Function::kMoveToLimit:
		return "move to limit";
		break;
	case HomeMethod::Function::kThisIsHome:
		return "this is home";
		break;
	case HomeMethod::Function::kMoveToIndex:
		return "move to index";
		break;
	default:
		return "?";
		break;
	}
}

std::string Copley::to_string(const HomingWindow::Mode mode)
{
	switch (mode) {
	case HomingWindow::Mode::kIdle:
		return "idle";
		break;
	case HomingWindow::Mode::kFirstEdgeSearch:
		return "first edge search";
		break;
	case HomingWindow::Mode::kSecondEdgeSearch:
		return "second edge search";
		break;
	case HomingWindow::Mode::kMovingToHome:
		return "move to home";
		break;
	default:
		return "?";
		break;
	}
}

bool Copley::bit_set(const uint32_t value, const int8_t bit)
{
	return value & (0b1 << bit);
}


} // namespace momentum
