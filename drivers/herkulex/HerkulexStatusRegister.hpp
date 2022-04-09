/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Herkulex motor status register.
 **/

#pragma once

#include <drivers/herkulex/HerkulexStatus.hpp>

namespace momentum
{

/// Status registers of a Herkulex motor.
class HerkulexStatusRegister
{
public:
	/// Status register bitfield masks
	enum class StatusFieldMask : uint8_t {
		kExceedInputVoltageLimit = 0x01,
		kExceedPositionLimit	 = 0x02,
		kExceedTemperatureLimit  = 0x04,
		kPacketInvalid			 = 0x08,
		kOverloadDetected		 = 0x10,
		kDriverFault			 = 0x20,
		kEepRegisterDistorted	= 0x40,
		kReserved				 = 0x80
	};

	/// Status detail register bitfield masks
	enum class StatusDetailFieldMask : uint8_t {
		kMoving						 = 0x01,
		kInPosition					 = 0x02,
		kPacketChecksumError		 = 0x04,
		kPacketUnknownCommand		 = 0x08,
		kPacketExceededRegisterRange = 0x10,
		kPacketGarbageDetected		 = 0x20,
		kTorqueOn					 = 0x40,
		kReserved					 = 0x80
	};

	/// Default constructor.
	HerkulexStatusRegister(void);

	/// Initialization constructor.
	/// @param [in] status_error The value of the status error register.
	/// @param [in] status_detail The value of the status detail register.
	explicit HerkulexStatusRegister(const uint8_t status_error, const uint8_t status_detail);

	/// Default destructor.
	~HerkulexStatusRegister(void);

	/// Clears the status
	void clear(void);

	/// Parses the status register and raises errors in a motor status object on rising edges.
	/// @param [in] status_register New status register.
	/// @param [in,out] motor_status The status object that will raise errors if they occur.
	/// @param [in,out] quantities Herkulex quantities to update.
	void update(const HerkulexStatusRegister &status_register,
				HerkulexStatus &motor_status);

	/// Returns the status code for the a

	/// Gets the raw status register.
	/// @return raw status register.
	uint8_t get_raw_status(void) const;

	/// Gets the raw status detail.
	/// @return raw status detail register.
	uint8_t get_raw_status_detail(void) const;

	/// Status indicates an error.
	/// @return true if an error has occurred.
	bool is_error(void) const;

	/// Status indicates exceeded input voltage limit.
	/// @return true if error condition is present.
	bool voltage_limit(void) const;

	/// Status indicates exceeded position limit.
	/// @return true if error condition is present.
	bool position_limit(void) const;

	/// Status indicates exceeded temperature limit.
	/// @return true if error condition is present.
	bool temperature_limit(void) const;

	/// Status indicates invalid packet detected.
	/// @return true if error condition is present.
	/// @note This is a superposition of four error
	/// flags present in status detail.
	bool packet_invalid(void) const;

	/// Status indicates driver fault.
	/// @return true if error condition is present.
	bool driver_fault(void) const;

	/// Status indicates non-volatile EEP register is distorted.
	/// @return true if error condition is present.
	bool eep_register_distorted(void) const;

	/// Status indicates the motor is moving.
	/// @return true if the motor is currently moving.
	bool moving(void) const;

	/// Status indicates the motor is in requested position.
	/// @return true if the motor is currently in position.
	bool in_position(void) const;

	/// Status indicates the motor torque is on.
	/// @return true if the motor torque is on.
	bool torque_on(void) const;

	// -- begin packet errors -- //

	/// Status indicates checksum error.
	/// @return true if error condition is present.
	bool packet_checksum_error(void) const;

	/// Status indicates an unknown command received.
	/// @return true if error condition is present.
	bool packet_unknown_command(void) const;

	/// Status indicates a packet attempted to access an invalid (out-of-range) register.
	/// @return true if error condition is present.
	bool packet_exceeded_register_range(void) const;

	/// Status indicates a packet could not be parsed.
	/// @return true if error condition is present.
	bool packet_garbage_detected(void) const;

	// -- end packet errors -- //

	/// Sets the status without comparing to the previous value.
	/// @param [in] status_error The value of the status error register.
	/// @param [in] status_detail The value of the status detail register.
	void set(const uint8_t status_error, const uint8_t status_detail);

private:
	/// Status error register
	uint8_t _status_error;

	/// Status detail register
	uint8_t _status_detail;
};

} // namespace momentum
