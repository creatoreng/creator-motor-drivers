/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Copley drive event status register.
 **/

#pragma once

#include <drivers/copley/CopleyStatus.hpp>

namespace momentum
{

/// Copley amplifier motor event status register.
/// Container for drive feedback.
class CopleyEventRegister
{
public:
	/// Default constructor.
	CopleyEventRegister(void);

	/// Is the motor moving?
	/// @return true if the event register indidates the motor is moving.
	bool is_moving(void) const;

	/// Is the brake active?
	/// @return true if the event register indicates the motor brake is active.
	bool brake_is_active(void) const;

	/// Is the positive limit switch active?
	/// @return true if the event register indicates the positive limit switch is active.
	bool limit_pos_is_active(void) const;

	/// Is the negative limit switch active?
	/// @return true if the event register indicates the negative limit switch is active.
	bool limit_neg_is_active(void) const;

	/// Is the home switch active?
	/// @return true if the event register indicates the home switch is active.
	bool home_switch_is_active(void) const;

	/// Is software positive limit active?
	/// @return true if the event register indicated the positive software limit is active.
	bool software_limit_pos_is_active(void) const;

	/// Is software negative limit active?
	/// @return true if the event register indicated the negative software limit is active.
	bool software_limit_neg_is_active(void) const;

	/// Set the event register value and update motor status.
	/// Raises appropriate warnings/errors on rising edges of events.
	/// @param [in] event_register Numeric value of event register.
	/// @param [in,out] motor_status Motor status to update.
	/// @return true on the rising edge of a drive latching fault,
	/// indicating the latching fault register should be read.
	bool update(const uint32_t event_register, CopleyStatus &motor_status);

	/// Raise latching errors.
	/// Call when a the event status register reflects a latching drive failure (bit 22).
	/// @param [in] faults_register The value of the latching fault status register.
	/// @param [in,out] motor_status Motor status to update.
	void raise_latching(const uint32_t faults_register, CopleyStatus &motor_status) const;

	/// Reset the event register to its default state (0).
	void reset(void);

	/// Drive latching fault register bit maps.
	/// @see Copley Parameter Dictionary, parameter 0xA4 Latching Fault Status Register
	enum class LatchingFaultRegisterBit : uint32_t {
		kDriveFlashCrcFail			  = 1 << 0,  //!< kDriveFlashCrcFail
		kDriveInternalError			  = 1 << 1,  //!< kDriveInternalError
		kMotorShortCircuit			  = 1 << 2,  //!< kMotorShortCircuit
		kMotorDriveOverTemp			  = 1 << 3,  //!< kMotorDriveOverTemp
		kMotorTempActive			  = 1 << 4,  //!< kMotorTempActive
		kMotorOverVoltage			  = 1 << 5,  //!< kMotorOverVoltage
		kMotorUnderVoltage			  = 1 << 6,  //!< kMotorUnderVoltage
		kDriveFeedbackFault			  = 1 << 7,  //!< kDriveFeedbackFault
		kMotorPhaseError			  = 1 << 8,  //!< kMotorPhaseError
		kMotorTrackingError			  = 1 << 9,  //!< kMotorTrackingError
		kMotorCurrentLimited		  = 1 << 10, //!< kMotorCurrentLimited
		kDriveFpgaErrorType1		  = 1 << 11, //!< kDriveFpgaErrorType1
		kDriveCommandFault			  = 1 << 12, //!< kDriveCommandFault
		kDriveFpgaErrorType2		  = 1 << 13, //!< kDriveFpgaErrorType2
		kDriveSafetyCircuitFault	  = 1 << 14, //!< kDriveSafetyCircuitFault
		kDriveMotorWiringDisconnected = 1 << 15  //!< kDriveMotorWiringDisconnected
	};

private:
	/// Logical AND of a 32-bit register and a latching fault status register bit.
	/// @param [in] faults_register The 32-bit drive latching faults event register.
	/// @param [in] bit The fault bit to check.
	/// @return logical AND of register and fault bit is nonzero.
	static bool is_latching_fault(const uint32_t faults_register, const LatchingFaultRegisterBit fault);

	/// Convert a motor event register bit index to a status code.
	/// @param [in] event_register_bit_index The motor event register bit index to convert.
	/// @return The converted status code.
	static CopleyStatus::Code motor_event_register_bit_to_code(const uint8_t event_register_bit_index);

	/// Offset from a motor event register bit index to a status code.
	static constexpr status_code_t kMotorEventCodeOffset = 3000;

	/// Event register indicates motor is moving?
	bool _is_moving;

	/// Event register indicates brake is active?
	bool _brake_is_active;

	/// Event register indicates positive limit switch is active?
	bool _limit_pos_is_active;

	/// Event register indicates negative limit switch is active?
	bool _limit_neg_is_active;

	/// Event register indicates home switch is active?
	bool _home_switch_is_active;

	/// Event register indicates positive software limit is active
	bool _software_limit_pos_is_active;

	/// Event register indicates negative software limit is active
	bool _software_limit_neg_is_active;

	/// Raw event register value.
	uint32_t _event_register;
};

/// Logical OR of two latching fault register bits.
/// @param [in] lhs drive latching fault register bit left operand.
/// @param [in] rhs drive latching fault register bit right operand.
/// @return logical OR of left and right operands.
CopleyEventRegister::LatchingFaultRegisterBit operator|(const CopleyEventRegister::LatchingFaultRegisterBit lhs,
														const CopleyEventRegister::LatchingFaultRegisterBit rhs);

} // namespace momentum
