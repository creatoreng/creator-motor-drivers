/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/copley/CopleyEventRegister.hpp>
#include <drivers/copley/CopleyResponse.hpp>

namespace momentum
{

bool CopleyEventRegister::is_latching_fault(const uint32_t faults_register, const LatchingFaultRegisterBit fault)
{
	return faults_register & static_cast<uint32_t>(fault);
}

CopleyEventRegister::LatchingFaultRegisterBit operator|(const CopleyEventRegister::LatchingFaultRegisterBit lhs,
														const CopleyEventRegister::LatchingFaultRegisterBit rhs)
{
	return static_cast<CopleyEventRegister::LatchingFaultRegisterBit>(static_cast<uint32_t>(lhs) &
																	  static_cast<uint32_t>(rhs));
}

CopleyEventRegister::CopleyEventRegister(void)
  : _is_moving(),
	_brake_is_active(),
	_limit_pos_is_active(),
	_limit_neg_is_active(),
	_home_switch_is_active(),
	_software_limit_pos_is_active(),
	_software_limit_neg_is_active(),
	_event_register()
{
	reset();
}

CopleyStatus::Code CopleyEventRegister::motor_event_register_bit_to_code(const uint8_t status_register_bit_index)
{
	return static_cast<CopleyStatus::Code>(status_register_bit_index + kMotorEventCodeOffset);
}

bool CopleyEventRegister::is_moving(void) const
{
	return _is_moving;
}

bool CopleyEventRegister::brake_is_active(void) const
{
	return _brake_is_active;
}

bool CopleyEventRegister::limit_pos_is_active(void) const
{
	return _limit_pos_is_active;
}

bool CopleyEventRegister::limit_neg_is_active(void) const
{
	return _limit_neg_is_active;
}

bool CopleyEventRegister::home_switch_is_active(void) const
{
	return _home_switch_is_active;
}

bool CopleyEventRegister::software_limit_pos_is_active(void) const
{
	return _software_limit_pos_is_active;
}

bool CopleyEventRegister::software_limit_neg_is_active(void) const
{
	return _software_limit_neg_is_active;
}

bool CopleyEventRegister::update(const uint32_t event_register,
								 CopleyStatus &motor_status)
{
	bool latching_fault = false;

	// compare and selectively handle all status bits.
	for (uint32_t bit_index = 0; bit_index < 32; bit_index++) {
		// convert the bit index into a Copley code.
		const CopleyStatus::Code code = motor_event_register_bit_to_code(bit_index);

		// edge change?
		if ((_event_register ^ event_register) & (1 << bit_index)) {
			bool rising_edge = event_register & (1 << bit_index);

			switch (code) {
			// ignored
			case CopleyStatus::Code::kMotorStopAttempt:
			case CopleyStatus::Code::kMotorPwmOutputDisabled:
			case CopleyStatus::Code::kMotorReset:
			case CopleyStatus::Code::kMotorTrackingWindow:
				break;

			// info - do nothing
			case CopleyStatus::Code::kMotorSoftwareDisable:
			case CopleyStatus::Code::kMotorMoving:
			case CopleyStatus::Code::kMotorLimitPosActive:
			case CopleyStatus::Code::kMotorLimitNegActive:
			case CopleyStatus::Code::kMotorBrakeActive:
			case CopleyStatus::Code::kMotorHomeSwitchActive:
			case CopleyStatus::Code::kMotorSoftwarePosLimit:
			case CopleyStatus::Code::kMotorSoftwareNegLimit:
			case CopleyStatus::Code::kMotorVelocityLimit:
			case CopleyStatus::Code::kMotorAccelerationLimit:

			// warnings
			case CopleyStatus::Code::kMotorEncoderError:
			case CopleyStatus::Code::kMotorHardwareDisable:
			case CopleyStatus::Code::kMotorCurrentLimited:
			case CopleyStatus::Code::kMotorVoltageLimited:
			case CopleyStatus::Code::kMotorTempActive:
			case CopleyStatus::Code::kMotorPositionWrapped:
			case CopleyStatus::Code::kMotorTrackingWarning:
			case CopleyStatus::Code::kMotorVelocityWindow:
				if (rising_edge) {
					motor_status.raise_warning(code);
				}
				break;

			// errors
			case CopleyStatus::Code::kMotorShortCircuit:
			case CopleyStatus::Code::kMotorDriveOverTemp:
			case CopleyStatus::Code::kMotorOverVoltage:
			case CopleyStatus::Code::kMotorUnderVoltage:
			case CopleyStatus::Code::kMotorPhaseError:
			case CopleyStatus::Code::kMotorTrackingError:
			case CopleyStatus::Code::kMotorPhaseNotInit:
			case CopleyStatus::Code::kMotorCommandFault:
			case CopleyStatus::Code::kMotorUndefined:
				if (rising_edge) {
					motor_status.raise_error(code);
				}
				break;
			case CopleyStatus::Code::kMotorDriveFault:
				if (rising_edge) {
					motor_status.raise_error(code);
					latching_fault = true;
				}
				break;

			// unknown
			default:
				if (rising_edge) {
					motor_status.raise_warning(CopleyStatus::Code::kMotorUndefined, "unknown motor event status");
				}
				break;
			}
		}

		// update motor moving status
		if (code == CopleyStatus::Code::kMotorMoving) {
			_is_moving = event_register & (1 << bit_index);
		} else if (code == CopleyStatus::Code::kMotorBrakeActive) {
			_brake_is_active = event_register & (1 << bit_index);
		} else if (code == CopleyStatus::Code::kMotorLimitPosActive) {
			_limit_pos_is_active = event_register & (1 << bit_index);
		} else if (code == CopleyStatus::Code::kMotorLimitNegActive) {
			_limit_neg_is_active = event_register & (1 << bit_index);
		} else if (code == CopleyStatus::Code::kMotorHomeSwitchActive) {
			_home_switch_is_active = event_register & (1 << bit_index);
		} else if (code == CopleyStatus::Code::kMotorSoftwarePosLimit) {
			_software_limit_pos_is_active = event_register & (1 << bit_index);
		} else if (code == CopleyStatus::Code::kMotorSoftwareNegLimit) {
			_software_limit_neg_is_active = event_register & (1 << bit_index);
		}
	}

	_event_register = event_register;
	return latching_fault;
}

void CopleyEventRegister::raise_latching(const uint32_t faults_register, CopleyStatus &motor_status) const
{
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kDriveFlashCrcFail))
		motor_status.raise_error(CopleyStatus::Code::kDriveFlashCrcFail, "Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kDriveInternalError))
		motor_status.raise_error(CopleyStatus::Code::kDriveInternalError, "Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kMotorShortCircuit))
		motor_status.raise_error(CopleyStatus::Code::kMotorShortCircuit, "Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kMotorDriveOverTemp))
		motor_status.raise_error(CopleyStatus::Code::kMotorDriveOverTemp, "Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kMotorTempActive))
		motor_status.raise_error(CopleyStatus::Code::kMotorTempActive, "Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kMotorOverVoltage))
		motor_status.raise_error(CopleyStatus::Code::kMotorOverVoltage, "Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kMotorUnderVoltage))
		motor_status.raise_error(CopleyStatus::Code::kMotorUnderVoltage, "Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kDriveFeedbackFault))
		motor_status.raise_error(CopleyStatus::Code::kDriveFeedbackFault, "Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kMotorPhaseError))
		motor_status.raise_error(CopleyStatus::Code::kMotorPhaseError, "Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kMotorTrackingError))
		motor_status.raise_error(CopleyStatus::Code::kMotorTrackingError, "Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kMotorCurrentLimited))
		motor_status.raise_error(CopleyStatus::Code::kMotorCurrentLimited, "Limited by I2T algoirthm. Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kDriveFpgaErrorType1))
		motor_status.raise_error(CopleyStatus::Code::kDriveFpgaErrorType1, "Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kDriveCommandFault))
		motor_status.raise_error(CopleyStatus::Code::kDriveCommandFault, "Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kDriveFpgaErrorType2))
		motor_status.raise_error(CopleyStatus::Code::kDriveFpgaErrorType2, "Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kDriveSafetyCircuitFault))
		motor_status.raise_error(CopleyStatus::Code::kDriveSafetyCircuitFault, "Latching error.");
	if (is_latching_fault(faults_register, LatchingFaultRegisterBit::kDriveMotorWiringDisconnected))
		motor_status.raise_error(CopleyStatus::Code::kDriveMotorWiringDisconnected, "Latching error.");
}

void CopleyEventRegister::reset(void)
{
	_event_register				  = 0;
	_is_moving					  = false;
	_brake_is_active			  = false;
	_limit_pos_is_active		  = false;
	_limit_neg_is_active		  = false;
	_home_switch_is_active		  = false;
	_software_limit_pos_is_active = false;
	_software_limit_neg_is_active = false;
}

} // namespace momentum
