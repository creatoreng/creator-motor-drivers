/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/herkulex/HerkulexStatusRegister.hpp>

namespace momentum
{

HerkulexStatusRegister::HerkulexStatusRegister(void) : _status_error(), _status_detail()
{
	clear();
}

HerkulexStatusRegister::~HerkulexStatusRegister(void)
{
}

HerkulexStatusRegister::HerkulexStatusRegister(const uint8_t status_error, const uint8_t status_detail)
  : _status_error(status_error), _status_detail(status_detail)
{
}

void HerkulexStatusRegister::clear(void)
{
	set(0x00, 0x00);
}

void HerkulexStatusRegister::update(const HerkulexStatusRegister &status_register,
									HerkulexStatus &motor_status)
{
	// notify on rising edge of errors and warnings
	// note: more than one error may occur; raise all that are present.
	if (!voltage_limit() && status_register.voltage_limit()) {
		motor_status.raise_error(HerkulexStatus::Code::kMotorInputVoltageLimit);
	}
	if (!position_limit() && status_register.position_limit()) {
		// position limit is a warning only
		motor_status.raise_warning(HerkulexStatus::Code::kMotorPositionLimit);
	}
	if (!temperature_limit() && status_register.temperature_limit()) {
		motor_status.raise_error(HerkulexStatus::Code::kMotorTemperatureLimit);
	}
	if (!driver_fault() && status_register.driver_fault()) {
		motor_status.raise_error(HerkulexStatus::Code::kMotorDriverFault);
	}
	if (!eep_register_distorted() && status_register.eep_register_distorted()) {
		motor_status.raise_error(HerkulexStatus::Code::kMotorEepRegisterDistorted);
	}
	if (!packet_invalid() && status_register.packet_invalid()) {
		// logic errors - these indicate the host sent an invalid command
		if (!packet_unknown_command() && status_register.packet_unknown_command()) {
			motor_status.raise_error(HerkulexStatus::Code::kPacketUnknownCommand);
		}
		if (!packet_exceeded_register_range() && status_register.packet_exceeded_register_range()) {
			motor_status.raise_error(HerkulexStatus::Code::kPacketExceededRegisterRange);
		}

		// communucation error bits - since this driver uses an acknowledge all policy,
		// these should raise warnings but not errors, since errors will present
		// in an inability to acknowledge.
		// FIXME: Ignored for now until the driver resets these bits after being read.
		// Otherwise they are persistent.
		if (!packet_checksum_error() && status_register.packet_checksum_error()) {
			// motor_status.raise_warning( HerkulexStatus::Code::kChecksumFailAtDevice );
		}
		if (!packet_garbage_detected() && status_register.packet_garbage_detected()) {
			// motor_status.raise_warning( HerkulexStatus::Code::kGarbageAtDevice );
		}
	}

	set(status_register.get_raw_status(), status_register.get_raw_status_detail());
}

void HerkulexStatusRegister::set(const uint8_t status_error, const uint8_t status_detail)
{
	_status_error  = status_error;
	_status_detail = status_detail;
}

uint8_t HerkulexStatusRegister::get_raw_status(void) const
{
	return _status_error;
}

uint8_t HerkulexStatusRegister::get_raw_status_detail(void) const
{
	return _status_detail;
}

bool HerkulexStatusRegister::is_error(void) const
{
	// status register is nonzero iff an error has occurred,
	// with the exception of position error which this driver ignores.
	return _status_error ^ static_cast<uint8_t>(StatusFieldMask::kExceedPositionLimit);
}

bool HerkulexStatusRegister::voltage_limit(void) const
{
	return _status_error & static_cast<uint8_t>(StatusFieldMask::kExceedInputVoltageLimit);
}

bool HerkulexStatusRegister::position_limit(void) const
{
	return _status_error & static_cast<uint8_t>(StatusFieldMask::kExceedPositionLimit);
}

bool HerkulexStatusRegister::temperature_limit(void) const
{
	return _status_error & static_cast<uint8_t>(StatusFieldMask::kExceedTemperatureLimit);
}

bool HerkulexStatusRegister::packet_invalid(void) const
{
	return _status_error & static_cast<uint8_t>(StatusFieldMask::kPacketInvalid);
}

bool HerkulexStatusRegister::driver_fault(void) const
{
	return _status_error & static_cast<uint8_t>(StatusFieldMask::kDriverFault);
}

bool HerkulexStatusRegister::eep_register_distorted(void) const
{
	return _status_error & static_cast<uint8_t>(StatusFieldMask::kEepRegisterDistorted);
}

bool HerkulexStatusRegister::moving(void) const
{
	return _status_detail & static_cast<uint8_t>(StatusDetailFieldMask::kMoving);
}

bool HerkulexStatusRegister::in_position(void) const
{
	return _status_detail & static_cast<uint8_t>(StatusDetailFieldMask::kInPosition);
}

bool HerkulexStatusRegister::packet_checksum_error(void) const
{
	return _status_detail & static_cast<uint8_t>(StatusDetailFieldMask::kPacketChecksumError);
}

bool HerkulexStatusRegister::packet_unknown_command(void) const
{
	return _status_detail & static_cast<uint8_t>(StatusDetailFieldMask::kPacketUnknownCommand);
}

bool HerkulexStatusRegister::packet_exceeded_register_range(void) const
{
	return _status_detail & static_cast<uint8_t>(StatusDetailFieldMask::kPacketExceededRegisterRange);
}

bool HerkulexStatusRegister::packet_garbage_detected(void) const
{
	return _status_detail & static_cast<uint8_t>(StatusDetailFieldMask::kPacketGarbageDetected);
}

bool HerkulexStatusRegister::torque_on(void) const
{
	return _status_detail & static_cast<uint8_t>(StatusDetailFieldMask::kTorqueOn);
}

} // namespace momentum
