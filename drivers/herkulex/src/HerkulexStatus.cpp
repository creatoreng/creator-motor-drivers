/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 **/

#include <drivers/herkulex/HerkulexStatus.hpp>

namespace momentum {

// explicit instantiation of HerkulexContext
template class Status<HerkulexContext>;
constexpr const char HerkulexContext::_context[];

std::string HerkulexContext::context(void)
{
	return _context;
}

std::string HerkulexContext::code_text(const Code code)
{
	switch(code) {
		case Code::kOk: return "OK"; break;
		case Code::kInvalidConfiguration: return "Invalid configuration in motor memory."; break;
		case Code::kOpenFail: return "Unable to open the communicaton port."; break;
		case Code::kAlreadyOpen: return "Attempted to open a port that had already been opened."; break;
		case Code::kNotOpen: return "Attempted to access a port that had not yet been opened."; break;
		case Code::kNotConnected: return "Communication is not established."; break;
		case Code::kWriteFail: return "Unable to write to the communication port."; break;
		case Code::kReadFail: return "Unable to read from the communication port."; break;
		case Code::kTimeout: return "Communication timed out."; break;
		case Code::kGarbageAtHost: return "Garbage or unexpected data read at host."; break;
		case Code::kGarbageAtDevice: return "Garbage or unexpected data read at device."; break;
		case Code::kChecksumFailAtDevice: return "Device reported a failed checksum."; break;
		case Code::kChecksumFailAtHost: return "Host reported a failed checksum."; break;
		case Code::kIncorrectDevice: return "Incorrect device specified."; break;
		case Code::kAckMismatch: return "An acknowledgement was received but did not match the request."; break;
		case Code::kDataFlush: return "Data was discarded from a communication port."; break;
		case Code::kPacketInvalidMotorId: return "Invalid motor ID specified."; break;
		case Code::kPacketPayloadOversize: return "The requested payload of a packet is too large."; break;
		case Code::kPacketPayloadSizeIncorrect: return "Payload size did not match the request."; break;
		case Code::kPacketPayloadOutOfRange: return "Value of the payload is out-of-range."; break;
		case Code::kPacketUnknownCommand: return "An unknown command was received."; break;
		case Code::kPacketExceededRegisterRange: return "A register operation specified an out-of-range register."; break;
		case Code::kPacketPlaytimeBound: return "Playtime was decreased to fit within playtime bounds."; break;
		case Code::kMotorInputVoltageLimit: return "Motor input voltage limit exceeded."; break;
		case Code::kMotorPositionLimit: return "Motor position limit reached or exceeded."; break;
		case Code::kMotorTemperatureLimit: return "Motor temperature limit has been exceeded."; break;
		case Code::kMotorDriverFault: return "Motor driver generated a fault."; break;
		case Code::kMotorEepRegisterDistorted: return "Motor non-volatile (EEP) memory is corrupt."; break;
		case Code::kMotorParameterInvalid: return "A motor register or parameter is invalid or out-of-range."; break;
		case Code::kMotorModelUnknown: return "The motor model is unknown."; break;
		case Code::kMotorPositionOutOfBound: return "The motor was requested to drive to a position beyond its operating range."; break;
		case Code::kMotorVelocityOutOfBound: return "The motor was requested to drive at a velocity greater than its operating range."; break;
		case Code::kMotorVelocityUnsupported: return "The motor model does not support velocity control."; break;
		case Code::kMotorTorqueOutOfBound: return "The motor was requested to drive at a constant torque, but the requested torque was beyond maximum torque."; break;
		case Code::kMotorTorqueUnsupported: return "The motor was requested to drive at a constant torque, but the model does not support torque control."; break;
		case Code::kMotorStoppedInDeadband: return "The motor stopped in its deadband region. At next power-on it will not have absolute position until it moves out of the deadband."; break;
		case Code::kMotorTorquePercentOutOfBound: return "The torque percent requested is out the allowable range of [0, 100%]."; break;
		case Code::kMotor2ndPositionUnsupported: return "The motor model does not support read of absolute 2nd position."; break;
		case Code::kMotorTemperatureReadWriteUnsupported: return "The motor model does not support temperature register reads/writes."; break;
		case Code::kMotorMaxTemperatureSettingInvalid: return "Requested max temperature setting is out of valid range."; break;
		case Code::kMotorInvalidSoftwareLimits: return "The requested negative software limit was greater than the positive limit."; break;
		case Code::kMotorInvalidDeadZone: return "Dead zone should be greater than zero."; break;
		case Code::kInvalidRegisterValue: return "Requested value for register is outside of register range."; break;
		default: return "Unknown code."; break;
	}
}

}	// namespace momentum
