/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Herkulex error enumerations and strings.
 **/

#pragma once

#include <libs/status/Status.hpp>

namespace momentum {

/// Herkulex status context, which enumerates all errors for this context.
class HerkulexContext {
public:
	/// Herkulex error codes
	enum class Code : status_code_t {
		kOk = 0, ///< OK
		kInvalidConfiguration = 1, ///< Invalid configuration in motor memory.
		kOpenFail = 1000, ///< Unable to open the communicaton port.
		kAlreadyOpen = 1001, ///< Attempted to open a port that had already been opened.
		kNotOpen = 1002, ///< Attempted to access a port that had not yet been opened.
		kNotConnected = 1003, ///< Communication is not established.
		kWriteFail = 1005, ///< Unable to write to the communication port.
		kReadFail = 1006, ///< Unable to read from the communication port.
		kTimeout = 1007, ///< Communication timed out.
		kGarbageAtHost = 1008, ///< Garbage or unexpected data read at host.
		kGarbageAtDevice = 1009, ///< Garbage or unexpected data read at device.
		kChecksumFailAtDevice = 1010, ///< Device reported a failed checksum.
		kChecksumFailAtHost = 1011, ///< Host reported a failed checksum.
		kIncorrectDevice = 1012, ///< Incorrect device specified.
		kAckMismatch = 1013, ///< An acknowledgement was received but did not match the request.
		kDataFlush = 1014, ///< Data was discarded from a communication port.
		kPacketInvalidMotorId = 1500, ///< Invalid motor ID specified.
		kPacketPayloadOversize = 1501, ///< The requested payload of a packet is too large.
		kPacketPayloadSizeIncorrect = 1502, ///< Payload size did not match the request.
		kPacketPayloadOutOfRange = 1503, ///< Value of the payload is out-of-range.
		kPacketUnknownCommand = 1504, ///< An unknown command was received.
		kPacketExceededRegisterRange = 1505, ///< A register operation specified an out-of-range register.
		kPacketPlaytimeBound = 1506, ///< Playtime was decreased to fit within playtime bounds.
		kMotorInputVoltageLimit = 2000, ///< Motor input voltage limit exceeded.
		kMotorPositionLimit = 2001, ///< Motor position limit reached or exceeded.
		kMotorTemperatureLimit = 2002, ///< Motor temperature limit has been exceeded.
		kMotorDriverFault = 2003, ///< Motor driver generated a fault.
		kMotorEepRegisterDistorted = 2004, ///< Motor non-volatile (EEP) memory is corrupt.
		kMotorParameterInvalid = 2005, ///< A motor register or parameter is invalid or out-of-range.
		kMotorModelUnknown = 2006, ///< The motor model is unknown.
		kMotorPositionOutOfBound = 2007, ///< The motor was requested to drive to a position beyond its operating range.
		kMotorVelocityOutOfBound = 2008, ///< The motor was requested to drive at a velocity greater than its operating range.
		kMotorVelocityUnsupported = 2009, ///< The motor model does not support velocity control.
		kMotorTorqueOutOfBound = 2010, ///< The motor was requested to drive at a constant torque, but the requested torque was beyond maximum torque.
		kMotorTorqueUnsupported = 2011, ///< The motor was requested to drive at a constant torque, but the model does not support torque control.
		kMotorStoppedInDeadband = 2012, ///< The motor stopped in its deadband region. At next power-on it will not have absolute position until it moves out of the deadband.
		kMotorTorquePercentOutOfBound = 2013, ///< The torque percent requested is out the allowable range of [0, 100%].
		kMotor2ndPositionUnsupported = 2014, ///< The motor model does not support read of absolute 2nd position.
		kMotorTemperatureReadWriteUnsupported = 2015, ///< The motor model does not support temperature register reads/writes.
		kMotorMaxTemperatureSettingInvalid = 2016, ///< Requested max temperature setting is out of valid range.
		kMotorInvalidSoftwareLimits = 2017, ///< The requested negative software limit was greater than the positive limit.
		kMotorInvalidDeadZone = 2018, ///< Dead zone should be greater than zero.
		kInvalidRegisterValue = 2019, ///< Requested value for register is outside of register range.
	};

	/// Unique name of this context.
	/// @return context name.
	static std::string context(void);

	/// Formatted message for a code.
	/// @param [in] code The status code to format.
	/// @return formatted string.
	static std::string code_text(const Code code);

private:
	/// Name of this object.
	static constexpr const char _context[] = "Herkulex";
};

// Herkulex status types.
using HerkulexStatus = Status<HerkulexContext>;
extern template class Status<HerkulexContext>;

} // namespace momentum
