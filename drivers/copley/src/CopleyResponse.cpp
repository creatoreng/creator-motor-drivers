/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/copley/CopleyResponse.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>

namespace momentum
{

CopleyResponse::CopleyResponse(CopleyStatus &driver_status)
  : _type(ResponseType::kUnknown),
	_has_numeric_value(false),
	_response_completed(false),
	_full_response(),
	_value(0),
	_value_string(),
	_status(driver_status)
{
}

CopleyResponse::ResponseType CopleyResponse::get_type(void) const
{
	return _type;
}

CopleyResponse::AckType CopleyResponse::get_ack_type(void) const
{
	AckType ack = AckType::kNone;

	switch (_type) {
	// Positive acknowledgement.
	case ResponseType::kOk:
	case ResponseType::kRegister:
	case ResponseType::kValue:
		ack = AckType::kAck;
		break;

	// Error acknowledgement.
	case CopleyResponse::ResponseType::kError: {
		const CopleyStatus::Code amplifier_code = amplifier_error_to_code(_value);

		// Communication errors that indicate a message was received but not understood, hence not executed.
		if (amplifier_code == CopleyStatus::Code::kAmplifierTooMuchData ||
			amplifier_code == CopleyStatus::Code::kAmplifierChecksumError ||
			amplifier_code == CopleyStatus::Code::kAmplifierUnknownCommand ||
			amplifier_code == CopleyStatus::Code::kAmplifierNotEnoughData ||
			amplifier_code == CopleyStatus::Code::kAmplifierTooMuchDataSupplied ||
			amplifier_code == CopleyStatus::Code::kAmplifierIllegalMemoryPage ||
			amplifier_code == CopleyStatus::Code::kAmplifierInvalidOperationMode ||
			amplifier_code == CopleyStatus::Code::kAmplifierUnknownParameterId ||
			amplifier_code == CopleyStatus::Code::kAmplifierDataValueOutOfRange ||
			amplifier_code == CopleyStatus::Code::kAmplifierAttemptToModifyReadOnly ||
			amplifier_code == CopleyStatus::Code::kAmplifierParameterDoesNotExist ||
			amplifier_code == CopleyStatus::Code::kAmplifierIllegalSerialPortForwarding ||
			amplifier_code == CopleyStatus::Code::kAmplifierInvalidNodeForForwarding ||
			amplifier_code == CopleyStatus::Code::kAmplifierCanCommunicationFailure ||
			amplifier_code == CopleyStatus::Code::kAmplifierAsciiParsingError ||
			amplifier_code == CopleyStatus::Code::kAmplifierBadAxisLetter ||
			amplifier_code == CopleyStatus::Code::kAmplifierUnableToCalculateFilter) {
			ack = AckType::kComError;
		}
		// Other negative acknowlement received.
		else {
			ack = AckType::kNack;
		}
		break;
	}

	// Unknown response.
	case ResponseType::kUnknown:
	default:
		ack = AckType::kNone;
		break;
	}

	return ack;
}

CopleyParameter::value_t CopleyResponse::get_value(void) const
{
	return _value;
}

std::string CopleyResponse::get_value_string(void) const
{
	return _value_string;
}

bool CopleyResponse::has_numeric_value(void) const
{
	return _has_numeric_value;
}

bool CopleyResponse::message_is_completed(void) const
{
	return _response_completed;
}

CopleyStatus::Code CopleyResponse::amplifier_error_to_code(const uint32_t amplifier_error)
{
	return static_cast<CopleyStatus::Code>(amplifier_error + kAmplifierCodeOffset);
}

} // namespace momentum
