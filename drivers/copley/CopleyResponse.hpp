/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Copley Amplifier response type.
 **/

#pragma once

#include <drivers/copley/CopleyParameter.hpp>
#include <drivers/copley/CopleyStatus.hpp>

#include <deque>
#include <string>

namespace momentum
{

/// Copley amplifier response. Handles parsing and serializing.
class CopleyResponse
{
public:
	/// Copley response types.
	enum class ResponseType {
		kUnknown = 0, ///< Unknown response type.
		kOk,		  ///< Ok.
		kError,		  ///< Error.
		kValue,		  ///< Value returned.
		kRegister,	///< Contents of a register read.
	};

	/// Response acknowledgement type. These are an interpretation
	/// of the response type and value.
	enum class AckType {
		kNone,	 ///< Unknown acknowledgement received. Unknown if command executed.
		kAck,	  ///< Positive acknowledgement. Command executed.
		kNack,	 ///< Negative acknowledgement due to error. Command not executed.
		kComError, ///< Possible communication error. Command not executed and may be retried.
	};

	/// Constructor.
	CopleyResponse(CopleyStatus &driver_status);

	/// Desctructor
	virtual ~CopleyResponse(void) = default;

	/// Add a new byte to the response. Each child class will
	/// be response for parsing as bytes are recieved and marking
	/// as completed when response ready.
	/// @param [in] byte New byte to add to the full response
	virtual void add_byte(uint8_t byte) = 0;

	/// Get the response type for this response.
	/// @return Response type.
	ResponseType get_type(void) const;

	/// Determine if the command was positively acknowledged. Useful to verify a command was
	/// received, or if it was negatively acknowledged, or if acknowledgement is unknown.
	/// @return Acknowledgement type.
	AckType get_ack_type(void) const;

	/// Has a numeric value?
	/// @return true if value exists and is numeric.
	bool has_numeric_value(void) const;

	/// Has a full message been completed?
	/// @return True if a full response received
	bool message_is_completed(void) const;

	/// Does a response type have a value?
	/// @return True if response type has a value.
	virtual bool has_value(void) const = 0;

	/// Value of this response.
	/// @note If no numeric value exists, zero is returned.
	/// @return Value.
	CopleyParameter::value_t get_value(void) const;

	/// Value of the response (if applicable) as a string
	/// @return Value in string format.
	std::string get_value_string(void) const;

	/// Convert an amplifier error code to a status code.
	/// @param [in] amplifier_error The amplifier code to convert.
	/// @return The converted status code.
	static CopleyStatus::Code amplifier_error_to_code(const uint32_t amplifier_error);

	/// Return the response as string (for logging, debugging, etc.)
	/// @return String representation of response
	virtual std::string to_string(void) const = 0;

protected:
	/// Response type.
	ResponseType _type;

	/// Parameter exists and is numeric (otherwise string).
	bool _has_numeric_value;

	/// Has a full response already loaded
	bool _response_completed;

	/// Response recieved from Copley
	std::vector<uint8_t> _full_response;

	/// If respose value numeric type
	CopleyParameter::value_t _value;

	/// If respose value is a string
	std::string _value_string;

	/// Status for Copley
	CopleyStatus &_status;

	/// Offset from an amplifier error code to a status code.
	static constexpr status_code_t kAmplifierCodeOffset = 2000;
};

} // namespace momentum
