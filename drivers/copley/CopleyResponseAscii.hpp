/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Copley Amplifier response for ASCII protocol.
 **/

#pragma once

#include <drivers/copley/CopleyParameter.hpp>
#include <drivers/copley/CopleyResponse.hpp>

namespace momentum
{

/// Copley amplifier response for ASCII based communcation.
/// Handles parsing and serializing.
class CopleyResponseAscii : public CopleyResponse
{
public:
	/// Constructor.
	CopleyResponseAscii(CopleyStatus &driver_status);

	/// Copy Constructor.
	CopleyResponseAscii(const CopleyResponseAscii &obj);

	/// Assignment operator
	CopleyResponseAscii &operator=(const CopleyResponseAscii &rhs);

	/// Default Destructor
	~CopleyResponseAscii(void) = default;

	/// Add a new byte to the response. Once
	/// carriage return character is added, message
	/// will be parsed and marked complete.
	/// @param [in] byte New byte to add to repsonse.
	void add_byte(uint8_t byte) override;

	/// Return the response as string (for logging, debugging, etc.)
	/// @return String representation of response
	std::string to_string(void) const override;

	/// Does a response type have a value?
	/// @return true if response type has a value.
	bool has_value(void) const override;

private:
	/// Deserialize a byte stream into this response.
	/// @note This method assumes all communication up to but not including the terminating character is passed in.
	/// @param [in] stream The serial stream to read.
	/// @param [in,out] driver_status The driver status to merge.
	/// @return reference to this response.
	void deserialize(void);

	/// Deserialize the response type from a stream.
	/// @param [in] type_word The response word.
	/// @param [in,out] driver_status The driver status.
	void deserialize_type(std::string type_word);

	/// Deserialize values from a stream.
	/// @param [in] values The values.
	void deserialize_values(std::deque<std::string> &values);

	/// Offset from an amplifier error code to a status code.
	static constexpr status_code_t kAmplifierCodeOffset = 2000;

	/// Character marking the end of an ASCII response
	static constexpr unsigned char kCarriageReturn = '\r';
};

} // namespace momentum
