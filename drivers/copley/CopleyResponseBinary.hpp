/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Copley Amplifier response for Binary protocol.
 **/

#pragma once

#include <drivers/copley/CopleyParameter.hpp>
#include <drivers/copley/CopleyResponse.hpp>

namespace momentum
{

/// Copley amplifier response for binary communication protocol.
/// Handles parsing of message
class CopleyResponseBinary : public CopleyResponse
{
public:
	/// Constuctor.
	/// @param [in] driver_status Reference to motor status obejct
	CopleyResponseBinary(CopleyStatus &driver_status);

	/// Default destructor
	~CopleyResponseBinary(void) = default;

	/// Add new byte to the response. First 4 bytes are header
	/// containing length of message. Once full message received,
	/// data will be parsed.
	/// @param [in] byte New byte to add to response.
	void add_byte(uint8_t byte) override;

	/// Return the response as string (for logging, debugging, etc.)
	/// For binary response, will create a string of hex values.
	/// @return String representation of response
	std::string to_string(void) const override;

	/// Does a response type have a value?
	/// @return True if response type has a value.
	bool has_value(void) const override;

private:
	/// Validate the checksum of the entire response
	/// @return True if checksum calculation succeededs
	bool valid_checksum(void) const;

	/// Parse header data immediately after
	/// recieving the 4th byte.
	void parse_header(void);

	/// Use the data received as part of the response to
	/// set the value and/or value string
	void parse_data(void);

	/// Number of 16 bit words following header
	uint8_t _num_data_words;

	/// Error code from response
	uint8_t _error_code;

	/// Has the full header been received?
	bool _header_received;

	/// Data from response (full response - header)
	std::vector<uint8_t> _data;

	/// Size of response header in bytes
	static constexpr uint8_t kHeaderSize = 4;

	/// Error code will be zero on successs, specific error code otherwise
	static constexpr uint8_t kSuccess = 0;
};

} // namespace momentum
