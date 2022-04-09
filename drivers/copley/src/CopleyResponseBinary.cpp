/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/copley/CopleyResponseBinary.hpp>
#include <libs/util/util.hpp>

#include <boost/algorithm/string.hpp>

namespace momentum
{

CopleyResponseBinary::CopleyResponseBinary(CopleyStatus &driver_status)
  : CopleyResponse(driver_status), _num_data_words(0), _error_code(0), _header_received(false), _data()
{
}

void CopleyResponseBinary::add_byte(uint8_t byte)
{
	// Respones for the binary protocol will
	// always include a 4 byte header. Additional
	// data will be recieved right after (if required).

	if (!_response_completed) {
		_full_response.push_back(byte);

		if (_header_received) {
			_data.push_back(byte);
			_response_completed = (_data.size() / 2 == _num_data_words);
		} else if (_full_response.size() == kHeaderSize) {
			parse_header();
		}

		// After receiving full response.
		// Invalid checksum will take precedence over error code since
		// the invalid checksum makes the entire response suspect.
		if (_response_completed) {
			if (!valid_checksum()) {
				_type = ResponseType::kUnknown;
				_status.raise_error(CopleyStatus::Code::kPacketInvalidChecksum);
			} else if (_error_code != kSuccess) {
				_type  = ResponseType::kError;
				_value = _error_code;
			} else if (_num_data_words == 0) {
				_type = ResponseType::kOk;
			} else {
				_type = ResponseType::kValue;
				parse_data();
			}
		}
	} else {
		_status.raise_error(CopleyStatus::Code::kPacketTooMuchData);
	}
}

void CopleyResponseBinary::parse_header(void)
{
	_header_received = true;

	// Header  Structure
	// Byte    Description
	// ---------------------
	//	1      Reserved
	//	2      Checksum
	//	3      Data Size
	//	4      Error Code

	// Amount of data expected after header
	_num_data_words = _full_response[2];
	if (_num_data_words == 0)
		_response_completed = true;

	// Save error code to be checked the same
	// time as checksum to make sure only one
	// error is raised.
	_error_code = _full_response[3];
}

void CopleyResponseBinary::parse_data(void)
{
	// Max length for a numeric value is 4 bytes. Assume anything 4 or less
	// bytes is a numeric value, anything else a string.
	// OK if response is a list of values, the Copley driver will just
	// ask for the raw data.
	if (_data.size() <= 4) {
		_value = 0;
		for (const auto &val : _data)
			_value = (_value << 8) + val;
		_has_numeric_value = true;
	} else {
		// For string values, seems like the amplifier will return the complete
		// register. (i.e. all availables bytes in memory for the string).
		// Results in a significant number of null values returned.
		// When parsing the data, just look for the first null value to mark the end of string.
		std::string data_str(_data.begin(), _data.end());
		_value_string	  = data_str.substr(0, data_str.find_first_of('\0'));
		_has_numeric_value = false;
	}
}

std::string CopleyResponseBinary::to_string(void) const
{
	std::string response_str;
	for (auto byte : _full_response)
		response_str += util::int_to_hex(byte) + " ";
	boost::trim(response_str); // Trim unneeded space

	return response_str;
}

bool CopleyResponseBinary::has_value(void) const
{
	return _num_data_words > 0;
}

bool CopleyResponseBinary::valid_checksum(void) const
{
	// From Copley Control document: Binary Serial Interface v. 1.4, page 3
	// "The checksum is calculated by performing an exclusive OR operation on every byte
	// of the packet (including the header and checksum value). The result should
	// equal the hex constant 5A."
	static constexpr uint8_t kChecksumTarget = 0x5A;

	uint8_t checksum = 0;
	for (const auto byte : _full_response) {
		checksum ^= byte;
	}

	return checksum == kChecksumTarget;
}

} // namespace momentum
