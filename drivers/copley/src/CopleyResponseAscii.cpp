/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/copley/CopleyResponseAscii.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

namespace momentum
{

CopleyResponseAscii::CopleyResponseAscii(CopleyStatus &driver_status) : CopleyResponse(driver_status)
{
}

CopleyResponseAscii::CopleyResponseAscii(const CopleyResponseAscii &obj) : CopleyResponse(obj._status)
{
	_type				= obj._type;
	_has_numeric_value  = obj._has_numeric_value;
	_response_completed = obj._response_completed;
	_full_response		= obj._full_response;
	_value				= obj._value;
	_value_string		= obj._value_string;
}

CopleyResponseAscii &CopleyResponseAscii::operator=(const CopleyResponseAscii &rhs)
{
	_type				= rhs._type;
	_has_numeric_value  = rhs._has_numeric_value;
	_response_completed = rhs._response_completed;
	_full_response		= rhs._full_response;
	_value				= rhs._value;
	_value_string		= rhs._value_string;

	return *this;
}

void CopleyResponseAscii::add_byte(uint8_t byte)
{
	if (!_response_completed) {
		if (byte == static_cast<uint8_t>(kCarriageReturn)) {
			_response_completed = true;
			deserialize();
		} else {
			_full_response.push_back(byte);
		}
	}
}

bool CopleyResponseAscii::has_value(void) const
{
	// all response types except for OK have a single parameter.
	return _type != ResponseType::kOk && _type != ResponseType::kUnknown;
}

void CopleyResponseAscii::deserialize(void)
{
	// no error?
	if (_status.ok()) {
		std::string response(_full_response.begin(), _full_response.end());

		// trim whitespace
		boost::trim(response);

		// split into words
		std::deque<std::string> response_words;
		boost::split(response_words, response, boost::is_any_of(" "));

		deserialize_type(response_words[0]);

		if (_type != ResponseType::kUnknown) {
			// pop type word
			response_words.pop_front();
			deserialize_values(response_words);
		}
	}
}

void CopleyResponseAscii::deserialize_type(std::string type_word)
{
	_type = ResponseType::kUnknown;

	// convert to lowercase
	boost::to_lower(type_word);

	if (type_word == "ok") {
		_type = ResponseType::kOk;
	} else if (type_word == "e") {
		_type = ResponseType::kError;
	} else if (type_word == "v") {
		_type = ResponseType::kValue;
	} else if (type_word == "r") {
		_type = ResponseType::kRegister;
	} else if (type_word.empty()) {
		_status.raise_error(CopleyStatus::Code::kComNotAcknowledged);
	} else {
		_status.raise_error(CopleyStatus::Code::kPacketUnknownResponse);
	}
}

void CopleyResponseAscii::deserialize_values(std::deque<std::string> &values)
{
	_value = 0;
	_value_string.clear();
	_has_numeric_value = false;

	// type has no parameter?
	if (!has_value()) {
		if (values.size() > 0) {
			_status.raise_warning(CopleyStatus::Code::kComDataFlush);
			values.clear();
		}
	}
	// has parameter but none received?
	else if (values.empty()) {
		_status.raise_error(CopleyStatus::Code::kPacketPayloadUndersize, "Expected a value, but no value received.");
	}
	// parameter expected and present
	else {
		// if response size is 1, attempt to read a numeric value
		if (values.size() == 1) {
			// use boost::lexical cast which automatically handles string types, signs, and base prefixes.
			try {
				_value			   = boost::lexical_cast<CopleyParameter::value_t>(values.front());
				_has_numeric_value = true;
			} catch (boost::bad_lexical_cast &) {
				_value			   = 0;
				_has_numeric_value = false;
			}
		}

		while (!values.empty()) {
			_value_string += values.front();
			values.pop_front();
			if (!values.empty()) {
				_value_string += " ";
			}
		}
	}
}

std::string CopleyResponseAscii::to_string(void) const
{
	return {_full_response.begin(), _full_response.end()};
}

} // namespace momentum
