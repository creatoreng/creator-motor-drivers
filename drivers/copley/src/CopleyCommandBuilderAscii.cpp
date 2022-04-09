/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/copley/CopleyCommandBuilderAscii.hpp>

#include <libs/util/util.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

namespace momentum
{

CopleyCommandBuilderAscii::CopleyCommandBuilderAscii(const uint8_t node_id, const uint8_t axis)
  : CopleyCommandBuilder(node_id, axis), _location(CopleyParameter::MemoryLocation::kVolatile), _data()
{
}

CopleyCommand CopleyCommandBuilderAscii::build_set_command(const CopleyParameter &parameter,
														   const CopleyParameter::MemoryLocation location,
														   const CopleyParameter::value_t value)
{
	return build_set_command(parameter, location, std::to_string(value));
}

CopleyCommand CopleyCommandBuilderAscii::build_set_command(const CopleyParameter &parameter,
														   const CopleyParameter::MemoryLocation location,
														   const std::string &value_string)
{
	_op_code  = OpCode::kSet;
	_location = location;

	_data.clear();
	_data.push_back(parameter.to_string(location)); // [memory bank (r/f)][parameter ID]
	_data.push_back(value_string);					// [value]

	return build_command();
}

CopleyCommand CopleyCommandBuilderAscii::build_set_command(const CopleyParameter &parameter,
														   const CopleyParameter::MemoryLocation location,
														   const std::vector<int16_t> words)
{
	_op_code  = OpCode::kSet;
	_location = location;

	_data.clear();
	_data.push_back(parameter.to_string(location));

	for (const auto &w : words) {
		_data.push_back(std::to_string(w));
	}

	return build_command();
}

CopleyCommand CopleyCommandBuilderAscii::build_get_command(const CopleyParameter &parameter,
														   const CopleyParameter::MemoryLocation location)
{
	_op_code  = OpCode::kGet;
	_location = location;

	_data.clear();
	_data.push_back(parameter.to_string(location)); // [memory bank (r/f)][parameter ID]

	return build_command();
}

CopleyCommand CopleyCommandBuilderAscii::build_copy_command(const CopleyParameter &parameter,
															const CopleyParameter::MemoryLocation location_from)
{
	_op_code  = OpCode::kCopy;
	_location = location_from;

	_data.clear();
	_data.push_back(parameter.to_string(location_from)); // [memory bank (r/f)][parameter ID]

	return build_command();
}

CopleyCommand CopleyCommandBuilderAscii::build_reset_command(void)
{
	_op_code  = OpCode::kReset;
	_location = CopleyParameter::MemoryLocation::kVolatile;
	_data.clear();

	return build_command();
}

CopleyCommand CopleyCommandBuilderAscii::build_trajectory_command(const CopleyParameter::value_t trajectory)
{
	_op_code  = OpCode::kTrajectory;
	_location = CopleyParameter::MemoryLocation::kVolatile;

	_data.clear();
	_data.push_back(std::to_string(trajectory));

	return build_command();
}

CopleyCommand CopleyCommandBuilderAscii::build_encoder_command(const CopleyEncoderSubCommand sub_cmd,
															   bool motor_encoder,
															   std::vector<uint16_t> data)
{
	CopleyCommand command;

	// Ascii protocol only supports a ability to clear encoder errors
	if (sub_cmd == CopleyEncoderSubCommand::kResetErrors) {
		std::string str = motor_encoder ? "enc clear" : "ldenc clear";
		str += kCarriageReturn;

		command.packet			 = std::vector<uint8_t>(str.begin(), str.end());
		command.expects_response = expects_response();
		command.write_to_flash   = false;
		command.protocol		 = CopleyCommand::Protocol::kAscii;
	}

	return command;
}

CopleyCommand CopleyCommandBuilderAscii::build_command(void)
{
	CopleyCommand command;
	command.packet			 = serialize();
	command.expects_response = expects_response();
	command.write_to_flash   = _location == CopleyParameter::MemoryLocation::kNonVolatile;
	command.protocol		 = CopleyCommand::Protocol::kAscii;

	return command;
}

std::vector<uint8_t> CopleyCommandBuilderAscii::serialize(void) const
{
	std::string serialized_stream;

	// format: [node ID][<.>axis letter] [command code] [command parameters]<CR>
	serialized_stream += std::to_string(_node_id);
	if (_axis > 0 && _op_code != OpCode::kReset) {
		// The axis letter has no effect with reset command because
		// the reset command applies to all axes of a multi-axis drive.
		serialized_stream += ".";
		serialized_stream += axis_letter(_axis);
	}
	serialized_stream += " ";
	serialized_stream += op_code_to_ascii(_op_code);
	serialized_stream += " ";
	for (const auto &d : _data) {
		serialized_stream += d;
		serialized_stream += " ";
	}
	serialized_stream.erase(serialized_stream.end() - 1); // erase trailing " "
	serialized_stream += kCarriageReturn;

	return std::vector<uint8_t>(serialized_stream.begin(), serialized_stream.end());
}

uint8_t CopleyCommandBuilderAscii::op_code_to_ascii(OpCode code) const
{
	switch (code) {
	case OpCode::kSet:
		return util::to_underlying(CommandType::kSet);
	case OpCode::kGet:
		return util::to_underlying(CommandType::kGet);
	case OpCode::kCopy:
		return util::to_underlying(CommandType::kCopy);
	case OpCode::kReset:
		return util::to_underlying(CommandType::kReset);
	case OpCode::kTrajectory:
		return util::to_underlying(CommandType::kTrajectory);
	case OpCode::kVirtualMachine:
		return util::to_underlying(CommandType::kCvmRegister);

	// To avoid compiler warning
	case OpCode::kOperatingMode:
	case OpCode::kGetFlashCrc:
	case OpCode::kSwapMode:
	case OpCode::kTrace:
	case OpCode::kErrorLog:
	case OpCode::kEncoderCommand:
	case OpCode::kGetCanObj:
	case OpCode::kSetCanObj:
	default:
		break;
	}

	return 0;
}

std::string CopleyCommandBuilderAscii::axis_letter(const uint8_t axis_number)
{
	switch (axis_number) {
	case 0:
		return "a";
	case 1:
		return "b";
	case 2:
		return "c";
	case 3:
		return "d";
	}

	return "";
}

CopleyCommand::Protocol CopleyCommandBuilderAscii::get_protocol_type(void) const
{
	return CopleyCommand::Protocol::kAscii;
}

} // namespace momentum
