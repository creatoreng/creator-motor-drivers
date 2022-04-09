/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/copley/CopleyCommandBuilderBinary.hpp>

#include <libs/util/util.hpp>

namespace momentum
{

CopleyCommandBuilderBinary::CopleyCommandBuilderBinary(const uint8_t node_id, const uint8_t axis)
  : CopleyCommandBuilder(node_id, axis), _data()
{
}

CopleyCommand CopleyCommandBuilderBinary::build_set_command(const CopleyParameter &parameter,
															const CopleyParameter::MemoryLocation location,
															const CopleyParameter::value_t value)
{
	_op_code = OpCode::kSet;
	_data	= create_variable_id(parameter, location);

	// Numeric values can be 2 or 4 bytes depending on parameter.
	// Values are added most significant byte first.
	for (int ii = parameter.bytes - 1; ii >= 0; --ii) {
		_data.push_back(static_cast<uint8_t>(value >> 8 * ii) & 0xFF);
	}

	return build_command(location);
}

CopleyCommand CopleyCommandBuilderBinary::build_set_command(const CopleyParameter &parameter,
															const CopleyParameter::MemoryLocation location,
															const std::string &value_string)
{
	_op_code = OpCode::kSet;
	_data	= create_variable_id(parameter, location);

	// Create bytes from string.
	for (auto it = value_string.cbegin(); it != value_string.cend(); ++it) {
		_data.push_back(*it);
	}

	// Length needs to be a multiple of 2
	if (_data.size() % 2 != 0)
		_data.push_back(0);

	return build_command(location);
}

CopleyCommand CopleyCommandBuilderBinary::build_set_command(const CopleyParameter &parameter,
															const CopleyParameter::MemoryLocation location,
															const std::vector<int16_t> words)
{
	_op_code = OpCode::kSet;
	_data	= create_variable_id(parameter, location);

	for (const auto &w : words) {
		_data.push_back(static_cast<uint8_t>((w >> 8) & 0xFF)); // Upper byte
		_data.push_back(static_cast<uint8_t>(w & 0xFF));		// Lower byte
	}

	return build_command(location);
}

CopleyCommand CopleyCommandBuilderBinary::build_get_command(const CopleyParameter &parameter,
															const CopleyParameter::MemoryLocation location)
{
	_op_code = OpCode::kGet;
	_data	= create_variable_id(parameter, location);

	return build_command(location);
}

CopleyCommand CopleyCommandBuilderBinary::build_copy_command(const CopleyParameter &parameter,
															 const CopleyParameter::MemoryLocation location_from)
{
	_op_code = OpCode::kCopy;
	_data	= create_variable_id(parameter, location_from);

	return build_command(location_from);
}

CopleyCommand CopleyCommandBuilderBinary::build_reset_command(void)
{
	_op_code = OpCode::kReset;
	_data.clear();

	return build_command();
}

CopleyCommand CopleyCommandBuilderBinary::build_trajectory_command(const CopleyParameter::value_t trajectory)
{
	_op_code = OpCode::kTrajectory;

	//	Bits	Description
	//	0-3		Trajectory sub-command
	//	4-11	Reserved for future use
	//	12		If set, apply command to axis 1
	//	13		If set, apply command to axis 2
	//	14		If set, apply command to axis 3
	//	15		If set, apply command to axis 4
	// If none of bits 12-15 are set, command is applied to axis 1 by default
	static constexpr uint8_t kSubCommandBitMask = 0b1111;
	static constexpr uint8_t kAxisOffset		= 4; // Offset from start of 2nd byte

	// From experimentation, the micro module doesn't like packet with an
	// axis bit set. For convenience, if building for command to first axis
	uint8_t axis = 0;
	if (_axis > 0) {
		axis = 0b1 << (kAxisOffset + _axis);
	}

	uint8_t sub_cmd = static_cast<uint8_t>(trajectory) & kSubCommandBitMask;

	_data = {axis, sub_cmd};
	return build_command();
}

CopleyCommand CopleyCommandBuilderBinary::build_encoder_command(const CopleyEncoderSubCommand sub_cmd,
																bool motor_encoder,
																std::vector<uint16_t> data)
{
	_op_code = OpCode::kEncoderCommand;

	// First Word for Encoder
	//	Bits	Description
	//	0       Clear for motor encoder, set for load encoder
	//	4-7     Sub Command
	//	12-13   Axis number
	static constexpr uint8_t kSubCmdOffset = 4;
	static constexpr uint8_t kAxisOffset   = 4; // Offset from start of 2nd byte

	uint8_t low_byte;
	low_byte = motor_encoder ? 0 : 1;
	low_byte += util::to_underlying(sub_cmd) << kSubCmdOffset;

	uint8_t high_byte = _axis << kAxisOffset;
	_data			  = {high_byte, low_byte};

	// If Read or write, expect additional data
	if (sub_cmd == CopleyEncoderSubCommand::kReadRegister || sub_cmd == CopleyEncoderSubCommand::kSetRegister) {
		for (const auto &d : data) {
			_data.push_back(static_cast<uint8_t>((d >> 8) & 0xFF)); // Upper byte
			_data.push_back(static_cast<uint8_t>(d & 0xFF));		// Lower byte
		}
	}

	return build_command();
}

CopleyCommand CopleyCommandBuilderBinary::build_command(const CopleyParameter::MemoryLocation location) const
{
	CopleyCommand command;

	command.packet = create_header();
	for (const auto &byte : _data)
		command.packet.push_back(byte);

	command.expects_response = expects_response();
	command.write_to_flash   = location == CopleyParameter::MemoryLocation::kNonVolatile;
	command.protocol		 = CopleyCommand::Protocol::kBinary;

	return command;
}

std::vector<uint8_t> CopleyCommandBuilderBinary::create_variable_id(const CopleyParameter &parameter,
																	const CopleyParameter::MemoryLocation location)
{
	//	Bits	Description
	//	0-8		Variable ID
	//	9-11	Reserved for future use
	//	12		Memory location (0:ram, 1:flash)
	//	13-15	Axis Number (0 for axis 1, 1 for axis 2, etc.)
	static constexpr int kMemoryLocationOffset = 12;
	static constexpr int kAxisNumberOffset	 = 13;

	uint16_t id = parameter.id;
	if (location == CopleyParameter::MemoryLocation::kNonVolatile)
		id += 0b1 << kMemoryLocationOffset;
	id += _axis << kAxisNumberOffset;

	return {static_cast<uint8_t>((id & 0xFF00) >> 8), static_cast<uint8_t>(id & 0x00FF)};
}

std::vector<uint8_t> CopleyCommandBuilderBinary::create_header(void) const
{
	// From Copley Control document: Binary serial interface v. 1.4 page 2-3
	// Header consists of following bytes in order:
	// 1. Node Number
	// 2. Checksum
	// 3. Data Size - number of 16 bit words in data
	// 4. Op Code

	std::vector<uint8_t> header;

	header.push_back(_node_id + kHeaderNodeOffset);
	header.push_back(calculate_checksum());
	header.push_back(_data.size() / 2);
	header.push_back(util::to_underlying(_op_code));

	return header;
}

uint8_t CopleyCommandBuilderBinary::calculate_checksum(void) const
{
	// From Copley Control document: Binary Serial Interface v. 1.4 page 3
	// "The checksum is calculated by performing an exclusive OR operation on every byte
	// of the command packet (including the header and checksum value). The result should
	// equal the hex constant 5A."
	static constexpr uint8_t kChecksumTarget = 0x5A;

	// From Header
	uint8_t xor_total = (_node_id + kHeaderNodeOffset) ^ util::to_underlying(_op_code) ^ (_data.size() / 2);

	// From Data
	for (const auto &byte : _data) {
		xor_total ^= byte;
	}

	return xor_total ^ kChecksumTarget;
}

CopleyCommand::Protocol CopleyCommandBuilderBinary::get_protocol_type(void) const
{
	return CopleyCommand::Protocol::kBinary;
}

} // namespace momentum
