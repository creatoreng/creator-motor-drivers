/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Builder class for Copley Amplifier commands using binary protocol.
 **/

#pragma once

#include <drivers/copley/CopleyCommandBuilder.hpp>

#include <string>
#include <vector>

namespace momentum
{

/// Copley amplifier command for binary communicatio protocol.
/// Handles parsing and serializing for single command.
/// All details from Copley Controls: Binary Serial Interface document
class CopleyCommandBuilderBinary : public CopleyCommandBuilder
{
public:
	/// Initialize constuctor.
	/// @param [in] node_id The node ID.
	/// @param [in] driver_status Status to raise error if invalid node_id
	/// @param [in] axis For multi-axis amplifier, which axis is motor?
	CopleyCommandBuilderBinary(const uint8_t node_id, const uint8_t axis = 0);

	/// Destructor
	virtual ~CopleyCommandBuilderBinary(void) = default;

	/// Build command to write a numerical value to the amplifier
	/// @param [in] parameter What Copley parameter to set
	/// @param [in] location Write to flash or RAM
	/// @param [in] value Numeric value to write
	/// @return CopleyCommand ready to send over serial port
	CopleyCommand build_set_command(const CopleyParameter &parameter,
									const CopleyParameter::MemoryLocation location,
									const CopleyParameter::value_t value) override;

	/// Build command to write a string value to the amplifier
	/// @param [in] driver_status Reference to status for error handling
	/// @param [in] parameter What Copley parameter to set
	/// @param [in] location Write to flash or RAM
	/// @param [in] value String value to write
	/// @return CopleyCommand ready to send over serial port
	CopleyCommand build_set_command(const CopleyParameter &parameter,
									const CopleyParameter::MemoryLocation location,
									const std::string &value_string) override;

	/// Build command to write to registers that aren't strings or single values
	/// An exmple is the Velocity Loop Output filter which is a register of
	/// 9 or 14 word parameter.
	/// @param [in] parameter What Copley parameter to set
	/// @param [in] location Write to flash or RAM
	/// @param [in] words Vector of 16 bit words to write
	/// @return CopleyCommand ready to send over serial port
	CopleyCommand build_set_command(const CopleyParameter &parameter,
									const CopleyParameter::MemoryLocation location,
									const std::vector<int16_t> words) override;

	/// Build command to read a value from an amplifier register
	/// @param [in] parameter What Copley parameter to read
	/// @param [in] location Read from flash or RAM
	/// @return CopleyCommand ready to send over serial port
	CopleyCommand build_get_command(const CopleyParameter &parameter,
									const CopleyParameter::MemoryLocation location) override;

	/// Build command to copy a variable from one bank to another
	/// @param [in] driver_status Reference to status for error handling
	/// @param [in] parameter What parameter to copy
	/// @param [in] location_from Ban to read variable FROM
	/// @return CopleyCommand ready to send over serial port
	CopleyCommand build_copy_command(const CopleyParameter &parameter,
									 const CopleyParameter::MemoryLocation location_from) override;

	/// Build command to reset the amplifier immediately
	/// @return CopleyCommand ready to send over serial port
	CopleyCommand build_reset_command(void) override;

	/// Build command to send to trajectory generator
	/// @param [in] trajectory The trajectory sub-command
	/// @return CopleyCommand ready to send over serial port
	CopleyCommand build_trajectory_command(const CopleyParameter::value_t trajectory) override;

	/// Build command to perform special functions with compatible absolute encoder
	/// @param sub_cmd What function to request for encoder
	/// @param motor_encoder Is encoder configured as motor or load encoder
	/// @param data Additional data to append to command (if applicable)
	CopleyCommand build_encoder_command(const CopleyEncoderSubCommand sub_cmd,
										bool motor_encoder,
										std::vector<uint16_t> data = {}) override;

	/// @return Protocol type for builder object.
	CopleyCommand::Protocol get_protocol_type(void) const override;

private:
	/// Serialize this command into a stream.
	/// @param [in,out] driver_status The driver status to merge.
	/// @param [in] location Memory location for command
	/// @return Object containing the complete command.
	CopleyCommand
	build_command(const CopleyParameter::MemoryLocation location = CopleyParameter::MemoryLocation::kVolatile) const;

	/// Create a variable identifier used in the data packet for multiple commands.
	std::vector<uint8_t> create_variable_id(const CopleyParameter &parameter,
											const CopleyParameter::MemoryLocation location);

	/// Build the 4 byte header required for all binary commands.
	/// See Copley Control document: Binary Serial Interface v. 1.4 page 2
	/// @return Vector of length 4 with header filled
	std::vector<uint8_t> create_header(void) const;

	/// Calcuate the checksum for Copley communication header
	/// @return Calculated checksum
	uint8_t calculate_checksum(void) const;

	/// Additional data required for commands
	std::vector<uint8_t> _data;

	/// From Copley Control document: Binary Serial Interface v. 1.4 page 2
	/// " ...The value passed in this byte should always be either zero
	/// (when not using multi-drop communication), or equal to the value 0x80
	/// plus the node ID of the destination drive."
	static constexpr uint8_t kHeaderNodeOffset = 0x80;
};

} // namespace momentum
