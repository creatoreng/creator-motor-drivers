/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Class for building Copley Command for the ASCII protocol.
 **/

#pragma once

#include <drivers/copley/CopleyCommandBuilder.hpp>
#include <drivers/copley/CopleyParameter.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace momentum
{

/// Derived class for creating the command packet to set to
/// a Copley amplifier using ASCII protocol
class CopleyCommandBuilderAscii : public CopleyCommandBuilder
{
public:
	/// Constructor with arguments for initialization
	/// @param [in] node_id The node ID.
	/// @param [in] axis For multi-axis amplifier, which axis is motor?
	CopleyCommandBuilderAscii(const uint8_t node_id, const uint8_t axis = 0);

	/// Destructor
	virtual ~CopleyCommandBuilderAscii(void) = default;

	/// Build command to write a numerical value to the amplifier
	/// @param [in] parameter What Copley parameter to set
	/// @param [in] location Write to flash or RAM
	/// @param [in] value Numeric value to write
	/// @return CopleyCommand ready to send over serial port
	CopleyCommand build_set_command(const CopleyParameter &parameter,
									const CopleyParameter::MemoryLocation location,
									const CopleyParameter::value_t value) override;

	/// Build command to write a string value to the amplifier
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
	/// ASCII characters for Op Code for ASCII based command.
	enum class CommandType : unsigned char {
		kSet		 = 's', ///< Set a value of a paramter in RAM or flash.
		kGet		 = 'g', ///< Read the value of a parameter in RAM or flash.
		kCopy		 = 'c', ///< Copy the value of a parameter from RAM to flash or vice versa.
		kReset		 = 'r', ///< Reset the drive.
		kTrajectory  = 't', ///< Trajectory generator command.
		kCvmRegister = 'i'  ///< Read or write the value of a CVM program register.
	};

	/// Create the command cpoley with all required data
	/// @return Fully formed command
	CopleyCommand build_command(void);

	/// Serialize this command into a stream.
	/// @param [in,out] driver_status The driver status to merge.
	/// @return Stream containing the complete command.
	std::vector<uint8_t> serialize(void) const;

	/// Get the ASCII representation for a Copley operational code
	/// @param [in] code Operation code
	/// @return Character used in ASCII protocol for op code
	uint8_t op_code_to_ascii(OpCode code) const;

	/// Convert numerical axis number to corresponding
	/// axis character for Copley amplifier (e.g. axis 0 -> "A")
	/// @param [in] axis_number Integer axis number (starts at 0)
	/// @return Corresponding axis character for Copley amp
	static std::string axis_letter(const uint8_t axis_number);

	/// Target memory location for command
	CopleyParameter::MemoryLocation _location;

	/// Additional data required for commands
	std::vector<std::string> _data;

	/// Carriage return sequence.
	static constexpr unsigned char kCarriageReturn = '\r';
};

} // namespace momentum
