/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Abstract Class for building Copley Command. (could be ASCII or Binary command protocol)
 **/

#pragma once

#include <drivers/copley/CopleyParameter.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace momentum
{

/// Available sub-commands for Copley Encoder Command
enum class CopleyEncoderSubCommand : uint8_t {
	kReadRegister		  = 0,
	kResetErrors		  = 1,
	kZeroInternalPosition = 2,
	kSetRegister		  = 4
};

/// Object to represent a single CopleyCommand
/// Contains the data to send over the serial port
/// and some additional information relating to command.
struct CopleyCommand {
	/// Two protocol options for the serial communication
	enum class Protocol { kAscii, kBinary };

	/// Packet of data to send to Copley amplifier
	std::vector<uint8_t> packet;

	/// True if command is expecting data back from amplifier
	bool expects_response;

	/// True if writing to flash, false if ram
	/// Used to determine communication timeout on write
	bool write_to_flash;

	/// Is the command using ascii or binary protocol
	Protocol protocol;

	CopleyCommand(void) : packet(), expects_response(false), write_to_flash(false), protocol(Protocol::kAscii)
	{
	}
};

/// Abstract class for creating the command packet to send over
/// the serial port to communicate with a Copley amplifier
class CopleyCommandBuilder
{
public:
	/// Constructor
	/// @param [in] node_id The node id of amp receiving command
	/// @param [in] axis For multi-axis amplifier, which axis is motor?
	CopleyCommandBuilder(const uint8_t node_id, const uint8_t axis);

	/// Destructor
	virtual ~CopleyCommandBuilder(void) = default;

	/// Build command to write a numerical value to the amplifier
	/// @param [in] parameter What Copley parameter to set
	/// @param [in] location Write to flash or RAM
	/// @param [in] value Numeric value to write
	/// @return CopleyCommand ready to send over serial port
	virtual CopleyCommand build_set_command(const CopleyParameter &parameter,
											const CopleyParameter::MemoryLocation location,
											const CopleyParameter::value_t value) = 0;

	/// Build command to write a string value to the amplifier
	/// @param [in] parameter What Copley parameter to set
	/// @param [in] location Write to flash or RAM
	/// @param [in] value String value to write
	/// @return CopleyCommand ready to send over serial port
	virtual CopleyCommand build_set_command(const CopleyParameter &parameter,
											const CopleyParameter::MemoryLocation location,
											const std::string &value) = 0;

	/// Build command to write to registers that aren't strings or single values
	/// An exmple is the Velocity Loop Output filter which is a register of
	/// 9 or 14 word parameters.
	/// @param [in] parameter What Copley parameter to set
	/// @param [in] location Write to flash or RAM
	/// @param [in] words Vector of 16 bit words to write
	/// @return CopleyCommand ready to send over serial port
	virtual CopleyCommand build_set_command(const CopleyParameter &parameter,
											const CopleyParameter::MemoryLocation location,
											const std::vector<int16_t> values) = 0;

	/// Build command to read a value of an amplifier register
	/// @param [in] parameter What Copley parameter to read
	/// @param [in] location Read from flash or RAM
	/// @return CopleyCommand ready to send over serial port
	virtual CopleyCommand build_get_command(const CopleyParameter &parameter,
											const CopleyParameter::MemoryLocation location) = 0;

	/// Build command to copy a variable from one bank to another
	/// @param [in] driver_status Reference to status for error handling
	/// @param [in] parameter What parameter to copy
	/// @param [in] location_from Bank to read variable FROM
	/// @return CopleyCommand ready to send over serial port
	virtual CopleyCommand build_copy_command(const CopleyParameter &parameter,
											 const CopleyParameter::MemoryLocation location_from) = 0;

	/// Build command to reset the amplifier immediately
	/// @return CopleyCommand ready to send over serial port
	virtual CopleyCommand build_reset_command(void) = 0;

	/// Build command to send to trajectory generator
	/// @param [in] trajectory The trajectory sub-command
	/// @return CopleyCommand ready to send over serial port
	virtual CopleyCommand build_trajectory_command(const CopleyParameter::value_t trajectory) = 0;

	/// Build command to perform special functions with compatible absolute encoder
	/// @param [in] sub_cmd What function to request for encoder
	/// @param [in] motor_encoder Is encoder configured as motor or load encoder
	/// @param [in] data Additional data to append to command (if applicable)
	virtual CopleyCommand build_encoder_command(const CopleyEncoderSubCommand sub_cmd,
												bool motor_encoder,
												std::vector<uint16_t> data = {}) = 0;

	/// @return Protocol type for builder object.
	virtual CopleyCommand::Protocol get_protocol_type(void) const = 0;

protected:
	/// Copley communication op-code.
	/// @note Not all operations are currently supported by command builder -- psherman (04/27/2020)
	enum class OpCode : uint8_t {
		kNop			= 0,  ///< No operation is performed.
		kOperatingMode  = 7,  ///< Returns current amp operating mode
		kGetFlashCrc	= 10, ///< Get 32-bit CRC value calculated by amp
		kSwapMode		= 11, ///< Switch between normal and boot mode
		kGet			= 12, ///< Read the value of a parameter in RAM or flash.
		kSet			= 13, ///< Write Value of a parameter on amp
		kCopy			= 14, ///< Copy the value of a parameter from RAM to flash or vice versa.
		kTrace			= 15, ///< Access amp trace feature
		kReset			= 16, ///< Reset the drive.
		kTrajectory		= 17, ///< Trajectory generator command.
		kErrorLog		= 18, ///< Access amp error log
		kVirtualMachine = 20, ///< Handles communicating with CVM
		kEncoderCommand = 27, ///< Perform special functions with absolute encoder
		kGetCanObj		= 28, ///< Read value of object in CANopen object dict
		kSetCanObj		= 29, ///< Set of the value of object in CANopen dict
	};

	/// Does this command type expect a response?
	/// @return True if the amplifier is expected to respond to this command.
	bool expects_response(void) const;

	/// Code for what type of command to build
	OpCode _op_code;

	/// Node ID
	const uint16_t _node_id;

	/// Axis number
	const uint8_t _axis;
};

} // namespace momentum
