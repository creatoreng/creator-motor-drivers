/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Herkulex communication packet type.
 **/

#pragma once

#include <drivers/herkulex/HerkulexCommand.hpp>
#include <drivers/herkulex/HerkulexStatusRegister.hpp>

#include <deque>
#include <list>

namespace momentum
{

/// Communication packet for communicating with a Herkulex motor.
/// Handles parsing, flatting and unflatting of the packet structure.
/// Use to read and write packets from and to a byte stream.
class HerkulexPacket
{
public:
	/// Response acknowledgement type. These are an interpretation
	/// of the response type and status register.
	enum class AckType {
		kNone,	 ///< Unknown acknowledgement received. Unknown if command executed.
		kAck,	  ///< Positive acknowledgement. Command executed.
		kNack,	 ///< Negative acknowledgement due to error. Command not executed.
		kComError, ///< Possible communication error. Command not executed and may be retried.
	};

	/// Initialization constructor.
	/// @param [in] motor_id The motor ID.
	/// @param [in] command The command for this packet.
	HerkulexPacket(const uint8_t motor_id, const HerkulexCommand command);

	/// Initialization constructor.
	/// @param [in] motor_id The motor ID.
	/// @param [in] command The command for this packet.
	/// @param [in] payload The payload for this packet.
	HerkulexPacket(const uint8_t motor_id, const HerkulexCommand command, const std::deque<uint8_t> &payload);

	/// No argument constructor.
	HerkulexPacket(void);

	/// Resets this packet to default values.
	void clear(void);

	/// Whether or not this packet is an acknowledgement packet.
	/// @return true if an acknowledgement packet.
	bool is_ack(void) const;

	/// Determine if the command was positively acknowledged. Useful to verify a command was
	/// received, or if it was negatively acknowledged, or if acknowledgement is unknown.
	/// @return Acknowledgement type.
	AckType get_ack_type(void) const;

	/// Determine error status of communication errors only. All other errors are ignored.
	/// @return communication error code.
	HerkulexStatus::Code com_error_code(void) const;

	/// Whether or not sending this packet should prompt a response from the motor.
	/// @param [in] ack_policy The acknowledgement policy of the motor.
	/// @return true if an acknowledgement packet is expected.
	bool expects_response(const HerkulexAckPolicy ack_policy) const;

	/// Get the motor ID for this packet.
	/// @return Motor ID.
	uint8_t get_motor_id(void) const;

	/// Sets the motor ID for this packet.
	/// @param [in] motor_id The motor ID.
	void set_motor_id(const uint8_t motor_id);

	/// Sets the number of transmission attempts.
	/// @param [in] Number of transmission attempts needed to deserialize this packet.
	void set_transmission_attempts(const int transmission_attempts);

	/// Gets the number of transmission attempts.
	/// @returns number of transmission attempts needed to deserialize this packet.
	int get_transmission_attempts(void) const;

	/// Get the command for this packet.
	/// @return Command.
	HerkulexCommand get_command(void) const;

	/// Set the command for this packet.
	/// @param [in] command The command.
	void set_command(const HerkulexCommand command);

	/// Get the payload of this packet.
	/// @return Copy of the payload.
	std::deque<uint8_t> get_payload(void) const;

	/// Sets the payload of this packet.
	/// @param [in] payload The payload to copy.
	void set_payload(const std::deque<uint8_t> &payload);

	/// Returns the status registers from an ACK packet
	HerkulexStatusRegister get_status_register(void) const;

	/// Serialize this packet into a byte stream.
	/// @param [in,out] driver_status The driver status to merge.
	/// @return Byte stream containing the complete packet.
	std::deque<uint8_t> serialize(HerkulexStatus &driver_status) const;

	/// Deserialize a byte stream into this packet.
	/// This method may be repeadedly called as bytes arrive on a communication port.
	/// @param [in,out] buffer The byte stream to read. Packet is popped if read.
	/// @param [in,out] driver_status The driver status to merge.
	/// @return true if a packet was extracted.
	bool deserialize(std::deque<uint8_t> &buffer, HerkulexStatus &driver_status);

	/// Maximum size of a packet payload.
	static constexpr uint8_t kPayloadMax = 216;

	/// Smallest size of a complete packet. This is the size of a packet
	/// with no payload.
	/// = header (2) + size(1) + pID (1) + CMD (1) + cxsum (1+1)
	static constexpr uint8_t kBasePacketSize = 7;

	/// Size of an acknowledgement packet, which is base packet size + status registers.
	static constexpr uint8_t kAckPacketSize = kBasePacketSize + 2;

	/// Acknowledge bit that is set in response command IDs
	static constexpr uint8_t kAckMask = 0x40;

	/// Maximum value of a motor ID.
	static constexpr uint8_t kMotorIdMax = 0xFE;

private:
	/// Trim data from a stream that preceedes a header.
	/// Preceeding data is considered garbage and is discarded.
	/// @note No effect if !status()
	/// @param [in,out] buffer The buffer to trim.
	/// @param [in,out] driver_status The driver status to merge.
	/// @return true if the header is found. The first element of the
	///		buffer will be the first byte of the header.
	bool trim_to_head(std::deque<uint8_t> &buffer, HerkulexStatus &driver_status);

	/// Peek to determine the the packet size element of a packet.
	/// @note No effect if !status()
	/// @note It is safe to call this method for any size buffer.
	/// @warning Call deserialize_header() first to trim garbage.
	/// @param [in,out] buffer The aligned buffer containing a packet.
	/// @param [out] size Receives the size of the packet.
	/// @return true if the packet size was extracted and in range.
	bool peek_packet_size(const std::deque<uint8_t> &buffer, uint8_t &size);

	/// Unpack the motor ID from the byte stream.
	/// @param [in,out] value The value of the packet field.
	void deserialize_motor_id(const uint8_t value);

	/// Unpack the command from the byte stream.
	/// @param [in,out] value The value of the packet field.
	void deserialize_command(const uint8_t value);

	/// Unpack the payload from the byte stream.
	/// @warning Does not prevent buffer overrun.
	/// 	Callee must ensure distance( byte, buffer.end() )
	///		is greater than payload size = packet_size - (kPacketSizeMin + 2).
	/// @param [in,out] byte The stream iterator.
	/// @param [in] packet_size Size of the packet. Payload size is a function of this.
	void deserialize_payload(std::deque<uint8_t>::iterator &byte, const uint8_t packet_size);

	/// Unpack the status registers from the byte stream.
	/// @warning Does not prevent buffer overrun.
	/// 	Callee must ensure distance( byte, buffer.end() )
	///		is greater than status size = 2.
	/// @param [in,out] byte The stream iterator.
	void deserialize_status(std::deque<uint8_t>::iterator &byte);

	/// Verify checksums match.
	/// @param [in] packet_begin Iterator at the first element after the header.
	/// @param [in] packet_end Iterator at the character beyond the last byte of the packet; may be end().
	/// @param [in] cxsum1 The first checksum value read from the packet.
	/// @param [in] cxsum2 The second checksum value read from the packet.
	/// @param [in,out] driver_status The driver status to merge.
	void validate_cxsum(std::deque<uint8_t>::iterator packet_begin,
						std::deque<uint8_t>::iterator packet_end,
						const uint8_t cxsum1,
						const uint8_t cxsum2,
						HerkulexStatus &driver_status) const;

	/// Validate all data fields.
	/// @param [in] packet_size The size of the packet, in bytes.
	/// @param [in,out] driver_status The driver status to merge.
	void validate_fields(const uint8_t packet_size, HerkulexStatus &driver_status);

	/// Whether or not this packet is an acknowledgement packet.
	/// Mutable so it can be updated when flattening.
	mutable bool _is_ack;

	/// Motor ID.
	uint8_t _motor_id;

	/// Number of communication attempts needed to receive this packet.
	/// Unused if packet is to be transmitted.
	int _transmission_attempts;

	/// Command
	HerkulexCommand _command;

	/// Payload
	std::deque<uint8_t> _payload{};

	/// Stores the status registers.
	HerkulexStatusRegister _status_register{};
};

} // namespace momentum
