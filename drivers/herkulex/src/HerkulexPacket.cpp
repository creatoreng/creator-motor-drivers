/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/herkulex/HerkulexPacket.hpp>
#include <drivers/herkulex/HerkulexRegister.hpp>

namespace momentum
{

HerkulexPacket::HerkulexPacket(const uint8_t motor_id, const HerkulexCommand command)
  : _is_ack(false), _motor_id(motor_id), _transmission_attempts(0), _command(command)
{
}

HerkulexPacket::HerkulexPacket(const uint8_t motor_id,
							   const HerkulexCommand command,
							   const std::deque<uint8_t> &payload)
  : HerkulexPacket(motor_id, command)
{
	_payload = payload;
}

HerkulexPacket::HerkulexPacket(void) : HerkulexPacket(0xFF, HerkulexCommand::STAT)
{
}

void HerkulexPacket::clear(void)
{
	_is_ack				   = false;
	_transmission_attempts = 0;
	// set motor_id to an invalid ID.
	// This will produce a runtime error in the case the motor ID is not set.
	_motor_id = 0xFF;
	// set to a safe command
	_command = HerkulexCommand::STAT;
	_payload.clear();
	_status_register.clear();
}

bool HerkulexPacket::is_ack(void) const
{
	return _is_ack;
}

HerkulexPacket::AckType HerkulexPacket::get_ack_type(void) const
{
	// not an acknowledgement
	if (!is_ack()) {
		return AckType::kNone;
	}
	// Communication errors that indicate a message was received but not understood, hence not executed.
	else if (com_error_code() != HerkulexStatus::Code::kOk) {
		return AckType::kComError;
	}
	// other error?
	else if (_status_register.is_error()) {
		return AckType::kNack;
	}
	// otherwise a positive acknowledgement.
	else {
		return AckType::kAck;
	}
}

HerkulexStatus::Code HerkulexPacket::com_error_code(void) const
{
	if (_status_register.packet_garbage_detected()) {
		return HerkulexStatus::Code::kGarbageAtDevice;
	} else if (_status_register.packet_checksum_error()) {
		return HerkulexStatus::Code::kChecksumFailAtDevice;
	} else {
		return HerkulexStatus::Code::kOk;
	}
}

bool HerkulexPacket::expects_response(const HerkulexAckPolicy ack_policy) const
{
	// ack policy = all, command = stat, or ack policy = read and command = read
	if ((_command == HerkulexCommand::STAT) || (ack_policy == HerkulexAckPolicy::kReplyAll) ||
		((ack_policy == HerkulexAckPolicy::kReplyRead) &&
		 (_command == HerkulexCommand::EEP_READ || _command == HerkulexCommand::RAM_READ))) {
		return true;
	}

	return false;
}

uint8_t HerkulexPacket::get_motor_id(void) const
{
	return _motor_id;
}

void HerkulexPacket::set_motor_id(const uint8_t motor_id)
{
	_motor_id = motor_id;
}

int HerkulexPacket::get_transmission_attempts(void) const
{
	return _transmission_attempts;
}

void HerkulexPacket::set_transmission_attempts(const int transmission_attempts)
{
	_transmission_attempts = transmission_attempts;
}

HerkulexCommand HerkulexPacket::get_command(void) const
{
	return _command;
}

void HerkulexPacket::set_command(const HerkulexCommand command)
{
	_command = command;
}

std::deque<uint8_t> HerkulexPacket::get_payload(void) const
{
	return _payload;
}

void HerkulexPacket::set_payload(const std::deque<uint8_t> &payload)
{
	_payload = payload;
}

HerkulexStatusRegister HerkulexPacket::get_status_register(void) const
{
	return _status_register;
}

std::deque<uint8_t> HerkulexPacket::serialize(HerkulexStatus &driver_status) const
{
	std::deque<uint8_t> serialized_stream;
	uint8_t checksum = 0;

	// serialize is used only for transmitting, hence this is not an ack packet
	_is_ack = false;

	// motor ID out of range?
	if (_motor_id > HerkulexPacket::kMotorIdMax) {
		driver_status.raise_error(HerkulexStatus::Code::kPacketInvalidMotorId,
								  "Attempted to serialize an invalid motor ID.");
	}
	// invalid data size?
	else if (_payload.size() >= HerkulexPacket::kPayloadMax) {
		driver_status.raise_error(HerkulexStatus::Code::kPacketPayloadOversize,
								  "Attempted to serialize an oversize payload.");
	}
	// no error
	else {
		// serialized_stream size = min_packet_size + data (n)
		const uint8_t packet_size = HerkulexPacket::kBasePacketSize + _payload.size();

		// header { 0xFF, 0xFF }
		serialized_stream.push_back(0xFF);
		serialized_stream.push_back(0xFF);
		// serialized_stream size
		serialized_stream.push_back(packet_size);

		// motor ID
		serialized_stream.push_back(_motor_id);

		// command
		serialized_stream.push_back(static_cast<uint8_t>(_command));

		// first calculate compound XOR, used in both checksum1 and checksum 2
		// checksum = PacketSize ^ pID ^ CMD ^ data[0] ^ ... ^ data[n]
		checksum = packet_size ^ _motor_id ^ (uint8_t)_command;
		for (auto payload_iterator = _payload.begin(); payload_iterator != _payload.end(); payload_iterator++) {
			checksum ^= *payload_iterator;
		}

		// checksum1 = (compound XOR) & 0xFE
		serialized_stream.push_back(checksum & 0xFE);

		// checksum2 = ~(compound XOR) & 0xFE
		serialized_stream.push_back((~checksum) & 0xFE);

		// data
		serialized_stream.insert(serialized_stream.end(), _payload.begin(), _payload.end());
	}

	return serialized_stream;
}

bool HerkulexPacket::deserialize(std::deque<uint8_t> &buffer, HerkulexStatus &driver_status)
{
	uint8_t packet_size = 0;
	uint8_t cxsum1		= 0;
	uint8_t cxsum2		= 0;

	// set is_ack false; it will be set true if the packet is correctly read
	_is_ack = false;

	// error already occurred?
	if (!driver_status.ok()) {
		return false;
	}

	// Advancing to the header removes garbage from the stream.
	if (!trim_to_head(buffer, driver_status)) {
		// header not found
		return false;
	}

	// can we peek ahead to find the packet size?
	if (!peek_packet_size(buffer, packet_size)) {
		// couldn't peek ahead; buffer not yet large enough
		return false;
	}
	// packet size out of range?
	// min = kAckPacketSize
	// max = kBasePacketSize + kPayloadMax
	else if (packet_size < HerkulexPacket::kAckPacketSize ||
			 packet_size > HerkulexPacket::kBasePacketSize + HerkulexPacket::kPayloadMax) {
		// packet size is out of range - this indicates stream
		// is misaligned or has garbage.
		driver_status.raise_warning(HerkulexStatus::Code::kDataFlush);
		// pop first element to force re-aligning of the stream.
		buffer.pop_front();
		return false;
	}
	// is the buffer lage enough to contain a packet?
	else if (buffer.size() < packet_size) {
		return false;
	}

	// Invariant: buffer is aligned with the first byte of the header
	// as the first element, the packet size in range, and the buffer
	// is large enough to contain packet.
	//
	// If the packet fails to read after this point, either the packet is
	// corrupt or the stream is misaligned. Pop the header
	// now, so that if this method fails, on the next execution it will force
	// the packet out of the stream. This is optimal in both execution time
	// and recoverability. Use an iterator from now on and discard data if and only
	// if a packet was succesfully read.
	buffer.pop_front();
	buffer.pop_front();

	// skip over size since we already peeked ahead
	auto byte = buffer.begin() + 1;

	// unflatten the packet
	deserialize_motor_id(*byte++);
	deserialize_command(*byte++);
	cxsum1 = *byte++;
	cxsum2 = *byte++;
	deserialize_payload(byte, packet_size);
	deserialize_status(byte);
	// at this point the iterator has advanced one beyond the last byte of the packet.
	validate_cxsum(buffer.begin(), byte, cxsum1, cxsum2, driver_status);
	validate_fields(packet_size, driver_status);

	if (driver_status.ok()) {
		buffer.erase(buffer.begin(), byte);
		return true;
	}
	return true;
}

bool HerkulexPacket::trim_to_head(std::deque<uint8_t> &buffer, HerkulexStatus &driver_status)
{
	if (driver_status.ok()) {
		// clear any content before the header
		while (!buffer.empty()) {
			// buffer size >= 2, check for header
			if (buffer.size() >= 2) {
				if (buffer.at(0) == 0xFF && buffer.at(1) == 0xFF) {
					// header found
					return true;
				} else {
					// 0xFF found but not proceeded by another 0xFF; discard.
					// This is not the drone you are looking for.
					buffer.pop_front();
					if (driver_status != HerkulexStatus::Code::kDataFlush) {
						driver_status.raise_warning(HerkulexStatus::Code::kDataFlush,
													"Flushed data from serial stream before finding packet header.");
					}
				}
			}
			// buffer size = 1, verify it is the start of the header
			else if (buffer.at(0) != 0xFF) {
				// discard anything before the header
				buffer.pop_front();
				if (driver_status != HerkulexStatus::Code::kDataFlush) {
					driver_status.raise_warning(HerkulexStatus::Code::kDataFlush,
												"Flushed data from serial stream before finding packet header.");
				}
			}
			// buffer size = 1 and buffer[0] = 0xFF
			else {
				return false;
			}
		}
	}

	return false;
}

bool HerkulexPacket::peek_packet_size(const std::deque<uint8_t> &buffer, uint8_t &packet_size)
{
	packet_size = 0;
	// buffer large enough to contain header and size?
	if (buffer.size() >= 3) {
		// verify packet size is in range
		packet_size = buffer.at(2);
		return true;
	}

	return false;
}

void HerkulexPacket::deserialize_motor_id(const uint8_t value)
{
	_motor_id = value;
}

void HerkulexPacket::deserialize_command(const uint8_t value)
{
	const uint8_t command = value;
	_is_ack				  = command & HerkulexPacket::kAckMask;
	// remove acknowldegement mask from the command
	set_command((HerkulexCommand)(command & ~HerkulexPacket::kAckMask));
}

void HerkulexPacket::deserialize_payload(std::deque<uint8_t>::iterator &byte, const uint8_t packet_size)
{
	// payload is a function of packet size
	// subtract acknowledgement packet size
	_payload.clear();
	const int32_t payload_size = packet_size - kAckPacketSize;
	if (payload_size < kPayloadMax && payload_size >= 0) {
		std::copy(byte, byte + payload_size, std::inserter(_payload, _payload.begin()));
		byte += payload_size;
	}
}

void HerkulexPacket::deserialize_status(std::deque<uint8_t>::iterator &byte)
{
	_status_register.clear();
	const uint8_t error_status  = *byte++;
	const uint8_t status_detail = *byte++;
	_status_register.set(error_status, status_detail);
}

void HerkulexPacket::validate_cxsum(std::deque<uint8_t>::iterator packet_begin,
									std::deque<uint8_t>::iterator packet_end,
									const uint8_t cxsum1,
									const uint8_t cxsum2,
									HerkulexStatus &driver_status) const
{
	if (driver_status.ok()) {
		// expected checksum = PacketSize ^ pID ^ CMD ^ data[0] ^ ... ^ data[n]
		uint8_t cxsum_expected = 0xFF;
		for (auto packet_iterator = packet_begin; packet_iterator != packet_end; packet_iterator++) {
			cxsum_expected ^= *packet_iterator;
		}

		// cxsum1 = (compound XOR) & 0xFE
		// cxsum2 = ~(compound XOR) & 0xFE
		if ((cxsum1 != (cxsum_expected & 0xFE)) || (cxsum2 != (~cxsum_expected & 0xFE))) {
			// checksum failed
			driver_status.raise_error(HerkulexStatus::Code::kChecksumFailAtHost);
		}
	}
}

void HerkulexPacket::validate_fields(const uint8_t packet_size, HerkulexStatus &driver_status)
{
	if (driver_status.ok()) {
		const int32_t payload_size = packet_size - kAckPacketSize;

		if (_motor_id > kMotorIdMax) {
			driver_status.raise_error(HerkulexStatus::Code::kGarbageAtHost,
									  "Received a response from an invalid motor ID.");
		} else if (!_is_ack) {
			driver_status.raise_error(HerkulexStatus::Code::kGarbageAtHost,
									  "Deserialize received packet not marked as a command acknowledgement.");
		} else if (payload_size > kPayloadMax) {
			driver_status.raise_error(HerkulexStatus::Code::kGarbageAtHost,
									  "Payload size field is larger than maximum payload.");
		} else if (_status_register.get_raw_status() < HerkulexRegister::kRamStatusError.range.min ||
				   _status_register.get_raw_status() > HerkulexRegister::kRamStatusError.range.max ||
				   _status_register.get_raw_status_detail() < HerkulexRegister::kRamStatusDetail.range.min ||
				   _status_register.get_raw_status_detail() > HerkulexRegister::kRamStatusDetail.range.max) {
			// just a warning here; packet passed checksum so may be ok.
			driver_status.raise_warning(HerkulexStatus::Code::kGarbageAtHost, "Status register values out-of-range.");
		}
	}
}

} // namespace momentum
