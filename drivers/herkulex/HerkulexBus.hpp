/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Low-level command interface for Herkulex motors.
 **/

#pragma once

#include <drivers/herkulex/HerkulexPacket.hpp>

#include <libserial/SerialStream.h>
#include <libs/util/util.hpp>

namespace momentum
{

/// Low-level register interface to a Herkulex bus.
/// Allows reading and writing Herkulex over a standard serial port.
class HerkulexBus
{
public:
	/// Construct and subscriber a Logger.
	HerkulexBus(void);

	/// Destructor.
	virtual ~HerkulexBus(void);

	/// Status of this driver.
	/// @return status.
	const HerkulexStatus &status(void) const;

	/// Status string of this driver.
	/// @return status string.
	std::string status_string(void) const;

	/// Attempt to ping a motor before connecting.
	/// @param [in] motor_id The motor id to ping.
	/// @return True if response received.
	bool ping(const uint8_t motor_id);

	/// Connect to the Herkulex bus. Opens serial port and initializes communication.
	/// @param [in] port The device address of the serial port (e.g. /dev/ttyUSB0).
	/// @param [in] baud Communication baud rate.
	/// @return true if the communication channel has been opened.
	bool connect(const std::string &port, const LibSerial::BaudRate baud_rate = kCommunicationBaudDefault);

	/// Disconnect. Clears the communication buffer. Does not notify motors.
	void disconnect(void);

	/// Determine if the communication channel is open and the device is responding.
	/// @return true if connected.
	bool is_connected(void);

	/// Flushes the input buffer, clears the status object, and resets
	/// _communication_timeout_ms and _communication_max_attempts
	/// Does not disconnect/reconnect the bus.
	void reset(void);

	/// Set response timeout.
	/// @param [in] timeout_ms The timeout, in ms.
	void set_timeout_ms(const uint32_t timeout_ms);

	/// Get the number of communication transmit attempts.
	/// @return number of communication transmit attempts.
	uint32_t get_transmit_attempts(void) const;

	/// Set transmission attempts. Retransmission occurs
	/// only if the motor responds that the previous message
	/// was not correctly received (and hence the command was not
	/// executed).
	///
	/// @note If a timeout occurs waiting for an acknowledgement,
	/// this driver cannot determine if the previous command was
	/// received and executed by the motor. If this happpens,
	/// there is a chance the first command was executed but the
	/// acknowledgement was not received, and retransmitting may
	/// result in the command being executed twice with potentially
	/// dangerous results.
	///
	/// @param [in] transmit_attempts The number of attempts to transmit
	/// a message. A large number of transmission attempts will result in
	/// increased blocking read time, and hence this number should be kept small.
	void set_transmit_attempts(const uint32_t transmit_attempts);

	/// Transmit a command over the serial port. Automatically
	/// forms a packet, checksum, etc.
	/// @note If the motor indicates a communication error has occurred
	/// and the command was not executed, the command may be retransmitted
	/// depending on the maximum transmit attempt value.
	/// @param [in] packet The packet to transmit.
	/// @param [in] ack_policy The motor acknowledgement policy.
	/// @param [in,out] driver_status The status of the driver corresponding to motor_id.
	/// @param [in] previous_transmission_attempts The number of previous attempts to transmit this packet
	/// 	without receiving a valid response. Optional. Use when a packet is valid and well-formed but contains
	/// 	invalid data, such as a register read returning the incorrect register address.
	/// @return Response packet. Verify it is an acknowlegement packet, otherwise a default value is returned.
	HerkulexPacket write_packet(const HerkulexPacket &packet,
								const HerkulexAckPolicy ack_policy,
								HerkulexStatus &driver_status,
								const int previous_transmission_attempts = 0);

	/// Updates the bus baud rate
	/// @param [in] baud_rate the new baud rate
	/// @return true if the baud rate was changed successfully
	bool set_baud_rate(const LibSerial::BaudRate baud_rate);

	/// @return current bus baud rate
	LibSerial::BaudRate get_baud_rate(void);

	/// @return current bus baud rate, as an integer
	unsigned int get_baud_rate_int(void);

	/// The Herkulex motor will periodically check its input buffer and clear any incomplete
	/// packages (garbage). The default value of 0x12 (18 * 11.2ms = 201ms) is exceptionally
	/// large and prevents timely retransmission attempts. This parameter sets the garbage
	/// check period to a smaller value that is used when attempting to retransmit.
	/// This number is in units of device "tick periods", where each tick period is 11.2ms (defined in HerkulexModel).
	static constexpr uint8_t kDeviceGarbageCollectionTickPeriods = 1;

	/// Default baud rate.
	static constexpr LibSerial::BaudRate kCommunicationBaudDefault = LibSerial::BaudRate::BAUD_115200;

private:
	/// Receive an acknowledgement or response from the serial port.
	/// This is a minimally blocking function.
	/// @param [in] motor_id The motor ID that is expected to send the packet.
	/// @param [in] command The expected command acknowledgement to receive.
	/// @param [in,out] read_status The status result of the read.
	/// @return packet read.
	HerkulexPacket read_packet(const uint8_t motor_id, const HerkulexCommand command, HerkulexStatus &read_status);

	/// Write raw data to the serial port.
	/// @param [in] data The data to write.
	void write_raw(const std::deque<uint8_t> &data);

	/// Flush all bytes in the input stream.
	/// Raises a warning if data is lost.
	void flush_input(void);

	/// Converts POSIX speed_t to a baud rate, as an integer.  The values of the
	/// constants for speed_t are not themselves portable.
	/// @param [in] speed POSIX speed type
	/// @return integer baud rate
	static int speed_to_baud(const speed_t speed);

	/// Converts a baud rate (as an integer) to a POSIX speed_t. If the baud rate is not
	/// defined in termios.h, then return the next valid baud less than the desired baud rate.
	/// @param [in] baud integer baud rate
	/// @return POSIX speed type
	static speed_t baud_to_speed(const unsigned int baud);

	/// Default number of transmission attempts when an invalid response is received or response times out.
	static constexpr uint32_t kDefaultTransmitAttempts = 5;

	/// Default communication timeout, in ms.
	/// Assume slowest baud rate of 57600.
	/// ACK packet is 15 bytes (round up to 16), baud is 57600 8:1:1 (10 baud / 8 bits)
	/// transmit time = 16 bytes * (8 bits/byte) * (10 baud / 8 bits)  / (57600 baud / s) = 2.776 ms.
	/// Double for minimum round-trip time, 5.552ms.
	/// In practice 10ms works well in many situations but on larger motor chains timeouts still occur.
	/// So even after all of that math, this value does not apparently account for the behavior of
	/// the device. Empirically we have found that for up to 16 motors, a timeout value of 25ms is reliable.
	/// @wtf Dongbu Robotics?
	static constexpr uint32_t kDefaultTimeoutMs = 25;

	/// Motor ID for broadcast messages.
	static constexpr uint8_t kBroadcastId = 0xFE;

	/// Response timeout, in ms.
	uint32_t _communication_timeout_ms;

	/// Time at given baud rate for a byte to be received, in us.
	/// transmit time = 1 byte * (8 bits/byte) * (10 baud / 8 bits) / (baud / s) * 1e6 us / s
	uint32_t _communication_byte_transmit_time_us;

	/// Maximum packet transmission attempts.
	uint32_t _communication_max_attempts;

	/// POSIX serial port
	LibSerial::SerialStream _port;

	/// Status of this driver.
	HerkulexStatus _status;
};

} // namespace momentum
