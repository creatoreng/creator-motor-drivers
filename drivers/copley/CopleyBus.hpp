/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Low-level command interface for Copley Amplifiers.
 **/

#pragma once

#include <drivers/copley/CopleyCommandBuilder.hpp>
#include <drivers/copley/CopleyResponse.hpp>
#include <drivers/copley/CopleyResponseAscii.hpp>
#include <libserial/SerialPort.h>
#include <libs/util/util.hpp>

namespace momentum
{

/// Low-level register interface to a Copley bus.
/// Allows reading and writing Copley over a standard serial port.
class CopleyBus
{
public:
	/// Construct and subscriber a Logger.
	explicit CopleyBus(void);

	/// Non-copyable.
	CopleyBus(const CopleyBus &) = delete;

	/// Destructor.
	virtual ~CopleyBus(void);

	/// Status of this driver.
	/// @return status.
	const CopleyStatus &status(void) const;

	/// Status string of this driver.
	/// @return status string.
	std::string status_string(void) const;

	/// Attempt to ping a motor before connecting.
	/// @param [in] node_id The node id to ping.
	/// @return true if response received.
	bool ping(const uint8_t node_id);

	/// Connect to the Copley bus. Opens serial port and initializes communication.
	/// @param [in] port_address The device address of the serial port (e.g. /dev/ttyUSB0).
	/// @return true if the communication channel has been opened.
	bool connect(const std::string &port_address, const speed_t baud = kRunningBaudRate);

	/// Disconnect. Clears the communication buffer. Does not notify motors.
	void disconnect(void);

	/// Reset driver. Clears status. If already connected, will disconnect and reconnect.
	void reset(void);

	/// Determine if the communication channel is open and the device is responding.
	/// @return true if connected.
	bool is_connected(void);

	/// Get the response timeout setting.
	/// @return timeout setting, in ms.
	uint32_t get_timeout_ms(void) const;

	/// Set response timeout.
	/// @param [in] timeout_ms The timeout, in ms
	void set_timeout_ms(const uint32_t timeout_ms);

	/// Set transmission attempts. Retransmission occurs
	/// only if the amplifier responds that the previous message
	/// was not correctly received (and hence the command was not
	/// executed).
	///
	/// @note If a timeout occurs waiting for an acknowledgement,
	/// this driver cannot determine if the previous command was
	/// received and executed by the amplifier. If this happpens,
	/// there is a chance the first command was executed but the
	/// acknowledgement was not received, and retransmitting may
	/// result in the command being executed twice with potentially
	/// dangerous results. For example, if drive_relative_deg(45)
	/// is executed but the acknowledgement is lost, retransmitting
	/// drive_relative_deg(45) would result in a net 90 degree
	/// rotation when the application intended only 45 degrees.
	/// If more robust communication is needed, this driver could
	/// implement "safe to retransmit" lists, or it could store
	/// a message counter on the amplifier and update it for each
	/// packet sent (which doubles communication time).
	///
	/// @param [in] transmit_attempts The number of attempts to transmit
	/// a message. A large number of transmission attempts will result in
	/// increased blocking read time, and hence this number should be kept small.
	void set_communication_attempts_max(const uint32_t transmit_attempts);

	/// Change baud rate at run-time.
	void set_baud_rate(const speed_t baud = kRunningBaudRate);

	/// Transmit a command over the serial port.
	/// @note If the amplifier indicates a communication error has occurred
	/// and the command was not executed, the command may be retransmitted
	/// depending on the maximum transmit attempt value.
	/// @param [in] command The command to transmit
	/// @param [in,out] driver_status The status of the driver corresponding to node_id.
	/// @return response (if any)
	std::unique_ptr<CopleyResponse> write_command(const CopleyCommand &command, CopleyStatus &driver_status);

	/// Carriage return sequence.
	static constexpr unsigned char kCarriageReturn = '\r';

	/// Node ID of the serial master.
	static constexpr uint8_t kSerialNodeId = 0x00;

	/// Node ID of the CAN bus master.
	static constexpr uint8_t kCanMasterNodeId = 0x00;

	/// Baud rate of the amplifier at power-on.
	static constexpr speed_t kPowerOnBaudRate = B9600;

	/// Baud rate to communicate after connect.
	static constexpr speed_t kRunningBaudRate = B115200;

	/// Default communication timeout, in ms.
	/// Typical response is only a few characters (round up to 16), power-on baud is 9600 8:1:1 (10 baud / 8 bits)
	/// one-way time = 16 bytes * (8 bits/byte) * (10 baud / 8 bits)  / (9600 baud / s) = 16.667 ms
	/// * 2 for roundtrip
	/// Add a decent buffer to accomodate delays. This value is determined empirically.
	static constexpr uint32_t kDefaultTimeoutMs = 50;

	/// Communication timeout, in ms, for acknowledgement of flash memory writes.
	/// Flash memory writes seem to take longer to acknowledge, hence a longer timeout is warranted.
	/// This value is determined empirically.
	static constexpr uint32_t kNonVolatileWriteTimeoutMs = kDefaultTimeoutMs + 150;

private:
	/// Set port baud rate to power-on default
	/// and attempt to configure baud rate of the amplifier.
	void base_baud_rate(void);

	/// Receive a response from the motor.
	/// @note This is a minimally blocking function.
	/// @note Verify driver status is OK before using response.
	/// @param [in,out] driver_status The status of the motor driver.
	/// @param [in] timeout_ms The maximum amount of time to wait for a response, in ms.
	/// @param [in] type Is the response expected to be ASCII or binary protocol type.
	/// @return Response read from serial port.
	std::unique_ptr<CopleyResponse>
	read_response(CopleyStatus &driver_status, const uint32_t timeout_ms, const CopleyCommand::Protocol type);

	/// Write carriage return characters to the serial port.
	/// Use during connections to unknown or changed baud rates.
	/// @param [in] num_returns The number of carriage returns to transmit.
	/// @param [in] spacing_us The time between successive return transmissions, in us.
	void write_returns(const uint32_t num_returns, const uint32_t spacing_us);

	/// Flush all bytes in the input and output buffers.
	/// Raises a warning if data is lost from the input buffer.
	void flush_buffers(void);

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

	/// Delay after changing baud rate, in ms. See Copley ASCII programmers guide 4.1.
	/// Add a 10% buffer for good measure.
	static constexpr uint32_t kBaudRateChangeDelayMs = 100 * 1.1;

	/// Default number of transmission attempts when an invalid response is received or response times out.
	static constexpr uint32_t kDefaultTransmitAttempts = 2;

	/// Response timeout, in ms.
	uint32_t _communication_timeout_ms;

	/// Maximum packet transmission attempts.
	uint32_t _communication_max_attempts;

	/// POSIX serial port
	LibSerial::SerialPort _port;

	/// POSIX serial port address
	std::string _communication_port_address;

	/// Status of this driver.
	CopleyStatus _status;

	/// Baud rate selected for port communication
	/// Used for re-connection during reset
	speed_t _baud;
};

} // namespace momentum
