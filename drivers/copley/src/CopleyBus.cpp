/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/copley/Copley.hpp>
#include <drivers/copley/CopleyBus.hpp>
#include <drivers/copley/CopleyCommandBuilderAscii.hpp>
#include <drivers/copley/CopleyResponseAscii.hpp>
#include <drivers/copley/CopleyResponseBinary.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/chrono.hpp>
#include <boost/timer/timer.hpp>

#include <array>
#include <iterator>
#include <thread>

namespace momentum
{

CopleyBus::CopleyBus(void)
  : _communication_timeout_ms(kDefaultTimeoutMs),
	_communication_max_attempts(kDefaultTransmitAttempts),
	_port(),
	_communication_port_address(""),
	_status(),
	_baud(kPowerOnBaudRate)
{
}

CopleyBus::~CopleyBus(void)
{
	disconnect();
}

const CopleyStatus &CopleyBus::status(void) const
{
	return _status;
}

std::string CopleyBus::status_string(void) const
{
	return _status.to_string();
}

uint32_t CopleyBus::get_timeout_ms(void) const
{
	return _communication_timeout_ms;
}

void CopleyBus::set_timeout_ms(const uint32_t timeout_ms)
{
	_communication_timeout_ms = timeout_ms;
	}

void CopleyBus::set_communication_attempts_max(const uint32_t transmit_attempts)
{
	_communication_max_attempts = transmit_attempts;
	}

bool CopleyBus::ping(const uint8_t node_id)
{
	bool ping_success = false;

	if (_status.ok()) {
		CopleyStatus motor_status;
		if (node_id <= Copley::kNodeIdMax) {
			CopleyCommandBuilderAscii cmd_builder(node_id);
			const CopleyCommand command = cmd_builder.build_get_command(CopleyParameter::kDriveEventStatus,
																		CopleyParameter::MemoryLocation::kVolatile);

			// attempt to read status register
      // temporarily suppress reporting of all motor errors.
			auto scoped_suppression = motor_status.suppress_all();
			write_command(command, motor_status);
			ping_success = motor_status.ok();
		} else {
			motor_status.raise_error(CopleyStatus::Code::kPacketInvalidMotorId,
									 "Attempted to ping motor with invalid node ID.");
		}
	}

	return ping_success;
}

bool CopleyBus::connect(const std::string &port_address, const speed_t baud)
{
	// error already occurred?
	if (!_status.ok()) {
	}
	// already open?
	else if (is_connected() && port_address == _communication_port_address) {
	}
	// trying to open a different port before first disconnecting?
	else if (is_connected()) {
		_status.raise_error(CopleyStatus::Code::kComAlreadyOpen);
	}
	// no error
	else {
		_communication_port_address = port_address;
		// catch ostream exceptions
		try {
			_port.Open(_communication_port_address);

			// should be open here.
			if (_port.IsOpen()) {
				// configure the baud rate
				base_baud_rate();
				set_baud_rate(baud);
				_baud = baud;
			} else {
				_status.raise_error(CopleyStatus::Code::kComOpenFail);
			}
		} catch (...) {
			_status.raise_error(CopleyStatus::Code::kComOpenFail, "unhandled I/O exception.");
		}
	}
	return is_connected();
}

void CopleyBus::disconnect(void)
{
	if (is_connected()) {
		set_baud_rate(kPowerOnBaudRate); // for easier restart, reset baud rate to power-on mode
		_port.Close();
	}
}

void CopleyBus::reset(void)
{
	_status.clear();

	if (_port.IsOpen()) {
		_port.FlushIOBuffers();
		_port.Close();
	}

	if (_communication_port_address != "")
		connect(_communication_port_address, _baud);
}

bool CopleyBus::is_connected(void)
{
	return _port.IsOpen();
}

void CopleyBus::base_baud_rate(void)
{
	if (_status.ok()) {
		// When attempting to connect to a Copley amplifier
		// whose baud rate is unknown, it helps to dump
		// carriage returns to the serial port. It appears
		// the amplifier may automatically detect the baud
		// rate from these carriage returns. By initially
		// dumping a small number of them, and a larger number
		// on subsequent attempts, the overall number of attempts
		// to connect decreases dramatically.
		// These values are determined empirically and annoyingly.
		constexpr int connect_attempts = 10;  // number of times to attempt to connect
		constexpr int num_preflush	 = 10;  // number of CR to transmit before connecting
		constexpr int num_retry_flush  = 100; // number of CR to transmit before retrying
		constexpr int flush_period_us  = 25;  // delay between CR transmissions

		// Error messages that are expected and normal while baud rate
		// is autodetected and communication established. These errors
		// are suppressed and cleared, except for the final attempt.
		std::array<CopleyContext::Code, 9> suppressed_codes = {CopleyStatus::Code::kComTimeout,
															   CopleyStatus::Code::kComBaudUnknown,
															   CopleyStatus::Code::kAmplifierUnknownCommand,
															   CopleyStatus::Code::kPacketUnknownResponse,
															   CopleyStatus::Code::kComGarbage,
															   CopleyStatus::Code::kAmplifierAsciiParsingError,
															   CopleyStatus::Code::kComNotAcknowledged,
															   CopleyStatus::Code::kAmplifierDataValueOutOfRange,
															   CopleyStatus::Code::kPacketPayloadUndersize};
		for (const CopleyStatus::Code code : suppressed_codes) {
			_status.suppress_push(code);
		}

		// preflush
		write_returns(num_preflush, flush_period_us);

		// connect attempts
		bool baud_established = false;
		for (int attempt = 0; attempt < connect_attempts && !baud_established; attempt++) {
			// clear suppressed errors from previous attempts
			for (const CopleyContext::Code code : suppressed_codes) {
				_status.clear(code);
			}

			// attempt to set baud rate
			set_baud_rate(kPowerOnBaudRate);

			// connection successful?
			if (_status.ok()) {
				baud_established = true;
			}
			// connection failed, but will attempt to reconnect
			else if (attempt + 1 < connect_attempts) {
				write_returns(num_retry_flush, flush_period_us);
			}
		}

		// unsuppress errors
		for (const CopleyContext::Code code : suppressed_codes) {
			_status.suppress_pop(code);
		}
		if (!baud_established) {
			_status.raise_error(CopleyStatus::Code::kComTimeout, "unable to establish communication with bus master.");
		}
	}
}

void CopleyBus::set_baud_rate(const speed_t baud)
{
	if (_status.ok()) {
		// Use Ascii builder by default. This command isn't run during real-time critical operations.
		CopleyCommandBuilderAscii cmd_builder(kSerialNodeId);

		const CopleyCommand set_baud = cmd_builder.build_set_command(CopleyParameter::kDriveSerialBaudRate,
																	 CopleyParameter::MemoryLocation::kVolatile,
																	 std::to_string(speed_to_baud(baud)));
		const CopleyCommand get_baud = cmd_builder.build_get_command(CopleyParameter::kDriveSerialBaudRate,
																	 CopleyParameter::MemoryLocation::kVolatile);

		// send baud change command; this is acknowledged but
		// is disregarded here while configuring the port as
		// sometimes the response is missed during the baud rate change.
		_status.suppress_push(CopleyStatus::Code::kComTimeout);
		write_command(set_baud, _status);
		_status.clear(CopleyStatus::Code::kComTimeout);
		_status.suppress_pop(CopleyStatus::Code::kComTimeout);

		// reset port
		usleep(kBaudRateChangeDelayMs * 1000);
		_port.SetBaudRate(static_cast<LibSerial::BaudRate>(baud));
		_port.FlushInputBuffer();

		// verify by reading back the baud rate
		std::unique_ptr<CopleyResponse> response = write_command(get_baud, _status);
		if (!(_status.ok() && response && response->get_type() == CopleyResponse::ResponseType::kValue)) {
			_status.raise_error(CopleyStatus::Code::kComBaudUnknown, "failed to set baud rate.");
		}
	}
}

std::unique_ptr<CopleyResponse> CopleyBus::write_command(const CopleyCommand &command, CopleyStatus &driver_status)
{

	std::unique_ptr<CopleyResponse> response;
	if (command.protocol == CopleyCommand::Protocol::kAscii) {
		response = static_cast<std::unique_ptr<CopleyResponse>>(
			std::unique_ptr<CopleyResponseAscii>(new CopleyResponseAscii(driver_status)));
	} else {
		response = static_cast<std::unique_ptr<CopleyResponse>>(
			std::unique_ptr<CopleyResponseBinary>(new CopleyResponseBinary(driver_status)));
	}

	// error already occurred?
	if (!_status.ok() || !driver_status.ok()) {
	}
	// not connected?
	else if (!is_connected()) {
		driver_status.raise_error(CopleyStatus::Code::kComNotConnected, "attempted to write to an unopen port.");
	}
	// no error
	else {

		bool retry				  = true;
		uint32_t transmit_attempt = 0;
		while (retry && driver_status.ok()) {
			transmit_attempt++;
			retry = false; // default to not retransmitting, and override if a retry attempt is warranted

			// Transmit command
			try {
				// Write command to the serial port
				// Okay not to wait for DrainWriteBuffer command as
				// this driver always waits for a response from the amplifier
				_port.Write(command.packet);

			} catch (...) {
				_status.raise_error(CopleyStatus::Code::kComWriteFail, "unhandled I/O exception.");
			}

			// Read response if command expecting reply
			if (command.expects_response) {
				// special case: writing to to some non-volatile paramters takes longer to receive a response
				uint32_t timeout = command.write_to_flash ? kNonVolatileWriteTimeoutMs : _communication_timeout_ms;

				response = read_response(driver_status, timeout, command.protocol);
				if (driver_status.ok()) {
					const CopleyStatus::Code amplifier_code =
						CopleyResponse::amplifier_error_to_code(response->get_value());

					std::string command_str;
					if (command.protocol == CopleyCommand::Protocol::kAscii) {
						command_str = std::string(command.packet.begin(), command.packet.end());
					} else {
						for (const auto &ch : command.packet)
							command_str += util::int_to_hex(ch) + " ";
						boost::trim(command_str); // Trim unneeded space
					}
					switch (response->get_ack_type()) {
					// Positive acknowlegement. Everybody's cool.
					case CopleyResponse::AckType::kAck:
						break;

					// This driver attempts to recover in hopes the message was corrupt due to a physical
					// communication
					// channel error.
					case CopleyResponse::AckType::kComError:
						if (transmit_attempt < _communication_max_attempts) {
							driver_status.raise_warning(amplifier_code, "In response to command: " + command_str +
																			"Command will be retried.");
							flush_buffers();
							retry = true;
						} else {
							driver_status.raise_error(amplifier_code, "In response to command: " + command_str +
																		  ". Maximum retransmit attempts reached.");
						}
						break;

					// Other negative acknowledgement received.
					case CopleyResponse::AckType::kNack:
						driver_status.raise_error(amplifier_code, "In response to command: " + command_str +
																	  " Response was \"" + response->to_string() +
																	  "\".");
						break;

					// Unknown acknowledgement received.
					case CopleyResponse::AckType::kNone:
					default:
						if (transmit_attempt < _communication_max_attempts) {
							driver_status.raise_warning(CopleyStatus::Code::kPacketUnknownResponse,
														"unexpected response \"" + response->to_string() +
															"\" in response to command: " + command_str +
															"Command will be retried.");
							flush_buffers();
							retry = true;
						} else {
							driver_status.raise_error(CopleyStatus::Code::kPacketUnknownResponse,
													  "unexpected response \"" + response->to_string() +
														  "\" in response to command: " + command_str +
														  ". Maximum retransmit attempts reached.");
						}
						break;
					}
				}
			}
		}
	}

	return response;
}

std::unique_ptr<CopleyResponse>
CopleyBus::read_response(CopleyStatus &driver_status, const uint32_t timeout_ms, const CopleyCommand::Protocol type)
{
	std::unique_ptr<CopleyResponse> response;
	if (type == CopleyCommand::Protocol::kAscii) {
		response = static_cast<std::unique_ptr<CopleyResponse>>(
			std::unique_ptr<CopleyResponseAscii>(new CopleyResponseAscii(driver_status)));
	} else {
		response = static_cast<std::unique_ptr<CopleyResponse>>(
			std::unique_ptr<CopleyResponseBinary>(new CopleyResponseBinary(driver_status)));
	}

	if (_status.ok() && driver_status.ok()) {
		if (!is_connected()) {
			driver_status.raise_error(CopleyStatus::Code::kComNotConnected,
									  "attempted to read from a port that is not connected.");
		} else {
			try {
				bool found							 = false;
				const bool previous_driver_status_ok = driver_status.ok();

				// read until timeout or message completed
				while (_status.ok() && driver_status.ok() && !found) {
					unsigned char byte;
					_port.ReadByte(byte, timeout_ms);
					response->add_byte(byte);
					if (response->message_is_completed()) {
						if (previous_driver_status_ok && !driver_status.ok()) {
							driver_status.raise_error(CopleyStatus::Code::kComGarbage,
													  "unable to deserialize reponse \"" + response->to_string() +
														  "\".");
						} else {
							found = true;
						}
					}
				}
			} catch (const LibSerial::ReadTimeout &) {
				driver_status.raise_error(CopleyStatus::Code::kComTimeout,
										  "Bus master did not respond within " + std::to_string(timeout_ms) + "ms." +
											  " Receive buffer contents: " + response->to_string() + ".");
			} catch (...) {
				_status.raise_error(CopleyStatus::Code::kComReadFail, "unhandled I/O exception.");
			}
		}
	}

	return response;
}

void CopleyBus::write_returns(const uint32_t num_returns, const uint32_t spacing_us)
{
	if (_status.ok() && is_connected()) {
		try {
			for (uint32_t return_count = 0; return_count < num_returns; return_count++) {
				_port.WriteByte(kCarriageReturn);
				_port.DrainWriteBuffer();
				usleep(spacing_us);
			}
		} catch (...) {
			_status.raise_error(CopleyStatus::Code::kComReadFail, "unhandled I/O exception.");
		}
	}
}

void CopleyBus::flush_buffers(void)
{
	// ignore general errors here, as long as the port can be access it should be flushed.
	// this allows recovery from garbage in the stream.

	/// transmit time = 1 byte * (8 bits/byte) * (10 baud / 8 bits) / (baud / s) * (1e6 us / s)
	uint32_t communication_byte_transmit_time_us = (1e6 * 10) / speed_to_baud(static_cast<speed_t>(_baud));
	std::chrono::microseconds input_wait(communication_byte_transmit_time_us * 2);
	std::chrono::steady_clock::time_point input_timeout = std::chrono::steady_clock::now() + (input_wait * 50);

	if (is_connected()) {
		bool data_lost = false;

		std::string discarded_buffer;

		// catch ostream exceptions
		try {
			// wait for at least two byte periods to capture any input
			std::this_thread::sleep_for(input_wait);
			// clear any garbage currently on the port
			while (_port.IsDataAvailable() && std::chrono::steady_clock::now() <= input_timeout) {
				unsigned char byte;
				_port.ReadByte(byte, _communication_timeout_ms);
				discarded_buffer += " " + util::int_to_hex(byte);
				data_lost = true;
				// wait for another two byte period to ensure no further input remains.
				std::this_thread::sleep_for(input_wait);
			}
		} catch (const LibSerial::ReadTimeout &) {
			_status.raise_error(CopleyStatus::Code::kComTimeout, "Bus master did not respond within " +
																	 std::to_string(_communication_timeout_ms) +
																	 "ms during input flush.");
		} catch (...) {
			_status.raise_error(CopleyStatus::Code::kComReadFail, "unhandled I/O exception during input flush.");
		}

		// raise warning if data lost.
		if (data_lost) {
			_status.raise_warning(CopleyStatus::Code::kComDataFlush, "Discarded: " + discarded_buffer);
		}

		// Flush at the port level... just in case there's something left
		_port.FlushIOBuffers();
	}
}

// Credit: Tatu Ylonen <ylo@cs.hut.di>
// See: https://opensource.apple.com/source/OpenSSH/OpenSSH-7.1/openssh/ttymodes.c?txt
int CopleyBus::speed_to_baud(const speed_t speed)
{
	switch (speed) {
	case B0:
		return 0;
	case B50:
		return 50;
	case B75:
		return 75;
	case B110:
		return 110;
	case B134:
		return 134;
	case B150:
		return 150;
	case B200:
		return 200;
	case B300:
		return 300;
	case B600:
		return 600;
	case B1200:
		return 1200;
	case B1800:
		return 1800;
	case B2400:
		return 2400;
	case B4800:
		return 4800;
	case B9600:
		return 9600;

#ifdef B19200
	case B19200:
		return 19200;
#else // B19200
#ifdef EXTA
	case EXTA:
		return 19200;
#endif // EXTA
#endif // B19200

#ifdef B38400
	case B38400:
		return 38400;
#else // B38400
#ifdef EXTB
	case EXTB:
		return 38400;
#endif // EXTB
#endif // B38400

#ifdef B7200
	case B7200:
		return 7200;
#endif // B7200
#ifdef B14400
	case B14400:
		return 14400;
#endif // B14400
#ifdef B28800
	case B28800:
		return 28800;
#endif // B28800
#ifdef B57600
	case B57600:
		return 57600;
#endif // B57600
#ifdef B76800
	case B76800:
		return 76800;
#endif // B76800
#ifdef B115200
	case B115200:
		return 115200;
#endif // B115200
#ifdef B230400
	case B230400:
		return 230400;
#endif // B230400
	default:
		return 9600;
	}
}

speed_t CopleyBus::baud_to_speed(const unsigned int baud)
{
	if (baud < 50) {
		return B0;
	} else if (baud < 75) {
		return B50;
	} else if (baud < 110) {
		return B75;
	} else if (baud < 134) {
		return B110;
	} else if (baud < 150) {
		return B134;
	} else if (baud < 200) {
		return B150;
	} else if (baud < 300) {
		return B200;
	} else if (baud < 600) {
		return B300;
	} else if (baud < 1200) {
		return B600;
	} else if (baud < 1800) {
		return B1200;
	} else if (baud < 2400) {
		return B1800;
	} else if (baud < 4800) {
		return B2400;
	} else if (baud < 9600) {
		return B4800;
	} else if (baud < 19200) {
		return B9600;
	} else if (baud < 38400) {
		return B19200;
	} else if (baud < 57600) {
		return B38400;
	} else if (baud < 115200) {
		return B57600;
	} else if (baud < 230400) {
		return B115200;
	} else {
		return B230400;
	}
}

} // namespace momentum
