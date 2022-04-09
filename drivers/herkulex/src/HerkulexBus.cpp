/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/herkulex/HerkulexBus.hpp>
#include <drivers/herkulex/HerkulexModel.hpp>
#include <drivers/herkulex/HerkulexStatusRegister.hpp>

#include <boost/chrono.hpp>
#include <boost/timer/timer.hpp>
#include <iterator>

namespace momentum
{

HerkulexBus::HerkulexBus(void)
  : _communication_timeout_ms(kDefaultTimeoutMs),
	_communication_byte_transmit_time_us((1e6 * 10) /
										 speed_to_baud(static_cast<speed_t>(kCommunicationBaudDefault))),
	_communication_max_attempts(kDefaultTransmitAttempts),
	_port(),
	_status()
{
}
HerkulexBus::~HerkulexBus(void)
{
	disconnect();
}

const HerkulexStatus &HerkulexBus::status(void) const
{
	return _status;
}

std::string HerkulexBus::status_string(void) const
{
	return _status.to_string();
}

void HerkulexBus::set_timeout_ms(const uint32_t timeout_ms)
{
	_communication_timeout_ms = timeout_ms;
	}

uint32_t HerkulexBus::get_transmit_attempts(void) const
{
	return _communication_max_attempts;
}

void HerkulexBus::set_transmit_attempts(const uint32_t transmit_attempts)
{
	_communication_max_attempts = transmit_attempts;
	}

bool HerkulexBus::ping(const uint8_t motor_id)
{
	bool ping_success = false;

	if (_status.ok()) {
		HerkulexStatus motor_status;
		const HerkulexPacket stat{motor_id, HerkulexCommand::STAT, {}};
		const HerkulexPacket response = write_packet(stat, HerkulexAckPolicy::kNoReply, motor_status);
		ping_success				  = motor_status.ok();
	}

	return ping_success;
}

bool HerkulexBus::connect(const std::string &port, const LibSerial::BaudRate baud_rate)
{
	// error already occurred?
	if (!_status.ok()) {
	}
	// already open?
	else if (is_connected()) {
	}
	// no error
	else if (_status.ok()) {
		// catch ostream exceptions
		try {
						_port.Open(port);
			_port.SetBaudRate(baud_rate);

			/// transmit time = 1 byte * (8 bits/byte) * (10 baud / 8 bits) / (baud / s) * (1e6 us / s)
			_communication_byte_transmit_time_us = (1e6 * 10) / speed_to_baud(static_cast<speed_t>(baud_rate));

			// should be open here.
			if (_port.IsOpen()) {
				// clear any garbage currently on the port
				flush_input();
			} else {
				_status.raise_error(HerkulexStatus::Code::kOpenFail);
			}
		} catch (...) {
			_status.raise_error(HerkulexStatus::Code::kOpenFail, "Unhandled I/O exception.");
		}
	}

	return is_connected();
}

void HerkulexBus::disconnect(void)
{
	if (is_connected()) {
		// clear any garbage currently on the port
		flush_input();
	}
	_port.Close();
}

bool HerkulexBus::is_connected(void)
{
	return _port.IsOpen() && _port.good();
}

void HerkulexBus::reset(void)
{
	// status clear should follow flush_input in case it raises an error
	flush_input();
	_status.clear();
}

HerkulexPacket
HerkulexBus::read_packet(const uint8_t motor_id, const HerkulexCommand command, HerkulexStatus &read_status)
{
	using namespace boost::chrono;
	HerkulexPacket packet;

	// error already occurred?
	if (!_status.ok() || !read_status.ok()) {
	}
	// not connected?
	else if (!is_connected()) {
		read_status.raise_error(HerkulexStatus::Code::kNotConnected,
								"Motor " + util::int_to_hex(motor_id) +
									" attempted to read from a bus that is not connected.");
	}
	// no error
	else {
		std::deque<uint8_t> buffer;
		bool timeout = false;
		bool found   = false;
		boost::timer::cpu_timer timer;
		timer.start();

		try {
			// read until timeout or a packet is found.
			while (_status.ok() && read_status.ok() && !timeout && !found) {
				if (_port.IsDataAvailable()) {
					const uint8_t byte = _port.get();
					buffer.push_back(byte);
				} else {
					// no bytes at the port - wait for a one byte transmit time period
					usleep(_communication_byte_transmit_time_us);
				}
				timeout = duration_cast<milliseconds>(nanoseconds(timer.elapsed().wall)) >=
						  milliseconds(_communication_timeout_ms);
				found = packet.deserialize(buffer, read_status);
			}
		} catch (...) {
			_status.raise_error(HerkulexStatus::Code::kReadFail, "Unhandled I/O exception.");
		}

		// timeout?
		if (timeout) {
			std::string timeout_message;
			timeout_message = "Motor " + util::int_to_hex(motor_id) + " did not respond within " +
							  std::to_string(_communication_timeout_ms) + "ms." + " Execution time was " +
							  std::to_string(duration_cast<milliseconds>(nanoseconds(timer.elapsed().wall)).count()) +
							  "ms." + " Packet " + std::string(found ? "was" : "was not") + " found in this time.";
			if (!found) {
				timeout_message += " Receive buffer:";
				if (!buffer.empty()) {
					for (uint8_t byte : buffer) {
						timeout_message += " " + util::int_to_hex(byte);
					}
				} else {
					timeout_message += " empty";
				}
				timeout_message += ".";
			}
			read_status.raise_error(HerkulexStatus::Code::kTimeout, timeout_message);
		}
		// if found, verify the packet contains expected content
		if (found) {
			// communication warning from device?
			if (packet.get_ack_type() == HerkulexPacket::AckType::kComError) {
				// FIXME: It appears the communucation failure bits in the status
				// register are persistent, and this driver does not yet reset them.
				// Since this driver exclusively uses an acknowledge all packet policy.
				// Communcation errors will present as timeouts, garbage or failed
				// checksums at the host, so the packet error bits on the device
				// may be safely ignored. Commenting out until these bits are reset
				// after being read.
				// read_status.raise_warning( packet.com_error_code() );
			}

			// motor ID correct?
			// exception: when the motor_id is broadcast address
			if (motor_id != HerkulexPacket::kMotorIdMax && motor_id != packet.get_motor_id()) {
				read_status.raise_error(HerkulexStatus::Code::kIncorrectDevice,
										"Expected response from motor " + util::int_to_hex(motor_id) +
											" but received response from motor " +
											std::to_string(packet.get_motor_id()) + ".");
			}
			// command matches?
			else if (command != packet.get_command()) {
				read_status.raise_error(HerkulexStatus::Code::kAckMismatch,
										"Expected acknowledgement to command " +
											std::to_string(static_cast<uint8_t>(command)) +
											" but received acknowledgement of command " +
											std::to_string(static_cast<uint8_t>(packet.get_command())) + ".");
			}
			// otherwise packet is valid
		}
	}

	return packet;
}

HerkulexPacket HerkulexBus::write_packet(const HerkulexPacket &packet,
										 const HerkulexAckPolicy ack_policy,
										 HerkulexStatus &driver_status,
										 const int previous_transmission_attempts)
{
	HerkulexPacket response;

	// error already occurred?
	if (!_status.ok() || !driver_status.ok()) {
	}
	// not connected?
	else if (!is_connected()) {
		driver_status.raise_error(HerkulexStatus::Code::kNotConnected, "Attempted to write to an unopen port.");
	}
	// no error
	else {
		const std::deque<uint8_t> serialized_packet = packet.serialize(driver_status);

		// Transmit command and receive response until positively acknowledged or error
		bool succeeded = false;
		for (int transmit_attempt = previous_transmission_attempts;
			 !succeeded && static_cast<uint32_t>(transmit_attempt) < _communication_max_attempts && driver_status.ok();
			 transmit_attempt++) {
			write_raw(serialized_packet);
			if (!packet.expects_response(ack_policy)) {
				succeeded = true;
			} else {
				HerkulexStatus read_status;
				response = read_packet(packet.get_motor_id(), packet.get_command(), read_status);

				if (read_status.ok()) {
					succeeded = true;
					response.set_transmission_attempts(transmit_attempt + 1);
				} else {
					if (static_cast<uint32_t>(transmit_attempt + 1) < _communication_max_attempts) {
						driver_status.raise(read_status, HerkulexStatus::Severity::kWarning,
											" Command will be retried.");
						// wait for garbage collection period to pass so device clears its buffer before
						// attempting to retransmit; add a 25% buffer for good measure
						usleep(kDeviceGarbageCollectionTickPeriods * HerkulexModel::tick_period_ms * 1.25);
						flush_input();
					} else {
						driver_status.raise(read_status, HerkulexStatus::Severity::kError,
											" Maximum retransmit attempts reached.");
					}
				}
			}
		}
	}

	return response;
}

void HerkulexBus::write_raw(const std::deque<uint8_t> &data)
{
	// ignore errors here, since some messages are meant to be written regardless
	// of error status. higher-level methods will determine whether or not this
	// method should be called.
	if (is_connected()) {
		// copy deque to the serial stream
		try {
			std::copy(data.cbegin(), data.cend(), (std::ostream_iterator<uint8_t>)_port);
		} catch (...) {
			_status.raise_error(HerkulexStatus::Code::kWriteFail, "Unhandled I/O exception.");
		}
	}
}

void HerkulexBus::flush_input(void)
{
	// ignore general errors here, as long as the port can be access it should be flushed.
	// this allows recovery from garbage in the stream.
	if (is_connected()) {
		bool data_lost = false;

		std::string discarded_buffer;

		// catch ostream exceptions
		try {
			// wait for at least two byte periods to capture any input
			usleep(2 * _communication_byte_transmit_time_us);
			// clear any garbage currently on the port
			while (_port.IsDataAvailable()) {
				const uint8_t byte = _port.get();
				discarded_buffer += " " + util::int_to_hex(byte);
				data_lost = true;
				// wait for another two byte period to ensure no further input remains.
				usleep(2 * _communication_byte_transmit_time_us);
			}
		} catch (...) {
			_status.raise_error(HerkulexStatus::Code::kReadFail, "Unhandled I/O exception during input flush.");
		}

		// raise warning if data lost.
		if (data_lost) {
			_status.raise_warning(HerkulexStatus::Code::kDataFlush, "Discarded: " + discarded_buffer);
		}
	}
}

bool HerkulexBus::set_baud_rate(const LibSerial::BaudRate baud_rate)
{
	if (_status.ok()) {
		_port.SetBaudRate(baud_rate);

		// Should reflect the success of SetBaudRate
		if (_port.good()) {
			/// transmit time = 1 byte * (8 bits/byte) * (10 baud / 8 bits) / (baud / s) * (1e6 us / s)
			_communication_byte_transmit_time_us = (1e6 * 10) / speed_to_baud(static_cast<speed_t>(baud_rate));

			// clear any garbage currently on the port
			flush_input();
		}
	}
	return is_connected();
}

LibSerial::BaudRate HerkulexBus::get_baud_rate(void)
{
	return _port.GetBaudRate();
}

unsigned int HerkulexBus::get_baud_rate_int(void)
{
  return speed_to_baud(static_cast<speed_t>(get_baud_rate()));
}

// Credit: Tatu Ylonen <ylo@cs.hut.di>
// See: https://opensource.apple.com/source/OpenSSH/OpenSSH-7.1/openssh/ttymodes.c?txt
int HerkulexBus::speed_to_baud(const speed_t speed)
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

speed_t HerkulexBus::baud_to_speed(const unsigned int baud)
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
