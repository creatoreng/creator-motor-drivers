/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Herkulex bus scanner.
 **/

#pragma once

#include <drivers/herkulex/Herkulex.hpp>

#include <libs/util/util.hpp>

namespace momentum
{

/// Herkulex amplifier scanner.
/// @warning Do not use for machine control. This class uses blocking methods and does not
/// control motor values.
class HerkulexScan
{
private:
	/// Discovered herkulex motor. Motor is weakly connected.
	/// @warning Do not use for machine control. Scanned motors are not configured and state is not managed.
	class Discovered : public Herkulex
	{
	public:
		/// Default constructor.
		/// @param [in,out] bus The Herkulex bus.
		explicit Discovered(const std::shared_ptr<HerkulexBus> bus);

		/// Connect to the motor. Weak connection only.
		/// @note The communication bus must be connected before calling this method.
		/// @param [in] motor_id The unique ID of this motor.
		/// @return true if connected.
		bool connect(const uint8_t motor_id) override;

		/// Disconnect the motor. Weak disconnect only.
		void disconnect(void) override;
	};

public:
	/// Construct a scanner.
	HerkulexScan(void);

	/// Status of this object.
	std::string status_string(void) const;

  /// Connect to the bus.
	/// @param [in] address The serial port address of the bus.
	/// @return true if connected.
	bool connect_uart(const std::string &address);

	/// Disconnect from the UART bus
	/// @return true if connected.
	void disconnect_bus(void);

	/// Scan for a Herkulex motor on the UART bus.
	/// @param [in] starting_motor_id The ID from which to begin the search.
	/// @return the ID of the first motor found; kMotorIdMax if none found.
	uint8_t scan(const uint8_t stating_motor_id = 0);

	/// Scan for all Herkulex motors on the UART bus.
	/// @param [in] starting_motor_id The motor ID from which to begin the search.
	/// @return comma-separated list of all IDs found.
	std::string scan_all(const uint8_t starting_motor_id = 0);

	/// Assign the network ID of a Herkulex motor.
	/// @param [in] current_motor_id The current address of the motor.
	/// @param [in] new_id The new address of the motor.
	void assign_address(const uint8_t current_motor_id, const uint8_t new_motor_id);

	/// Read an EEP ROM register from a Herkulex motor.
	/// @param [in] motor_id The motor ID.
	/// @param [in] addresss The register address to read.
	/// @return Value of the register (either 1 or 2 bytes depending on the register).
	uint16_t read_eep_register(const uint8_t motor_id, const uint8_t address);

	/// Read RAM register from a Herkulex motor.
	/// @param [in] motor_id The motor ID.
	/// @param [in] addresss The register address to read.
	/// @return Value of the register (either 1 or 2 bytes depending on the register).
	uint16_t read_ram_register(const uint8_t motor_id, const uint8_t address);

	/// Write an EEP ROM register to a Herkulex motor.
	/// @param [in] motor_id The motor ID.
	/// @param [in] addresss The register address to write.
	/// @param [in] Value to write to the register (either 1 or 2 bytes depending on the register).
	void write_eep_register(const uint8_t motor_id, const uint8_t address, const uint16_t value);

	/// Write a RAM register to a Herkulex motor.
	/// @param [in] motor_id The motor ID.
	/// @param [in] addresss The register address to write.
	/// @param [in] Value to write to the register (either 1 or 2 bytes depending on the register).
	void write_ram_register(const uint8_t motor_id, const uint8_t address, const uint16_t value);

	/// Read all known memory addresses.
	/// @param [in] motor_id The motor address.
	/// @return memory map.
	std::map<std::pair<uint8_t, HerkulexRegister::MemoryLocation>, uint16_t> map_memory(const uint8_t motor_id);

	/// Read all known memory addresses and print as a beautiful string.
	/// @param [in] motor_id The motor ID to scan.
	/// @return formatted memory map.
	std::string map_memory_pretty(const uint8_t motor_id);

	/// Sets the baud rate for the underlying bus object
	/// @param [in] baud_rate the integer value of a LibSerial baud define
	bool set_baud_rate(const uint16_t baud_rate);

	/// Gets the baud rate for the underlying bus object
	/// @return the current baud rate, as an integer
	int get_baud_rate(void);

private:
	/// Attempt to discover a motor. If a motor is discovered, it is wealy connected and
	/// a unique pointer is returned. If no motor is discovered, nullptr is returned.
	/// @param [in] motor_id The motor id to discover.
	/// @return unique pointer to a connected motor; nullptr if connection failed.
	std::unique_ptr<Discovered> discover(const uint8_t motor_id);

	/// Read a register from a Herkulex motor.
	/// @param [in,out] motor The scanned motor. Motor must be connected before calling this method.
	/// @param [in] addresss The register address to read.
	/// @param [in] location The memory bank to read.
	/// @return Value of the register (either 1 or 2 bytes depending on the register).
	uint16_t
	read_register(const uint8_t motor_id, const uint8_t address, const HerkulexRegister::MemoryLocation location);

	/// Read a register from a Herkulex motor.
	/// @param [in,out] motor The scanned motor. Motor must be connected before calling this method.
	/// @param [in] reg The register to read.
	/// @return Value of the register (either 1 or 2 bytes depending on the register).
	uint16_t read_register(Herkulex &motor, const HerkulexRegister &reg);

	/// Write a register to a Herkulex motor.
	/// @param [in,out] motor The scanned motor. Motor must be connected before calling this method.
	/// @param [in] addresss The register address to write.
	/// @param [in] location The memory bank to write.
	/// @param [in] Value to write to the register (either 1 or 2 bytes depending on the register).
	void write_register(const uint8_t motor_id,
						const uint8_t address,
						const HerkulexRegister::MemoryLocation location,
						const uint16_t value);

	/// Write a register to a Herkulex motor.
	/// @param [in,out] motor The scanned motor. Motor must be connected before calling this method.
	/// @param [in] reg The register to write.
	/// @param [in] Value to write to the register (either 1 or 2 bytes depending on the register).
	void write_register(Herkulex &motor, const HerkulexRegister &reg, const uint16_t value);

	/// Return a hexidecimal string formatted 0x__ with variable width.
	/// @param [in] number The number to convert.
	/// @param [in] digits The number of digits to print.
	/// @return formatted string
	static std::string to_hex(unsigned int number, int digits);

	/// Herkulex bus.
	std::shared_ptr<HerkulexBus> _bus;
};

} // namespace momentum
