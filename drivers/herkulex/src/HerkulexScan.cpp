/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * See header file for documentation.
 **/

#include <drivers/herkulex/HerkulexScan.hpp>

#include <iomanip>
#include <sstream>
#include <iostream>

namespace momentum
{

HerkulexScan::Discovered::Discovered(const std::shared_ptr<HerkulexBus> bus)
  : Herkulex(bus)
{
}

bool HerkulexScan::Discovered::connect(const uint8_t motor_id)
{
	return connect_weak(motor_id);
}

void HerkulexScan::Discovered::disconnect(void)
{
	disconnect_weak();
}

HerkulexScan::HerkulexScan(void) : _bus(std::make_shared<HerkulexBus>())
{
}

std::string HerkulexScan::to_hex(unsigned int number, int digits)
{
	std::stringstream ss;
	ss << "0x" << std::setfill('0') << std::setw(digits) << std::uppercase << std::hex << number;
	return ss.str();
}

std::string HerkulexScan::status_string(void) const
{
	return _bus->status().to_string();
}

bool HerkulexScan::connect_uart(const std::string &port)
{
	return _bus->connect(port, LibSerial::BaudRate::BAUD_115200);
}

void HerkulexScan::disconnect_bus(void)
{
	_bus->disconnect();
}

std::unique_ptr<HerkulexScan::Discovered> HerkulexScan::discover(const uint8_t motor_id)
{
	std::unique_ptr<Discovered> discovered(new Discovered(_bus));
	if (discovered->connect(motor_id)) {
		std::cout << "Discovered motor " + std::to_string(motor_id) + "." << std::endl;
	} else {
		std::cout << "Failed to discover motor " + std::to_string(motor_id) + "." << std::endl;
		discovered.reset(nullptr);
	}
	return discovered;
}

uint8_t HerkulexScan::scan(const uint8_t starting_motor_id)
{
	for (uint8_t motor_id = starting_motor_id; motor_id < HerkulexPacket::kMotorIdMax && _bus->status().ok();
		 motor_id++) {
		if (_bus->ping(motor_id)) {
			std::cout << "Found motor " + std::to_string(motor_id) + "." << std::endl;
			return motor_id;
		}
	}
	std::cout << "No additional motors found." << std::endl;
	return HerkulexPacket::kMotorIdMax;
}

std::string HerkulexScan::scan_all(const uint8_t starting_motor_id)
{
	std::string found;
	uint8_t motor_id = scan(starting_motor_id);
	while (motor_id < HerkulexPacket::kMotorIdMax) {
		if (!found.empty()) {
			found += ",";
		}
		found += std::to_string(motor_id);
		motor_id = scan(motor_id + 1);
	}
	return found;
}

void HerkulexScan::assign_address(const uint8_t current_motor_id, const uint8_t new_motor_id)
{
	if (!_bus->ping(new_motor_id)) {
		std::cout << "New motor id " + std::to_string(new_motor_id) + " is available." << std::endl;
		;
		std::unique_ptr<Herkulex> motor = discover(current_motor_id);
		if (motor) {
			// write new address, reboot and disconnect.
			std::cout << "Writing new address " + std::to_string(new_motor_id) + " to EEP ROM." << std::endl;
			motor->write_register(HerkulexRegister::kEepId, new_motor_id);
			motor->reboot();
			motor->disconnect();

			// attempt to ping the new motor id.
			std::cout << "Attempting to ping motor ID " + std::to_string(new_motor_id) + "." << std::endl;
			if (_bus->ping(new_motor_id)) {
				std::cout << "Ping successful. Motor ID changed." << std::endl;
			} else {
				std::cerr << "Ping unsuccessful. Motor ID change failed." << std::endl;
			}
		}
	} else {
		std::cerr << "New motor ID is already in use. Motor ID unchanged." << std::endl;
	}
}

uint16_t HerkulexScan::read_eep_register(const uint8_t motor_id, const uint8_t address)
{
	return read_register(motor_id, address, HerkulexRegister::MemoryLocation::kNonVolatile);
}

uint16_t HerkulexScan::read_ram_register(const uint8_t motor_id, const uint8_t address)
{
	return read_register(motor_id, address, HerkulexRegister::MemoryLocation::kVolatile);
}

uint16_t HerkulexScan::read_register(const uint8_t motor_id,
									 const uint8_t address,
									 const HerkulexRegister::MemoryLocation location)
{
	uint16_t value					  = 0;
	const HerkulexRegister *const reg = HerkulexRegister::lookup(address, location);
	if (reg) {
		std::unique_ptr<Herkulex> motor = discover(motor_id);
		if (motor) {
			value = read_register(*motor, *reg);
		}
	} else {
		std::cerr << "Register " +
						 (location == HerkulexRegister::MemoryLocation::kNonVolatile ? std::string("e")
																					 : std::string("r")) +
						 to_hex(address, 2) + " is unsupported."
				  << std::endl;
	}

	return value;
}

uint16_t HerkulexScan::read_register(Herkulex &motor, const HerkulexRegister &reg)
{
	uint16_t value = 0;
	if (motor.is_connected() && motor.status().ok()) {
		value = motor.read_register(reg);
	} else {
		std::cerr << "Motor not connected." << std::endl;
	}
	return value;
}

void HerkulexScan::write_eep_register(const uint8_t motor_id, const uint8_t address, const uint16_t value)
{
	write_register(motor_id, address, HerkulexRegister::MemoryLocation::kNonVolatile, value);
}

void HerkulexScan::write_ram_register(const uint8_t motor_id, const uint8_t address, const uint16_t value)
{
	write_register(motor_id, address, HerkulexRegister::MemoryLocation::kVolatile, value);
}

void HerkulexScan::write_register(const uint8_t motor_id,
								  const uint8_t address,
								  const HerkulexRegister::MemoryLocation location,
								  const uint16_t value)
{
	const HerkulexRegister *const reg = HerkulexRegister::lookup(address, location);
	if (reg) {
		std::unique_ptr<Herkulex> motor = discover(motor_id);
		if (motor) {
			write_register(*motor, *reg, value);
		}
	} else {
		std::cerr << "Register " +
						 (location == HerkulexRegister::MemoryLocation::kNonVolatile ? std::string("e")
																					 : std::string("r")) +
						 to_hex(address, 2) + " is unsupported."
				  << std::endl;
	}
}

void HerkulexScan::write_register(Herkulex &motor, const HerkulexRegister &reg, const uint16_t value)
{
	if (motor.is_connected() && motor.status().ok()) {
		motor.write_register(reg, value);
	} else {
		std::cerr << "Motor not connected." << std::endl;
	}
}

std::map<std::pair<uint8_t, HerkulexRegister::MemoryLocation>, uint16_t>
HerkulexScan::map_memory(const uint8_t motor_id)
{
	std::map<std::pair<uint8_t, HerkulexRegister::MemoryLocation>, uint16_t> memory_map;

	std::unique_ptr<Discovered> motor = discover(motor_id);
	if (motor) {
		for (uint8_t address = 0x00; address < 0xFF && motor->status().ok(); address++) {
			std::cout << "Reading address " + to_hex(address, 2) + "." << std::endl;
			const HerkulexRegister *const eep =
				HerkulexRegister::lookup(address, HerkulexRegister::MemoryLocation::kNonVolatile);
			const HerkulexRegister *const ram =
				HerkulexRegister::lookup(address, HerkulexRegister::MemoryLocation::kVolatile);
			if (eep) {
				uint16_t value = motor->read_register(*eep);
				memory_map.insert({{eep->address, eep->location}, value});
			}
			if (ram) {
				uint16_t value = motor->read_register(*ram);
				memory_map.insert({{ram->address, ram->location}, value});
			}
		}
		if (motor->status().ok()) {
			std::cout << "Memory map complete." << std::endl;
		}
	}

	return memory_map;
}

std::string HerkulexScan::map_memory_pretty(const uint8_t motor_id)
{
	std::string eep;
	std::string ram;
	std::map<std::pair<uint8_t, HerkulexRegister::MemoryLocation>, uint16_t> regs = map_memory(motor_id);
	for (auto &&reg : regs) {
		// lookup the register
		const HerkulexRegister *const named_reg = HerkulexRegister::lookup(reg.first.first, reg.first.second);
		if (named_reg) {
			// format the string as e0x01 (e1) = 0x00 0x01 (0 1) [Register Type]
			std::string &bank		= named_reg->location == HerkulexRegister::MemoryLocation::kNonVolatile ? eep : ram;
			std::string bank_prefix = named_reg->location == HerkulexRegister::MemoryLocation::kNonVolatile ? "e" : "r";

			bank += bank_prefix + to_hex(named_reg->address, 2) + " (" + bank_prefix + std::to_string(reg.first.first) +
					")";

			if (named_reg->bytes == 1) {
				bank += " = " + to_hex(reg.second, 2) + " (" + std::to_string(reg.second) + ")";
			} else if (named_reg->bytes == 2) {
				bank += " = " + to_hex(reg.second, 4) + " (" + std::to_string(reg.second) + ")";
			}

			bank += " [" + named_reg->type + "]\n";
		}
	}

	return eep + ram;
}

bool HerkulexScan::set_baud_rate(const uint16_t baud_rate)
{
	return _bus->set_baud_rate(static_cast<LibSerial::BaudRate>(baud_rate));
}

int HerkulexScan::get_baud_rate(void)
{
	return _bus->get_baud_rate_int();
}

} // namespace momentum
