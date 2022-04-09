/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/copley/CopleyResponse.hpp>
#include <drivers/copley/CopleyScan.hpp>

#include <iomanip>
#include <sstream>
#include <typeinfo>
#include <iostream>

namespace momentum
{

CopleyScan::Discovered::Discovered(const std::shared_ptr<CopleyBus> bus, const uint8_t node_id)
  : Copley(bus, CopleyHardwareSpecification::invalid, node_id)
{
}

CopleyScan::Discovered::~Discovered(void)
{
	disconnect();
}

bool CopleyScan::Discovered::connect(const uint8_t node_id)
{
	return connect_weak(node_id);
}

void CopleyScan::Discovered::disconnect(void)
{
	disconnect_weak();
}

bool CopleyScan::Discovered::read_parameter(const CopleyParameter &parameter,
											const CopleyParameter::MemoryLocation location,
											CopleyParameter::value_t &value,
											std::string *const value_string)
{
	value = 0x00;
	if (status().ok()) {
		value = write_get(parameter, location);
		return status().ok();
	}
	return false;
}

bool CopleyScan::Discovered::write_parameter(const CopleyParameter &parameter,
											 const CopleyParameter::MemoryLocation location,
											 const CopleyParameter::value_t value)
{
	// write to both non-volatile and volatile?
	if (location == CopleyParameter::MemoryLocation::kBoth) {
		return write_parameter(parameter, CopleyParameter::MemoryLocation::kNonVolatile, value) &&
			   write_parameter(parameter, CopleyParameter::MemoryLocation::kVolatile, value);
	} else {
		write_set(parameter, value, location);
	}
	return status().ok();
}

CopleyScan::CopleyScan(void)
: _bus(std::make_shared<CopleyBus>())
{
	// use a more conservative timeout for scanning
	_bus->set_timeout_ms(CopleyBus::kNonVolatileWriteTimeoutMs);
}

std::string CopleyScan::status_string(void) const
{
	return _bus->status().to_string();
}

std::string CopleyScan::to_hex(unsigned int number, int digits)
{
	std::stringstream ss;
	ss << "0x" << std::setfill('0') << std::setw(digits) << std::uppercase << std::hex << number;
	return ss.str();
}

bool CopleyScan::connect_uart(const std::string &address)
{
	return _bus->connect(address);
}

std::unique_ptr<CopleyScan::Discovered> CopleyScan::discover(const uint8_t node_id)
{
	std::unique_ptr<Discovered> discovered(new Discovered(_bus, node_id));
	if (!discovered->connect(node_id)) {
		// in case the amplifier is the device connected over UART, it responds only
		// to bus id 0, though its network ID may be different. attempt to connect
		// to id 0 and then see if its network ID matches
		discovered->clear_status();
		if (!(discovered->connect(CopleyBus::kSerialNodeId) && discovered->read_node_id() == node_id)) {
			discovered.reset(nullptr);
		}
	}
	if (discovered) {
		std::string message = "Discovered amplifier " + to_hex(node_id, 2) + " ";
		if (discovered->read_node_id() == CopleyBus::kCanMasterNodeId && discovered->is_serial_node()) {
			message += "(via serial, CAN master).";
		} else if (discovered->is_serial_node()) {
			message += "(via serial, CAN slave).";
		} else if (node_id == CopleyBus::kCanMasterNodeId) {
			message += "(via CAN, CAN master).";
		} else {
			message += "(via CAN, CAN slave).";
		}
		std::cout << message << std::endl;
	} else {
		std::cout << "Failed to discover amplifier " + std::to_string(node_id) + "." << std::endl;
	}
	return discovered;
}

int CopleyScan::scan(const int starting_node_id)
{
	for (uint8_t node_id = starting_node_id; node_id <= Copley::kNodeIdMax && _bus->status().ok(); node_id++) {
		if (_bus->ping(node_id)) {
			// when connected over serial, the serial device always
			// responds to node ID 0, even if it's CAN network ID is not.
			// Report the CAN network ID instead.
			if (node_id == CopleyBus::kSerialNodeId) {
				std::unique_ptr<Discovered> amplifier = discover(node_id);
				if (amplifier && amplifier->status().ok()) {
					node_id = amplifier->read_node_id();
				} else {
					// can ping but can't weakly connect... strange.
					std::cout << "Unable to read network node ID of serial port amplifier." << std::endl;
					continue;
				}
			}
			std::cout << "Found amplifier " + to_hex(node_id, 2) + "." << std::endl;
			return node_id;
		}
	}
	std::cout << "No additional amplifiers found." << std::endl;
	return Copley::kNodeIdMax + 1;
}

std::string CopleyScan::scan_all(const int starting_node_id)
{
	std::string found;
	uint8_t node_id = scan(starting_node_id);
	while (node_id <= Copley::kNodeIdMax) {
		if (!found.empty()) {
			found += ",";
		}
		found += to_hex(node_id, 2);
		node_id = scan(node_id + 1);
	}
	return found;
}

bool CopleyScan::assign_node_id(const int current_node_id, const int new_node_id)
{
	std::unique_ptr<Discovered> amplifier = discover(current_node_id);
	if (amplifier) {
		amplifier->write_node_id(new_node_id);
		return amplifier->status().ok();
	}
	return false;
}

int CopleyScan::read_nonvolatile_parameter(const int node_id, const int parameter_id)
{
	int32_t value = 0;
	read_parameter(node_id, parameter_id, CopleyParameter::MemoryLocation::kNonVolatile, value);
	return value;
}

int CopleyScan::read_volatile_parameter(const int node_id, const int parameter_id)
{
	int value = 0;
	read_parameter(node_id, parameter_id, CopleyParameter::MemoryLocation::kVolatile, value);
	return value;
}

bool CopleyScan::read_parameter(const uint8_t node_id,
								const uint16_t parameter_id,
								const CopleyParameter::MemoryLocation location,
								CopleyParameter::value_t &value)
{
	value								  = 0;
	std::unique_ptr<Discovered> amplifier = discover(node_id);
	if (amplifier) {
		const CopleyParameter *const parameter = CopleyParameter::lookup(parameter_id, location);
		if (parameter) {
			return amplifier->read_parameter(*parameter, location, value);
		} else {
			std::cout << __FUNCTION__ << " received unknown parameter." << std::endl;
		}
	}
	return false;
}

bool CopleyScan::write_nonvolatile_parameter(const int node_id, const int parameter_id, const int value)
{
	return write_parameter(node_id, parameter_id, CopleyParameter::MemoryLocation::kNonVolatile, value);
}

bool CopleyScan::write_volatile_parameter(const int node_id, const int parameter_id, const int value)
{
	return write_parameter(node_id, parameter_id, CopleyParameter::MemoryLocation::kVolatile, value);
}

bool CopleyScan::write_parameter(const uint8_t node_id,
								 const uint16_t parameter_id,
								 const CopleyParameter::MemoryLocation location,
								 const int32_t value)
{
	std::unique_ptr<Discovered> amplifier = discover(node_id);
	if (amplifier) {
		const CopleyParameter *const parameter = CopleyParameter::lookup(parameter_id, location);
		if (parameter) {
			return amplifier->write_parameter(*parameter, location, value);
		} else {
			std::cout << __FUNCTION__ << " received unknown parameter." << std::endl;
		}
	}
	return false;
}

int CopleyScan::read_hardware_enable_nonvolatile(const uint8_t node_id)
{
	int value = 0;
	read_parameter(node_id, static_cast<uint16_t>((CopleyParameter::kDigitalInput0Configuration).id),
				   CopleyParameter::MemoryLocation::kNonVolatile, value);
	return value;
}

int CopleyScan::read_hardware_enable_volatile(const uint8_t node_id)
{
	int value = 0;
	read_parameter(node_id, static_cast<uint16_t>((CopleyParameter::kDigitalInput0Configuration).id),
				   CopleyParameter::MemoryLocation::kVolatile, value);
	return value;
}

bool CopleyScan::write_hardware_enable(const uint8_t node_id)
{
	bool is_hardware_enable_configured =
		write_parameter(node_id, static_cast<uint16_t>((CopleyParameter::kDigitalInput0Configuration).id),
						CopleyParameter::MemoryLocation::kNonVolatile,
						static_cast<int32_t>(CopleyHardwareSpecification::InputPin::kDriveDisableActiveLow)) &&
		write_parameter(node_id, static_cast<uint16_t>((CopleyParameter::kDigitalInput0Configuration).id),
						CopleyParameter::MemoryLocation::kVolatile,
						static_cast<int32_t>(CopleyHardwareSpecification::InputPin::kDriveDisableActiveLow));
	return is_hardware_enable_configured;
}

std::string CopleyScan::write_bus(std::string command_string)
{
	std::string str = "";

	CopleyCommand command;
	command.protocol		 = CopleyCommand::Protocol::kAscii;
	command.expects_response = true;
	command.packet			 = std::vector<uint8_t>{command_string.begin(), command_string.end()};

	// Assume command will contain the format with parameter id in hex format
	std::size_t found = command_string.find("0x");
	if (found != std::string::npos) {
		command.write_to_flash = command_string[found - 1] == 'f';

		CopleyStatus amplifier_status;
		std::unique_ptr<CopleyResponse> response = _bus->write_command(command, amplifier_status);
		str										 = response->get_value_string();
	}

	return str;
}

CopleyScan::ParameterMap CopleyScan::map_parameters(const uint8_t node_id)
{
	ParameterMap parameter_map;

	std::unique_ptr<Discovered> amplifier = discover(node_id);
	if (amplifier) {
		for (CopleyParameter::MemoryLocation location :
			 {CopleyParameter::MemoryLocation::kVolatile, CopleyParameter::MemoryLocation::kNonVolatile}) {
			for (CopleyParameter::parameter_id_t parameter_id = 0x00; parameter_id <= 0x1A9; parameter_id++) {
				const CopleyParameter *const parameter = CopleyParameter::lookup(parameter_id, location);
				if (parameter) {
					std::string text_response;
					CopleyParameter::value_t value;
					if (amplifier->read_parameter(*parameter, location, value, &text_response)) {
						parameter_map.insert({{{parameter_id, location}, {value, text_response}}});
					} else {
						amplifier->clear_status();
					}
				}
			}
		}
		if (amplifier->status().ok()) {
			std::cout << "Memory map complete." << std::endl;
		}
	}

	return parameter_map;
}

std::string CopleyScan::map_parameters_pretty(const int node_id)
{
	std::string eep;
	std::string ram;
	ParameterMap parameter_map = map_parameters(node_id);
	for (auto &&parameter_index : parameter_map) {
		const CopleyParameter::parameter_id_t parameter_id = parameter_index.first.first;
		const CopleyParameter::MemoryLocation location	 = parameter_index.first.second;
		const CopleyParameter::value_t value			   = parameter_index.second.first;
		const std::string value_string					   = parameter_index.second.second;
		const CopleyParameter *const parameter			   = CopleyParameter::lookup(parameter_id, location);
		if (!parameter) {
			std::cerr << "Found an unrecognized parameter in the parameter map." << std::endl;
			continue;
		}

		// format the string as f0x01 (f1) = 0x00 0x01 (0 1) [Parameter Type]
		std::string &bank = location == CopleyParameter::MemoryLocation::kNonVolatile ? eep : ram;

		bank += parameter->to_string(location);
		bank += " = ";
		if (!value_string.empty()) {
			bank += value_string;
		} else {
			bank += to_hex(value, parameter->bytes) + " (" + std::to_string(value) + ")";
		}

		bank += " [" + parameter->type + "]\n";
	}

	std::cout << eep + ram << std::endl;
	return eep + ram;
}

} // namespace momentum
