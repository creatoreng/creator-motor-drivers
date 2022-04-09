/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Copley Amplifier bus scanner.
 **/

#pragma once

#include <drivers/copley/Copley.hpp>

namespace momentum
{

/// Copley amplifier scanner.
/// @warning Do not use for machine control. This class uses blocking methods and does not control amplifier values.
class CopleyScan
{
private:
	/// Parameter map type.
	/// < <id, location>, <value_int, value_string> >
	using ParameterMap = std::map<std::pair<CopleyParameter::parameter_id_t, CopleyParameter::MemoryLocation>,
					 std::pair<CopleyParameter::value_t, std::string>>;

	/// Discovered Copley amplifier. Amplifier is weakly connected.
	/// @warning Do not use for machine control. Scanned amplifiers are not configured and state is not managed.
	class Discovered : public Copley
	{
	public:
		/// Default constructor.
		/// @param [in,out] bus The Copley bus.
		/// @param [in] node_id The unique ID of this amplifier.
		explicit Discovered(const std::shared_ptr<CopleyBus> bus,
												const uint8_t node_id);

		/// Destructor
		~Discovered(void);

		/// Connect to the amplifier. Weak connection only.
		/// @note The communication bus must be connected before calling this method.
		/// @param [in] node_id The network node ID.
		/// @return true if connected.
		bool connect(const uint8_t node_id);

		/// Disconnect the amplifier. Weak disconnect only.
		void disconnect(void) override;

		/// Read a parameter from this amplifier.
		/// @param [in] parameter The parameter to read.
		/// @param [in] location The memory bank to read.
		/// @param [out] value Value of the parameter (1-4 bytes depending on the parameter).
		/// @param [out] value_string Optional buffer to receive string value of parameter.
		/// @return true if the read was successful.
		bool read_parameter(const CopleyParameter &parameter,
							const CopleyParameter::MemoryLocation location,
							CopleyParameter::value_t &value,
							std::string *const value_string = nullptr);

		/// Write a [arameter to this amplifier.
		/// @param [in] parameter The parameter to write.
		/// @param [in] location The memory bank to write.
		/// @param [in] Value to write to the parameter (1-4 bytes depending on the parameter).
		/// @return true if write was successful.
		bool write_parameter(const CopleyParameter &parameter,
							 const CopleyParameter::MemoryLocation location,
							 const CopleyParameter::value_t value);
	};

public:
	/// Default constructor.
	CopleyScan(void);

	/// Status of this object.
	std::string status_string(void) const;

	/// Connect to the bus.
	/// @param [in] address The serial port address of the bus.
	/// @return true if connected.
	bool connect_uart(const std::string &address);

	/// Scan for a Copley amplifier on the CAN bus.
	/// @param [in] starting_node_id The ID from which to begin the search.
	/// @return the node ID of the first amplifier found; CopleyCommand::kAmplifierIdMax if none found.
	int scan(const int starting_node_id = 0);

	/// Scan for all Copley amplifiers on the CAN bus.
	/// @param [in] starting_node_id The amplifier ID from which to begin the search.
	/// @return comma-separated list of all IDs found.
	std::string scan_all(const int starting_node_id = 0);

	/// Assign the network node ID of an amplifier.
	/// @note This method uses simple datatypes to facilitate code generation.
	/// @param [in] current_node_id The current ID of the amplifier.
	/// @param [in] new_id The new ID of the amplifier.
	/// @return true if successful.
	bool assign_node_id(const int current_node_id, const int new_node_id);

	/// Read a non-volatile parameter from an amplifier.
	/// @note This method uses simple datatypes to facilitate code generation.
	/// @param [in] node_id The amplifier ID.
	/// @param [in] parameter_id The parameter ID to read.
	/// @return value of the parameter.
	int read_nonvolatile_parameter(const int node_id, const int parameter_id);

	/// Read a volatile parameter from a Copley amplifier.
	/// @note This method uses simple datatypes to facilitate code generation.
	/// @param [in] node_id The amplifier ID.
	/// @param [in] parameter_id The parameter ID to read.
	/// @return value of the parameter.
	int read_volatile_parameter(const int node_id, const int parameter_id);

	/// Write a non-volatile parameter to a Copley amplifier.
	/// @note This method uses simple datatypes to facilitate code generation.
	/// @param [in] node_id The amplifier ID.
	/// @param [in] parameter_id The parameter ID to write.
	/// @param [in] Value to write to the parameter (1-4 bytes depending on the parameter).
	/// @return true if write was successful.
	bool write_nonvolatile_parameter(const int node_id, const int parameter_id, const int value);

	/// Write a volatile parameter to a Copley amplifier.
	/// @note This method uses simple datatypes to facilitate code generation.
	/// @param [in] node_id The amplifier ID.
	/// @param [in] parameter_id The parameter ID to write.
	/// @param [in] Value to write to the parameter (1-4 bytes depending on the parameter).
	/// @return true if write was successful.
	bool write_volatile_parameter(const int node_id, const int parameter_id, const int value);

	/// Write directly to the Copley bus.
	/// @param [in] command_string Command to write.
	/// @return The value returned from the bus.
	std::string write_bus(std::string command_string);

	/// Read all known parameters.
	/// @param [in] node_id The amplifier node ID.
	/// @param [in] location The memory bank to read. kBoth is not supported and will return RAM memory only.
	/// @return parameter map.
	ParameterMap map_parameters(const uint8_t node_id);

	/// Read all known parameters and print as a handsome string.
	/// @param [in] node_id The amplifier node ID.
	/// @return formatted parameter map.
	std::string map_parameters_pretty(const int node_id);

	/// Read hardware enable parameter from a Copley amplifier's flash memory.
	/// @param [in] node_id node_id of the scanned amplifier. Amplifier must be connected before calling this method.
	/// @return integer value of hardware enable pin configuration parameter.
	int read_hardware_enable_nonvolatile(const uint8_t node_id);

	/// Read hardware enable parameter from a Copley amplifier's RAM.
	/// @param [in] node_id node_id of the scanned amplifier. Amplifier must be connected before calling this method.
	/// @return integer value of hardware enable pin configuration parameter.
	int read_hardware_enable_volatile(const uint8_t node_id);

	/// Write hardware enable pin configuration to Amp Enable HI on both Flash & RAM on a Copley amplifier
	/// @param [in] node_id The amplifier node ID.
	bool write_hardware_enable(const uint8_t node_id);

private:
	/// Attempt to discover a amplifier. If a amplifier is discovered, it is wealy connected and
	/// a unique pointer is returned. If no amplifier is discovered, nullptr is returned.
	/// @param [in] node_id The amplifier id to discover.
	/// @return unique pointer to a connected amplifier; nullptr if connection failed.
	std::unique_ptr<Discovered> discover(const uint8_t node_id);

	/// Read a parameter from a Copley amplifier.
	/// @param [in,out] amplifier The scanned amplifier. Amplifier must be connected before calling this method.
	/// @param [in] parameter_id The parameter ID to read.
	/// @param [in] location The memory bank to read.
	/// @param [out] value Value of the parameter (1-4 bytes depending on the parameter).
	/// @return true if read was successful.
	bool read_parameter(const uint8_t node_id,
						const CopleyParameter::parameter_id_t parameter_id,
						const CopleyParameter::MemoryLocation location,
						CopleyParameter::value_t &value);

	/// Write a parameter to a Copley amplifier.
	/// @param [in] node_id The amplifier node ID.
	/// @param [in] id The parameter ID to write.
	/// @param [in] location The memory bank to write.
	/// @param [in] Value to write to the parameter (1-4 bytes depending on the parameter).
	/// @return true if write was successful.
	bool write_parameter(const uint8_t node_id,
						 const CopleyParameter::parameter_id_t parameter_id,
						 const CopleyParameter::MemoryLocation location,
						 const int32_t value);

	/// Return a hexidecimal string formatted 0x__ with variable width.
	/// @param [in] number The number to convert.
	/// @param [in] digits The number of digits to print.
	/// @return formatted string
	static std::string to_hex(unsigned int number, int digits);

	/// Copley bus.
	std::shared_ptr<CopleyBus> _bus;
};

} // namespace momentum
