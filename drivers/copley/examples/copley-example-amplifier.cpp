/**
* @author Creator, Inc.
* @copyright Copyright (c) 2022, Creator, Inc. MIT license.
**/

#include <drivers/copley/Copley.hpp>

#include <iostream>

using namespace momentum;

int main(int argc, char *argv[])
{
  const uint8_t node_id = 0x00;  // CAN bus ID; 0 for UART bus master
  const uint8_t motor_axis = 0;  // motor axis ID (used for dual-axis motors)
  const std::string bus_address = "/dev/tty0";

  std::shared_ptr<CopleyBus> bus(new CopleyBus());
  bus->connect(bus_address);

  if(bus->is_connected()){
    Copley motor(bus,
      	CopleyHardwareSpecification::invalid, // replace with your configuration
      	node_id,
      	motor_axis);

    motor.connect();
    if(motor.is_connected()){
      std::cout << "node " << node_id
        << " serial number: " << motor.read_serial_number()
        << std::endl;
    }
		std::cout << "motor status: " << motor.status().to_string() << std::endl;
  }
	std::cout << "bus status: " << bus->status().to_string() << std::endl;

	return 0;
}
