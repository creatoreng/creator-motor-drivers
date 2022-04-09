/**
* @author Creator, Inc.
* @copyright Copyright (c) 2022, Creator, Inc. MIT license.
**/

#include <drivers/herkulex/Herkulex.hpp>

#include <iostream>

using namespace momentum;

int main(int argc, char *argv[])
{
	const uint8_t motor_id = 0x00;  // motor ID
  const std::string bus_address = "/dev/tty0";

  std::shared_ptr<HerkulexBus> bus(new HerkulexBus());
  bus->connect(bus_address);

  if(bus->is_connected()){
    Herkulex motor(bus);
    motor.connect(motor_id);
    if(motor.is_connected()){
      std::cout << "motor " << motor_id
        << " voltage: " << motor.read_voltage()
        << std::endl;
    }
		std::cout << "motor status: " << motor.status().to_string() << std::endl;
  }
	std::cout << "bus status: " << bus->status().to_string() << std::endl;

	return 0;
}
