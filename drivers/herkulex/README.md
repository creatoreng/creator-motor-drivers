# Creator Herkulex Driver
> C++ drivers for Dongbu Robotics Herkulex motors

C++ drivers for Donbu Robotics Herkulex motors.

# Supported hardware
The following Herkulex motor models are supported
* DRS-0101
* DRS-0201
* DRS-0401
* DRS-0402
* DRS-0601
* DRS-0602

An optional gear ratio parameter may be used to allow for additional
gearing applied after the motor shaft.

# Example Program
```c++
#include <drivers/herkulex/Herkulex.hpp>
#include <iostream>

using namespace momentum;

int main(int argc, char *argv[])
{
  const uint8_t motor_id = 0x00;  // motor ID
  const std::string bus_address = "/dev/tty0";

  std::shared_ptr<HerkulexBus> bus(new HerkulexBus());
  bus->connect(bus_address);

  Herkulex motor(bus);
  motor.connect(motor_id);
  std::cout << "voltage: " << motor.read_voltage() << std::endl;
  std::cout << "motor status: " << motor.status().to_string();
  std::cout << "bus status: " << bus->status().to_string() << std::endl;

  return 0;
}
```

# Authors
* Jeff C. Jensen (elgeeko1)
* Patrick Sherman (pdsherman-creator)
* Cam Bennett (cbennett24)
