# Creator Copley Amplifier Driver
> C++ driver for Copley Controls amplifiers

C++ driver to communicate with Copley Controls motor amplifiers.

# Supported hardware
In general any DC brushless motor, incremental encoder, and shaft encoder may be
used by adding the appropriate configuration to `CopleyModel.cpp`.

Amplifiers in the AccelNet Micro and AccelNet Plus family can also be added by
modifying `CopleyModel.cpp`. Models outside this family may require modification
of this driver.

See example [examples/copley-example-custom.cpp](examples/copley-example-custom.cpp)
for how to build a custom configuration for an arbitrary motor, primary encoder and load encoder.

### Copley Controls Amplifiers supported
* AccelNet ACK_055_10
* AccelNet ACK_090_20
* AccelNet BP2_090_20
* AccelNet BPL_090_30

### Motors supported
* Anaheim Automation BLD172D-24V-2000
* Anaheim Automation BLY171D-24V-4000
* Anaheim Automation BLWRPG173S-24V-4000
* Minebea BLDC36P16A-24V
* Maxon Ec-i 40 449464
* Teknic CPM-MCVC-2321P-RLN
* Teknic M-3432V-LN-02D

### Primary incremental encoders supported
* Avago HEDL-5540, 2000 CPR
* Teknic CPM-MCVC series, 2000 CPR
* Teknic CPM-MCVC series, 4000 CPR
* Teknic CPM-MCVC series, 8000 CPR
* US Digital E5-1000
* US Digital EM1-0-500
* MILE Encoder 453233
* Maxon MR encoder

### Load encoders supported
* Avago HEDS-9700#E50, 800 CPR
* RLS super small rotary
* RLS rotary magnetic encoder
* RLS absolute BiSS encoder

Complete control algorithms included in this library are parameters found
by combining an motor, primary encoder, and load encoder, and tuning
in Copley CME2:

| Motor       | Primary Encoder | Load Encoder |
| ----------- | --------------- | ------------ |
| Anaheim Automation BLY171D-24V-4000 | US Digital E5-1000 | none |
| Anaheim Automation BLD172D-24V-2000 | US Digital E5-1000 | none |
| Anaheim Automation BLD172D-24V-2000 | US Digital E5-1000 | RLS absolute BiSS encoder |
| Anaheim Automation BLWRPG173S-24V-4000 | none (use Hall feedback) | none |
| Teknic CPM-MCVC-2321P-RLN | Teknic CPM-MCVC series, 4000 CPR | none |
| Teknic CPM-MCVC-2321P-RLN | Teknic CPM-MCVC series, 4000 CPR | RLS absolute BiSS encoder |
| Teknic CPM-MCVC-2321P-RLN | Teknic CPM-MCVC series, 4000 CPR | RLS rotary magnetic encoder |
| Teknic M-3432V-LN-02D | Teknic CPM-MCVC series, 2000 CPR | none |
| Minebea BLDC36P16A-24V | none (use Hall feedback) | Avago HEDS-9700#E50, 800 CPR |
| Maxon Ec-i 40 449464 | none (use Hall feedback) | Avago HEDS-9700#E50, 800 CPR |

# Example Program

### Communicate with any amplifier (not controllable)
```c++
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

  Copley motor(bus,
    CopleyHardwareSpecification::invalid, // replace with your configuration
    node_id,
    motor_axis);

  motor.connect();
  std::cout << motor.read_serial_number() << std::endl;
  std::cout << "motor status: " << motor.status().to_string() << std::endl;
  std::cout << "bus status: " << bus->status().to_string() << std::endl;

  return 0;
}
```

See example [examples/copley-example-custom.cpp](examples/copley-example-custom.cpp)
for how to build a custom configuration for an arbitrary motor, primary encoder and load encoder.

# Authors
* Jeff C. Jensen (elgeeko1)
* Patrick Sherman (pdsherman-creator)
* Cam Bennett (cbennett24)
