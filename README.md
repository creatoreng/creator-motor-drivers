# Creator Motor Drivers
> C++ drivers for Copley Controls amplifiers
> and Dongbu Robotics Herkulex motors

C++ applications to communicate with Copley Controls motor amplifiers
controlling DC brushless (DCBL) motors, and Donbu Robotics Herkulex motors,
over UART serial ports in Linux.

These drivers communicate to a bus master using a
standard Linux serial port, such as `/dev/tty0`, and any other motors
connected to the CAN bus of the UART amplifier.

Drivers are designed
to be used in a synchronous, single-threaded environment, and methods
are minimally blocking. Methods are provided to start motion and
periodically poll the status. Low-level motor control, such as
trajectory planning and detecting limit switches, are offloaded to
the motor controller itself.

These drivers are user-space (and not kernel) and generally portable.
We have tested these extensively in amd64 and armhf platforms.

This software is _not_ designed to be used in real-time applications,
but could presumably be adapted for them.

See [drivers/copley/README.md](drivers/copley/README.md) for the Copley driver,
and [drivers/herkulex/README.md](drivers/herkulex/README.md) for the Herkulex driver.


# Compile and Run Examples
Compile all code and examples
```sh
mkdir -p build
cd build
cmake ../
make all
```

Run examples
```sh
# scan for all Copley amplifiers on a bus
./build/drivers/copley/examples/copley-scan

# connect to a Copley amplifier
./build/drivers/copley/examples/copley-example-amplifier

# scan for all Herkulex motors on a bus
./build/drivers/herkulex/examples/herkulex-scan

# connect to a Herkulex motor
./build/drivers/herkulex/examples/herkulex-example
```
_Depending on your system, you may need to run
these commands with root or `sudo` privileges._

### Prerequesites
Compiling
* CMake 3.12 of later (tested through 3.23)
* gcc supporting c++11 or later (tested through c++20)
* boost 1.65 or later (tested through 1.71)

Runtime
* Linux (POSIX)

# Authors
* Jeff C. Jensen (elgeeko1)
* Patrick Sherman (pdsherman-creator)
* Cam Bennett (cbennett24)
