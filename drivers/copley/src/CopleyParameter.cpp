/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/copley/CopleyParameter.hpp>

#include <iomanip>
#include <sstream>

namespace momentum
{

std::map<const CopleyParameter::parameter_id_t, const CopleyParameter *const> CopleyParameter::parameter_map;

CopleyParameter::CopleyParameter(const parameter_id_t src_id,
								 const std::string &src_type,
								 const size_t src_bytes,
								 const AccessMode src_access,
								 const MemoryLocation src_location,
								 const bool src_is_string,
								 const bool src_is_signed)
  : id(src_id),
	type(src_type),
	bytes(src_bytes),
	access(src_access),
	location(src_location),
	is_string(src_is_string),
	is_signed(src_is_signed)
{
	parameter_map.insert({id, this});
}

std::string CopleyParameter::to_string(const MemoryLocation location) const
{
	std::stringstream parameter_stream;
	parameter_stream << "0x" << std::setfill('0') << std::setw(3) << std::hex << id;
	return std::string(location == CopleyParameter::MemoryLocation::kNonVolatile ? "f" : "r") + parameter_stream.str();
}

const CopleyParameter *CopleyParameter::lookup(const parameter_id_t id, const MemoryLocation location)
{
	auto search = parameter_map.find(id);
	// found and memory bank matches?
	if (search != parameter_map.end() && (static_cast<int>(search->second->location) & static_cast<int>(location))) {
		return search->second;
	}
	return nullptr;
}

// clang-format off
// Current Control Loop

const CopleyParameter CopleyParameter::kCurrentLoopCp = {
		.id		 	= 0x00,
		.type 		= "Current Control Loop Proportional Gain",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kCurrentLoopCi = {
		.id		 	= 0x01,
		.type 		= "Current Control Loop Integral Gain",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kCurrentLoopProgrammedValue = {
		.id		 	= 0x02,
		.type 		= "Current Control Loop Programmed Value",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentWindingA = {
		.id		 	= 0x03,
		.type 		= "Winding A Current",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentWindingB = {
		.id		 	= 0x04,
		.type 		= "Winding B Current",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentOffsetA = {
		.id		 	= 0x05,
		.type 		= "Current Offset A",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentOffsetB = {
		.id		 	= 0x06,
		.type 		= "Current Offset B",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentStatorX = {
		.id		 	= 0x07,
		.type 		= "Stator Current Vector X",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentStatorY = {
		.id		 	= 0x08,
		.type 		= "Stator Current Vector Y",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLoopOutputStatorX = {
		.id		 	= 0x09,
		.type 		= "Current Control Loop Output Stator X",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLoopOutputStatorY = {
		.id		 	= 0x0A,
		.type 		= "Current Control Loop Output Stator Y",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLoopActualRotorD = {
		.id		 	= 0x0B,
		.type 		= "Current Control Loop Actual Rotor D",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLoopActualRotorQ = {
		.id		 	= 0x0C,
		.type 		= "Current Control Loop Actual Rotor Q",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLoopCommandedRotorD = {
		.id		 	= 0x0D,
		.type 		= "Current Control Loop Commanded Rotor D",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLoopCommandedRotorQ = {
		.id		 	= 0x0E,
		.type 		= "Current Control Loop Commanded Rotor Q",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLoopErrorRotorD = {
		.id		 	= 0x0F,
		.type 		= "Current Control Loop Error Rotor D",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLoopErrorRotorQ = {
		.id		 	= 0x10,
		.type 		= "Current Control Loop Error Rotor Q",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLoopIntegralRotorD = {
		.id		 	= 0x11,
		.type 		= "Current Control Loop Integral Value Rotor D",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLoopIntegralRotorQ = {
		.id		 	= 0x12,
		.type 		= "Current Control Loop Integral Value Rotor Q",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLoopOutputRotorD = {
		.id		 	= 0x13,
		.type 		= "Current Control Loop Output Rotor D",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLoopOutputRotorQ = {
		.id		 	= 0x14,
		.type 		= "Current Control Loop Output Rotor Q",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLoopCommanded = {
		.id		 	= 0x15,
		.type 		= "Commanded Motor Current",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kUnknown016 = {
		.id		 	= 0x16,
		.type 		= "?",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// measurements

const CopleyParameter CopleyParameter::kPositionActual = {
		.id		 	= 0x17,
		.type 		= "Actual Position",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kVelocityActual = {
		.id		 	= 0x18,
		.type 		= "Actual Velocity",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

// analog and A/D

const CopleyParameter CopleyParameter::kAnalogReferenceScaling = {
		.id		 	= 0x19,
		.type 		= "Analog Reference Scaling Factor",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kAnalogReferenceOffset = {
		.id		 	= 0x1A,
		.type 		= "Analog Reference Offset",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kAnalogEncoderSineInputVoltage = {
		.id		 	= 0x1B,
		.type 		= "Analog Encoder Sine Input Voltage",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kAnalogEncoderCosineInputVoltage = {
		.id		 	= 0x1C,
		.type 		= "Analog Encoder Cosine Input Voltage",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kAnalogReferenceInputVoltage = {
		.id		 	= 0x1D,
		.type 		= "Analog Reference Input Voltage",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kHighVoltage = {
		.id		 	= 0x1E,
		.type 		= "High Voltage",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kUnknown01F = {
		.id		 	= 0x1F,
		.type 		= "?",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveTemperature = {
		.id		 	= 0x20,
		.type 		= "Drive Temperature",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

// absolute limits

const CopleyParameter CopleyParameter::kCurrentLimitPeak = {
		.id		 	= 0x21,
		.type 		= "Peak Current Limit",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLimitContinuous = {
		.id		 	= 0x22,
		.type 		= "Continuous Current Limit",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLimitPeakTime = {
		.id		 	= 0x23,
		.type 		= "Peak Current Limit Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// motion control

const CopleyParameter CopleyParameter::kDriveMode = {
		.id		 	= 0x24,
		.type 		= "Drive Mode",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kLimitedMotorCurrentCommand = {
		.id		 	= 0x25,
		.type 		= "Limited Motor Current Command",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kAnalogReferenceInputDeadband = {
		.id		 	= 0x26,
		.type 		= "Limited Analog Reference Input Deadband",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kVelocityLoopVp = {
		.id		 	= 0x27,
		.type 		= "Velocity Control Loop Proportional Gain",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kVelocityLoopVi = {
		.id		 	= 0x28,
		.type 		= "Velocity Control Loop Integral Gain",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kVelocityLoopLimitedVelocity = {
		.id		 	= 0x29,
		.type 		= "Velocity Control Loop Limited Velocity",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kVelocityLoopError = {
		.id		 	= 0x2A,
		.type 		= "Velocity Control Loop Error",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kVelocityLoopIntegral = {
		.id		 	= 0x2B,
		.type 		= "Velocity Control Loop Integral",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kVelocityCommanded = {
		.id		 	= 0x2C,
		.type 		= "Commanded Velocity",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionCommanded = {
		.id		 	= 0x2D,
		.type 		= "Commanded Position",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kVelocityLoopAff = {
		.id		 	= 0x2E,
		.type 		= "Velocity Control Loop Acceleration Feed-Forward",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kVelocityProgrammed = {
		.id		 	= 0x2F,
		.type 		= "Programmed Velocity",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionLoopPp = {
		.id		 	= 0x30,
		.type 		= "Position Control Loop Proportional Gain",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kVelocityLoopShift = {
		.id		 	= 0x31,
		.type 		= "Velocity Control Loop Shift",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kActualMotorPosition = {
		.id		 	= 0x32,
		.type 		= "Actual Motor Position",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionLoopVff = {
		.id		 	= 0x33,
		.type 		= "Position Control Loop Velocity Feed Forward Gain",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kPositionLoopAff = {
		.id		 	= 0x34,
		.type 		= "Position Control Loop Acceleration Feed Forward Gain",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kPositionLoopError = {
		.id		 	= 0x35,
		.type 		= "Position Control Loop Error",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kVelocityLoopAccelLimit = {
		.id		 	= 0x36,
		.type 		= "Velocity Control Loop Acceleration Limit",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kVelocityLoopDecelLimit = {
		.id		 	= 0x37,
		.type 		= "Velocity Control Loop Deceleration Limit",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kCurrentActual = {
		.id		 	= 0x38,
		.type 		= "Actual Current",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kVelocityLoopEmergencyAccelLimit = {
		.id		 	= 0x39,
		.type 		= "Velocity Control Loop Emergency Stop Deceleration Limit",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kVelocityLoopVelocityLimit = {
		.id		 	= 0x3A,
		.type 		= "Veloctiy Control Loop Velocity Limit",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kTrajectoryCommandedVelocity = {
		.id		 	= 0x3B,
		.type 		= "Instantaneous Commanded Velocity",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kTrajectoryCommandedAcceleration = {
		.id		 	= 0x3C,
		.type 		= "Instantaneous Commanded Acceleration",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kTrajectoryPositionDestination = {
		.id		 	= 0x3D,
		.type 		= "Destination Position",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kVelocityWindow = {
		.id		 	= 0x3E,
		.type 		= "Velocity Control Loop Window",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kVelocityWindowTime = {
		.id		 	= 0x3F,
		.type 		= "Velocity Control Loop Window Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// Motor specifications

const CopleyParameter CopleyParameter::kMotorType = {
		.id		 	= 0x40,
		.type 		= "Motor Type",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kMotorManufacturer = {
		.id		 	= 0x41,
		.type 		= "Motor Manufacturer",
		.bytes		= 1,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kMotorModel = {
		.id		 	= 0x42,
		.type 		= "Motor Model",
		.bytes		= 1,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kMotorUnits = {
		.id		 	= 0x43,
		.type 		= "Motor Units",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorInertia = {
		.id		 	= 0x44,
		.type 		= "Motor Inertia",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorPolePairs = {
		.id		 	= 0x45,
		.type 		= "Motor Pole Pairs",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorBrakeType = {
		.id		 	= 0x46,
		.type 		= "Motor Brake Type",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kMotorTempSensorType = {
		.id		 	= 0x47,
		.type 		= "Motor Temperature Sensor Type",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kMotorTorqueConstant = {
		.id		 	= 0x48,
		.type 		= "Motor Torque Constant",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorResistance = {
		.id		 	= 0x49,
		.type 		= "Motor Resistance",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorInductance = {
		.id		 	= 0x4A,
		.type 		= "Motor Inductance",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorPeakTorque = {
		.id		 	= 0x4B,
		.type 		= "Motor Peak Torque",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorContinuousTorque = {
		.id		 	= 0x4C,
		.type 		= "Motor Continuous Torque",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorMaxVelocity = {
		.id		 	= 0x4D,
		.type 		= "Motor Max Velocity",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorWiring = {
		.id		 	= 0x4E,
		.type 		= "Motor Wiring",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kMotorHallOffset = {
		.id		 	= 0x4F,
		.type 		= "Motor Hall Offset",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorHallType = {
		.id		 	= 0x50,
		.type 		= "Motor Hall Type",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kUnknown051 = {
		.id		 	= 0x51,
		.type 		= "?",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kMotorHallWiring = {
		.id		 	= 0x52,
		.type 		= "Motor Hall Wiring",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorBrakeActivationTime = {
		.id		 	= 0x53,
		.type 		= "Motor Brake Activation Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kMotorBrakeDelayTime = {
		.id		 	= 0x54,
		.type 		= "Motor Brake Delay Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kMotorBrakeActivationVelocity = {
		.id		 	= 0x55,
		.type 		= "Motor Brake Activation Velocity",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorBackEmfConstant = {
		.id		 	= 0x56,
		.type 		= "Motor Back EMF Constant",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kStepperMicrostepsPerRev = {
		.id		 	= 0x57,
		.type 		= "Microsteps per Revolution",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kMotorGearRatio = {
		.id		 	= 0x58,
		.type 		= "Motor Gear Ratio",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kHallVelocityShift = {
		.id		 	= 0x59,
		.type 		= "Hall Velocity Shift",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// encoder

const CopleyParameter CopleyParameter::kEncoderOutputConfiguration = {
		.id		 	= 0x5A,
		.type 		= "Encoder Output Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kLoadEncoderResolution = {
		.id		 	= 0x5B,
		.type 		= "Load Encoder Resolution",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kLoadEncoderDirection = {
		.id		 	= 0x5C,
		.type 		= "Load Encoder Direction",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kLoadEncoderType = {
		.id		 	= 0x5D,
		.type 		= "Load Encoder Type",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kLoadEncoderVelocity = {
		.id		 	= 0x5E,
		.type 		= "Load Encoder Velocity",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kVelocityLoopOutputFilter = {
		.id		 	= 0x5F,
		.type 		= "Velocity Control Loop Output Filter",
		.bytes		= 28,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorEncoderType = {
		.id		 	= 0x60,
		.type 		= "Motor Encoder Type",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kLinearEncoderUnits = {
		.id		 	= 0x61,
		.type 		= "Linear Motor Encoder Units",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorEncoderCountsPerRevolution = {
		.id		 	= 0x62,
		.type 		= "Rotary Encoder Counts Per Revolution",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kLinearEncoderResolution = {
		.id		 	= 0x63,
		.type 		= "Linear Encoder Resolution",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kLinearEncoderElectricalDistance = {
		.id		 	= 0x64,
		.type 		= "Linear Encoder Electrical Distance",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kEncoderDirection = {
		.id		 	= 0x65,
		.type 		= "Encoder Direction",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kUnknown066 = {
		.id		 	= 0x66,
		.type 		= "?",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kAnalogEncoderShift = {
		.id		 	= 0x67,
		.type 		= "Analog Encoder Shift",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

// position capture

const CopleyParameter CopleyParameter::kPositionCaptureIndex = {
		.id		 	= 0x68,
		.type 		= "Position Capture Index",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorEncoderVelocityUnfiltered = {
		.id		 	= 0x69,
		.type 		= "Motor Encoder Velocity Unfiltered",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentCommandedRampLimit = {
		.id		 	= 0x6A,
		.type 		= "Current Commanded Ramp Limit",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// misc

const CopleyParameter CopleyParameter::kVelocityLoopInputFilter = {
		.id		 	= 0x6B,
		.type 		= "Velocity Control Loop Input Filter",
		.bytes		= 28,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kPositionCaptureControl = {
		.id			= 0x6C,
		.type		= "Position Capture Control",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionCaptureStatus = {
		.id			= 0x6D,
		.type		= "Position Capture Status",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kResolverCyclesPerRev = {
		.id		 	= 0x6E,
		.type 		= "Resolve Cycles per Revolution",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

// I/O

const CopleyParameter CopleyParameter::kPwmMode = {
		.id		 	= 0x6F,
		.type 		= "PWM Mode and Status",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDigitalOutput0Configuration = {
		.id		 	= 0x70,
		.type 		= "Digital Output 0 Configuration",
		.bytes		= 10,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalOutput1Configuration = {
		.id		 	= 0x71,
		.type 		= "Digital Output 1 Configuration",
		.bytes		= 10,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalOutput2Configuration = {
		.id		 	= 0x72,
		.type 		= "Digital Output 2 Configuration",
		.bytes		= 10,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalOutput3Configuration = {
		.id		 	= 0x73,
		.type 		= "Digital Output 3 Configuration",
		.bytes		= 10,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalOutput4Configuration = {
		.id		 	= 0x74,
		.type 		= "Digital Output 4 Configuration",
		.bytes		= 10,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalOutput5Configuration = {
		.id		 	= 0x75,
		.type 		= "Digital Output 5 Configuration",
		.bytes		= 10,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalOutput6Configuration = {
		.id		 	= 0x76,
		.type 		= "Digital Output 6 Configuration",
		.bytes		= 10,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalOutput7Configuration = {
		.id		 	= 0x77,
		.type 		= "Digital Output 7 Configuration",
		.bytes		= 10,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput0Configuration = {
		.id		 	= 0x78,
		.type 		= "Input 0 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput1Configuration = {
		.id		 	= 0x79,
		.type 		= "Input 1 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput2Configuration = {
		.id		 	= 0x7A,
		.type 		= "Input 2 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput3Configuration = {
		.id		 	= 0x7B,
		.type 		= "Input 3 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput4Configuration = {
		.id		 	= 0x7C,
		.type 		= "Input 4 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput5Configuration = {
		.id		 	= 0x7D,
		.type 		= "Input 5 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput6Configuration = {
		.id		 	= 0x7E,
		.type 		= "Input 6 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput7Configuration = {
		.id		 	= 0x7F,
		.type 		= "Input 7 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// drive configuration

const CopleyParameter CopleyParameter::kDriveModelNumber = {
		.id		 	= 0x80,
		.type 		= "Drive Model Number",
		.bytes		= 0,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveSerialNumber = {
		.id		 	= 0x81,
		.type 		= "Drive Serial Number",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveCurrentLimitPeak = {
		.id		 	= 0x82,
		.type 		= "Drive Peak Current Limit",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveCurrentLimitContinuous = {
		.id		 	= 0x83,
		.type 		= "Drive Continuous Current Limit",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveCurrentAtMaxA2D = {
		.id		 	= 0x84,
		.type 		= "Drive Current at Max A/D",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPwmPeriod = {
		.id		 	= 0x85,
		.type 		= "PWM Period",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kPwmLoopUpdatePeriod = {
		.id		 	= 0x86,
		.type 		= "PWM Loop Update Period",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveProductFamily = {
		.id		 	= 0x87,
		.type 		= "Drive Product Family",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveCurrentLimitPeakTime = {
		.id		 	= 0x88,
		.type 		= "Drive Peak Current Limit Time",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveVoltageMax = {
		.id		 	= 0x89,
		.type 		= "Drive Maximum Voltage",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveVoltageAtMaxA2D = {
		.id		 	= 0x8A,
		.type 		= "Drive Voltage at Max A/D",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveVoltageMin = {
		.id		 	= 0x8B,
		.type 		= "Drive Minimum Voltage",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveTempMax = {
		.id		 	= 0x8C,
		.type 		= "Drive Maximum Temperature",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveManufaturingInfo = {
		.id		 	= 0x8D,
		.type 		= "Drive Manufacturing Information",
		.bytes		= 0,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kAnalogInputScale = {
		.id		 	= 0x8E,
		.type 		= "Analog Input Scale",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveSerialBaudRate = {
		.id		 	= 0x90,
		.type 		= "Serial Port Baud Rate",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveSerialCommandWordsMax = {
		.id		 	= 0x91,
		.type 		= "Max Data Words per Command",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveName = {
		.id		 	= 0x92,
		.type 		= "Drive Name",
		.bytes		= 1,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kUnknown093 = {
		.id		 	= 0x93,
		.type 		= "?",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveFirmwareVersion = {
		.id		 	= 0x94,
		.type 		= "Drive Firmware Version",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveHostConfiguration = {
		.id		 	= 0x95,
		.type 		= "Host Configuration",
		.bytes		= 40,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kAnalogReferenceCalibrationOffset = {
		.id		 	= 0x96,
		.type 		= "Analog Reference Calibration Offset",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveTempCutoutHysteresis = {
		.id		 	= 0x97,
		.type 		= "Drive Over Temperature Cutout Hysteresis",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

// function generator

const CopleyParameter CopleyParameter::kFunctionGeneratorConfiguration = {
		.id		 	= 0x98,
		.type 		= "Function Generator Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kFunctionGeneratorFrequency = {
		.id		 	= 0x99,
		.type 		= "Function Generator Frequency",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kFunctionGeneratorAmplitude = {
		.id		 	= 0x9A,
		.type 		= "Function Generator Amplitude",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kFunctionGeneratorDutyCycle = {
		.id		 	= 0x9B,
		.type 		= "Function Generator Duty Cycle",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// misc - drive voltage

const CopleyParameter CopleyParameter::kDriveVoltageCutoutHysteresis = {
		.id		 	= 0x9C,
		.type 		= "Drive Over Voltage Cutout Hysteresis",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

// PWM

const CopleyParameter CopleyParameter::kPwmDeadTimeAtContinuousCurrentLimit = {
		.id		 	= 0x9D,
		.type 		= "PWM Dead Time at Continuous Current Limit",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kPwmOffTimeMin = {
		.id		 	= 0x9E,
		.type 		= "PWM Minimum Off Time",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kPwmDeadTimeAtZeroCurrent = {
		.id		 	= 0x9F,
		.type 		= "PWM Dead Time at Zero Current",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

// state and I/O

const CopleyParameter CopleyParameter::kDriveEventStatus = {
		.id		 	= 0xA0,
		.type 		= "Drive Event Status",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveLatchedEventStatus = {
		.id		 	= 0xA1,
		.type 		= "Drive Latched Event Status",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,	// special case: writable to clear bits
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kHallInputState = {
		.id		 	= 0xA2,
		.type 		= "Hall Input State",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kUnknown0A3 = {
		.id		 	= 0xA3,
		.type 		= "?",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveLatchingFaultStatus = {
		.id		 	= 0xA4,
		.type 		= "Latching Fault Status Register",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInputPullupConfiguration = {
		.id		 	= 0xA5,
		.type 		= "Digital Input Pullup Configuration Register",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInputState = {
		.id		 	= 0xA6,
		.type 		= "Digital Input Pin States",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveLatchingFaultMask = {
		.id		 	= 0xA7,
		.type 		= "Drive Latching Fault Mask",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInputCommandConfiguration = {
		.id		 	= 0xA8,
		.type 		= "Digital Input Command Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDigitalInputCommandScaling = {
		.id		 	= 0xA9,
		.type 		= "Digital Command Input Scaling Factor",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDigitalInputStateRaw = {
		.id		 	= 0xAA,
		.type 		= "Digital Input State Raw",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalOutputState = {
		.id		 	= 0xAB,
		.type 		= "Digital Output State",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveEventStatusSticky = {
		.id		 	= 0xAC,
		.type 		= "Drive Event Status Sticky Register",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveHardwareType = {
		.id		 	= 0xAD,
		.type 		= "Drive Hardware Type",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,	// error in datasheet, this register is flash only
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCurrentLoopOffset = {
		.id		 	= 0xAE,
		.type 		= "Current Control Loop Offset",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveOptions = {
		.id		 	= 0xAF,
		.type 		= "Drive Options",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// phasing

const CopleyParameter CopleyParameter::kPhaseAngle = {
		.id		 	= 0xB0,
		.type 		= "Motor Phase Angle",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPhaseMicrostepIncrementRate = {
		.id		 	= 0xB1,
		.type 		= "Phase Microstep Increment Rate",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPhaseMode = {
		.id		 	= 0xB2,
		.type 		= "Commutation Mode",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kAnalogEncoderScale = {
		.id		 	= 0xB3,
		.type 		= "Analog Encoder Scaling",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorEncoderPhaseAngle = {
		.id		 	= 0xB4,
		.type 		= "Motor Encoder Phase Angle",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

// misc

const CopleyParameter CopleyParameter::kHomeAdjustment = {
		.id		 	= 0xB5,
		.type 		= "Homing Adjustment",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPwmInputFrequency = {
		.id		 	= 0xB6,
		.type 		= "PWM Input Frequency",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveTime = {
		.id		 	= 0xB7,
		.type 		= "System Time",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

// position tracking

const CopleyParameter CopleyParameter::kPositionLimitPositive = {
		.id		 	= 0xB8,
		.type 		= "Positive Software Limit",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionLimitNegative = {
		.id		 	= 0xB9,
		.type 		= "Negative Software Limit",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionFollowingErrorLimit = {
		.id		 	= 0xBA,
		.type 		= "Position Control Loop Following Error Limit",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionFollowingWarningLimit = {
		.id		 	= 0xBB,
		.type 		= "Position Control Loop Following Warning Limit",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionTrackingWindow = {
		.id		 	= 0xBC,
		.type 		= "Position Control Loop Tracking Window",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionTrackingWindowTime = {
		.id		 	= 0xBD,
		.type 		= "Position Control Loop Tracking Window Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kPositionLimitDeceleration = {
		.id		 	= 0xBE,
		.type 		= "Position Software Limit Deceleration",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// misc - homing

const CopleyParameter CopleyParameter::kHomeHardStopCurrentDelayTime = {
		.id		 	= 0xBF,
		.type 		= "Homing Limit Hard Stop Current Delay Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// network

const CopleyParameter CopleyParameter::kNetworkNodeId = {
		.id		 	= 0xC0,
		.type 		= "Network Node ID",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kNetworkNodeIdConfiguration = {
		.id		 	= 0xC1,
		.type 		= "Network Node ID Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// homing

const CopleyParameter CopleyParameter::kHomeMethodConfiguration = {
		.id		 	= 0xC2,
		.type 		= "Home Method Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kHomeVelocityFast = {
		.id		 	= 0xC3,
		.type 		= "Home Velocity (fast moves)",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kHomeVelocitySlow = {
		.id		 	= 0xC4,
		.type 		= "Home Velocity (slow moves)",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kHomeAcceleration = {
		.id		 	= 0xC5,
		.type 		= "Home Acceleration/Deceleration",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kHomeOffset = {
		.id		 	= 0xC6,
		.type 		= "Home Offset",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kHomeHardStopCurrentLimit = {
		.id		 	= 0xC7,
		.type 		= "Home Hard Stop Current Limit",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// trajectory

const CopleyParameter CopleyParameter::kTrajectoryProfile = {
		.id		 	= 0xC8,
		.type 		= "Trajectory Profile Mode",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kTrajectoryStatus = {
		.id		 	= 0xC9,
		.type 		= "Trajectory Status",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kTrajectoryPosition = {
		.id		 	= 0xCA,
		.type 		= "Trajectory Generator Position",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kTrajectoryMaxVelocity = {
		.id		 	= 0xCB,
		.type 		= "Trajectory Generator Maximum Velocity",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kTrajectoryMaxAccel = {
		.id		 	= 0xCC,
		.type 		= "Trajectory Generator Maximum Acceleration",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kTrajectoryMaxDecel = {
		.id		 	= 0xCD,
		.type 		= "Trajectory Generator Maximum Deceleration",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kTrajectoryMaxJerk = {
		.id		 	= 0xCE,
		.type 		= "Trajectory Generator Maximum Jerk",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kTrajectoryAbortDecel = {
		.id		 	= 0xCF,
		.type 		= "Trajectory Generator Abort Deceleration",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// I/O

const CopleyParameter CopleyParameter::kDigitalInput8Configuration = {
		.id		 	= 0xD0,
		.type 		= "Digital Input 8 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput9Configuration = {
		.id		 	= 0xD1,
		.type 		= "Digital Input 9 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput10Configuration = {
		.id		 	= 0xD2,
		.type 		= "Digital Input 10 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput11Configuration = {
		.id		 	= 0xD3,
		.type 		= "Digital Input 11 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput12Configuration = {
		.id		 	= 0xD4,
		.type 		= "Digital Input 12 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput13Configuration = {
		.id		 	= 0xD5,
		.type 		= "Digital Input 13 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput14Configuration = {
		.id		 	= 0xD6,
		.type 		= "Digital Input 14 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput15Configuration = {
		.id		 	= 0xD7,
		.type 		= "Digital Input 15 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// regen resistor
const CopleyParameter CopleyParameter::kRegenResistance = {
		.id		 	= 0xD8,
		.type 		= "Regen Resistor Resistance",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kRegenPowerContinuous = {
		.id		 	= 0xD9,
		.type 		= "Regen Resistor Continuous Power",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kRegenPowerPeak = {
		.id		 	= 0xDA,
		.type 		= "Regen Resistor Peak Power",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kRegenPowerPeakTime = {
		.id		 	= 0xDB,
		.type 		= "Regen Resistor Power Peak Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kRegenVoltageOn = {
		.id		 	= 0xDC,
		.type 		= "Regen Resistor On Voltage",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kRegenVoltageOff = {
		.id		 	= 0xDD,
		.type 		= "Regen Resistor Off Voltage",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kRegenInternalCurrentPeak = {
		.id		 	= 0xDE,
		.type 		= "Drive Internal Regen Resistor Peak Current",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kRegenInternalCurrentContinuous = {
		.id		 	= 0xDF,
		.type 		= "Drive Internal Regen Resistor Continuous Current",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kRegenInternalCurrentPeakTime = {
		.id		 	= 0xE0,
		.type 		= "Drive Internal Regen Resistor Peak Current Time",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kRegenModel = {
		.id		 	= 0xE1,
		.type 		= "Regen Resistor Model",
		.bytes		= 0,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kRegenStatus = {
		.id		 	= 0xE2,
		.type 		= "Regen Resistor Status",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

// misc

const CopleyParameter CopleyParameter::kPositionLoopGainsMultiplier = {
		.id			= 0xE3,
		.type		= "Position Control Loop Gains Multiplier",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kPhaseInitializationCurrent = {
		.id		 	= 0xE4,
		.type 		= "Algorithmic Phase Initialization Current",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPhaseInitializationTimeout = {
		.id		 	= 0xE5,
		.type 		= "Algorithmic Phase Initialization Timeout",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// stepping

const CopleyParameter CopleyParameter::kStepperLoopVelocityAdjustmentMax = {
		.id		 	= 0xE6,
		.type 		= "Stepper Loop Maximum Velocity Adjustment",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kStepperLoopProportionalGain = {
		.id		 	= 0xE7,
		.type 		= "Stepper Loop Proportional Gain",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kStepperMicrostepHoldCurrent = {
		.id		 	= 0xE8,
		.type 		= "Stepper Microstepping Holding Current",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kStepperMicrostepRunToHoldTime = {
		.id		 	= 0xE9,
		.type 		= "Stepper Microstepping Run to Hold Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kStepperMicrostepDetentCorrectionGain = {
		.id		 	= 0xEA,
		.type 		= "Stepper Microstepping Detent Correction Gain",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kUnknown0EB = {
		.id		 	= 0xEB,
		.type 		= "?",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kUnknown0EC = {
		.id		 	= 0xEC,
		.type 		= "?",
		.bytes		= 18,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kStepperMicrostepHoldingCurrentToFixedVoltageTime = {
		.id		 	= 0xED,
		.type 		= "Stepper Microstepping Holding Current to Fixed Voltage Output Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kStepperConfiguration = {
		.id		 	= 0xEE,
		.type 		= "Stepper Configuration and Status",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kUnknown0EF = {
		.id		 	= 0xEF,
		.type 		= "?",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// I/O debouncing

const CopleyParameter CopleyParameter::kDigitalInput0DebounceTime = {
		.id		 	= 0xF0,
		.type 		= "Digital Input 0 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput1DebounceTime = {
		.id		 	= 0xF1,
		.type 		= "Digital Input 1 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput2DebounceTime = {
		.id		 	= 0xF2,
		.type 		= "Digital Input 2 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput3DebounceTime = {
		.id		 	= 0xF3,
		.type 		= "Digital Input 3 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput4DebounceTime = {
		.id		 	= 0xF4,
		.type 		= "Digital Input 4 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput5DebounceTime = {
		.id		 	= 0xF5,
		.type 		= "Digital Input 5 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput6DebounceTime = {
		.id		 	= 0xF6,
		.type 		= "Digital Input 6 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput7DebounceTime = {
		.id		 	= 0xF7,
		.type 		= "Digital Input 7 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput8DebounceTime = {
		.id		 	= 0xF8,
		.type 		= "Digital Input 8 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput9DebounceTime = {
		.id		 	= 0xF9,
		.type 		= "Digital Input 9 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput10DebounceTime = {
		.id		 	= 0xFA,
		.type 		= "Digital Input 10 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput11DebounceTime = {
		.id		 	= 0xFB,
		.type 		= "Digital Input 11 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput12DebounceTime = {
		.id		 	= 0xFC,
		.type 		= "Digital Input 12 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput13DebounceTime = {
		.id		 	= 0xFD,
		.type 		= "Digital Input 13 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput14DebounceTime = {
		.id		 	= 0xFE,
		.type 		= "Digital Input 14 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput15DebounceTime = {
		.id		 	= 0xFF,
		.type 		= "Digital Input 15 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// network

const CopleyParameter CopleyParameter::kNetworkCANLimitStatusMask = {
		.id		 	= 0x100,
		.type 		= "CAN Limit Status Mask",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kNetworkCANAddressSwitch = {
		.id		 	= 0x101,
		.type 		= "CAN Network Address Switch",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kNetworkStatus = {
		.id		 	= 0x102,
		.type 		= "Network Status",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kNetworkNodeIdPinMapping = {
		.id		 	= 0x103,
		.type 		= "Network Node ID Input Pin Mapping",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

// misc - phase initialization configuration

const CopleyParameter CopleyParameter::kPhaseInitializationConfiguration = {
		.id		 	= 0x104,
		.type 		= "Algorithmic Phase Initialization Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// camming

const CopleyParameter CopleyParameter::kCammingConfiguration = {
		.id		 	= 0x105,
		.type 		= "Camming Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kCammingForwardDelay = {
		.id		 	= 0x106,
		.type 		= "Camming Forward Motion Delay",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCammingReverseDelay = {
		.id		 	= 0x107,
		.type 		= "Camming Reverse Motion Delay",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// wtf?
const CopleyParameter CopleyParameter::kNetworkCANSendPdoType254 = {
		.id		 	= 0x108,
		.type 		= "Network CAN Send PDO Type 254",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kCammingVelocityMaster = {
		.id		 	= 0x109,
		.type 		= "Camming Master Velocity",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// misc

const CopleyParameter CopleyParameter::kHomePositionCapture = {
		.id			= 0x10A,
		.type		= "Captured Home Position",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveFirmwareVersionExtended = {
		.id		 	= 0x10B,
		.type 		= "Drive Firmware Version Number (extended)",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

// network

const CopleyParameter CopleyParameter::kNetworkCANHeartbeatTime = {
		.id		 	= 0x10C,
		.type 		= "CAN Heartbeat Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kNetworkCANNodeGuardingTime = {
		.id		 	= 0x10D,
		.type 		= "CAN Node Guarding Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kNetworkCANNodeGuardingLifetimeFactor = {
		.id		 	= 0x10E,
		.type 		= "CAN Node Guarding Lifetime Factor",
		.bytes		= 1,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveModePulsePositionRegistrationOffset = {
		.id		 	= 0x10F,
		.type 		= "Pulse & Direction Mode Position Registration Offset",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionCaptureHighspeedTimestamp = {
		.id		 	= 0x110,
		.type 		= "Position Capture High Speed Time Stamp",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionCaptureHighspeedPosition = {
		.id		 	= 0x111,
		.type 		= "Position Capture High Speed Position",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kLoadEncoderPosition = {
		.id		 	= 0x112,
		.type 		= "Load Encoder Position",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kNetworkCANEmergencyInhibitTime = {
		.id		 	= 0x113,
		.type 		= "CAN Emergency Inhibit Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// loop gains

const CopleyParameter CopleyParameter::kVelocityLoopViDrain = {
		.id			= 0x114,
		.type		= "Velocity Control Loop Integral Drain",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kTrajectoryBuffer = {
		.id		 	= 0x115,
		.type 		= "Trajectory Buffer",
		.bytes		= 10,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kNetworkCANQuickStopOption = {
		.id		 	= 0x116,
		.type 		= "CAN Quick Stop Option Code",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kNetworkCANShutdownOption = {
		.id		 	= 0x117,
		.type 		= "CAN Shutdown Option Code",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kNetworkCANDisableOption = {
		.id		 	= 0x118,
		.type 		= "CAN Disable Option Code",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kNetworkCANHaltOption = {
		.id		 	= 0x119,
		.type 		= "CAN Halt Option Code",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveUnitScaling = {
		.id		 	= 0x11A,
		.type 		= "Drive Scaling Configuration",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveAxisCount = {
		.id		 	= 0x120,
		.type 		= "Drive Axis Count",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kNetworkOptions = {
		.id		 	= 0x121,
		.type 		= "Network Options",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kRegenInternalCurrentLimit = {
		.id		 	= 0x122,
		.type 		= "Regen Resistor Internal Current Limit",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

// encoder

const CopleyParameter CopleyParameter::kMotorEncoderPositionWrap = {
		.id		 	= 0x123,
		.type 		= "Motor Encoder Wrap Position",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kLoadEncoderPositionWrap = {
		.id		 	= 0x124,
		.type 		= "Load Encoder Wrap Position",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kEncoderCaptureConfiguration = {
		.id		 	= 0x125,
		.type 		= "MACRO Drive Encoder Capture Circuit Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// misc

const CopleyParameter CopleyParameter::kFpgaFirmwareVersion = {
		.id		 	= 0x126,
		.type 		= "FPGA Firmware Version Number",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

// gain scheduling

const CopleyParameter CopleyParameter::kGainSchedulingConfiguration = {
		.id		 	= 0x127,
		.type 		= "Gain Scheduling Configuration",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kGainSchedulingKey = {
		.id		 	= 0x128,
		.type 		= "Gain Scheduling Key",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveHardwareOptions = {
		.id		 	= 0x129,
		.type 		= "Drive Hardware Options",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kMotorEncoderOptions = {
		.id		 	= 0x12A,
		.type 		= "Motor Encoder Options",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kLoadEncoderOptions = {
		.id		 	= 0x12B,
		.type 		= "Load Encoder Options",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveSecondaryFirmwareVersion = {
		.id		 	= 0x12C,
		.type 		= "Drive Secondary Firmware Version",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kMotorEncoderStatus = {
		.id		 	= 0x12E,
		.type 		= "Motor Encoder Status",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kLoadEncoderStatus = {
		.id		 	= 0x12F,
		.type 		= "Load Encoder Status",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

// drive current

const CopleyParameter CopleyParameter::kDriveCurrentRMSPeriod = {
		.id		 	= 0x130,
		.type 		= "RMS Current Calculation Period",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveCurrentRMS = {
		.id		 	= 0x131,
		.type 		= "RMS Current",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveCurrentUserLimitRunningSum = {
		.id		 	= 0x132,
		.type 		= "User Current Limit Running Sum",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveCurrentAmpLimitRunningSum = {
		.id		 	= 0x133,
		.type 		= "Amplifier Current Limit Running Sum",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

// analog

const CopleyParameter CopleyParameter::kAnalogOutputConfiguration = {
		.id		 	= 0x134,
		.type 		= "Analog Output Configuration",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kAnalogOutputValue = {
		.id		 	= 0x135,
		.type 		= "Analog Output Value",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kAnalogInputSecondaryReferenceValue = {
		.id		 	= 0x136,
		.type 		= "Secondary Analog Input Reference Value",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kAnalogInputSecondaryReferenceOffset = {
		.id		 	= 0x137,
		.type 		= "Secondary Analog Input Reference Offset",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kAnalogInputSecondaryCalibrationOffset = {
		.id		 	= 0x138,
		.type 		= "Secondary Analog Input Calibration Offset",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// motor

const CopleyParameter CopleyParameter::kDriveSafetyCircuitStatus = {
		.id		 	= 0x139,
		.type 		= "Drive Safety Circuit Status",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorTempSensorVoltage = {
		.id		 	= 0x13A,
		.type 		= "Motor Temperature Sensor Voltage",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorTempSensorLimit = {
		.id		 	= 0x13B,
		.type 		= "Motor Temperature Sensor Limit",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// misc

const CopleyParameter CopleyParameter::kPwmPulseWidthMin = {
		.id		 	= 0x13C,
		.type 		= "PWM Minimum Pulse Width",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPwmPulseWidthMax = {
		.id		 	= 0x13D,
		.type 		= "PWM Maximum Pulse Width",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// Control Loop filters, configuration and gains

const CopleyParameter CopleyParameter::kVelocityLoopOutputFilter2 = {
		.id		 	= 0x150,
		.type 		= "Velocity Control Loop Output Filter 2",
		.bytes		= 28,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kVelocityLoopOutputFilter3 = {
		.id		 	= 0x151,
		.type 		= "Velocity Control Loop Output Filter 3",
		.bytes		= 28,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kCurrentLoopInputFilter = {
		.id		 	= 0x152,
		.type 		= "Current Control Loop Input Filter",
		.bytes		= 28,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kCurrentLoopInputFilter2 = {
		.id		 	= 0x153,
		.type 		= "Current Control Loop Input Filter 2",
		.bytes		= 28,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kServoLoopConfiguration = {
		.id		 	= 0x154,
		.type 		= "Servo Loop Configuration",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionLoopPi = {
		.id		 	= 0x155,
		.type 		= "Position Control Loop Integral Gain",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionLoopPd = {
		.id		 	= 0x156,
		.type 		= "Position Control Loop Derivative Gain",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kVelocityLoopCommandFf = {
		.id		 	= 0x157,
		.type 		= "Velocity Control Loop Command Feed Forward",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionLoopIntegralDrain = {
		.id		 	= 0x158,
		.type 		= "Position Control Loop Integral Drain",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// misc

const CopleyParameter CopleyParameter::kNetworkCANAbortOption = {
		.id		 	= 0x159,
		.type 		= "CAN Abort Option Code",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};


const CopleyParameter CopleyParameter::kDigitalOptions = {
		.id		 	= 0x15A,
		.type 		= "I/O Options",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kMotorBrakeEnableDelay = {
		.id		 	= 0x15B,
		.type 		= "Motor Brake Enable Delay",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

// digital I/O

const CopleyParameter CopleyParameter::kDigitalInputStateExtended = {
		.id		 	= 0x15C,
		.type 		= "Digital Input State (extended)",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInputStateRawExtended = {
		.id		 	= 0x15D,
		.type 		= "Digital Input State Raw (exented)",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInputPullupConfigurationExtended = {
		.id		 	= 0x15E,
		.type 		= "Digital Input Pullup Configuration (extended)",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput16Configuration = {
		.id		 	= 0x160,
		.type 		= "Digital Input 16 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput17Configuration = {
		.id		 	= 0x161,
		.type 		= "Digital Input 17 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput18Configuration = {
		.id		 	= 0x162,
		.type 		= "Digital Input 18 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput19Configuration = {
		.id		 	= 0x163,
		.type 		= "Digital Input 19 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput20Configuration = {
		.id		 	= 0x164,
		.type 		= "Digital Input 20 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput21Configuration = {
		.id		 	= 0x165,
		.type 		= "Digital Input 20 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput22Configuration = {
		.id		 	= 0x166,
		.type 		= "Digital Input 21 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput23Configuration = {
		.id		 	= 0x167,
		.type 		= "Digital Input 22 Configuration",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput16DebounceTime = {
		.id		 	= 0x170,
		.type 		= "Digital Input 16 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput17DebounceTime = {
		.id		 	= 0x171,
		.type 		= "Digital Input 17 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput18DebounceTime = {
		.id		 	= 0x172,
		.type 		= "Digital Input 18 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput19DebounceTime = {
		.id		 	= 0x173,
		.type 		= "Digital Input 19 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput20DebounceTime = {
		.id		 	= 0x174,
		.type 		= "Digital Input 20 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput21DebounceTime = {
		.id		 	= 0x175,
		.type 		= "Digital Input 21 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput22DebounceTime = {
		.id		 	= 0x176,
		.type 		= "Digital Input 22 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalInput23DebounceTime = {
		.id		 	= 0x177,
		.type 		= "Digital Input 23 Debounce Time",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// drive modes

const CopleyParameter CopleyParameter::kDriveModeUVConfiguration = {
		.id		 	= 0x180,
		.type 		= "UV Configuration",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDriveModeUV_U_Input = {
		.id		 	= 0x181,
		.type 		= "U Input",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveModeUV_V_Input = {
		.id		 	= 0x182,
		.type 		= "V Input",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveModePulseCounterRaw = {
		.id		 	= 0x183,
		.type 		= "Pulse & Direction Mode Counter Raw",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

// misc - position loop input filter

const CopleyParameter CopleyParameter::kPositionLoopInputFilter = {
		.id		 	= 0x184,
		.type 		= "Position Control Loop Input Shaping Filter",
		.bytes		= 80,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// I/O output comparators

const CopleyParameter CopleyParameter::kDigitalOutputCompareConfiguration = {
		.id		 	= 0x185,
		.type 		= "Digital Output Compare Configuration",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalOutputCompareStatus = {
		.id		 	= 0x186,
		.type 		= "Digital Output Compare Status",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalOutputCompareValue1 = {
		.id		 	= 0x187,
		.type 		= "Digital Output Compare Value 1",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDigitalOutputCompareValue2 = {
		.id		 	= 0x188,
		.type 		= "Digital Output Compare Value 2",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDigitalOutputCompareIncrement = {
		.id		 	= 0x189,
		.type 		= "Digital Output Compare Increment",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDigitalOutputComparePulseWidth = {
		.id		 	= 0x18A,
		.type 		= "Digital Output Compare Pulse Width",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

// misc - trajectory options, I/O extension

const CopleyParameter CopleyParameter::kTrajectoryOptions = {
		.id		 	= 0x18B,
		.type 		= "Trajectory Options",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kDriveIoExtensionConfiguration = {
		.id		 	= 0x18C,
		.type 		= "Drive I/O Extension Configuration",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// SPI

const CopleyParameter CopleyParameter::kSpiTransmit = {
		.id		 	= 0x18D,
		.type 		= "I/O Extension Transmit Data",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kSpiReceive = {
		.id		 	= 0x18E,
		.type 		= "I/O Extension Receive Data",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

// encoder

const CopleyParameter CopleyParameter::kAnalogEncoderSineOffset = {
		.id		 	= 0x18F,
		.type 		= "Analog Encoder Sine Offset",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kAnalogEncoderCosineOffset = {
		.id		 	= 0x190,
		.type 		= "Analog Encoder Cosine Offset",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kAnalogEncoderCosineScaling = {
		.id		 	= 0x191,
		.type 		= "Analog Encoder Cosine Scaling Factor",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kMotorEncoderCalibration = {
		.id		 	= 0x192,
		.type 		= "Motor Encoder Calibration",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kLoadEncoderCalibration = {
		.id		 	= 0x193,
		.type 		= "Load Encoder Calibration",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// misc

const CopleyParameter CopleyParameter::kPwmInputDutyCycle = {
		.id		 	= 0x194,
		.type 		= "PWM Input Duty Cycle",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kTrajectoryAbortJerk = {
		.id		 	= 0x195,
		.type 		= "Trajectory Abort Jerk",
		.bytes		= 4,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kAnalogEncoderMagnitude = {
		.id		 	= 0x196,
		.type 		= "Analog Encoder Magnitude",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

// position control loop gains - multi-axis cross-coupling

const CopleyParameter CopleyParameter::kPositionLoopCrossCouplingKp = {
		.id		 	= 0x197,
		.type 		= "Position Control Loop Cross Coupling Kp",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionLoopCrossCouplingKi = {
		.id		 	= 0x198,
		.type 		= "Position Control Loop Cross Couping Ki",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionLoopCrossCouplingKd = {
		.id		 	= 0x199,
		.type 		= "Position Control Loop Cross Coupling Kd",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

// misc

const CopleyParameter CopleyParameter::kMotorTempSensorSteinhartConstants = {
		.id		 	= 0x19A,
		.type 		= "Motor Temperature Sensor Steinhart Constants",
		.bytes		= 10,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kPwmDeadtimeCurrent = {
		.id		 	= 0x19B,
		.type 		= "PWM Deadtime Current",
		.bytes		= 2,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kNonVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kLoadEncoderPassiveHighspeedCapture = {
		.id		 	= 0x19C,
		.type 		= "Passive Load Encoder High Speed Capture",
		.bytes		= 4,
		.access		= AccessMode::kReadOnly,
		.location	= MemoryLocation::kVolatile,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kMotorWiringCheckCurrent = {
		.id		 	= 0x19D,
		.type 		= "Open Motor Wiring Check Current",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kPositionLoopErrorTimeout = {
		.id		 	= 0x19E,
		.type 		= "Position Error Timeout",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= false
};

// I/O - digital output configuration

const CopleyParameter CopleyParameter::kDigitalOutput9Configuration = {
		.id		 	= 0x1A0,
		.type 		= "Digital Output 9 Configuration",
		.bytes		= 10,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalOutput10Configuration = {
		.id		 	= 0x1A1,
		.type 		= "Digital Output 10 Configuration",
		.bytes		= 10,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= true,
		.is_signed	= false
};


const CopleyParameter CopleyParameter::kDigitalOutput11Configuration = {
		.id		 	= 0x1A2,
		.type 		= "Digital Output 11 Configuration",
		.bytes		= 10,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= true,
		.is_signed	= false
};

const CopleyParameter CopleyParameter::kDigitalOutput12Configuration = {
		.id		 	= 0x1A3,
		.type 		= "Digital Output 12 Configuration",
		.bytes		= 10,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= true,
		.is_signed	= false
};

//encoder

const CopleyParameter CopleyParameter::kMotorEncoderDownShift = {
		.id		 	= 0x1A8,
		.type 		= "Motor Encoder Down-Shift",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};

const CopleyParameter CopleyParameter::kLoadEncoderDownShift = {
		.id		 	= 0x1A9,
		.type 		= "Load Encoder Down-Shift",
		.bytes		= 2,
		.access		= AccessMode::kReadWrite,
		.location	= MemoryLocation::kBoth,
		.is_string	= false,
		.is_signed	= true
};
// clang-format on

} // namespace momentum
