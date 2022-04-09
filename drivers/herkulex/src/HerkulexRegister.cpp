/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/herkulex/HerkulexRegister.hpp>

namespace momentum
{

std::map<const std::pair<uint8_t, HerkulexRegister::MemoryLocation>, const HerkulexRegister *const>
	HerkulexRegister::register_map;

HerkulexRegister::HerkulexRegister(const uint8_t src_address,
								   const std::string &src_type,
								   const uint8_t src_bytes,
								   const Range src_range,
								   const AccessMode src_access,
								   const MemoryLocation src_location)
  : address(src_address), type(src_type), bytes(src_bytes), range(src_range), access(src_access), location(src_location)
{
	register_map.insert({{address, location}, this});
}

const HerkulexRegister *HerkulexRegister::lookup(const uint8_t address, const MemoryLocation location)
{
	auto search = register_map.find({address, location});
	if (search != register_map.end()) {
		return search->second;
	} else {
		return nullptr;
	}
}

// clang-format off
//--------------
// EEP registers
//--------------
const HerkulexRegister HerkulexRegister::kEepModelNo1 = {
		.address =	0,
		.type =		"Model No1",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepModelNo2 = {
		.address =	1,
		.type =		"Model No2",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepVersion1 = {
		.address =	2,
		.type =		"Version1",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepVersion2 = {
		.address =	3,
		.type =		"Version2",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepBaudRate = {
		.address =	4,
		.type =		"Baud Rate",
		.bytes =	1,
		.range =	{ 0x01, 0x22 },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepReserved5 = {
		.address =	5,
		.type =		"Reserved",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReserved,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepId = {
		.address =	6,
		.type =		"ID",
		.bytes =	1,
		.range =	{ 0x00, 0xFE },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepAckPolicy = {
		.address =	7,
		.type =		"Ack Policy",
		.bytes =	1,
		.range =	{ 0x00, 0x02 },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepAlarmLedPolicy = {
		.address =	8,
		.type =		"Alarm LED Policy",
		.bytes =	1,
		.range =	{ 0x00, 0x7F },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepTorquePolicy = {
		.address =	9,
		.type =		"Torque Policy",
		.bytes =	1,
		.range =	{ 0x00, 0x7F },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepReserved10 = {
		.address =	10,
		.type =		"Reserved",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReserved,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepMaxTemp = {
		.address =	11,
		.type =		"Max Temp",
		.bytes =	1,
		.range =	{ 0x00, 0x6E },	// max of any Herkulex is 110'C
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepMinVoltage = {
		.address =	12,
		.type =		"Min Voltage",
		.bytes =	1,
		.range =	{ 0x5B, 0xC8 },	// min 6.714V (101), max 20 (602)
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepMaxVoltage = {
		.address =	13,
		.type =		"Max Voltage",
		.bytes =	1,
		.range =	{ 0x00, 0xFE },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepAccelRatio = {
		.address =	14,
		.type =		"Accel Ratio",
		.bytes =	1,
		.range =	{ 0x00, 0x32 },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepMaxAccelTime = {
		.address =	15,
		.type =		"Max Accel Time",
		.bytes =	1,
		.range =	{ 0x00, 0xFE },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepDeadZone = {
		.address =	16,
		.type =		"Dead Zone",
		.bytes =	1,
		.range =	{ 0x00, 0xFE },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepSaturatorOffset = {
		.address =	17,
		.type =		"Saturator Offset",
		.bytes =	1,
		.range =	{ 0x00, 0xFE },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepSaturatorSlope = {
		.address =	18,
		.type =		"Saturator Slope",
		.bytes =	2,
		.range =	{ 0x0000, 0x7FFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepPwmOffset = {
		.address =	20,
		.type =		"PWM Offset",
		.bytes =	1,
		.range =	{ 0x00, 0x00 },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepMinPwm = {
		.address =	21,
		.type =		"Min PWM",
		.bytes =	1,
		.range =	{ 0x00, 0xFE },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepMaxPwm = {
		.address =	22,
		.type =		"Max PWM",
		.bytes =	2,
		.range =	{ 0x0000, 0x03FF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepOverloadPwmThreshold = {
		.address =	24,
		.type =		"Overload PWM Threshold",
		.bytes =	2,
		.range =	{ 0x0000, 0x7FFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepMinPosition = {
		.address =	26,
		.type =		"Min Position",
		.bytes =	2,
		.range =	{ 0x0000, 0x7FFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepMaxPosition = {
		.address =	28,
		.type =		"Max Position",
		.bytes =	2,
		.range =	{ 0x0000, 0x7FFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepPositionKp = {
		.address =	30,
		.type =		"Position Kp",
		.bytes =	2,
		.range =	{ 0x0000, 0x7FFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepPositionKd = {
		.address =	32,
		.type =		"Position Kd",
		.bytes =	2,
		.range =	{ 0x0000, 0x7FFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepPositionKi = {
		.address =	34,
		.type =		"Position Ki",
		.bytes =	2,
		.range =	{ 0x0000, 0x7FFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepPositionFeedFwdGain1 = {
		.address =	36,
		.type =		"Position Fwd Gain 1",
		.bytes =	2,
		.range =	{ 0x0000, 0x7FFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepPositionFeedFwdGain2 = {
		.address =	38,
		.type =		"Position Fwd Gain 2",
		.bytes =	2,
		.range =	{ 0x0000, 0x7FFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepVelocityKp = {
		.address =	40,
		.type =		"Velocity Kp",
		.bytes =	2,
		.range =	{ 0x0000, 0x7FFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepVelocityKi = {
		.address =	42,
		.type =		"Velocity Ki",
		.bytes =	2,
		.range =	{ 0x0000, 0x7FFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepLedBlinkPeriod = {
		.address =	44,
		.type =		"LED Blink Period",
		.bytes =	1,
		.range =	{ 0x00, 0xFE },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepAdcFaultCheckPeriod = {
		.address =	45,
		.type =		"ADC Fault Check Period",
		.bytes =	1,
		.range =	{ 0x00, 0xFE },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepPacketGarbageCheckPeriod = {
		.address =	46,
		.type =		"Packet Garbage Check Period",
		.bytes =	1,
		.range =	{ 0x00, 0xFE },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepStopDetectPeriod = {
		.address =	47,
		.type =		"Stop Detect Period",
		.bytes =	1,
		.range =	{ 0x00, 0xFE },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepOverloadDetectPeriod = {
		.address =	48,
		.type =		"Overload Detect Period",
		.bytes =	1,
		.range =	{ 0x00, 0xFE },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepStopThreshold = {
		.address =	49,
		.type =		"Overload Detect Period",
		.bytes =	1,
		.range =	{ 0x00, 0xFE },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepInpositionMargin = {
		.address =	50,
		.type =		"Inposition Margin",
		.bytes =	1,
		.range =	{ 0x00, 0xFE },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepReserved51 = {
		.address =	51,
		.type =		"Reserved",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReserved,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};

// <!-- begin calibration difference -->
// for models with a 2-byte calibration difference
const HerkulexRegister HerkulexRegister::kEepCalibrationDifferenceLower = {
		.address =	52,
		.type =		"Calibration Difference Lower",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
const HerkulexRegister HerkulexRegister::kEepCalibrationDifferenceUpper = {
		.address =	53,
		.type =		"Calibration Difference Upper",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
// for models with a 1-byte calibration difference
// const HerkulexRegister HerkulexRegister::&kEepCalibrationDifference = kEepCalibrationDifferenceUpper;
const HerkulexRegister HerkulexRegister::kEepCalibrationDifference = {
		.address =	53,
		.type =		"Calibration Difference",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kNonVolatile
};
// <!-- end calibration difference -->

//--------------
// RAM registers
//--------------
const HerkulexRegister HerkulexRegister::kRamId = {
		.address =	0,
		.type =		kEepId.type,
		.bytes =	kEepId.bytes,
		.range =	kEepId.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamAckPolicy = {
		.address =	1,
		.type =		kEepAckPolicy.type,
		.bytes =	kEepAckPolicy.bytes,
		.range =	kEepAckPolicy.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamAlarmLedPolicy = {
		.address =	2,
		.type =		kEepAlarmLedPolicy.type,
		.bytes =	kEepAlarmLedPolicy.bytes,
		.range =	kEepAlarmLedPolicy.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamTorquePolicy = {
		.address =	3,
		.type =		kEepTorquePolicy.type,
		.bytes =	kEepTorquePolicy.bytes,
		.range =	kEepTorquePolicy.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamReserved4 = {
		.address =	4,
		.type =		kEepReserved10.type,
		.bytes =	kEepReserved10.bytes,
		.range =	kEepReserved10.range,
		.access =	HerkulexRegister::AccessMode::kReserved,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamMaxTemp = {
		.address =	5,
		.type =		kEepMaxTemp.type,
		.bytes =	kEepMaxTemp.bytes,
		.range =	kEepMaxTemp.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamMinVoltage = {
		.address =	6,
		.type =		kEepMinVoltage.type,
		.bytes =	kEepMinVoltage.bytes,
		.range =	kEepMinVoltage.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamMaxVoltage = {
		.address =	7,
		.type =		kEepMaxVoltage.type,
		.bytes =	kEepMaxVoltage.bytes,
		.range =	kEepMaxVoltage.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamAccelRatio = {
		.address =	8,
		.type =		kEepAccelRatio.type,
		.bytes =	kEepAccelRatio.bytes,
		.range =	kEepAccelRatio.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamMaxAccelTime = {
		.address =	9,
		.type =		kEepMaxAccelTime.type,
		.bytes =	kEepMaxAccelTime.bytes,
		.range =	kEepMaxAccelTime.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamDeadZone = {
		.address =	10,
		.type =		kEepDeadZone.type,
		.bytes =	kEepDeadZone.bytes,
		.range =	kEepDeadZone.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamSaturatorOffset = {
		.address =	11,
		.type =		kEepSaturatorOffset.type,
		.bytes =	kEepSaturatorOffset.bytes,
		.range =	kEepSaturatorOffset.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamSaturatorSlope = {
		.address =	12,
		.type =		kEepSaturatorSlope.type,
		.bytes =	kEepSaturatorSlope.bytes,
		.range =	kEepSaturatorSlope.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamPwmOffset = {
		.address =	14,
		.type =		kEepPwmOffset.type,
		.bytes =	kEepPwmOffset.bytes,
		.range =	kEepPwmOffset.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamMinPwm = {
		.address =	15,
		.type =		kEepMinPwm.type,
		.bytes =	kEepMinPwm.bytes,
		.range =	kEepMinPwm.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamMaxPwm = {
		.address =	16,
		.type =		kEepMaxPwm.type,
		.bytes =	kEepMaxPwm.bytes,
		.range =	kEepMaxPwm.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamOverloadPwmThreshold = {
		.address =	18,
		.type =		kEepOverloadPwmThreshold.type,
		.bytes =	kEepOverloadPwmThreshold.bytes,
		.range =	kEepOverloadPwmThreshold.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamMinPosition = {
		.address =	20,
		.type =		kEepMinPosition.type,
		.bytes =	kEepMinPosition.bytes,
		.range =	kEepMinPosition.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamMaxPosition = {
		.address =	22,
		.type =		kEepMaxPosition.type,
		.bytes =	kEepMaxPosition.bytes,
		.range =	kEepMaxPosition.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamPositionKp = {
		.address =	24,
		.type =		kEepPositionKp.type,
		.bytes =	kEepPositionKp.bytes,
		.range =	kEepPositionKp.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamPositionKd = {
		.address =	26,
		.type =		kEepPositionKd.type,
		.bytes =	kEepPositionKd.bytes,
		.range =	kEepPositionKd.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamPositionKi = {
		.address =	28,
		.type =		kEepPositionKi.type,
		.bytes =	kEepPositionKi.bytes,
		.range =	kEepPositionKi.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamPositionFeedFwdGain1 = {
		.address =	30,
		.type =		kEepPositionFeedFwdGain1.type,
		.bytes =	kEepPositionFeedFwdGain1.bytes,
		.range =	kEepPositionFeedFwdGain1.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamPositionFeedFwdGain2 = {
		.address =	32,
		.type =		kEepPositionFeedFwdGain2.type,
		.bytes =	kEepPositionFeedFwdGain2.bytes,
		.range =	kEepPositionFeedFwdGain2.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamVelocityKp = {
		.address =	34,
		.type =		kEepVelocityKp.type,
		.bytes =	kEepVelocityKp.bytes,
		.range =	kEepVelocityKp.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamVelocityKi = {
		.address =	36,
		.type =		kEepVelocityKi.type,
		.bytes =	kEepVelocityKi.bytes,
		.range =	kEepVelocityKi.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamLedBlinkPeriod = {
		.address =	38,
		.type =		kEepLedBlinkPeriod.type,
		.bytes =	kEepLedBlinkPeriod.bytes,
		.range =	kEepLedBlinkPeriod.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamAdcFaultCheckPeriod = {
		.address =	39,
		.type =		kEepAdcFaultCheckPeriod.type,
		.bytes =	kEepAdcFaultCheckPeriod.bytes,
		.range =	kEepAdcFaultCheckPeriod.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamPacketGarbageCheckPeriod = {
		.address =	40,
		.type =		kEepPacketGarbageCheckPeriod.type,
		.bytes =	kEepPacketGarbageCheckPeriod.bytes,
		.range =	kEepPacketGarbageCheckPeriod.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamStopDetectPeriod = {
		.address =	41,
		.type =		kEepStopDetectPeriod.type,
		.bytes =	kEepStopDetectPeriod.bytes,
		.range =	kEepStopDetectPeriod.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamOverloadDetectPeriod = {
		.address =	42,
		.type =		kEepOverloadDetectPeriod.type,
		.bytes =	kEepOverloadDetectPeriod.bytes,
		.range =	kEepOverloadDetectPeriod.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamStopThreshold = {
		.address =	43,
		.type =		kEepStopThreshold.type,
		.bytes =	kEepStopThreshold.bytes,
		.range =	kEepStopThreshold.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamInpositionMargin = {
		.address =	44,
		.type =		kEepInpositionMargin.type,
		.bytes =	kEepInpositionMargin.bytes,
		.range =	kEepInpositionMargin.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamReserved45 = {
		.address =	45,
		.type =		kEepReserved51.type,
		.bytes =	kEepReserved51.bytes,
		.range =	kEepReserved51.range,
		.access =	HerkulexRegister::AccessMode::kReserved,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};

//<!-- begin calibration difference -->
// for models with a 2-byte calibration difference
const HerkulexRegister HerkulexRegister::kRamCalibrationDifferenceLower = {
		.address =	46,
		.type =		kEepCalibrationDifferenceLower.type,
		.bytes =	kEepCalibrationDifferenceLower.bytes,
		.range =	kEepCalibrationDifferenceLower.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamCalibrationDifferenceUpper = {
		.address =	47,
		.type =		kEepCalibrationDifferenceUpper.type,
		.bytes =	kEepCalibrationDifferenceUpper.bytes,
		.range =	kEepCalibrationDifferenceUpper.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
// for models with a 1-byte calibration difference
// const HerkulexRegister HerkulexRegister::&kRamCalibrationDifference = kRamCalibrationDifferenceUpper;
const HerkulexRegister HerkulexRegister::kRamCalibrationDifference = {
		.address =	47,
		.type =		kEepCalibrationDifferenceUpper.type,
		.bytes =	kEepCalibrationDifferenceUpper.bytes,
		.range =	kEepCalibrationDifferenceUpper.range,
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
// <!-- end calibration difference -->

const HerkulexRegister HerkulexRegister::kRamStatusError = {
		.address =	48,
		.type =		"Status Error",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamStatusDetail = {
		.address =	49,
		.type =		"Status Detail",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamAux1 = {
		.address =	50,
		.type =		"Aux 1",
		.bytes =	1,
		.range =	{ 0x00, 0x06 },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamReserved51 = {
		.address =	51,
		.type =		"Reserved",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReserved,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamTorqueControl = {
		.address =	52,
		.type =		"Torque Control",
		.bytes =	1,
		.range =	{ 0x00, 0x60 },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamLedControl = {
		.address =	53,
		.type =		"LED Control",
		.bytes =	1,
		.range =	{ 0x00, 0x07 },
		.access =	HerkulexRegister::AccessMode::kReadWrite,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamVoltage = {
		.address =	54,
		.type =		"Voltage",
		.bytes =	1,
		.range =	{ 0x00, 0xC8 },	// max voltage is 20V (601)
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamTemp = {
		.address =	55,
		.type =		"Temperature",
		.bytes =	1,
		.range =	{ 0x00, 0x6E },	// max temp is 110'C (601)
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamCurrentControlMode = {
		.address =	56,
		.type =		"Current Control Mode",
		.bytes =	1,
		.range =	{ 0x00, 0x01 },
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamTick = {
		.address =	57,
		.type =		"Tick",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamCalibratedPosition = {
		.address =	58,
		.type =		"Calibrated Positon",
		.bytes =	2,
		.range =	{ 0x0000, 0x03FF },
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamAbsolutePosition = {
		.address =	60,
		.type =		"Absolute Position",
		.bytes =	2,
		.range =	{ 0x0000, 0x03FF },
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamDifferentialPosition = {
		.address =	62,
		.type =		"Differential Position",
		.bytes =	2,
		.range =	{ 0x0000, 0x03FF },
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamPwm = {
		.address =	64,
		.type =		"PWM / Torque Raw Data",
		.bytes =	2,
		.range =	{ 0x0000, 0x03FF },
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamAbsoluteSecondPosition = {
		.address =	66,
		.type =		"Absolute 2nd Position",
		.bytes =	2,
		.range =	{ 0x0000, 0xFFFF },
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamAbsoluteGoalPosition = {
		.address =	68,
		.type =		"Absolute Goal Position",
		.bytes =	2,
		.range =	{ 0x0000, 0x03FF },
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamAbsoluteDesiredTrajectoryPosition = {
		.address =	70,
		.type =		"Absolute Desired Trajectory Position",
		.bytes =	2,
		.range =	{ 0x0000, 0x03FF },
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
const HerkulexRegister HerkulexRegister::kRamDesiredVelocity = {
		.address =	72,
		.type =		"Desired Velocity",
		.bytes =	1,
		.range =	{ 0x00, 0xFF },
		.access =	HerkulexRegister::AccessMode::kReadOnly,
		.location =	HerkulexRegister::MemoryLocation::kVolatile
};
// clang-format on

} // namespace momentum
