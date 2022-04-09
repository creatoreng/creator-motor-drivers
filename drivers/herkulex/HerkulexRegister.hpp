/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Herkulex motor registers.
 **/

#pragma once

#include <map>
#include <stdint.h>
#include <string>

namespace momentum
{

/// Register on a Herkulex motor.
struct HerkulexRegister {
	/// Acceptable range for the value of a register.
	/// This range is the union of all Herkulex models
	/// and may not reflect the exact range for a specific model.
	struct Range {
		/// Minimum value for this register.
		uint16_t min;
		/// Maximum value for this register.
		uint16_t max;
	};
	/// Access mode of a register.
	enum class AccessMode {
		/// HerkulexRegister is reserved and used by no current Herkulex models.
		kReserved,
		/// Regiser is read-only.
		kReadOnly,
		/// HerkulexRegister is read/write.
		kReadWrite
	};
	/// Physical memory type of a register.
	enum class MemoryLocation {
		/// Non-volatile (EEP) memory.
		kNonVolatile,
		/// Volatile (RAM) memory.
		kVolatile
	};

	/// Find a register by address.
	/// @param [in] address The address to find.
	/// @param [in] location The memory bank to search.
	/// @return Pointer to the register object. nullptr if not found.
	static const HerkulexRegister *lookup(const uint8_t address, const MemoryLocation location);

	/// Absolute address of this register.
	uint8_t address;
	/// Brief name of this register.
	std::string type;
	/// Number of bytes stored in this register.
	uint8_t bytes;
	/// Acceptable range for the value of this register.
	Range range;
	/// Access mode of this register.
	AccessMode access;
	/// Memory bank of this register.
	MemoryLocation location;

	// all Herkulex registers
	/// Non-volatile (EEP) register: First two digits of model number (e.g. 401 -> 0x00 0x04).
	static const HerkulexRegister kEepModelNo1;
	/// Non-volatile (EEP) register: Second two digits of model number (e.g. 401 -> 0x00 0x01).
	static const HerkulexRegister kEepModelNo2;
	/// Non-volatile (EEP) register: First two digits of firmware version number.
	static const HerkulexRegister kEepVersion1;
	/// Non-volatile (EEP) register: Second two digits of firmware version number.
	static const HerkulexRegister kEepVersion2;
	/// Non-volatile (EEP) register: Baud rate code.
	static const HerkulexRegister kEepBaudRate;
	/// Non-volatile (EEP) register: Reserved - unused.
	static const HerkulexRegister kEepReserved5;
	/// Non-volatile (EEP) register: Unique device ID.
	static const HerkulexRegister kEepId;
	/// Non-volatile (EEP) register: Packet acknowledgement policy.
	static const HerkulexRegister kEepAckPolicy;
	/// Non-volatile (EEP) register: LED alarm policy.
	static const HerkulexRegister kEepAlarmLedPolicy;
	/// Non-volatile (EEP) register: Torque policy.
	static const HerkulexRegister kEepTorquePolicy;
	/// Non-volatile (EEP) register: Reserved - unused.
	static const HerkulexRegister kEepReserved10;
	/// Non-volatile (EEP) register: Maximum operating temperature.
	static const HerkulexRegister kEepMaxTemp;
	/// Non-volatile (EEP) register: Minimum operating voltage.
	static const HerkulexRegister kEepMinVoltage;
	/// Non-volatile (EEP) register: Maximum operating voltage.
	static const HerkulexRegister kEepMaxVoltage;
	/// Non-volatile (EEP) register: Acceleration ratio.
	static const HerkulexRegister kEepAccelRatio;
	/// Non-volatile (EEP) register: Maximum acceleration time.
	static const HerkulexRegister kEepMaxAccelTime;
	/// Non-volatile (EEP) register: Dead zone.
	static const HerkulexRegister kEepDeadZone;
	/// Non-volatile (EEP) register: Saturator offset.
	static const HerkulexRegister kEepSaturatorOffset;
	/// Non-volatile (EEP) register: Saturator slope.
	static const HerkulexRegister kEepSaturatorSlope;
	/// Non-volatile (EEP) register: PWM offset.
	static const HerkulexRegister kEepPwmOffset;
	/// Non-volatile (EEP) register: Minimum PWM duty cycle.
	static const HerkulexRegister kEepMinPwm;
	/// Non-volatile (EEP) register: Maxiumum PWM duty cycle.
	static const HerkulexRegister kEepMaxPwm;
	/// Non-volatile (EEP) register: Overload PWM threshold.
	static const HerkulexRegister kEepOverloadPwmThreshold;
	/// Non-volatile (EEP) register: Minimum position.
	static const HerkulexRegister kEepMinPosition;
	/// Non-volatile (EEP) register: Maximum postion.
	static const HerkulexRegister kEepMaxPosition;
	/// Non-volatile (EEP) register: Position control proportional gain, Kp.
	static const HerkulexRegister kEepPositionKp;
	/// Non-volatile (EEP) register: Position control derivative gain, Kd.
	static const HerkulexRegister kEepPositionKd;
	/// Non-volatile (EEP) register: Position control integral gain, Ki.
	static const HerkulexRegister kEepPositionKi;
	/// Non-volatile (EEP) register: Position control feed-forward gain 1.
	static const HerkulexRegister kEepPositionFeedFwdGain1;
	/// Non-volatile (EEP) register: Position control feed-forward gain 2.
	static const HerkulexRegister kEepPositionFeedFwdGain2;
	/// Non-volatile (EEP) register: Velocity control proportional gain, Kp.
	static const HerkulexRegister kEepVelocityKp;
	/// Non-volatile (EEP) register: Velocity control integral gain, Ki.
	static const HerkulexRegister kEepVelocityKi;
	/// Non-volatile (EEP) register: LED blink period.
	static const HerkulexRegister kEepLedBlinkPeriod;
	/// Non-volatile (EEP) register: ADC fault check period.
	static const HerkulexRegister kEepAdcFaultCheckPeriod;
	/// Non-volatile (EEP) register: Packet garbage check period.
	static const HerkulexRegister kEepPacketGarbageCheckPeriod;
	/// Non-volatile (EEP) register: Stop detect period.
	static const HerkulexRegister kEepStopDetectPeriod;
	/// Non-volatile (EEP) register: Overload detect period.
	static const HerkulexRegister kEepOverloadDetectPeriod;
	/// Non-volatile (EEP) register: Stop threshold.
	static const HerkulexRegister kEepStopThreshold;
	/// Non-volatile (EEP) register: In-position error margin.
	static const HerkulexRegister kEepInpositionMargin;
	/// Non-volatile (EEP) register: Reserved - unused.
	static const HerkulexRegister kEepReserved51;
	/// Non-volatile (EEP) register: Calibration difference lower-byte, for models with 2-byte calibration difference.
	static const HerkulexRegister kEepCalibrationDifferenceLower;
	/// Non-volatile (EEP) register: Calibration difference upper-byte, for models with 2-byte calibration difference.
	static const HerkulexRegister kEepCalibrationDifferenceUpper;
	/// Non-volatile (EEP) register: Calibration difference, for models with 1-byte calibration difference.
	static const HerkulexRegister kEepCalibrationDifference;

	/// Volatile (RAM) register: Unique device ID.
	static const HerkulexRegister kRamId;
	/// Volatile (RAM) register: Packet acknowledgement policy.
	static const HerkulexRegister kRamAckPolicy;
	/// Volatile (RAM) register: LED alarm policy.
	static const HerkulexRegister kRamAlarmLedPolicy;
	/// Volatile (RAM) register: Torque policy.
	static const HerkulexRegister kRamTorquePolicy;
	/// Volatile (RAM) register: Reserved - unused.
	static const HerkulexRegister kRamReserved4;
	/// Volatile (RAM) register: Maximum operating temperature.
	static const HerkulexRegister kRamMaxTemp;
	/// Volatile (RAM) register: Minimum operating temperature.
	static const HerkulexRegister kRamMinVoltage;
	/// Volatile (RAM) register: Maximum operating voltage.
	static const HerkulexRegister kRamMaxVoltage;
	/// Volatile (RAM) register: Acceleration ratio.
	static const HerkulexRegister kRamAccelRatio;
	/// Volatile (RAM) register: Maximum acceleration time.
	static const HerkulexRegister kRamMaxAccelTime;
	/// Volatile (RAM) register: Dead zone.
	static const HerkulexRegister kRamDeadZone;
	/// Volatile (RAM) register: Saturator offset.
	static const HerkulexRegister kRamSaturatorOffset;
	/// Volatile (RAM) register: Saturator slope.
	static const HerkulexRegister kRamSaturatorSlope;
	/// Volatile (RAM) register: PWM offset.
	static const HerkulexRegister kRamPwmOffset;
	/// Volatile (RAM) register: Minimum PWM duty cycle.
	static const HerkulexRegister kRamMinPwm;
	/// Volatile (RAM) register: Maximum PWM duty cycle.
	static const HerkulexRegister kRamMaxPwm;
	/// Volatile (RAM) register: PWM overload threshold.
	static const HerkulexRegister kRamOverloadPwmThreshold;
	/// Volatile (RAM) register: Minimum position.
	static const HerkulexRegister kRamMinPosition;
	/// Volatile (RAM) register: Maximum position.
	static const HerkulexRegister kRamMaxPosition;
	/// Volatile (RAM) register: Position control proportional gain, Kp.
	static const HerkulexRegister kRamPositionKp;
	/// Volatile (RAM) register: Position control derivative gain, Kd.
	static const HerkulexRegister kRamPositionKd;
	/// Volatile (RAM) register: Position control integral gain, Ki.
	static const HerkulexRegister kRamPositionKi;
	/// Volatile (RAM) register: Position control feed-forward gain 1.
	static const HerkulexRegister kRamPositionFeedFwdGain1;
	/// Volatile (RAM) register: Position control feed-forward gain 2.
	static const HerkulexRegister kRamPositionFeedFwdGain2;
	/// Volatile (RAM) register: Velocity control proportional gain, Kp.
	static const HerkulexRegister kRamVelocityKp;
	/// Volatile (RAM) register: Velocity control integral gain, Ki.
	static const HerkulexRegister kRamVelocityKi;
	/// Volatile (RAM) register: LED blink period.
	static const HerkulexRegister kRamLedBlinkPeriod;
	/// Volatile (RAM) register: ADC fault check period.
	static const HerkulexRegister kRamAdcFaultCheckPeriod;
	/// Volatile (RAM) register: Packet garbage check period.
	static const HerkulexRegister kRamPacketGarbageCheckPeriod;
	/// Volatile (RAM) register: Stop detect peroid.
	static const HerkulexRegister kRamStopDetectPeriod;
	/// Volatile (RAM) register: Overload detect period.
	static const HerkulexRegister kRamOverloadDetectPeriod;
	/// Volatile (RAM) register: Stop threshold.
	static const HerkulexRegister kRamStopThreshold;
	/// Volatile (RAM) register: In-position error margin.
	static const HerkulexRegister kRamInpositionMargin;
	/// Volatile (RAM) register: Reserved - unused.
	static const HerkulexRegister kRamReserved45;
	/// Volatile (RAM) register: Calibration difference lower-byte, for models with 2-byte calibration difference.
	static const HerkulexRegister kRamCalibrationDifferenceLower;
	/// Volatile (RAM) register: Calibration difference upper-byte, for models with 2-byte calibration difference.
	static const HerkulexRegister kRamCalibrationDifferenceUpper;
	/// Volatile (RAM) register: Calibration difference, for models with 1-byte calibration difference.
	static const HerkulexRegister kRamCalibrationDifference;
	/// Volatile (RAM) register: Error status.
	static const HerkulexRegister kRamStatusError;
	/// Volatile (RAM) register: Detail of the error status.
	static const HerkulexRegister kRamStatusDetail;
	/// Volatile (RAM) register: Auxilliary register 1.
	static const HerkulexRegister kRamAux1;
	/// Volatile (RAM) register: Reserved - unused.
	static const HerkulexRegister kRamReserved51;
	/// Volatile (RAM) register: Torque control.
	static const HerkulexRegister kRamTorqueControl;
	/// Volatile (RAM) register: LED control.
	static const HerkulexRegister kRamLedControl;
	/// Volatile (RAM) register: Current operating voltage.
	static const HerkulexRegister kRamVoltage;
	/// Volatile (RAM) register: Current operating temperature.
	static const HerkulexRegister kRamTemp;
	/// Volatile (RAM) register: Current control mode.
	static const HerkulexRegister kRamCurrentControlMode;
	/// Volatile (RAM) register: Clock tick count.
	static const HerkulexRegister kRamTick;
	/// Volatile (RAM) register: Current calibrated position.
	static const HerkulexRegister kRamCalibratedPosition;
	/// Volatile (RAM) register: Current abosolute (raw) position.
	static const HerkulexRegister kRamAbsolutePosition;
	/// Volatile (RAM) register: Current differential position.
	static const HerkulexRegister kRamDifferentialPosition;
	/// Volatile (RAM) register: Current PWM duty cycle / torque output.
	static const HerkulexRegister kRamPwm;
	/// Volatile (RAM) register: Absolute second position.
	static const HerkulexRegister kRamAbsoluteSecondPosition;
	/// Volatile (RAM) register: Current absolute goal position.
	static const HerkulexRegister kRamAbsoluteGoalPosition;
	/// Volatile (RAM) register: Current absolute desired trajectory position.
	static const HerkulexRegister kRamAbsoluteDesiredTrajectoryPosition;
	/// Volatile (RAM) register: Desired velocity.
	static const HerkulexRegister kRamDesiredVelocity;

private:
	/// Construct a static HerkulexRegister and adds it to the static register map.
	/// @param [in] src_address The register address.
	/// @param [in] src_type The register type description.
	/// @param [in] src_bytes The number of bytes of the register.
	/// @param [in] src_range The permitted value range of the register.
	/// @param [in] src_access The memory access mode of the register.
	/// @param [in] src_location The memory location of the register.
	HerkulexRegister(const uint8_t src_address,
					 const std::string &src_type,
					 const uint8_t src_bytes,
					 const Range src_range,
					 const AccessMode src_access,
					 const MemoryLocation src_location);

	/// Make this struct non-copyable. Client code should always reference a static member.
	/// @param [in] src The source register to copy.
	HerkulexRegister(const HerkulexRegister &src) = delete;

	/// Static set of all HerkulexRegisters
	static std::map<const std::pair<uint8_t, MemoryLocation>, const HerkulexRegister *const> register_map;
};

} // namespace momentum
