/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Copley motor parameters.
 **/

#pragma once

#include <map>
#include <memory>
#include <stdint.h>
#include <string>

namespace momentum
{

/// Copley amplifier parameter.
/// @see Copley Parameter Dictionary.
struct CopleyParameter {
public:
	/// Smallest type that can hold any non-string parameter value.
	using value_t = int32_t;

	/// Parameter ID type.
	using parameter_id_t = uint16_t;

	/// Access mode of a parameter.
	enum class AccessMode {
		kReadOnly, ///< Parameter is read-only.
		kReadWrite ///< Parameter is read/write.
	};

	/// Physical memory type of a parameter. Bitmapped value.
	enum class MemoryLocation {
		/// Non-volatile (flash) memory.
		kNonVolatile = 0x01,
		/// Volatile (RAM) memory.
		kVolatile = 0x02,
		/// Both non-volatile and volatile memory.
		kBoth = kNonVolatile | kVolatile
	};

	// helper methods

	/// Find a parameter by id.
	/// @param [in] id The parameter id.
	/// @param [in] location The memory bank of the parameter.
	/// @return Pointer to the parameter object. nullptr if not found.
	static const CopleyParameter *lookup(const parameter_id_t id, const MemoryLocation location);

	/// Return a hexidecimal formatted string for the parameter location and parameter_id.
	/// @param [in] location The memory location.
	/// @return formatted string.
	std::string to_string(const MemoryLocation location) const;

	// member variables

	parameter_id_t id;		 ///< Absolute parameter_id of this parameter.
	std::string type;		 ///< Brief name of this parameter.
	size_t bytes;			 ///< Number of bytes stored in this parameter.
	AccessMode access;		 ///< Access mode of this parameter.
	MemoryLocation location; ///< Memory bank of this parameter.
	bool is_string;			 ///< Value type is a string?
	bool is_signed;			 ///< If numeric, is value type signed?

	// all Copley parameters, in ascending order of parameter_id

	// current control
	// clang-format off
	static const CopleyParameter kCurrentLoopCp;				///< Current control loop proportional gain (aka Cp).
	static const CopleyParameter kCurrentLoopCi;				///< Current control loop integral gain (Ci).
	static const CopleyParameter kCurrentLoopProgrammedValue;	///< Current control loop programmed value.
	static const CopleyParameter kCurrentWindingA;				///< Winding A current.
	static const CopleyParameter kCurrentWindingB;				///< Winding B current.
	static const CopleyParameter kCurrentOffsetA;				///< Current offset A.
	static const CopleyParameter kCurrentOffsetB;				///< Current offset B.
	static const CopleyParameter kCurrentStatorX;				///< Stator current vector X axis.
	static const CopleyParameter kCurrentStatorY;				///< Stator current vector Y axis.
	static const CopleyParameter kCurrentLoopOutputStatorX;		///< Current control loop output stator X axis.
	static const CopleyParameter kCurrentLoopOutputStatorY;		///< Current control loop output stator Y axis.
	static const CopleyParameter kCurrentLoopActualRotorD;		///< Actual current, D axis of rotor space.
	static const CopleyParameter kCurrentLoopActualRotorQ;		///< Actual current, Q axis of rotor space.
	static const CopleyParameter kCurrentLoopCommandedRotorD;	///< Current control loop commanded output, D axis of rotor space.
	static const CopleyParameter kCurrentLoopCommandedRotorQ;	///< Current control loop commanded output, Q axis of rotor space.
	static const CopleyParameter kCurrentLoopErrorRotorD;		///< Current control loop error, D axis of rotor space.
	static const CopleyParameter kCurrentLoopErrorRotorQ;		///< Current control loop error, Q axis of rotor space.
	static const CopleyParameter kCurrentLoopIntegralRotorD;	///< Current control loop integral value, D axis of rotor space.
	static const CopleyParameter kCurrentLoopIntegralRotorQ;	///< Current control loop integral value, Q axis of rotor space.
	static const CopleyParameter kCurrentLoopOutputRotorD;		///< Current control loop output, D axis of rotor space.
	static const CopleyParameter kCurrentLoopOutputRotorQ;		///< Current control loop output, Q axis of rotor space.
	static const CopleyParameter kCurrentLoopCommanded;			///< Current control loop commanded motor current.

	// unknown
	static const CopleyParameter kUnknown016;					///< Unknown parameter 0x016.

	// measurements
	static const CopleyParameter kPositionActual;				///< Actual position.
	static const CopleyParameter kVelocityActual;				///< Actual velocity.

	// analog and A/D
	static const CopleyParameter kAnalogReferenceScaling;		///< Analog reference scaling factor.
	static const CopleyParameter kAnalogReferenceOffset;			///< Analog reference offset.
	static const CopleyParameter kAnalogEncoderSineInputVoltage;	///< Analog encoder sine input voltage.
	static const CopleyParameter kAnalogEncoderCosineInputVoltage;	///< Analog encoder cosine input voltage.
	static const CopleyParameter kAnalogReferenceInputVoltage;		///< Analog reference input voltage.
	static const CopleyParameter kHighVoltage;					///< High voltage.
	static const CopleyParameter kUnknown01F;					///< Unknown parameter 0x01F.
	static const CopleyParameter kDriveTemperature;				///< Drive temperature.

	// absolute limits
	static const CopleyParameter kCurrentLimitPeak;				///< Peak current limit.
	static const CopleyParameter kCurrentLimitContinuous;		///< Continuous current limit.
	static const CopleyParameter kCurrentLimitPeakTime;			///< Peak current limit time.

	// motion control
	static const CopleyParameter kDriveMode;					///< Drive mode (aka "Desired State")
	static const CopleyParameter kLimitedMotorCurrentCommand;	///< Limited motor current command.
	static const CopleyParameter kAnalogReferenceInputDeadband;	///< Analog reference input deadband.
	static const CopleyParameter kVelocityLoopVp;				///< Velocity control loop proportional gain (aka Vp).
	static const CopleyParameter kVelocityLoopVi;				///< Velocity control loop integral gain (aka Vi).
	static const CopleyParameter kVelocityLoopLimitedVelocity;	///< Velocity control loop limited velocity.
	static const CopleyParameter kVelocityLoopError;			///< Velocity control loop error.
	static const CopleyParameter kVelocityLoopIntegral;			///< Velocity control loop integral value.
	static const CopleyParameter kVelocityCommanded;			///< Commanded velocity.
	static const CopleyParameter kPositionCommanded;			///< Commanded position.
	static const CopleyParameter kVelocityLoopAff;				///< Velocity control loop acceleration feed forward gain.
	static const CopleyParameter kVelocityProgrammed;			///< Programmed velocity.
	static const CopleyParameter kPositionLoopPp;				///< Position control loop proportional gain (aka Pp).
	static const CopleyParameter kVelocityLoopShift;			///< Velocity control loop output binary right-shift.
	static const CopleyParameter kActualMotorPosition;		    ///< Actual Motor position.
	static const CopleyParameter kPositionLoopVff;				///< Position control loop velocity feed forward gain (aka Vff).
	static const CopleyParameter kPositionLoopAff;				///< Position control loop acceleration feed forward gain (aka Aff).
	static const CopleyParameter kPositionLoopError;			///< Position control loop error.
	static const CopleyParameter kVelocityLoopAccelLimit;		///< Velocity control loop acceleration limit.
	static const CopleyParameter kVelocityLoopDecelLimit;		///< Velocity control loop develeration limit.
	static const CopleyParameter kCurrentActual;					///< Actual motor current.
	static const CopleyParameter kVelocityLoopEmergencyAccelLimit;	///< Velocity control loop emergency deceleration limit.
	static const CopleyParameter kVelocityLoopVelocityLimit;		///< Velocity control loop velocity limit.
	static const CopleyParameter kTrajectoryCommandedVelocity;		///< Trajectory generator instantaneous commanded velocity.
	static const CopleyParameter kTrajectoryCommandedAcceleration;	///< Trajectory generator instantaneous commanded acceleration.
	static const CopleyParameter kTrajectoryPositionDestination;	///< Trajectory generator position destination.
	static const CopleyParameter kVelocityWindow;				///< Velocity control loop window. If velocity error exceeds this value, a velocity tracking warning is reported.
	static const CopleyParameter kVelocityWindowTime;			///< Velocity control loop window time. Amount of time velocity must be within tracking window to clear tracking warning.

	// motor specifications
	static const CopleyParameter kMotorType;					///< Motor type.
	static const CopleyParameter kMotorManufacturer;			///< Motor manufacturer.
	static const CopleyParameter kMotorModel;					///< Motor model.
	static const CopleyParameter kMotorUnits;					///< Motor units.
	static const CopleyParameter kMotorInertia;					///< Motor inertia (mass).
	static const CopleyParameter kMotorPolePairs;				///< Motor pole pairs.
	static const CopleyParameter kMotorBrakeType;				///< Motor brake type.
	static const CopleyParameter kMotorTempSensorType;			///< Motor temperature sensor type.
	static const CopleyParameter kMotorTorqueConstant;			///< Motor torque constant.
	static const CopleyParameter kMotorResistance;				///< Motor armature resitance.
	static const CopleyParameter kMotorInductance;				///< Motor armature inductance.
	static const CopleyParameter kMotorPeakTorque;				///< Motor peak torque.
	static const CopleyParameter kMotorContinuousTorque;		///< Motor continuous torque.
	static const CopleyParameter kMotorMaxVelocity;				///< Motor max velocity.
	static const CopleyParameter kMotorWiring;					///< Motor wiring.
	static const CopleyParameter kMotorBrakeActivationTime;		///< Motor brake activation time.
	static const CopleyParameter kMotorBrakeDelayTime;			///< Motor brake delay time.
	static const CopleyParameter kMotorBrakeActivationVelocity;	///< Motor brake activation velocity
	static const CopleyParameter kMotorHallOffset;				///< Motor Hall offset. Offset angle to be applied to the Hall sensors.
	static const CopleyParameter kMotorHallType;				///< Motor Hall type.
	static const CopleyParameter kUnknown051;					///< Unknown parameter 0x051.
	static const CopleyParameter kMotorHallWiring;				///< Motor Hall wiring.
	static const CopleyParameter kMotorBackEmfConstant;			///< Motor back EMF constant.
	static const CopleyParameter kStepperMicrostepsPerRev;		///< Motor microsteps per revolution.
	static const CopleyParameter kMotorGearRatio;				///< Motor gear ratio.
	static const CopleyParameter kHallVelocityShift;			///< Hall velocity mode shift value.

	// encoder
	static const CopleyParameter kEncoderOutputConfiguration;	///< Encoder output configuration.
	static const CopleyParameter kLoadEncoderResolution;		///< Load encoder resolution.
	static const CopleyParameter kLoadEncoderDirection;			///< Load encoder direction.
	static const CopleyParameter kUnknown066;					///< Unknown parameter 0x066.
	static const CopleyParameter kLoadEncoderType;				///< Load encoder type.
	static const CopleyParameter kLoadEncoderVelocity;			///< Load encoder velocity.
	static const CopleyParameter kVelocityLoopOutputFilter;		///< A bi-quad filter which acts on the output of the Velocity control loop.
	static const CopleyParameter kMotorEncoderType;					///< Encoder type.
	static const CopleyParameter kLinearEncoderUnits;				///< Linear motor encoder units
	static const CopleyParameter kMotorEncoderCountsPerRevolution;	///< Encoder counts per revolution.
	static const CopleyParameter kLinearEncoderResolution;			///< Linear motor encoder resolution.
	static const CopleyParameter kLinearEncoderElectricalDistance;	///< Linear motor encoder electrical distance.
	static const CopleyParameter kEncoderDirection;					///< Encoder direction.
	static const CopleyParameter kAnalogEncoderShift;				///< Analog encoder right-shift amount.

	// position capture
	static const CopleyParameter kPositionCaptureIndex;				///< Position capture index.
	static const CopleyParameter kPositionCaptureControl;			///< Position capture control.
	static const CopleyParameter kPositionCaptureStatus;			///< Position capture status.

	// misc
	static const CopleyParameter kMotorEncoderVelocityUnfiltered;	///< Unfiltered encoder velocity.
	static const CopleyParameter kCurrentCommandedRampLimit;		///< Current commanded ramp limit.
	static const CopleyParameter kVelocityLoopInputFilter;			///< A bi-quad filter which acts on the input of the Velocity control loop.
	static const CopleyParameter kResolverCyclesPerRev;				///< Resolver feedback cylces per revolution.

	// I/O
	static const CopleyParameter kPwmMode;							///< PWM mode and status.
	static const CopleyParameter kDigitalOutput0Configuration;		///< Digital I/O bank Output 0 configuration.
	static const CopleyParameter kDigitalOutput1Configuration;		///< Digital I/O bank Output 1 configuration.
	static const CopleyParameter kDigitalOutput2Configuration;		///< Digital I/O bank Output 2 configuration.
	static const CopleyParameter kDigitalOutput3Configuration;		///< Digital I/O bank Output 3 configuration.
	static const CopleyParameter kDigitalOutput4Configuration;		///< Digital I/O bank Output 4 configuration.
	static const CopleyParameter kDigitalOutput5Configuration;		///< Digital I/O bank Output 5 configuration.
	static const CopleyParameter kDigitalOutput6Configuration;		///< Digital I/O bank Output 6 configuration.
	static const CopleyParameter kDigitalOutput7Configuration;		///< Digital I/O bank Output 7 configuration.
	static const CopleyParameter kDigitalInput0Configuration;		///< Digital I/O bank Input 0 configuration.
	static const CopleyParameter kDigitalInput1Configuration;		///< Digital I/O bank Input 1 configuration.
	static const CopleyParameter kDigitalInput2Configuration;		///< Digital I/O bank Input 2 configuration.
	static const CopleyParameter kDigitalInput3Configuration;		///< Digital I/O bank Input 3 configuration.
	static const CopleyParameter kDigitalInput4Configuration;		///< Digital I/O bank Input 4 configuration.
	static const CopleyParameter kDigitalInput5Configuration;		///< Digital I/O bank Input 5 configuration.
	static const CopleyParameter kDigitalInput6Configuration;		///< Digital I/O bank Input 6 configuration.
	static const CopleyParameter kDigitalInput7Configuration;		///< Digital I/O bank Input 7 configuration.

	// drive configuration
	static const CopleyParameter kDriveModelNumber;					///< Drive model number.
	static const CopleyParameter kDriveSerialNumber;				///< Drive serial number.
	static const CopleyParameter kDriveCurrentLimitPeak;			///< Drive peak current limit.
	static const CopleyParameter kDriveCurrentLimitContinuous;		///< Drive continuous current limit.
	static const CopleyParameter kDriveCurrentAtMaxA2D;				///< Current corresponding to max A/D reading.
	static const CopleyParameter kPwmPeriod;						///< PWM period.
	static const CopleyParameter kPwmLoopUpdatePeriod;				///< PWM loop update period.
	static const CopleyParameter kDriveProductFamily;				///< Drive product family.
	static const CopleyParameter kDriveCurrentLimitPeakTime;		///< Drive current limit peak time.
	static const CopleyParameter kDriveVoltageMax;					///< Drive maximum voltage.
	static const CopleyParameter kDriveVoltageAtMaxA2D;				///< Drive voltage at max A/D reading.
	static const CopleyParameter kDriveVoltageMin;					///< Drive minimum voltage;
	static const CopleyParameter kDriveTempMax;						///< Drive maximum temperature.
	static const CopleyParameter kDriveManufaturingInfo;			///< Drive manufaturing information.
	static const CopleyParameter kAnalogInputScale;					///< Analog input scale factor; the voltage that corresponds to the max A/D value.
	static const CopleyParameter kDriveSerialBaudRate;				///< Serial port baud rate.
	static const CopleyParameter kDriveSerialCommandWordsMax;		///< Serial port maxiumum number of words in a command.
	static const CopleyParameter kDriveName;						///< Drive name.
	static const CopleyParameter kUnknown093;						///< Unknown parameter 0x093.
	static const CopleyParameter kDriveFirmwareVersion;				///< Drive firmware version.
	static const CopleyParameter kDriveHostConfiguration;			///< Drive host configuration;
	static const CopleyParameter kAnalogReferenceCalibrationOffset;	///< Analog reference calibration offset.
	static const CopleyParameter kDriveTempCutoutHysteresis;		///< Drive over temperature cutout hysteresis.

	// function generator
	static const CopleyParameter kFunctionGeneratorConfiguration;	///< Function generator configuration.
	static const CopleyParameter kFunctionGeneratorFrequency;		///< Function generator frequency.
	static const CopleyParameter kFunctionGeneratorAmplitude;		///< Function generator amplitude.
	static const CopleyParameter kFunctionGeneratorDutyCycle;		///< Function generator duty cycle.

	// misc - drive voltage
	static const CopleyParameter kDriveVoltageCutoutHysteresis;		///< Drive over voltage cutout hysteresis.

	// PWM
	static const CopleyParameter kPwmDeadTimeAtContinuousCurrentLimit;	///< PWM dead time at continuous current limit.
	static const CopleyParameter kPwmOffTimeMin;						///< PWM minimum off time.
	static const CopleyParameter kPwmDeadTimeAtZeroCurrent;				///< PWM dead time at zero current.

	// state and I/O
	static const CopleyParameter kDriveEventStatus;					///< Drive event status.
	static const CopleyParameter kDriveLatchedEventStatus;			///< Drive latched event status.
	static const CopleyParameter kHallInputState;					///< Hall input pin state.
	static const CopleyParameter kUnknown0A3;						///< Unknown parameter 0x0A3.
	static const CopleyParameter kDriveLatchingFaultStatus;			///< Latching fault status parameter.
	static const CopleyParameter kDigitalInputPullupConfiguration;	///< Digital input pullup configuration.
	static const CopleyParameter kDigitalInputState;				///< Digital input states.
	static const CopleyParameter kDriveLatchingFaultMask;			///< Latching fault mask.
	static const CopleyParameter kDigitalInputCommandConfiguration;	///< Digital input command configuration.
	static const CopleyParameter kDigitalInputCommandScaling;		///< Digital input command scaling factor.
	static const CopleyParameter kDigitalInputStateRaw;				///< Digital input state (raw).
	static const CopleyParameter kDigitalOutputState;				///< Digital outout state and program control.
	static const CopleyParameter kDriveEventStatusSticky;			///< Drive event status (sticky).
	static const CopleyParameter kDriveHardwareType;				///< Drive hardware type.
	static const CopleyParameter kCurrentLoopOffset;				///< Current control loop offset.
	static const CopleyParameter kDriveOptions;						///< Miscellaneous drive options.

	// phasing
	static const CopleyParameter kPhaseAngle;					///< Phase angle.
	static const CopleyParameter kPhaseMicrostepIncrementRate;	///< Phase microstep increment rate.
	static const CopleyParameter kPhaseMode;					///< Commutation (phase) mode.
	static const CopleyParameter kAnalogEncoderScale;			///< Analog encoder scaling factor.
	static const CopleyParameter kMotorEncoderPhaseAngle;		///< Encoder phase angle.

	// misc
	static const CopleyParameter kHomeAdjustment;				///< Homing adjustment.
	static const CopleyParameter kPwmInputFrequency;			///< PWM input frequency.
	static const CopleyParameter kDriveTime;					///< System time.

	// position tracking
	static const CopleyParameter kPositionLimitPositive;		///< Positive software enforced limit.
	static const CopleyParameter kPositionLimitNegative;		///< Negative software enforced limit.
	static const CopleyParameter kPositionFollowingErrorLimit;	///< Position control loop following error limit. If position error exceeds this limit, a tracking fault is reported.
	static const CopleyParameter kPositionFollowingWarningLimit;///< Position control loop following error warning limit. If position error exceeds this limit, a tracking warning is reported.
	static const CopleyParameter kPositionTrackingWindow;		///< Position control loop tracking window. If position error exceeds this value, a position tracking warning is reported.
	static const CopleyParameter kPositionTrackingWindowTime;	///< Position control loop tracking error window time.  Amount of time the position must be within tracking window before a tracking warning is cleared.
	static const CopleyParameter kPositionLimitDeceleration;	///< Position software limit deceleration.

	// misc - homing
	static const CopleyParameter kHomeHardStopCurrentDelayTime;	///< Homing limit hard stop current delay time.

	// network
	static const CopleyParameter kNetworkNodeId;				///< Network node ID.
	static const CopleyParameter kNetworkNodeIdConfiguration;	///< Network node ID configuration.

	// homing
	static const CopleyParameter kHomeMethodConfiguration;		///< Home method configuration.
	static const CopleyParameter kHomeVelocityFast;				///< Home velocity (fast moves).
	static const CopleyParameter kHomeVelocitySlow;				///< Home velocity (slow moves).
	static const CopleyParameter kHomeAcceleration;				///< Home acceleration (and deceleration).
	static const CopleyParameter kHomeOffset;					///< Home offset.
	static const CopleyParameter kHomeHardStopCurrentLimit;		///< Home current limit.

	// trajectory
	static const CopleyParameter kTrajectoryProfile;			///< Trajectory profile type.
	static const CopleyParameter kTrajectoryStatus;				///< Trajectory status parameter.
	static const CopleyParameter kTrajectoryPosition;			///< Trajectory generator position command.
	static const CopleyParameter kTrajectoryMaxVelocity;		///< Trajectory generator velocity limit.
	static const CopleyParameter kTrajectoryMaxAccel;			///< Trajectory generator acceleration limit.
	static const CopleyParameter kTrajectoryMaxDecel;			///< Trajectory generator deceleration limit.
	static const CopleyParameter kTrajectoryMaxJerk;			///< Trajectory generator jerk limit.
	static const CopleyParameter kTrajectoryAbortDecel;			///< Trajectory generator abort deceleration.

	// I/O
	static const CopleyParameter kDigitalInput8Configuration;	///< Digital input 8 configuration.
	static const CopleyParameter kDigitalInput9Configuration;	///< Digital input 9 configuration.
	static const CopleyParameter kDigitalInput10Configuration;	///< Digital input 10 configuration.
	static const CopleyParameter kDigitalInput11Configuration;	///< Digital input 11 configuration.
	static const CopleyParameter kDigitalInput12Configuration;	///< Digital input 12 configuration.
	static const CopleyParameter kDigitalInput13Configuration;	///< Digital input 13 configuration.
	static const CopleyParameter kDigitalInput14Configuration;	///< Digital input 14 configuration.
	static const CopleyParameter kDigitalInput15Configuration;	///< Digital input 15 configuration.

	// regen resistor
	static const CopleyParameter kRegenResistance;				///< Regen resistor resistance.
	static const CopleyParameter kRegenPowerContinuous;			///< Regen resistor continuous power.
	static const CopleyParameter kRegenPowerPeak;				///< Regen resistor peak power.
	static const CopleyParameter kRegenPowerPeakTime;			///< Regen resistor peak power time.
	static const CopleyParameter kRegenVoltageOn;				///< Regen resistor on voltage.
	static const CopleyParameter kRegenVoltageOff;				///< Regen resistor off voltage.
	static const CopleyParameter kRegenInternalCurrentPeak;		///< Regen resistor internal peak current.
	static const CopleyParameter kRegenInternalCurrentContinuous;///< Regen resistor internal continuous current.
	static const CopleyParameter kRegenInternalCurrentPeakTime;	///< Regen resistor internal peak current time.
	static const CopleyParameter kRegenModel;					/// Regen resistor model.
	static const CopleyParameter kRegenStatus;					/// Regen resistor status.

	// misc
	static const CopleyParameter kPositionLoopGainsMultiplier;	///< Position control loop gains multiplier. Multiplies output of the position control loop.
	static const CopleyParameter kPhaseInitializationCurrent;	///< Phase initialization current.
	static const CopleyParameter kPhaseInitializationTimeout;	///< Phase initialization timeout.

	// stepping
	static const CopleyParameter kStepperLoopVelocityAdjustmentMax;	///< Maximum velocity adjustment.
	static const CopleyParameter kStepperLoopProportionalGain;		///< Stepper position control loop proportional gain.
	static const CopleyParameter kStepperMicrostepHoldCurrent;		///< Stepper microstep hold current.
	static const CopleyParameter kStepperMicrostepRunToHoldTime;		///< Stepper microstep run to hold time.
	static const CopleyParameter kStepperMicrostepDetentCorrectionGain;	///< Stepper microstep detent correction gain.
	static const CopleyParameter kUnknown0EB;							///< Unknown parameter 0x0EB.
	static const CopleyParameter kUnknown0EC;							///< Unknown parameter 0x0EC.
	static const CopleyParameter kStepperMicrostepHoldingCurrentToFixedVoltageTime;	///< Stepper microstep holding current to fixed voltage time.
	static const CopleyParameter kStepperConfiguration;				///< Stepper configuration.
	static const CopleyParameter kUnknown0EF;						///< Known parameter 0x0EF.

	// I/O debouncing
	static const CopleyParameter kDigitalInput0DebounceTime;	///< Digital input 0 debounce time.
	static const CopleyParameter kDigitalInput1DebounceTime;	///< Digital input 1 debounce time.
	static const CopleyParameter kDigitalInput2DebounceTime;	///< Digital input 2 debounce time.
	static const CopleyParameter kDigitalInput3DebounceTime;	///< Digital input 3 debounce time.
	static const CopleyParameter kDigitalInput4DebounceTime;	///< Digital input 4 debounce time.
	static const CopleyParameter kDigitalInput5DebounceTime;	///< Digital input 5 debounce time.
	static const CopleyParameter kDigitalInput6DebounceTime;	///< Digital input 6 debounce time.
	static const CopleyParameter kDigitalInput7DebounceTime;	///< Digital input 7 debounce time.
	static const CopleyParameter kDigitalInput8DebounceTime;	///< Digital input 8 debounce time.
	static const CopleyParameter kDigitalInput9DebounceTime;	///< Digital input 9 debounce time.
	static const CopleyParameter kDigitalInput10DebounceTime;	///< Digital input 10 debounce time.
	static const CopleyParameter kDigitalInput11DebounceTime;	///< Digital input 11 debounce time.
	static const CopleyParameter kDigitalInput12DebounceTime;	///< Digital input 12 debounce time.
	static const CopleyParameter kDigitalInput13DebounceTime;	///< Digital input 13 debounce time.
	static const CopleyParameter kDigitalInput14DebounceTime;	///< Digital input 14 debounce time.
	static const CopleyParameter kDigitalInput15DebounceTime;	///< Digital input 15 debounce time.

	// network
	static const CopleyParameter kNetworkCANLimitStatusMask;///< CAN limit status mask.
	static const CopleyParameter kNetworkCANAddressSwitch;	///< CAN parameter_id switch.
	static const CopleyParameter kNetworkStatus;			///< Network status.
	static const CopleyParameter kNetworkNodeIdPinMapping;	///< Network node ID input pin mapping.

	// misc - phase initialization configuration
	static const CopleyParameter kPhaseInitializationConfiguration;	///< Phase initialization configuration.

	// camming
	static const CopleyParameter kCammingConfiguration;		///< Camming configuration.
	static const CopleyParameter kCammingForwardDelay;		///< Camming forward delay.
	static const CopleyParameter kCammingReverseDelay;		///< Camming reverse delay.
	static const CopleyParameter kNetworkCANSendPdoType254;	///< Send any CANopen PDO objects configured with type 254.
	static const CopleyParameter kCammingVelocityMaster;	///< Camming master velocity.

	// misc
	static const CopleyParameter kHomePositionCapture;			///< Capture home position.
	static const CopleyParameter kDriveFirmwareVersionExtended;	///< Drive firwmare version (extended).

	// network
	static const CopleyParameter kNetworkCANHeartbeatTime;		///< CAN heartbeat time.
	static const CopleyParameter kNetworkCANNodeGuardingTime;	///< CAN node guarding time.
	static const CopleyParameter kNetworkCANNodeGuardingLifetimeFactor;	///< CAN node guarding lifetime factor.

	// misc
	static const CopleyParameter kDriveModePulsePositionRegistrationOffset;	///< Pulse & direction mode position registration offset.
	static const CopleyParameter kPositionCaptureHighspeedTimestamp;		///< Time stamp of last high speed position capture.
	static const CopleyParameter kPositionCaptureHighspeedPosition;			///< Captured position for high speed position capture.
	static const CopleyParameter kLoadEncoderPosition;				///< Load encoder parameter.
	static const CopleyParameter kNetworkCANEmergencyInhibitTime;	///< CAN emergency inhibit time.
	static const CopleyParameter kVelocityLoopViDrain;				///< Velocity control loop integral drain.
	static const CopleyParameter kTrajectoryBuffer;					///< Trajectory buffer.

	// network
	static const CopleyParameter kNetworkCANQuickStopOption;	///< CAN quick stop option code.
	static const CopleyParameter kNetworkCANShutdownOption;		///< CAN shutdown option code.
	static const CopleyParameter kNetworkCANDisableOption;		///< CAN disable option code.
	static const CopleyParameter kNetworkCANHaltOption;			///< CAN halt option code.

	// drive
	static const CopleyParameter kDriveUnitScaling;				///< Drive scaling configuration.
	static const CopleyParameter kDriveAxisCount;				///< Drive axis count.
	static const CopleyParameter kNetworkOptions;				///< Drive network options.
	static const CopleyParameter kRegenInternalCurrentLimit;	///< Regen resistor internal current limit.

	// encoder
	static const CopleyParameter kMotorEncoderPositionWrap;		///< Motor encoder position wrap.
	static const CopleyParameter kLoadEncoderPositionWrap;		///< Load encoder position wrap.
	static const CopleyParameter kEncoderCaptureConfiguration;	///< MACRO drive encoder capture circuit configuration.

	// misc
	static const CopleyParameter kFpgaFirmwareVersion;			///< FPGA firmware version.

	// gain scheduling
	static const CopleyParameter kGainSchedulingConfiguration;	///< Gain scheduling configuration.
	static const CopleyParameter kGainSchedulingKey;			///< Gain scheduling key.

	// misc
	static const CopleyParameter kDriveHardwareOptions;			///< Drive hardware options.
	static const CopleyParameter kMotorEncoderOptions;			///< Motor encoder options.
	static const CopleyParameter kLoadEncoderOptions;			///< Load encoder options.
	static const CopleyParameter kDriveSecondaryFirmwareVersion;///< Drive secondary firmware version.
	static const CopleyParameter kMotorEncoderStatus;			///< Motor encoder status.
	static const CopleyParameter kLoadEncoderStatus;			///< Load encoder status.

	// drive current
	static const CopleyParameter kDriveCurrentRMSPeriod;			///< RMS calculation period.
	static const CopleyParameter kDriveCurrentRMS;					///< RMS current.
	static const CopleyParameter kDriveCurrentUserLimitRunningSum;	///< Running sum of user current limit in percent.
	static const CopleyParameter kDriveCurrentAmpLimitRunningSum;	///< Running sum of amplifier current limit in percent.

	// analog output
	static const CopleyParameter kAnalogOutputConfiguration;			///< Analog output configuration.
	static const CopleyParameter kAnalogOutputValue;					///< Analog output value.
	static const CopleyParameter kAnalogInputSecondaryReferenceValue;	///< Secondary analog input reference value.
	static const CopleyParameter kAnalogInputSecondaryReferenceOffset;	///< Secondary analog input reference offset.
	static const CopleyParameter kAnalogInputSecondaryCalibrationOffset;///< Secondary analog input calibration offset.

	// drive
	static const CopleyParameter kDriveSafetyCircuitStatus;		///< Drive safety circuit status.
	static const CopleyParameter kMotorTempSensorVoltage;		///< Motor temperature sensor voltage.
	static const CopleyParameter kMotorTempSensorLimit;			///< Motor temperature sensor limit.

	// misc
	static const CopleyParameter kPwmPulseWidthMin;				///< Minimum PWM pulse width.
	static const CopleyParameter kPwmPulseWidthMax;				///< Maximum PWM pulse width.

	// control loop filters, configuration, and gains
	static const CopleyParameter kVelocityLoopOutputFilter2;	///< Velocity control loop second output filter.
	static const CopleyParameter kVelocityLoopOutputFilter3;	///< Velocity control loop third output filter.
	static const CopleyParameter kCurrentLoopInputFilter;		///< Current control loop first input filter.
	static const CopleyParameter kCurrentLoopInputFilter2;		///< Current control loop second input filter.
	static const CopleyParameter kServoLoopConfiguration;		///< Servo motor loop configuration.
	static const CopleyParameter kPositionLoopPd;				///< Position control loop derivative gain (aka Pd).
	static const CopleyParameter kPositionLoopPi;				///< Position control loop integral gain (aka Pi).
	static const CopleyParameter kVelocityLoopCommandFf;		///< Velocity control loop command feed-forward.
	static const CopleyParameter kPositionLoopIntegralDrain;	///< Position control loop integral drain.

	// misc
	static const CopleyParameter kNetworkCANAbortOption;		///< CAN abort option code.
	static const CopleyParameter kDigitalOptions;				///< Digital I/O options.
	static const CopleyParameter kMotorBrakeEnableDelay;		///< Motor brake enable delay.

	// digital I/O
	static const CopleyParameter kDigitalInputStateExtended;	///< Extended digital input state.
	static const CopleyParameter kDigitalInputStateRawExtended;	///< Extended raw digital input state.
	static const CopleyParameter kDigitalInputPullupConfigurationExtended;	///< Extended digital input pullup configuration.
	static const CopleyParameter kDigitalInput16Configuration;	///< Digital input 16 configuration.
	static const CopleyParameter kDigitalInput17Configuration;	///< Digital input 17 configuration.
	static const CopleyParameter kDigitalInput18Configuration;	///< Digital input 18 configuration.
	static const CopleyParameter kDigitalInput19Configuration;	///< Digital input 19 configuration.
	static const CopleyParameter kDigitalInput20Configuration;	///< Digital input 20 configuration.
	static const CopleyParameter kDigitalInput21Configuration;	///< Digital input 21 configuration.
	static const CopleyParameter kDigitalInput22Configuration;	///< Digital input 22 configuration.
	static const CopleyParameter kDigitalInput23Configuration;	///< Digital input 23 configuration.
	static const CopleyParameter kDigitalInput16DebounceTime;	///< Digital input 16 debounce time.
	static const CopleyParameter kDigitalInput17DebounceTime;	///< Digital input 17 debounce time.;
	static const CopleyParameter kDigitalInput18DebounceTime;	///< Digital input 18 debounce time.
	static const CopleyParameter kDigitalInput19DebounceTime;	///< Digital input 19 debounce time.
	static const CopleyParameter kDigitalInput20DebounceTime;	///< Digital input 20 debounce time.
	static const CopleyParameter kDigitalInput21DebounceTime;	///< Digital input 21 debounce time.
	static const CopleyParameter kDigitalInput22DebounceTime;	///< Digital input 22 debounce time.
	static const CopleyParameter kDigitalInput23DebounceTime;	///< Digital input 23 debounce time.

	// drive modes
	static const CopleyParameter kDriveModeUVConfiguration;		///< UV drive mode configuration.
	static const CopleyParameter kDriveModeUV_U_Input;			///< UV drive mode U input.
	static const CopleyParameter kDriveModeUV_V_Input;			///< UV drive mode V input.
	static const CopleyParameter kDriveModePulseCounterRaw;		///< Pulse & direction mode counter raw.

	// misc - position loop input filter
	static const CopleyParameter kPositionLoopInputFilter;		///< Position control loop input filter.

	// I/O output comparators
	static const CopleyParameter kDigitalOutputCompareConfiguration;///< Digital output compare configuration.
	static const CopleyParameter kDigitalOutputCompareStatus;		///< Digital output compare status.
	static const CopleyParameter kDigitalOutputCompareValue1;		///< Digital output compare value 1.
	static const CopleyParameter kDigitalOutputCompareValue2;		///< Digital output compare value 2.
	static const CopleyParameter kDigitalOutputCompareIncrement;	///< Digital output compare increment.
	static const CopleyParameter kDigitalOutputComparePulseWidth;	///< Digital output compare pulse width.

	// misc - trajectory options, I/O extension
	static const CopleyParameter kTrajectoryOptions;				///< Trajectory options.
	static const CopleyParameter kDriveIoExtensionConfiguration;	///< I/O extension configuration.

	// SPI
	static const CopleyParameter kSpiTransmit;					///< SPI transmit.
	static const CopleyParameter kSpiReceive;					///< SPI receive.

	// encoder
	static const CopleyParameter kAnalogEncoderSineOffset;		///< Analog encoder sine offset.
	static const CopleyParameter kAnalogEncoderCosineOffset;	///< Analog encoder cosine offset.
	static const CopleyParameter kAnalogEncoderCosineScaling;	///< Analog encoder cosine scaling factor.
	static const CopleyParameter kMotorEncoderCalibration;		///< Motor encoder calibration.
	static const CopleyParameter kLoadEncoderCalibration;		///< Load encoder calibration.

	// misc
	static const CopleyParameter kPwmInputDutyCycle;			///< PWM input duty cycle.
	static const CopleyParameter kTrajectoryAbortJerk;			///< Trajectory abort jerk.
	static const CopleyParameter kAnalogEncoderMagnitude;		///< Analog encoder magnitude.

	// position control loop gains - multi-axis cross-coupling
	static const CopleyParameter kPositionLoopCrossCouplingKp;	///< Position control loop cross-coupling proportional gain (Kp).
	static const CopleyParameter kPositionLoopCrossCouplingKi;	///< Position control loop cross-coupling integral grain (Ki).
	static const CopleyParameter kPositionLoopCrossCouplingKd;	///< Position control loop cross-coupling derivative gain (Kd).

	// misc
	static const CopleyParameter kMotorTempSensorSteinhartConstants;	///< Motor temperature sensor Steinhart constants.
	static const CopleyParameter kPwmDeadtimeCurrent;					///< Current at which PWM deadtime is used.
	static const CopleyParameter kLoadEncoderPassiveHighspeedCapture;	///< Passive load encoder high speed capture.
	static const CopleyParameter kMotorWiringCheckCurrent;				///< Open motor wiring check current.
	static const CopleyParameter kPositionLoopErrorTimeout;				///< Positon loop erorr timeout.

	// I/O - digital output configuration
	static const CopleyParameter kDigitalOutput9Configuration;		///< Digital output 9 configuration.
	static const CopleyParameter kDigitalOutput10Configuration;		///< Digital output 10 configuration.
	static const CopleyParameter kDigitalOutput11Configuration;		///< Digital output 11 configuration.
	static const CopleyParameter kDigitalOutput12Configuration;		///< Digital output 12 configuration.

	// encoder
	static const CopleyParameter kMotorEncoderDownShift;			///< Motor encoder down-shift.
	static const CopleyParameter kLoadEncoderDownShift;				///< Load encoder down-shift.
	// clang-format on

private:
	/// Construct a static CopleyParameter and add it to the static parameter map.
	/// @param [in] src_id The parameter ID.
	/// @param [in] src_type The parameter type description.
	/// @param [in] src_bytes The number of bytes of the parameter.
	/// @param [in] src_access The memory access mode of the parameter.
	/// @param [in] src_location The memory location of the parameter.
	/// @param [in] src_is_string The memory location refers to a string.
	/// @param [in] src_is_signed If numeric, value type is signed
	CopleyParameter(const parameter_id_t src_id,
					const std::string &src_type,
					const size_t src_bytes,
					const AccessMode src_access,
					const MemoryLocation src_location,
					const bool src_is_string,
					const bool src_is_signed);

	/// Make this struct non-copyable. Client code should always reference a static member.
	/// @param [in] src The source parameter to copy.
	CopleyParameter(const CopleyParameter &src) = delete;

	/// Static set of all parameters
	static std::map<const parameter_id_t, const CopleyParameter *const> parameter_map;
};

} // namespace momentum
