/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Model specifications for motors connected to Copley Controls Servo Drives.
 **/

#pragma once

#include <array>
#include <memory>
#include <stdint.h>
#include <string>
#include <vector>

namespace momentum
{

/// Copley amplifier configuration.
/// This object is not runtime configurable.
struct CopleyAmplifier {

	/// Copley amplifier models supported by driver
	enum class Model {
		AccelNet_ACK_055_10, ///< Amplifier Accelnet ACK-055-10
		AccelNet_ACK_090_20, ///< Amplifier Accelnet ACK-090-20
		AccelNet_BPL_090_30, ///< Amplifier Accelnet BPL-090-30
		AccelNet_BP2_090_20  ///< Amplifier Accelnet BP2-090-20 (2-axis,CANopen)
	};

	/// Model identifiers. Maps to static configurations.
	/// See register 0xAD for hardware type identifiers.
	enum class Identifier : uint16_t {
		AccelNet_Micro_Module   = 0x020C, ///< AccelNet Micro Module
		AccelNet_Plus_CAN		= 0x1040, ///< Accelnet Plus Panel CAN
		AccelNet_Plus_CAN_2axis = 0x1060, ///< Accelnet Plus Panel CAN - 2axis
	};

	const Model model;						 ///< Model for this amplifier
	const Identifier identifier;			 ///< Identifier for this amplifier.
	const std::string manufacturer;			 ///< Motor manufacturer.
	const std::string model_name;			 ///< Model string.
	const double current_peak_limit_A;		 ///< Peak current limit of the amplifier, in A.
	const double current_continuous_limit_A; ///< Continuous current limit of the amplifier, in A.
	const bool supports_Pi;					 ///< Supports position control loop integral gain.
	const bool supports_Pd;					 ///< Supports position control loop derivative gain.
	const uint8_t num_inputs;				 ///< Number of digital inputs on amplifier.
	const uint8_t num_outputs;				 ///< Number of digital outputs on amplifier.

	/// Set of input pins that can enable/disable internal pull-up resisitor
	const std::vector<int> inputs_with_pullup_option;

	/// Look up a static amplifier.
	/// @param [in] id The amplifier ID to lookup.
	/// @return Pointer to the amplifier model. nullptr is returned if not found.
	static const CopleyAmplifier *lookup(const Model model);

	/// Invalid amplifier. Use as a placeholder if a configuration will not be used.
	static const CopleyAmplifier invalid;
};

/// Copley motor encoder configuration.
/// This object is not runtime configurable.
struct CopleyMotorEncoder {
	/// Model identifiers. Maps to static configurations.
	enum class Identifier {
		Avago_HEDL_5540,			 ///< Avago HEDL-5540, 2000 CPR.
		Teknic_CPM_MCVC_xxxx_xxx_02, ///< Teknic CPM-MCVC series, 2000 CPR.
		Teknic_CPM_MCVC_xxxx_xxx_04, ///< Teknic CPM-MCVC series, 4000 CPR.
		Teknic_CPM_MCVC_xxxx_xxx_08, ///< Teknic CPM-MCVC series, 8000 CPR.
		USDigital_E5_1000,			 ///< US Digital E5-1000.
		USDigital_EM1_0_500,		 ///< US Digital EM1-0-500.
		MILE_Encoder,				 ///< MILE Encoder 453233.
		Encoder_MR,					 ///< Maxon MR encoder
		Custom,              ///< Custom encoder
		Hall_Feedback,				 ///< No motor encoder. The amplifier uses hall feedback for phasing
	};

	/// Encoder type.
	/// @note This driver supports primary encoders and digital hall feedback.
	enum class Type : uint16_t {
		kPrimary	 = 0, ///< Primary (differential) quadrature encoder.
		kDigitalHall = 6, ///< Use digital hall signals for position & velocity estimates.
	};

	/// Encoder direction.
	enum class Direction : uint16_t {
		kNormal  = 0, ///< Normal direction.
		kReverse = 1,
	};

	const Identifier identifier;	///< Identifier.
	const std::string manufacturer; ///< Motor manufacturer.
	const std::string model_name;   ///< Model string.
	const Type type;				///< Encoder type.
	const Direction direction;		///< Encoder direction.
	const double resolution_cpr;	///< Encoder resolution, counts per revolution.

	/// Lookup a static encoder.
	/// @param [in] id The encoder ID to look up.
	/// @return Pointer to the encoder model. Returns nullptr if not found.
	static const CopleyMotorEncoder *lookup(const Identifier id);

	/// Invalid encoder. Use as a placeholder if a configuration will not be used.
	static const CopleyMotorEncoder invalid;
};

/// Copley load encoder configuration.
/// This object is not runtime configurable.
struct CopleyLoadEncoder {
	/// Model identifiers. Maps to static configurations.
	enum class Identifier {
		Avago_HEDS_9700_E50, ///< Avago HEDS-9700#E50, 800 CPR.
		RLS_RM08,			 ///< RLS super small rotary
		RLS_RM22,			 ///< RLS rotary magnetic encoder
		RLS_BR10,			 ///< RLS absolute BiSS encoder
		Custom,        ///< Custom load encoder
		No_Load_Encoder,	 ///< No load encoder
	};

	/// Encoder type.
	enum class Type : uint16_t {
		kNone	  = 0,  ///< No load encoder.
		kPrimary   = 1,  ///< Primary (differential) quadrature encoder.
		kSsiSerial = 12, ///< SSI serial encoder.
		kBiSS	  = 13, ///< BiSS-C absolute encoder.
	};

	/// Encoder direction.
	enum class Direction : uint16_t {
		kNormal  = 0, ///< Normal direction.
		kReverse = 1,
	};

	const Identifier identifier;	///< Identifier.
	const std::string manufacturer; ///< Motor manufacturer.
	const std::string model_name;   ///< Model string.
	const Type type;				///< Encoder type.
	const double resolution_cpr;	///< Encoder resolution, counts per revolution.
	const bool passive_mode;		///< If true, don't use for feedback.
	const uint32_t options;			///< Load encoder options register value

	/// Lookup a static encoder.
	/// @param [in] id The encoder ID to look up.
	/// @return Pointer to the encoder model. Returns nullptr if not found.
	static const CopleyLoadEncoder *lookup(const Identifier id);

	/// Invalid encoder. Use as a placeholder if a configuration will not be used.
	static const CopleyLoadEncoder invalid;

	/// If set, then don't use this encoder for position feedback.
	static constexpr uint8_t kPositionFeedbackBitOffset = 5;
};

/// Container class for storing model-specific parameters.
/// This object is not runtime configurable.
struct CopleyMotor {
	/// Model identifiers. Maps to static configurations.
	enum class Identifier {
		AnaheimAutomation_BLY172D_24V_2000,	///< Anaheim Automation BLD172D-24V-2000.
		AnaheimAutomation_BLY171D_24V_4000,	///< Anaheim Automation BLY171D-24V-4000.
		AnaheimAutomation_BLWRPG173S_24V_4000, ///< Anaheim Automation BLWRPG173S-24V-4000.
		Minebea_BLDC36P16A_24V,				   ///< Minebea BLDC36P16A-24V.
		Maxon_449464,						   ///< Maxon Ec-i 40 449464.
		Teknic_CPM_MCVC_2321P_RLN,			   ///< Teknic CPM-MCVC-2321P-RLN.
		Teknic_CPM_MCVC_3432V_RLN,			   ///< Teknic M-3432V-LN-02D
		Custom,                            ///< Custom motor
	};

	/// Motor type. May be written directly to motor type register.
	/// @note This driver only supports rotary motors.
	enum class Type : uint16_t {
		kRotary = 0, ///< Rotary motor.
	};

	/// Motor architecture. Shifted values may be written directly to motor type register.
	/// @note This driver only supports brushless servos.
	enum class Architecture : uint16_t {
		kUnspecified	= 0 << 4, ///< Unspecified architecture.
		kBrushlessServo = 3 << 4, ///< Brushless servo motor.
	};

	/// Motor wiring.
	/// @note Swap UV corresponds to Motor Invert Output on.
	enum class Wiring : uint16_t {
		kStandard = 0, ///< Standard wiring.
		kSwapUV   = 1, ///< Swap U and V outputs.
	};

	/// Hall sensor type.
	/// @note This driver assumes digital Hall effect sensors.
	enum class HallType : uint16_t {
		kDigital = 1, /// < Digital Hall sensors.
	};

	/// Hall sensor wiring.
	enum class HallWiring : uint16_t {
		UVW = 0, ///< U V W
		UWV = 1, ///< U W V
		VUW = 2, ///< V U W
		VWU = 3, ///< V W U
		WVU = 4, ///< W V U
		WUV = 5, ///< W U V
	};

	/// Hall sensor wiring inversion. Inversion occurs after Hall wiring is applied.
	/// @note This driver does not support individual Hall input inversion.
	enum class HallInversion : uint16_t {
		kNone	  = 0,			 ///< No Hall wires are inverted.
		kInvertW   = 0b1 << 4,   ///< Invert W.
		kInvertV   = 0b1 << 5,   ///< Invert V.
		kInvertU   = 0b1 << 6,   ///< Invert U.
		kInvertAll = 0b111 << 4, ///< All Hall wires are inverted.
	};

	/// Commutation mode.
	/// @note Only standard mode (Halls + primary encoder) and trapezoidal mode (Halls only) are supported.
	enum class CommutationMode : uint16_t {
		kStandard	= 0, ///< Standard mode. Hall + encoder.
		kTrapezoidal = 1, ///< Trapezoidal (Hall based) phasing.
	};

	//-----
	// values based on motor datasheet except where noted otherwise
	//-----
	const Identifier identifier;		///< Identifier.
	const std::string manufacturer;		///< Motor manufacturer.
	const std::string model_name;		///< Model string.
	const Type type;					///< Motor type.
	const Architecture architecture;	///< Motor architecture.
	const double voltage_nominal_V;		///< Nominal voltage, in V.
	const double torque_constant_Nm_A;  ///< Torque constant, in Nm / A.
	const double resistance_ohm;		///< Armature resistance, in ohm.
	const double inductance_mH;			///< Armature inductance, in mH.
	const double torque_peak_Nm;		///< Peak (stall) torque, in Nm.
	const double torque_continuous_Nm;  ///< Continuous (nominal) torque, in Nm.
	const double current_peak_A;		///< Peak (stall) current, in A.
	const double current_continuous_A;  ///< Continuous (nominal) current, in A.
	const double back_emf_V_Krpm;		///< Motor back EMF constant, in V / Krpm.
	const double velocity_max_rpm;		///< Maximum velocity, in rpm.
	const double inertia_Kg_cm2;		///< Motor inertia, in Kg / cm^2
	const int16_t pole_pairs;			///< Pole pairs. Number of pole pairs (electrical phases) per rotation.
	const Wiring wiring;				///< Motor wiring.
	const double hall_offset_deg;		///< Motor Hall offset, in deg.
	const HallType hall_sensor;			///< Hall sensor type.
	const HallWiring hall_wiring;		///< Hall sensor wiring.
	const HallInversion hall_inversion; ///< Hall sensor wiring inversion.
	const CommutationMode commutation;  ///< Commutation mode.

	/// Lookup a static motor.
	/// @param [in] id The motor ID to look up.
	/// @return Pointer to the motor model. Returns nullptr if not found.
	static const CopleyMotor *lookup(const Identifier id);

	/// Invalid motor. Use as a placeholder if a configuration will not be used.
	static const CopleyMotor invalid;
};

/// Copley amplifier control parameters.
/// These settings are specific to a motor and encoder pair.
/// Values calculated from Copley Controls CME 2.
struct CopleyAlgorithm {
	double current_gain_Cp;	///< Current control loop proportional gain. Cp in Copley documentation. Must be in the
							   /// range [0, 32767].
	double current_gain_Ci;	///< Current control loop integral gain. Ci in Copley documentation. Must be in the range
							   ///[0, 32767].
	double current_peak_max_A; ///< Current control loop peak current, in A.
	double current_continuous_max_A;	   ///< Current control loop continuous current limit, in A.
	double current_I2t_ms;				   ///< Current control loop I^2*t time limit, in ms.
	double current_offset_A;			   ///< Current control loop offset, in A.
	double velocity_gain_Vp;			   ///< Velocity control loop proportional gain. Vp in Copley documentation.
	double velocity_gain_Vi;			   ///< Velocity control loop integral gain. Vi in Copley documentation.
	double velocity_gain_Vi_drain;		   ///< Velocity control loop integral drain. Vi drain in Copley documentation.
	double velocity_max_rpm;			   ///< Velocity control loop maximum velocity, in rpm.
	double velocity_accel_max_rps2;		   ///< Velocity control loop maximum acceleration, in rps^2
	double velocity_tracking_window_rpm;   ///< Velocity control loop tracking window, in rpm.
	double velocity_tracking_time_ms;	  ///< Velocity control loop tracking window time, in ms.
	int16_t velocity_loop_shift;		   ///< Velocity loop shift value.
	double position_gain_Pp;			   ///< Position control loop proportional gain. Pp in Copley documentation.
	double position_gain_Vff;			   ///< Position control loop velocity feed forward gain.
	double position_gain_Aff;			   ///< Position control loop acceleration feed forward gain.
	double position_following_error_cts;   ///< Position control loop following error limit, in encoder counts.
	double position_following_warning_cts; ///< Position control loop following warning limit, in encoder counts.
	double position_gains_multiplier_percent; ///< Position control loop gains multiplier; a value of 100.0 corresponds
											  /// to unity gain. Default is 1.0.
	double trajectory_velocity_max_rpm;		  ///< Trajectory control loop maximum velocity, in rpm.
	double trajectory_accel_max_rps2;		  ///< Trajectory control loop maximum acceleration, in rps^2.
	double trajectory_jerk_max_rps3;		  ///< Trajectory control loop maximum jerk, in rps^3.
	double home_fast_velocity_max_rpm;		  ///< Homing fast move velocity max, in rpm.
	double home_slow_velocity_max_rpm;		  ///< Homing slow move velocity max, in rpm.
	double home_accel_max_rps2;				  ///< Homing acceleration max, in rps^2.

	static constexpr double kGainMin = 0;	 ///< Control loop gain (proportional, integral, derivate) minimum value.
	static constexpr double kGainMax = 32767; ///< Control loop gain (proportional, integral, derivate) minimum value.

	/// Lookup an algorithm for a motor, motor encoder, and load encoder combination.
	/// @param [in] motor The motor identifier.
	/// @param [in] motor_encoder The motor encoder identifier.
	/// @param [in] load_encoder The load encoder identifier.
	/// @return Pointer to the initial control algorithm. nullptr if the motor + encoder pair is unsupported.
	static const CopleyAlgorithm *lookup(const CopleyMotor::Identifier motor,
										 const CopleyMotorEncoder::Identifier motor_encoder,
										 const CopleyLoadEncoder::Identifier load_encoder);

	/// Invalid motor. Use as a placeholder if a configuration will not be used.
	static const CopleyAlgorithm invalid;
};

/// Struct for Velocity Loop output filter. A generic bi-quad filter which acts on the output of the velocity
/// loop. Following formula is used by Copley to transform input x(n) to ouput y(n):
/// 		y(n) = K * (b0*x(n) + b1*x(n-1) + b2*x(n-2) + a1*y(n-1) + a2*y(n-2)) / 32768 / 4096
/// Register to configure is 9 or 14 word (1 word = 16 bits) parameter depending on amplifier type.
/// For details see Velocity Loop Filters in the CME 2 User Guide and/or p. 73 of Parameter Dictionary.
struct CopleyOutputFilter {

	/// Array of data to write to Copley Register to set filter
	std::vector<int16_t> filter_params;

	/// Lookup a output filter configuration for a Copley amplifier.
	/// @param [in] id Model of Copley amplifier used
	/// @return Pointer to initial filter values for amplifier model. nullptr if values aren't defined
	static const CopleyOutputFilter *lookup(const CopleyAmplifier::Model model);

	/// Invalid output filter. Use as a placeholder if a configuration will not be used.
	static const CopleyOutputFilter invalid;

	/// For PLUS amplifier type, the filter type is in the first word with a 13 bit offset
	static constexpr uint8_t kFilterTypeOffset = 13;

	CopleyOutputFilter(void) : filter_params()
	{
	}

	CopleyOutputFilter(const std::vector<int16_t> &params) : filter_params(params)
	{
	}

	~CopleyOutputFilter(void) = default;
};

/// Specification of the hardware attached to a Copley amplifier.
struct CopleyHardwareSpecification {
	/// I/O input pin type for a single pin.
	/// Not all configurations are supported by this driver.
	enum class InputPin : uint16_t {
		kNoFunction				= 0,  ///< No function.
		kLimitPosActiveHigh		= 4,  ///< Positive limit switch, active high.
		kLimitPosActiveLow		= 5,  ///< Positive limit switch, active low.
		kLimitNegActiveHigh		= 6,  ///< Negative limit switch, active high.
		kLimitNegActiveLow		= 7,  ///< Negative limit switch, active low.
		kHomeActiveHigh			= 14, ///< Home switch, active high.
		kHomeActiveLow			= 15, ///< Home switch, active low.
		kDriveDisableActiveHigh = 16, ///< Drive disable, active high.
		kDriveDisableActiveLow  = 17, ///< Drive disable, active low.
	};

	/// Bit offset for axis parameter in input pin configuration.
	static constexpr uint8_t kInputPinConfigAxisOffset = 12;

	/// I/O output pin type for a single pin.
	/// Not all configurations are supported by this driver.
	enum class OutputPin : uint16_t {
		kTrackEventStatus		 = 0, ///< Track the equivalent bit in the event status register.
		kTrackLatchedEventStatus = 1, ///< Track the equivalent bit in the latched event status register.
		kTrackManualOutput		 = 2, ///< Track the equivalent bit in the manual output register.
		kTrackTrajectoryStatus   = 3, ///< Track the equivalent bit in the trajectory status register.
	};

	/// Bit offset for axis parameter in output pin configuration.
	static constexpr uint8_t kOutputPinConfigAxisOffset = 12;

	/// I/O output pin modifier for a single pin.
	/// These optional configurations supplement one of the required OutputPin configurations.
	/// Not all configurations are supported by this driver.
	enum class OutputPinModifier : uint16_t {
		kActiveLow  = 0,   ///< Set the output pin low when the corresponding bit is set.
		kActiveHigh = 256, ///< Set the output pin high when the corresponding bit is set.
	};

	/// Constructor requires all hardware specifications.
	CopleyHardwareSpecification(const CopleyAmplifier &amplifier,
								const CopleyMotor &motor,
								const CopleyMotorEncoder &motor_encoder,
								const CopleyLoadEncoder &load_encoder,
								const CopleyLoadEncoder::Direction &load_encoder_direction,
								const CopleyAlgorithm &algorithm_default,
								const CopleyOutputFilter &filter_default,
								const double gear_ratio);

	/// Copley amplifier model.
	const CopleyAmplifier &amplifier;
	/// Copley motor model.
	const CopleyMotor &motor;
	/// Copley motor encoder model.
	const CopleyMotorEncoder &motor_encoder;
	/// Copley load encoder model.
	const CopleyLoadEncoder &load_encoder;
	/// Copley load encoder direction.
	const CopleyLoadEncoder::Direction &load_encoder_direction;
	/// Copley control algorithm.
	const CopleyAlgorithm &algorithm_default;
	/// Velocity output loop filter
	const CopleyOutputFilter &filter_default;
	/// Gear ratio of the motor.
	const double gear_ratio;

	/// Invalid specification. Use as a placeholder if a configuration will not be used.
	static const CopleyHardwareSpecification invalid;
};

/// Overload the '|' operator in order to combine OutputPin options via their underlying bit representations
/// See http://blog.bitwigglers.org/using-enum-classes-as-type-safe-bitmasks/ for jumping off point
/// This method cannot be defined within OutputPin (because enum classes do not allow methods)
/// nor within the CopleyHardwareSpecification body (because the compiler will think the definition is overloading
/// CopleyHardwareSpecification::operator|)
CopleyHardwareSpecification::OutputPin operator|(CopleyHardwareSpecification::OutputPin lhs,
												 CopleyHardwareSpecification::OutputPinModifier rhs);

} // namespace momentum
