/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/copley/CopleyConversion.hpp>
#include <drivers/copley/CopleyModel.hpp>

#include <map>
#include <tuple>

namespace momentum
{

// put CopleyConversion values into this namespace for convenience
using namespace CopleyConversion;

const CopleyAmplifier CopleyAmplifier::invalid = {
		.model = static_cast<CopleyAmplifier::Model>(-1),
		.identifier = static_cast<CopleyAmplifier::Identifier>(-1),
		.manufacturer = "unknown",
		.model_name = "unknown",
		.current_peak_limit_A = 0.0,
		.current_continuous_limit_A = 0.0,
		.supports_Pi = false,
		.supports_Pd = false,
		.num_inputs = 0,
		.num_outputs = 0,
		.inputs_with_pullup_option = {}
};

const CopleyAmplifier * CopleyAmplifier::lookup( const Model model )
{
	// static array of all supported amplifiers
	static const std::array<CopleyAmplifier, 4> amps = {{
			CopleyAmplifier{	.model = CopleyAmplifier::Model::AccelNet_ACK_055_10,
								.identifier = CopleyAmplifier::Identifier::AccelNet_Micro_Module,
								.manufacturer = "Copley Controls",
								.model_name = "AccelNet Micro Module",
								.current_peak_limit_A = 10.0,
								.current_continuous_limit_A = 5.0,
								.supports_Pi = false,
								.supports_Pd = false,
								.num_inputs = 10,
								.num_outputs = 3,
								.inputs_with_pullup_option = {1, 2, 3, 4, 5, 6, 7, 8, 9}},
			CopleyAmplifier{	.model = CopleyAmplifier::Model::AccelNet_ACK_090_20,
								.identifier = CopleyAmplifier::Identifier::AccelNet_Micro_Module,
								.manufacturer = "Copley Controls",
								.model_name = "AccelNet Micro Module",
								.current_peak_limit_A = 20.0,
								.current_continuous_limit_A = 10.0,
								.supports_Pi = false,
								.supports_Pd = false,
								.num_inputs = 10,
								.num_outputs = 3,
								.inputs_with_pullup_option = {1, 2, 3, 4, 5, 6, 7, 8, 9}},
			CopleyAmplifier{	.model = CopleyAmplifier::Model::AccelNet_BPL_090_30,
								.identifier = CopleyAmplifier::Identifier::AccelNet_Plus_CAN,
								.manufacturer = "Copley Controls",
								.model_name = "Accelnet Plus CANopen",
								.current_peak_limit_A = 30.0,
								.current_continuous_limit_A = 15.0,
								.supports_Pi = false,
								.supports_Pd = false,
								.num_inputs = 11,
								.num_outputs = 4,
								.inputs_with_pullup_option = {0, 1, 2, 3, 4, 5}},
			CopleyAmplifier{	.model = CopleyAmplifier::Model::AccelNet_BP2_090_20,
								.identifier = CopleyAmplifier::Identifier::AccelNet_Plus_CAN_2axis,
								.manufacturer = "Copley Controls",
								.model_name = "2-Axis Accelnet Plus CANopen",
								.current_peak_limit_A = 20.0,
								.current_continuous_limit_A = 10.0,
								.supports_Pi = false,
								.supports_Pd = false,
								.num_inputs = 18,
								.num_outputs = 7,
								.inputs_with_pullup_option = {0, 1, 2, 3, 9, 10, 11, 12}}
	}};

	for( const CopleyAmplifier & amp : amps ){
		if( amp.model == model ){
			return &amp;
		}
	}
	return nullptr;
}

const CopleyMotorEncoder CopleyMotorEncoder::invalid = {
		.identifier = static_cast<CopleyMotorEncoder::Identifier>(-1),
		.manufacturer = "unknown",
		.model_name = "unknown",
		.type = static_cast<CopleyMotorEncoder::Type>(-1),
		.direction = static_cast<CopleyMotorEncoder::Direction>(-1),
		.resolution_cpr = 0.0
};

const CopleyMotorEncoder * CopleyMotorEncoder::lookup( const Identifier id )
{
	// static array of all supported encoders
	static const std::array<CopleyMotorEncoder, 9> encoders = {{
			CopleyMotorEncoder{
							.identifier = CopleyMotorEncoder::Identifier::Avago_HEDL_5540,
							.manufacturer = "Avago",
							.model_name = "HEDL-5540",
							.type = CopleyMotorEncoder::Type::kPrimary,
							.direction = CopleyMotorEncoder::Direction::kNormal,
							.resolution_cpr = 2000 },
			CopleyMotorEncoder{
							.identifier = CopleyMotorEncoder::Identifier::Hall_Feedback,
							.manufacturer = "N/A",
							.model_name = "N/A",
							.type = CopleyMotorEncoder::Type::kDigitalHall,
							.direction = CopleyMotorEncoder::Direction::kNormal,
							.resolution_cpr = 0 },
			CopleyMotorEncoder{
							.identifier = CopleyMotorEncoder::Identifier::Teknic_CPM_MCVC_xxxx_xxx_02,
					    .manufacturer = "Teknic",
					    .model_name = "CPM-MCVC-xxxx-xxx-02",
					    .type = CopleyMotorEncoder::Type::kPrimary,
					    .direction = CopleyMotorEncoder::Direction::kNormal,
					    .resolution_cpr = 2000 },
			CopleyMotorEncoder{
							.identifier = CopleyMotorEncoder::Identifier::Teknic_CPM_MCVC_xxxx_xxx_04,
							.manufacturer = "Teknic",
							.model_name = "CPM-MCVC-xxxx-xxx-04",
							.type = CopleyMotorEncoder::Type::kPrimary,
							.direction = CopleyMotorEncoder::Direction::kReverse,
							.resolution_cpr = 4000 },
			CopleyMotorEncoder{
							.identifier = CopleyMotorEncoder::Identifier::Teknic_CPM_MCVC_xxxx_xxx_08,
							.manufacturer = "Teknic",
							.model_name = "CPM-MCVC-xxxx-xxx-08",
							.type = CopleyMotorEncoder::Type::kPrimary,
							.direction = CopleyMotorEncoder::Direction::kNormal,
							.resolution_cpr = 8000 },
			CopleyMotorEncoder{
							.identifier = CopleyMotorEncoder::Identifier::USDigital_E5_1000,
							.manufacturer = "US Digital",
							.model_name = "E5-1000",
							.type = CopleyMotorEncoder::Type::kPrimary,
							.direction = CopleyMotorEncoder::Direction::kNormal,
							.resolution_cpr = 4000 },
			CopleyMotorEncoder{
							.identifier = CopleyMotorEncoder::Identifier::USDigital_EM1_0_500,
							.manufacturer = "US Digital",
							.model_name = "EM1-0-500",
							.type = CopleyMotorEncoder::Type::kPrimary,
							.direction = CopleyMotorEncoder::Direction::kReverse,
							.resolution_cpr = 5500 },
			CopleyMotorEncoder{
							.identifier = CopleyMotorEncoder::Identifier::MILE_Encoder,
							.manufacturer = "Maxon",
							.model_name = "453233",
							.type = CopleyMotorEncoder::Type::kPrimary,
							.direction = CopleyMotorEncoder::Direction::kReverse,
							.resolution_cpr = 4096 },
			CopleyMotorEncoder{
							.identifier = CopleyMotorEncoder::Identifier::Encoder_MR,
							.manufacturer = "Maxon",
							.model_name = "225778",
							.type = CopleyMotorEncoder::Type::kPrimary,
							.direction = CopleyMotorEncoder::Direction::kNormal,
							.resolution_cpr = 2048 },
	}};

	for( const CopleyMotorEncoder & encoder : encoders ){
		if( encoder.identifier == id ){
			return &encoder;
		}
	}
	return nullptr;
}

const CopleyLoadEncoder CopleyLoadEncoder::invalid = {
		.identifier      = static_cast<CopleyLoadEncoder::Identifier>(-1),
		.manufacturer    = "unknown",
		.model_name      = "unknown",
		.type            = static_cast<CopleyLoadEncoder::Type>(-1),
		.resolution_cpr  = 0.0,
		.passive_mode    = false,
		.options         = 0
};

const CopleyLoadEncoder * CopleyLoadEncoder::lookup( const Identifier id )
{
	// static array of all supported encoders
	static const std::array<CopleyLoadEncoder, 5> encoders = {{
			CopleyLoadEncoder{
							.identifier = CopleyLoadEncoder::Identifier::Avago_HEDS_9700_E50,
							.manufacturer = "Avago",
							.model_name = "HEDL-9700#E50",
							.type = CopleyLoadEncoder::Type::kPrimary,
							.resolution_cpr = 800,
							.passive_mode = false,
							.options = 0},
			CopleyLoadEncoder{
							.identifier = CopleyLoadEncoder::Identifier::RLS_RM08,
							.manufacturer = "N/A",
							.model_name = "N/A",
							.type = CopleyLoadEncoder::Type::kSsiSerial,
							.resolution_cpr = 2048,
							.passive_mode = true,
							.options = 11 + (0b1 << 25)}, // 11-bit resolution, use multi-mode encoder port
			CopleyLoadEncoder{
							.identifier = CopleyLoadEncoder::Identifier::RLS_RM22,
							.manufacturer = "RLS",
							.model_name = "RM22SC0012B",
							.type = CopleyLoadEncoder::Type::kSsiSerial,
							.resolution_cpr = 4096,
							.passive_mode = true,
							.options = 12            // 12-bit resolution
									+ (0b1 << 25)},  // Use multi-mode encoder port
			CopleyLoadEncoder{
							.identifier = CopleyLoadEncoder::Identifier::RLS_BR10,
							.manufacturer = "RLS",
							.model_name = "Orbis absolute rotary",
							.type = CopleyLoadEncoder::Type::kBiSS,
							.resolution_cpr = 16384,
							.passive_mode = true,
							.options = 14 		    // 14-bit resolution,
									+ (16 << 8)     // 16-bit multiturn counter
									+ (0b1 << 16)   // ModeC encoder
									+ (0b1 << 22)   // error bit sent before warning
									+ (0b1 << 28)}, // use multi-mode encoder port
			CopleyLoadEncoder{
							.identifier = CopleyLoadEncoder::Identifier::No_Load_Encoder,
							.manufacturer = "N/A",
							.model_name = "N/A",
							.type = CopleyLoadEncoder::Type::kNone,
							.resolution_cpr = 0,
							.passive_mode = false,
							.options = 0}
	}};

	for( const CopleyLoadEncoder & encoder : encoders ){
		if( encoder.identifier == id ){
			return &encoder;
		}
	}
	return nullptr;
}

const CopleyMotor CopleyMotor::invalid = {
		.identifier = static_cast<CopleyMotor::Identifier>(-1),
		.manufacturer = "unknown",
		.model_name = "unknown",
		.type = static_cast<CopleyMotor::Type>(-1),
		.architecture = static_cast<CopleyMotor::Architecture>(-1),
		.voltage_nominal_V = 0.0,
		.torque_constant_Nm_A = 0.0,
		.resistance_ohm = 0.0,
		.inductance_mH = 0.0,
		.torque_peak_Nm = 0.0,
		.torque_continuous_Nm = 0.0,
		.current_peak_A = 0.0,
		.current_continuous_A = 0.0,
		.back_emf_V_Krpm = 0.0,
		.velocity_max_rpm = 0.0,
		.inertia_Kg_cm2 = 0.0,
		.pole_pairs = 0,
		.wiring = static_cast<CopleyMotor::Wiring>(-1),
		.hall_offset_deg = 0.0,
		.hall_sensor = static_cast<CopleyMotor::HallType>(-1),
		.hall_wiring = static_cast<CopleyMotor::HallWiring>(-1),
		.hall_inversion = static_cast<CopleyMotor::HallInversion>(-1),
		.commutation = static_cast<CopleyMotor::CommutationMode>(-1)
};

const CopleyMotor * CopleyMotor::lookup( const Identifier id )
{
	static const std::array<CopleyMotor, 7> motors = {{
		CopleyMotor{
					.identifier = CopleyMotor::Identifier::AnaheimAutomation_BLY172D_24V_2000,
					.manufacturer = "Anaheim Automation",
					.model_name = "BLD172D-24V-2000",
					.type = CopleyMotor::Type::kRotary,
					.architecture = CopleyMotor::Architecture::kBrushlessServo,
					.voltage_nominal_V = 24,
					.torque_constant_Nm_A = 7.79 * scale_ozin_to_Nm,
					.resistance_ohm = 1.40,
					.inductance_mH = 2.25,
					.torque_peak_Nm = 54.0 * scale_ozin_to_Nm,
					.torque_continuous_Nm = 28.3 * scale_ozin_to_Nm,
					.current_peak_A = 6.93,			// = torque_peak_Nm / torque_constant_Nm_A
					.current_continuous_A = 3.63,	// = torque_continuous_Nm / torque_constant_Nm_A,
					.back_emf_V_Krpm = 5.66,
					.velocity_max_rpm = 2800,
					.inertia_Kg_cm2 = 0.000680 * scale_ozin_s2_to_kg_cm2,
					.pole_pairs = 4,
					.wiring = CopleyMotor::Wiring::kSwapUV,
					.hall_offset_deg = 0.0,
					.hall_sensor = CopleyMotor::HallType::kDigital,
					.hall_wiring = CopleyMotor::HallWiring::WVU,
					.hall_inversion = CopleyMotor::HallInversion::kNone,
					.commutation = CopleyMotor::CommutationMode::kStandard },
		CopleyMotor{
					.identifier = CopleyMotor::Identifier::AnaheimAutomation_BLY171D_24V_4000,
					.manufacturer = "Anaheim Automation",
					.model_name = "BLY171D-24V-4000",
					.type = CopleyMotor::Type::kRotary,
					.architecture = CopleyMotor::Architecture::kBrushlessServo,
					.voltage_nominal_V = 24,
					.torque_constant_Nm_A = 4.81 * scale_ozin_to_Nm,
					.resistance_ohm = 1.50,
					.inductance_mH = 2.10,
					.torque_peak_Nm = 27. * scale_ozin_to_Nm,
					.torque_continuous_Nm = 8.9 * scale_ozin_to_Nm,
					.current_peak_A = 5.61,			// = torque_peak_Nm / torque_constant_Nm_A (
					.current_continuous_A = 4.0,
					.back_emf_V_Krpm = 2.7,
					.velocity_max_rpm = 8000,
					.inertia_Kg_cm2 = 0.00034 * scale_ozin_s2_to_kg_cm2,
					.pole_pairs = 4,
					.wiring = CopleyMotor::Wiring::kSwapUV,
					.hall_offset_deg = 0.0,
					.hall_sensor = CopleyMotor::HallType::kDigital,
					.hall_wiring = CopleyMotor::HallWiring::WVU,
					.hall_inversion = CopleyMotor::HallInversion::kNone,
					.commutation = CopleyMotor::CommutationMode::kStandard },
		CopleyMotor{
					.identifier = CopleyMotor::Identifier::AnaheimAutomation_BLWRPG173S_24V_4000,
					.manufacturer = "Anaheim Automation",
					.model_name = "BLWRPG-173S-24V-4000",
					.type = CopleyMotor::Type::kRotary,
					.architecture = CopleyMotor::Architecture::kBrushlessServo,
					.voltage_nominal_V = 24,
					.torque_constant_Nm_A = 5.21 * scale_ozin_to_Nm,
					.resistance_ohm = 0.71,
					.inductance_mH = 0.86,
					.torque_peak_Nm = 63.7 * scale_ozin_to_Nm,
					.torque_continuous_Nm = 21.2 * scale_ozin_to_Nm,
					.current_peak_A = 12.23,	// = torque_peak_Nm / torque_constant_Nm_A
					.current_continuous_A = 4.1,
					.back_emf_V_Krpm = 4.0,
					.velocity_max_rpm = 4000,
					.inertia_Kg_cm2 = 0.000821 * scale_ozin_s2_to_kg_cm2,
					.pole_pairs = 4,
					.wiring = CopleyMotor::Wiring::kStandard,
					.hall_offset_deg = 30.0,
					.hall_sensor = CopleyMotor::HallType::kDigital,
					.hall_wiring = CopleyMotor::HallWiring::UVW,
					.hall_inversion = CopleyMotor::HallInversion::kNone,
					.commutation = CopleyMotor::CommutationMode::kStandard },
		CopleyMotor{
					.identifier = CopleyMotor::Identifier::Maxon_449464,
					.manufacturer = "Maxon",
					.model_name = "EC-i 40 449464",
					.type = CopleyMotor::Type::kRotary,
					.architecture = CopleyMotor::Architecture::kBrushlessServo,
					.voltage_nominal_V = 24,
					.torque_constant_Nm_A = 0.00895,
					.resistance_ohm = 0.501,
					.inductance_mH = 0.39,
					.torque_peak_Nm = 0.810,
					.torque_continuous_Nm = 0.0528,
					.current_peak_A = 10.0,			// from datasheet, also equal to torque_peak / torque_constant
					.current_continuous_A = 3.0, 	// from datasheet, approximately torque_continuous / torque_constant
					.back_emf_V_Krpm = 1.77,	// inverse of speed constant
					.velocity_max_rpm = 13200.0,
					.inertia_Kg_cm2 = 0.0105,
					.pole_pairs = 7,
					.wiring = CopleyMotor::Wiring::kStandard,
					.hall_offset_deg = 0.0,
					.hall_sensor = CopleyMotor::HallType::kDigital,
					.hall_wiring = CopleyMotor::HallWiring::UWV,
					.hall_inversion = CopleyMotor::HallInversion::kInvertAll,
					.commutation = CopleyMotor::CommutationMode::kTrapezoidal },
		CopleyMotor{
					.identifier = CopleyMotor::Identifier::Teknic_CPM_MCVC_2321P_RLN,
					.manufacturer = "Teknic",
					.model_name = "CPM-MCVC-2321P-RLN",
					.type = CopleyMotor::Type::kRotary,
					.architecture = CopleyMotor::Architecture::kBrushlessServo,
					.voltage_nominal_V = 48,
					.torque_constant_Nm_A = 14.189 * scale_ozin_to_Nm,	// Source: 3/20/15 output from TS Calc "M-2321P with SSt-E545 at 48V"
					.resistance_ohm = 0.616,					// Source: Teknic Industrial-Grade NEMA 23 Motors
																//    Alt: 3/20/15 output from TS Calc "M-2321P with SSt-E545 at 48V"
					.inductance_mH = 0.915,						// Source: Teknic Industrial-Grade NEMA 23 Motors
																//    Alt: 3/20/15 output from TS Calc "M-2321P with SSt-E545 at 48V"
					.torque_peak_Nm = 133.716 * scale_ozin_to_Nm,		// Source: 3/20/15 output from TS Calc "M-2321P with SSt-E545 at 48V"
					.torque_continuous_Nm = 116.2 * scale_ozin_to_Nm,	// Source: Teknic Industrial-Grade NEMA 23 Motors (also confirmed on motor label)
																//    Alt: 3/20/15 output from TS Calc "M-2321P with SSt-E545 at 48V" (calculated as 109.5)
					.current_peak_A = 33,						// Source: 3/20/15 output from TS Calc "M-2321P with SSt-E545 at 48V" (stated as 33A)
					.current_continuous_A = 8.5,				// Source: label on motor
																//    Alt: 3/20/15 output from TS Calc "M-2321P with SSt-E545 at 48V" (calculated as 9.0)
					.back_emf_V_Krpm = 11.65,					// Source: Teknic Industrial-Grade NEMA 23 Motors (also confirmed on motor label)
																//    Alt: 3/20/15 output from TS Calc "M-2321P with SSt-E545 at 48V" (calculated as 10.5)
					.velocity_max_rpm = 6000,					// Source: label on motor
					.inertia_Kg_cm2 = 0.002 * scale_ozin_s2_to_kg_cm2,	// Source: 3/20/15 output from TS Calc "M-2321P with SSt-E545 at 48V"
					.pole_pairs = 4,
					.wiring = CopleyMotor::Wiring::kStandard,					// Source: CME 2 auto-phasing tool. For seasoner carousel configuration is overwritten in SeasonerHardware1_5. See MBQFR-25 in jira for more information
					.hall_offset_deg = 0.0,									// Source: CME 2 auto-phasing tool
					.hall_sensor = CopleyMotor::HallType::kDigital,
					.hall_wiring = CopleyMotor::HallWiring::WUV,			// Source: CME 2 auto-phasing tool. For seasoner carousel configuration is overwritten in SeasonerHardware1_5. See MBQFR-25 in jira for more information
					.hall_inversion = CopleyMotor::HallInversion::kNone,	// Source: CME 2 auto-phasing tool
					.commutation = CopleyMotor::CommutationMode::kStandard },
		CopleyMotor{
					.identifier = CopleyMotor::Identifier::Teknic_CPM_MCVC_3432V_RLN,
					.manufacturer = "Teknic",
					.model_name = "M-3432V-LN-02D",
					.type = CopleyMotor::Type::kRotary,						// Everything beyond this point must be checked.*
					.architecture = CopleyMotor::Architecture::kBrushlessServo,
					.voltage_nominal_V = 48,
					.torque_constant_Nm_A = 23.27 * scale_ozin_to_Nm,		// Source: 10/12/20 output from TS Calc "M-3432V with Generic Drive at 48VDC"
					.resistance_ohm = 2.183,								// *Source: Teknic Industrial-Grade NEMA 34 Motors
																			//    Alt:
					.inductance_mH = 5.058,									// *Source: Teknic Industrial-Grade NEMA 23 Motors
																			//    Alt: 3/20/15 output from TS Calc "M-2321P with SSt-E545 at 48V"
					.torque_peak_Nm = 604.6 * scale_ozin_to_Nm,				// Source: 10/12/20 output from TS Calc "M-3432V with Generic Drive at 48VDC"
					.torque_continuous_Nm = 376.9 * scale_ozin_to_Nm,		// Source: Teknic Industrial-Grade NEMA 34 Motors (also confirmed on motor label)
																			//    Alt:
					.current_peak_A = 30,									// Source: 10/12/20 output from TS Calc "M-3432V with Generic Drive at 48VDC"
					.current_continuous_A = 19.1,							// Source: label on motor
																			//    Alt:
					.back_emf_V_Krpm = 16.9,								// Source: Teknic Industrial-Grade NEMA 34 Motors (also confirmed on motor label)
																			//    Alt:
					.velocity_max_rpm = 6000,								// Source: label on motor
					.inertia_Kg_cm2 = 0.030 * scale_ozin_s2_to_kg_cm2,		// Source: 10/12/20 output from TS Calc "M-3432V with Generic Drive at 48VDC"
					.pole_pairs = 4,
					.wiring = CopleyMotor::Wiring::kSwapUV,				// *Source: CME 2 auto-phasing tool
					.hall_offset_deg = 0.0,									// *Source: CME 2 auto-phasing tool
					.hall_sensor = CopleyMotor::HallType::kDigital,
					.hall_wiring = CopleyMotor::HallWiring::VUW,			// *Source: CME 2 auto-phasing tool
					.hall_inversion = CopleyMotor::HallInversion::kNone,	// *Source: CME 2 auto-phasing tool
					.commutation = CopleyMotor::CommutationMode::kStandard },
		CopleyMotor{
					.identifier = CopleyMotor::Identifier::Minebea_BLDC36P16A_24V,
					.manufacturer = "Minebea",
					.model_name = "BLDC36P16A-24V",
					.type = CopleyMotor::Type::kRotary,
					.architecture = CopleyMotor::Architecture::kBrushlessServo,
					.voltage_nominal_V = 24,
					.torque_constant_Nm_A = .0422,
					.resistance_ohm = 2.24,
					.inductance_mH = 1.0,
					.torque_peak_Nm = .226,
					.torque_continuous_Nm = .063,
					.current_peak_A = 5.0,	// = torque_peak_Nm / torque_constant_Nm_A (
					.current_continuous_A = 1.8, // = torque_continuous_Nm / torque_constant_Nm_A,
					.back_emf_V_Krpm = 4.52,
					.velocity_max_rpm = 5200,
					.inertia_Kg_cm2 = .0266,
					.pole_pairs = 3,
					.wiring = CopleyMotor::Wiring::kSwapUV,
					.hall_offset_deg = 0.0,
					.hall_sensor = CopleyMotor::HallType::kDigital,
					.hall_wiring = CopleyMotor::HallWiring::VWU,
					.hall_inversion = CopleyMotor::HallInversion::kInvertAll,
					.commutation = CopleyMotor::CommutationMode::kTrapezoidal }
	}};

	for( const CopleyMotor & motor : motors ){
		if( motor.identifier == id ){
			return &motor;
		}
	}
	return nullptr;
}

const CopleyAlgorithm CopleyAlgorithm::invalid = {
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

const CopleyAlgorithm * CopleyAlgorithm::lookup( const CopleyMotor::Identifier motor, const CopleyMotorEncoder::Identifier motor_encoder, const CopleyLoadEncoder::Identifier load_encoder )
{
	static const std::array< std::tuple< CopleyMotor::Identifier, CopleyMotorEncoder::Identifier, CopleyLoadEncoder::Identifier, CopleyAlgorithm >, 10 > algorithms = {{
			std::make_tuple<CopleyMotor::Identifier, CopleyMotorEncoder::Identifier, CopleyLoadEncoder::Identifier, CopleyAlgorithm>(
				CopleyMotor::Identifier::AnaheimAutomation_BLY172D_24V_2000,
				CopleyMotorEncoder::Identifier::USDigital_E5_1000,
				CopleyLoadEncoder::Identifier::RLS_BR10,
				CopleyAlgorithm{.current_gain_Cp = 291,
								.current_gain_Ci = 23,
								.current_peak_max_A = 6.93,
								.current_continuous_max_A = 3.63,
								.current_I2t_ms = 1000,
								.current_offset_A = 0,
								.velocity_gain_Vp = 227,
								.velocity_gain_Vi = 51,
								.velocity_gain_Vi_drain = 0,
								.velocity_max_rpm = 2000,
								.velocity_accel_max_rps2 = 333,
								.velocity_tracking_window_rpm = 600,
								.velocity_tracking_time_ms = 100,
								.velocity_loop_shift = 0,
								.position_gain_Pp = 1000,
								.position_gain_Vff = 16384,
								.position_gain_Aff = 0,
								.position_following_error_cts = 4000,
								.position_following_warning_cts = 2000,
								.position_gains_multiplier_percent = 100,
								.trajectory_velocity_max_rpm = 2800,
								.trajectory_accel_max_rps2 = 333,
								.trajectory_jerk_max_rps3 = 6680,
								.home_fast_velocity_max_rpm = 50,
								.home_slow_velocity_max_rpm = 10,
								.home_accel_max_rps2 = 17 }),
			std::make_tuple<CopleyMotor::Identifier, CopleyMotorEncoder::Identifier, CopleyLoadEncoder::Identifier, CopleyAlgorithm>(
				CopleyMotor::Identifier::AnaheimAutomation_BLY172D_24V_2000,
				CopleyMotorEncoder::Identifier::USDigital_E5_1000,
				CopleyLoadEncoder::Identifier::No_Load_Encoder,
				CopleyAlgorithm{.current_gain_Cp = 291,
								.current_gain_Ci = 23,
								.current_peak_max_A = 6.93,
								.current_continuous_max_A = 3.63,
								.current_I2t_ms = 1000,
								.current_offset_A = 0,
								.velocity_gain_Vp = 227,
								.velocity_gain_Vi = 51,
								.velocity_gain_Vi_drain = 0,
								.velocity_max_rpm = 2000,
								.velocity_accel_max_rps2 = 333,
								.velocity_tracking_window_rpm = 600,
								.velocity_tracking_time_ms = 100,
								.velocity_loop_shift = 0,
								.position_gain_Pp = 1000,
								.position_gain_Vff = 16384,
								.position_gain_Aff = 0,
								.position_following_error_cts = 4000,
								.position_following_warning_cts = 2000,
								.position_gains_multiplier_percent = 100,
								.trajectory_velocity_max_rpm = 2800,
								.trajectory_accel_max_rps2 = 333,
								.trajectory_jerk_max_rps3 = 6680,
								.home_fast_velocity_max_rpm = 50,
								.home_slow_velocity_max_rpm = 10,
								.home_accel_max_rps2 = 17 } ),
			std::make_tuple<CopleyMotor::Identifier, CopleyMotorEncoder::Identifier, CopleyLoadEncoder::Identifier, CopleyAlgorithm>(
				CopleyMotor::Identifier::AnaheimAutomation_BLY171D_24V_4000,
				CopleyMotorEncoder::Identifier::USDigital_E5_1000,
				CopleyLoadEncoder::Identifier::No_Load_Encoder,
				CopleyAlgorithm{.current_gain_Cp = 325,
								.current_gain_Ci = 42,
								.current_peak_max_A = 5.61,
								.current_continuous_max_A = 4.0,
								.current_I2t_ms = 1000,
								.current_offset_A = 0,
								.velocity_gain_Vp = 184,
								.velocity_gain_Vi = 41,
								.velocity_gain_Vi_drain = 0,
								.velocity_max_rpm = 5000,
								.velocity_accel_max_rps2 = 833,
								.velocity_tracking_window_rpm = 600,
								.velocity_tracking_time_ms = 100,
								.velocity_loop_shift = 0,
								.position_gain_Pp = 1000,
								.position_gain_Vff = 16384,
								.position_gain_Aff = 0,
								.position_following_error_cts = 4000,
								.position_following_warning_cts = 2000,
								.position_gains_multiplier_percent = 100,
								.trajectory_velocity_max_rpm = 5000,
								.trajectory_accel_max_rps2 = 833,
								.trajectory_jerk_max_rps3 = 6000,
								.home_fast_velocity_max_rpm = 20,
								.home_slow_velocity_max_rpm = 10,
								.home_accel_max_rps2 =  20} ),
			std::make_tuple<CopleyMotor::Identifier, CopleyMotorEncoder::Identifier, CopleyLoadEncoder::Identifier, CopleyAlgorithm>(
				CopleyMotor::Identifier::AnaheimAutomation_BLWRPG173S_24V_4000,
				CopleyMotorEncoder::Identifier::Hall_Feedback,
				CopleyLoadEncoder::Identifier::No_Load_Encoder,
				CopleyAlgorithm{.current_gain_Cp = 300,
								.current_gain_Ci = 70,
								.current_peak_max_A = 12.0,
								.current_continuous_max_A = 4.1,
								.current_I2t_ms = 1000,
								.current_offset_A = 0,
								.velocity_gain_Vp = 8000,
								.velocity_gain_Vi = 9500,
								.velocity_gain_Vi_drain = 0,
								.velocity_max_rpm = 4000,
								.velocity_accel_max_rps2 = 500,
								.velocity_tracking_window_rpm = 500,
								.velocity_tracking_time_ms = 100,
								.velocity_loop_shift = 0,
								.position_gain_Pp = 1000,
								.position_gain_Vff = 16384,
								.position_gain_Aff = 0,
								.position_following_error_cts = 4000,
								.position_following_warning_cts = 2000,
								.position_gains_multiplier_percent = 100,
								.trajectory_velocity_max_rpm = 3200,
								.trajectory_accel_max_rps2 = 350,
								.trajectory_jerk_max_rps3 = 5000,
								.home_fast_velocity_max_rpm = 20,
								.home_slow_velocity_max_rpm = 10,
								.home_accel_max_rps2 =  20} ),
			std::make_tuple<CopleyMotor::Identifier, CopleyMotorEncoder::Identifier, CopleyLoadEncoder::Identifier, CopleyAlgorithm>(
				CopleyMotor::Identifier::Teknic_CPM_MCVC_2321P_RLN,
				CopleyMotorEncoder::Identifier::Teknic_CPM_MCVC_xxxx_xxx_04,
				CopleyLoadEncoder::Identifier::No_Load_Encoder,
				CopleyAlgorithm{.current_gain_Cp = 225,
								.current_gain_Ci = 45,
								.current_peak_max_A = 15.0,
								.current_continuous_max_A = 8.5,
								.current_I2t_ms = 1000,
								.current_offset_A = 0,
								.velocity_gain_Vp = 367,
								.velocity_gain_Vi = 82,
								.velocity_gain_Vi_drain = 0,
								.velocity_max_rpm = 6000,
								.velocity_accel_max_rps2 = 1000,
								.velocity_tracking_window_rpm = 600,
								.velocity_tracking_time_ms = 100,
								.velocity_loop_shift = 0,
								.position_gain_Pp = 1000,
								.position_gain_Vff = 16384,
								.position_gain_Aff = 0,
								.position_following_error_cts = 4000,
								.position_following_warning_cts = 2000,
								.position_gains_multiplier_percent = 100,
								.trajectory_velocity_max_rpm = 4800,
								.trajectory_accel_max_rps2 = 800,
								.trajectory_jerk_max_rps3 = 20000,
								.home_fast_velocity_max_rpm = 150,
								.home_slow_velocity_max_rpm = 30,
								.home_accel_max_rps2 = 50 } ),
			std::make_tuple<CopleyMotor::Identifier, CopleyMotorEncoder::Identifier, CopleyLoadEncoder::Identifier, CopleyAlgorithm>(
				CopleyMotor::Identifier::Teknic_CPM_MCVC_2321P_RLN,
				CopleyMotorEncoder::Identifier::Teknic_CPM_MCVC_xxxx_xxx_04,
				CopleyLoadEncoder::Identifier::RLS_BR10,
				CopleyAlgorithm{.current_gain_Cp = 225,
								.current_gain_Ci = 45,
								.current_peak_max_A = 15.0,
								.current_continuous_max_A = 8.5,
								.current_I2t_ms = 1000,
								.current_offset_A = 0,
								.velocity_gain_Vp = 367,
								.velocity_gain_Vi = 82,
								.velocity_gain_Vi_drain = 0,
								.velocity_max_rpm = 6000,
								.velocity_accel_max_rps2 = 1000,
								.velocity_tracking_window_rpm = 600,
								.velocity_tracking_time_ms = 100,
								.velocity_loop_shift = 0,
								.position_gain_Pp = 1000,
								.position_gain_Vff = 16384,
								.position_gain_Aff = 0,
								.position_following_error_cts = 4000,
								.position_following_warning_cts = 2000,
								.position_gains_multiplier_percent = 100,
								.trajectory_velocity_max_rpm = 4800,
								.trajectory_accel_max_rps2 = 800,
								.trajectory_jerk_max_rps3 = 20000,
								.home_fast_velocity_max_rpm = 150,
								.home_slow_velocity_max_rpm = 30,
								.home_accel_max_rps2 = 50 } ),
			std::make_tuple<CopleyMotor::Identifier, CopleyMotorEncoder::Identifier, CopleyLoadEncoder::Identifier, CopleyAlgorithm>(
				CopleyMotor::Identifier::Teknic_CPM_MCVC_2321P_RLN,
				CopleyMotorEncoder::Identifier::Teknic_CPM_MCVC_xxxx_xxx_04,
				CopleyLoadEncoder::Identifier::RLS_RM22,
				CopleyAlgorithm{.current_gain_Cp = 225,
								.current_gain_Ci = 45,
								.current_peak_max_A = 15.0,
								.current_continuous_max_A = 8.5,
								.current_I2t_ms = 1000,
								.current_offset_A = 0,
								.velocity_gain_Vp = 367,
								.velocity_gain_Vi = 82,
								.velocity_gain_Vi_drain = 0,
								.velocity_max_rpm = 6000,
								.velocity_accel_max_rps2 = 1000,
								.velocity_tracking_window_rpm = 600,
								.velocity_tracking_time_ms = 100,
								.velocity_loop_shift = 0,
								.position_gain_Pp = 1000,
								.position_gain_Vff = 16384,
								.position_gain_Aff = 0,
								.position_following_error_cts = 4000,
								.position_following_warning_cts = 2000,
								.position_gains_multiplier_percent = 100,
								.trajectory_velocity_max_rpm = 4800,
								.trajectory_accel_max_rps2 = 800,
								.trajectory_jerk_max_rps3 = 20000,
								.home_fast_velocity_max_rpm = 150,
								.home_slow_velocity_max_rpm = 30,
								.home_accel_max_rps2 = 50 } ),
			std::make_tuple<CopleyMotor::Identifier, CopleyMotorEncoder::Identifier, CopleyLoadEncoder::Identifier, CopleyAlgorithm>(
				CopleyMotor::Identifier::Teknic_CPM_MCVC_3432V_RLN,
				CopleyMotorEncoder::Identifier::Teknic_CPM_MCVC_xxxx_xxx_02,
				CopleyLoadEncoder::Identifier::No_Load_Encoder,
				CopleyAlgorithm{.current_gain_Cp = 207,
								.current_gain_Ci = 35,
								.current_peak_max_A = 30,
								.current_continuous_max_A = 15,
								.current_I2t_ms = 1000,
								.current_offset_A = 0,
								.velocity_gain_Vp = 2500,
								.velocity_gain_Vi = 300,
								.velocity_gain_Vi_drain = 0,
								.velocity_max_rpm = 5000,
								.velocity_accel_max_rps2 = 1400,
								.velocity_tracking_window_rpm = 600,
								.velocity_tracking_time_ms = 100,
								.velocity_loop_shift = 0,
								.position_gain_Pp = 200,
								.position_gain_Vff = 16384,
								.position_gain_Aff = 0,
								.position_following_error_cts = 2000,
								.position_following_warning_cts = 1000,
								.position_gains_multiplier_percent = 100,
								.trajectory_velocity_max_rpm = 3000,
								.trajectory_accel_max_rps2 = 900,
								.trajectory_jerk_max_rps3 = 10000,
								.home_fast_velocity_max_rpm = 150,
								.home_slow_velocity_max_rpm = 30,
								.home_accel_max_rps2 = 50 } ),
			std::make_tuple<CopleyMotor::Identifier, CopleyMotorEncoder::Identifier, CopleyLoadEncoder::Identifier, CopleyAlgorithm>(
				CopleyMotor::Identifier::Minebea_BLDC36P16A_24V,
				CopleyMotorEncoder::Identifier::Hall_Feedback,
				CopleyLoadEncoder::Identifier::Avago_HEDS_9700_E50,
				CopleyAlgorithm{.current_gain_Cp = 160,
								.current_gain_Ci = 50,
								.current_peak_max_A = 3.5,
								.current_continuous_max_A = 1.8,
								.current_I2t_ms = 1000,
								.current_offset_A = 0,
								.velocity_gain_Vp = 8000,
								.velocity_gain_Vi = 1500,
								.velocity_gain_Vi_drain = 0,
								.velocity_max_rpm = 4000,
								.velocity_accel_max_rps2 = 200,
								.velocity_tracking_window_rpm = 50,
								.velocity_tracking_time_ms = 100,
								.velocity_loop_shift = 0,
								.position_gain_Pp = 1000,
								.position_gain_Vff = 0,
								.position_gain_Aff = 0,
								.position_following_error_cts = 5500,
								.position_following_warning_cts = 2750,
								.position_gains_multiplier_percent = 100,
								.trajectory_velocity_max_rpm = 4000,
								.trajectory_accel_max_rps2 = 200,
								.trajectory_jerk_max_rps3 = 5000,
								.home_fast_velocity_max_rpm = 500,
								.home_slow_velocity_max_rpm = 250,
								.home_accel_max_rps2 = 50 } ),
			std::make_tuple<CopleyMotor::Identifier, CopleyMotorEncoder::Identifier, CopleyLoadEncoder::Identifier, CopleyAlgorithm>(
				CopleyMotor::Identifier::Maxon_449464,
				CopleyMotorEncoder::Identifier::Hall_Feedback,
				CopleyLoadEncoder::Identifier::Avago_HEDS_9700_E50,
				CopleyAlgorithm{.current_gain_Cp = 60,
								.current_gain_Ci = 35,
								.current_peak_max_A = 10.0,
								.current_continuous_max_A = 3.0,
								.current_I2t_ms = 1000,
								.current_offset_A = 0,
								.velocity_gain_Vp = 7200,
								.velocity_gain_Vi = 5500,
								.velocity_gain_Vi_drain = 0,
								.velocity_max_rpm = 13000,
								.velocity_accel_max_rps2 = 550,
								.velocity_tracking_window_rpm = 550,
								.velocity_tracking_time_ms = 100,
								.velocity_loop_shift = 0,
								.position_gain_Pp = 1400,
								.position_gain_Vff = 16384*0.5,
								.position_gain_Aff = 0,
								.position_following_error_cts = 5500,
								.position_following_warning_cts = 2750,
								.position_gains_multiplier_percent = 100,
								.trajectory_velocity_max_rpm = 10000,
								.trajectory_accel_max_rps2 = 190,
								.trajectory_jerk_max_rps3 = 350,
								.home_fast_velocity_max_rpm = 170,
								.home_slow_velocity_max_rpm = 150,
								.home_accel_max_rps2 = 6.0 } )
	}};

	for( auto && algorithm : algorithms ){
		if( std::get<0>(algorithm) == motor
				&& std::get<1>(algorithm) == motor_encoder
				&& std::get<2>(algorithm) == load_encoder ){
			return &std::get<3>(algorithm);
		}
	}
	return nullptr;
}

const CopleyOutputFilter CopleyOutputFilter::invalid = {
		.filter_params = {0, 0, 0, 0, 0, 0, 0, 0, 0,}
};

const CopleyOutputFilter * CopleyOutputFilter::lookup( const CopleyAmplifier::Model model )
{
	// See "Parameter Dictionary" document from Copley Contols for details.
	// For the plus family of drives, a 14-word format
	// is used to describe the biquad filter coefficients.
	// For the first word.
	// Bits			Function
	// 0-3			Filter Family
	// 4			If set, filter not designed by amp
	// 5-7			Reserved
	// 8			Number of poles (0 for single, 1 for two-pole)
	// 9-12			Reserved
	// 13-15 		Filter Type

	// To disable, set filter type to value 7. All other words in the
	// register have no meaning when disabled and can be set to 0.
	std::vector<int16_t> disabled_plus_filter;
	int16_t word1 = 7 << kFilterTypeOffset;
	disabled_plus_filter.push_back(word1);
	for (size_t i = 0; i < 13; ++i)
		disabled_plus_filter.push_back(0);

	static const std::map<CopleyAmplifier::Model, CopleyOutputFilter> filters = {
		{ CopleyAmplifier::Model::AccelNet_ACK_055_10,
			CopleyOutputFilter{	std::vector<int16_t>{
					8448,   // word1
					200,    // word2
					0,      // word3
					775,    // b2
					1550,   // b1
					775,    // b0
					-12774, // a2
					32763,  // a1
					5813 }} // K
		},
		{ CopleyAmplifier::Model::AccelNet_ACK_090_20,
			CopleyOutputFilter{std::vector<int16_t>{
					8448,   // word1
					200,    // word2
					0,      // word3
					775,    // b2
					1550,   // b1
					775,    // b0
					-12774, // a2
					32763,  // a1
					5813 }} // K
		},
		{ CopleyAmplifier::Model::AccelNet_BPL_090_30,
			CopleyOutputFilter{disabled_plus_filter}
		},
		{ CopleyAmplifier::Model::AccelNet_BP2_090_20,
			CopleyOutputFilter{disabled_plus_filter}
		},
	};

	if( filters.find(model) != filters.end() ) { return &filters.at(model); }

	return nullptr;
}

const CopleyHardwareSpecification CopleyHardwareSpecification::invalid = {
		.amplifier = CopleyAmplifier::invalid,
		.motor = CopleyMotor::invalid,
		.motor_encoder = CopleyMotorEncoder::invalid,
		.load_encoder = CopleyLoadEncoder::invalid,
		.load_encoder_direction = CopleyLoadEncoder::Direction::kNormal,
		.algorithm_default = CopleyAlgorithm::invalid,
		.filter_default = CopleyOutputFilter::invalid,
		.gear_ratio = 1.0
};

CopleyHardwareSpecification::CopleyHardwareSpecification(
		const CopleyAmplifier &amplifier,
		const CopleyMotor &motor,
		const CopleyMotorEncoder &motor_encoder,
		const CopleyLoadEncoder &load_encoder,
		const CopleyLoadEncoder::Direction &load_encoder_direction,
		const CopleyAlgorithm &algorithm_default,
		const CopleyOutputFilter &filter_default,
		const double gear_ratio)
	: amplifier( amplifier ),
	  motor( motor ),
	  motor_encoder( motor_encoder ),
	  load_encoder( load_encoder ),
	  load_encoder_direction( load_encoder_direction ),
	  algorithm_default( algorithm_default ),
	  filter_default(filter_default),
	  gear_ratio( gear_ratio )
{
}

CopleyHardwareSpecification::OutputPin operator|(CopleyHardwareSpecification::OutputPin lhs,
												 CopleyHardwareSpecification::OutputPinModifier rhs)
{
	return static_cast<CopleyHardwareSpecification::OutputPin>(
		static_cast<std::underlying_type<CopleyHardwareSpecification::OutputPin>::type>(lhs) |
		static_cast<std::underlying_type<CopleyHardwareSpecification::OutputPinModifier>::type>(rhs));
}

} // namespace momentum
