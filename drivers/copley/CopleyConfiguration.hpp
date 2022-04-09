/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Configuration for Copley Amplifiers.
 **/

#pragma once

#include <drivers/copley/CopleyModel.hpp>

namespace momentum
{

/// Motor configuration.
class CopleyConfiguration
{
public:
	/// Construct a CopleyConfiguration.
	/// @throws std::logic_error if configuration is unsupported.
	/// @param [in] amplifier The amplifier identifier.
	/// @param [in] motor The motor model identifier.
	/// @param [in] motor_encoder The motor encoder identifier.
	/// @param [in] gearing Gear ratio.
	/// @param [in] load_encoder The load encoder identifier.
	/// @param [in] load_encoder_direction The load encoder direction.
	CopleyConfiguration(
		const CopleyAmplifier::Model model,
		const CopleyMotor::Identifier motor,
		const CopleyMotorEncoder::Identifier motor_encoder,
		const double gearing,
		const CopleyLoadEncoder::Identifier load_encoder		  = CopleyLoadEncoder::Identifier::No_Load_Encoder,
		const CopleyLoadEncoder::Direction load_encoder_direction = CopleyLoadEncoder::Direction::kNormal);

	/// Convert this configuration to a Copley hardware specification.
	/// If the specification is unsupported, an error is thrown.
	/// @returns hardware specification.
	operator CopleyHardwareSpecification(void) const;

	CopleyHardwareSpecification get_hardware_specification(void) const;

private:
	const double _gearing;										///< Gear ratio.
	const CopleyAmplifier *const _amplifier;					///< Amplifier.
	const CopleyMotor *const _motor;							///< Motor.
	const CopleyMotorEncoder *const _motor_encoder;				///< Motor encoder.
	const CopleyLoadEncoder *const _load_encoder;				///< Load encoder.
	const CopleyLoadEncoder::Direction _load_encoder_direction; ///< Load encoder direction.
	const CopleyAlgorithm *const _algorithm;					///< Algorithm.
	const CopleyOutputFilter *const _filter;					///< Filter.
};

} // namespace momentum
