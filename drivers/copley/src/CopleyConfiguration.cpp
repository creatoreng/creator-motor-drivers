/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/copley/CopleyConfiguration.hpp>

namespace momentum
{

CopleyConfiguration::CopleyConfiguration(const CopleyAmplifier::Model model,
										 const CopleyMotor::Identifier motor,
										 const CopleyMotorEncoder::Identifier motor_encoder,
										 const double gearing,
										 const CopleyLoadEncoder::Identifier load_encoder,
										 const CopleyLoadEncoder::Direction load_encoder_direction)
  : _gearing(gearing),
	_amplifier(CopleyAmplifier::lookup(model)),
	_motor(CopleyMotor::lookup(motor)),
	_motor_encoder(CopleyMotorEncoder::lookup(motor_encoder)),
	_load_encoder(CopleyLoadEncoder::lookup(load_encoder)),
	_load_encoder_direction(load_encoder_direction),
	_algorithm(CopleyAlgorithm::lookup(motor, motor_encoder, load_encoder)),
	_filter(CopleyOutputFilter::lookup(model))
{
	if (!_amplifier) {
		throw std::logic_error("Unsupported amplifier model.");
	} else if (!_motor) {
		throw std::logic_error("Unsupported motor model.");
	} else if (!_motor_encoder) {
		throw std::logic_error("Unsupported motor encoder model.");
	} else if (!_load_encoder) {
		throw std::logic_error("Unsupported load encoder model.");
	} else if (!_algorithm) {
		throw std::logic_error("Unsupported motor, motor encoder, and load encoder combination.");
	} else if (!_filter) {
		throw std::logic_error("Velocity output filter + amplifier combination.");
	}
}

CopleyConfiguration::operator CopleyHardwareSpecification(void) const
{
	return CopleyHardwareSpecification(*_amplifier, *_motor, *_motor_encoder, *_load_encoder, _load_encoder_direction,
									   *_algorithm, *_filter, _gearing);
}

CopleyHardwareSpecification CopleyConfiguration::get_hardware_specification(void) const
{
	return CopleyHardwareSpecification(*_amplifier, *_motor, *_motor_encoder, *_load_encoder, _load_encoder_direction,
									   *_algorithm, *_filter, _gearing);
}

} // namespace momentum
