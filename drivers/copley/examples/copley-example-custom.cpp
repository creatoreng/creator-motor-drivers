/**
* @author Creator, Inc.
* @copyright Copyright (c) 2022, Creator, Inc. MIT license.
**/

#include <drivers/copley/Copley.hpp>
#include <drivers/copley/CopleyConfiguration.hpp>
#include <drivers/copley/CopleyConversion.hpp>

#include <iostream>

using namespace momentum;
using namespace momentum::CopleyConversion;

/// Return a custom hardware specification defined by this method.
CopleyHardwareSpecification hardware_specifiction(void);

int main(int argc, char *argv[])
{
  const uint8_t node_id = 0x00;  // CAN bus ID; 0 for UART bus master
  const uint8_t motor_axis = 0;  // motor axis ID (used for dual-axis motors)
  const std::string bus_address = "/dev/tty0";

  std::shared_ptr<CopleyBus> bus(new CopleyBus());
  bus->connect(bus_address);

  if(bus->is_connected()){
    Copley motor(bus,
      	hardware_specifiction(),
      	node_id,
      	motor_axis);

    motor.connect();
    if(motor.is_connected()){
      std::cout << "node " << node_id
        << " serial number: " << motor.read_serial_number()
        << std::endl;
    }
		std::cout << "motor status: " << motor.status().to_string() << std::endl;
  }
	std::cout << "bus status: " << bus->status().to_string() << std::endl;

	return 0;
}

CopleyHardwareSpecification hardware_specifiction(void)
{
  // Example of how to build a custom configuration.
  // This is example duplicates the pre-configured
  // algorithm for the following combination:
  // amplifier: AccelNet BPL_090_30
  // motor:     Teknic CPM-MCVC-2321P-RLN
  // primary:   Teknic CPM-MCVC series, 4000 CPR
  // load:      none

  CopleyAmplifier amplifier_config {
        .model = CopleyAmplifier::Model::AccelNet_BPL_090_30,
        .identifier = CopleyAmplifier::Identifier::AccelNet_Plus_CAN,
        .manufacturer = "Copley Controls",
        .model_name = "Accelnet Plus CANopen",
        .current_peak_limit_A = 30.0,
        .current_continuous_limit_A = 15.0,
        .supports_Pi = false,
        .supports_Pd = false,
        .num_inputs = 11,
        .num_outputs = 4,
        .inputs_with_pullup_option = {0, 1, 2, 3, 4, 5} };

  CopleyMotor motor_config {
        .identifier = CopleyMotor::Identifier::Custom,
        .manufacturer = "Teknic",
        .model_name = "CPM-MCVC-2321P-RLN",
        .type = CopleyMotor::Type::kRotary,
        .architecture = CopleyMotor::Architecture::kBrushlessServo,
        .voltage_nominal_V = 48,
        .torque_constant_Nm_A = 14.189 * scale_ozin_to_Nm,
        .resistance_ohm = 0.616,
        .inductance_mH = 0.915,
        .torque_peak_Nm = 133.716 * scale_ozin_to_Nm,
        .torque_continuous_Nm = 116.2 * scale_ozin_to_Nm,
        .current_peak_A = 33,
        .current_continuous_A = 8.5,
        .back_emf_V_Krpm = 11.65,
        .velocity_max_rpm = 6000,
        .inertia_Kg_cm2 = 0.002 * scale_ozin_s2_to_kg_cm2,
        .pole_pairs = 4,
        .wiring = CopleyMotor::Wiring::kStandard,
        .hall_offset_deg = 0.0,
        .hall_sensor = CopleyMotor::HallType::kDigital,
        .hall_wiring = CopleyMotor::HallWiring::WUV,
        .hall_inversion = CopleyMotor::HallInversion::kNone,
        .commutation = CopleyMotor::CommutationMode::kStandard };

  CopleyMotorEncoder encoder_config {
        .identifier = CopleyMotorEncoder::Identifier::Custom,
        .manufacturer = "Teknic",
        .model_name = "CPM-MCVC-xxxx-xxx-04",
        .type = CopleyMotorEncoder::Type::kPrimary,
        .direction = CopleyMotorEncoder::Direction::kReverse,
        .resolution_cpr = 4000 };

  CopleyLoadEncoder load_encoder_config {
        .identifier = CopleyLoadEncoder::Identifier::No_Load_Encoder,
        .manufacturer = "N/A",
        .model_name = "N/A",
        .type = CopleyLoadEncoder::Type::kNone,
        .resolution_cpr = 0,
        .passive_mode = false,
        .options = 0 };

  CopleyAlgorithm control_algorithm {
        .current_gain_Cp = 225,
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
        .home_accel_max_rps2 = 50 };

  // Output filter. To disable, set filter type to value 7. All other words in the
	// register have no meaning when disabled and can be set to 0.
  CopleyOutputFilter output_filter { std::vector<int16_t>{
        static_cast<int16_t>(7 << CopleyOutputFilter::kFilterTypeOffset),   // word1
        0,      // word2
        0,      // word3
        0,      // b2
        0,      // b1
        0,      // b0
        0,      // a2
        0,      // a1
        0  }};  // K

  // alternately you can look up the default output filter by amplifier model:
  // output_filter = *CopleyOutputFilter::lookup(CopleyAmplifier::Model::AccelNet_BPL_090_30));

  CopleyHardwareSpecification hardware_config {
        amplifier_config,
        motor_config,
        encoder_config,
        load_encoder_config,
        CopleyLoadEncoder::Direction::kNormal,
        control_algorithm,
        output_filter,
        1.0 }; // gear ratio

  return hardware_config;
}
