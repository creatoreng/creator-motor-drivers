/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Conversion terms for Copley amplifiers.
 **/

#include <cmath>

namespace momentum
{

/// Conversion factors between registers and units.
namespace CopleyConversion
{
// unit conversion
/// deg to rad
static constexpr double scale_deg_to_rad = M_PI / 180.0;
/// RPM to deg/s.
static constexpr double scale_rpm_to_deg_s = 360.0 / 60.0;
/// RPS to deg/s
static constexpr double scale_rps_to_deg_s = 360.0;
/// Rotary inertia from oz-in/s^2 to kg/cm^2
static constexpr double scale_ozin_s2_to_kg_cm2 = 70.6154;
/// Torque constant from oz-in/A to Nm/A (equivalently oz-in to Nm)
static constexpr double scale_ozin_to_Nm = 0.00706155;

// motion conversion

/// Scale factor to convert raw position to encoder counts.
/// @see Copley Parameter Dictionary pg. 16 0x32
static constexpr double scale_raw_to_count = 1.0;
/// Scale factor to convert encoder counts to raw position.
static constexpr double scale_count_to_raw = 1.0 / scale_raw_to_count;
/// Scale factor to convert raw velocity to encoder counts per second (CPS)
/// @see Copley Parameter Dictionary pg. 13 0x18
static constexpr double scale_raw_to_cps = 0.1;
/// Scale factor to convert encoder counts per second (CPS) to raw velocity.
static constexpr double scale_cps_to_raw = 1.0 / scale_raw_to_cps;
/// Scale factor to convert raw acceleration to encoder counds per second^2 (CPS^2)
/// @see Copley Parameter Dictionary pg. 47 0xCC
static constexpr double scale_raw_to_cps2 = 10.0;
/// Scale factor to convert encoder acceleration (CPS^2) to raw acceleration.
static constexpr double scale_cps2_to_raw = 1.0 / scale_raw_to_cps2;
/// Scale factor to convert raw jerk to encoder counds per second^3 (CPS^3)
/// @see Copley Parameter Dictionary pg. 47 0xCE
static constexpr double scale_raw_to_cps3 = 100.0;
/// Scale factor to convert encoder jerk (CPS^3) to raw jerk.
static constexpr double scale_cps3_to_raw = 1.0 / scale_raw_to_cps3;
/// Scale factor to convert raw voltage to V.
/// @see Copley Parameter Dictionary pg. 13 0x1E
static constexpr double scale_raw_to_V = 0.1;
/// Scale factor to convert V into raw voltage.
static constexpr double scale_V_to_raw = 1.0 / scale_raw_to_V;
/// Scale factor to convert raw A to A.
/// @see Copley Parameter Dictionary pg. 14 0x21
static constexpr double scale_raw_to_A = 0.01;
/// Scale factor to convert A to raw A.
static constexpr double scale_A_to_raw = 1.0 / scale_raw_to_A;
/// Number of hall sensor switches per revolution
static constexpr double hall_sensor_cpr = 6.0;

// control parameters

/// Scale factor to convert a raw acceleration limit from counts to raw
static constexpr double scale_accel_limit_cps2_to_raw = 1.0 / 1000.0;
/// Scale factor to convert Vff in percent to raw Vff.
static constexpr double scale_vff_percent_to_raw = 16384.0 / 100.0;

// motor specification conversion

/// Scale factor to convert rotary motor inertia from Kg / cm^2 to raw.
static constexpr double scale_kg_cm2_to_raw = 1.0 / 0.000001;
/// Scale factor to convert torque constant from Nm / A to raw.
static constexpr double scale_Nm_A_to_raw = 1.0 / 0.00001;
/// Scale factor to convert armature resistance from Ohm to raw.
static constexpr double scale_Ohm_to_raw = 1000.0 / 10.0;
/// Scale factor to convert armature inductance from uH to raw.
static constexpr double scale_mH_to_raw = 1000.0 / 10.0;
/// Scale factor to convert torque from Nm to raw.
static constexpr double scale_Nm_to_raw = 1.0 / 0.00001;
/// Scale factor to convert back EMF (V/Krpm or V/mps) to raw.
static constexpr double scale_V_krpm_to_raw = 1.0 / 0.01;
} // namespace CopleyConversion

} // namespace momentum
