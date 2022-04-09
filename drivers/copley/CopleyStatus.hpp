/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Copley error enumerations and strings.
 **/

#pragma once

#include <libs/status/Status.hpp>

namespace momentum {

/// Copley status context, which enumerates all errors for this context.
class CopleyContext {
public:
	/// Copley error codes
	enum class Code : status_code_t {
		kOk = 0, ///< OK.
		kConfigFail = 1, ///< Driver configuration failed.
		kInvalidInputData = 100, ///< Requested value is not valid.
		kInvalidComProtocol = 101, ///< ASCII communication protocol not supported.
		kComOpenFail = 1000, ///< Unable to open the communicaton port.
		kComAlreadyOpen = 1001, ///< Attempted to open a port that had already been opened.
		kComNotOpen = 1002, ///< Attempted to access a port that had not yet been opened.
		kComNotConnected = 1003, ///< Communication is not established.
		kComNotConfigured = 1004, ///< Attempted to connect before motor parameters configured.
		kComWriteFail = 1005, ///< Unable to write to the communication port.
		kComReadFail = 1006, ///< Unable to read from the communication port.
		kComTimeout = 1007, ///< Communication timed out.
		kComGarbage = 1008, ///< Unexpected data.
		kComChecksumFail = 1009, ///< Checksum failed.
		kComUnderrun = 1010, ///< Less data received than expected.
		kComIncorrectDevice = 1011, ///< Incorrect device specified.
		kComAckMismatch = 1012, ///< An acknowledgement was received but did not match the request.
		kComDataFlush = 1013, ///< Data was discarded from a communication port.
		kComNotAcknowledged = 1014, ///< An acknowledgement was expected but not received.
		kComBaudUnknown = 1015, ///< Baud rate unknown.
		kComNodeIdInUse = 1016, ///< Network node ID is already in use.
		kPacketInvalidMotorId = 1500, ///< Invalid motor ID specified.
		kPacketPayloadUndersize = 1502, ///< Payload size is too small.
		kPacketPayloadOutOfRange = 1503, ///< Value of the payload is out-of-range.
		kPacketUnknownCommand = 1504, ///< An unknown command was transmitted.
		kPacketUnknownResponse = 1505, ///< An unknown response was received.
		kPacketExceededRegisterRange = 1506, ///< A register operation specified an out-of-range register.
		kPacketRegisterBankInvalid = 1507, ///< A register was read or written from an incorrect memory bank.
		kPacketRegisterNotWritable = 1508, ///< Attempted to write a register that is read-only.
		kPacketParameterNonNumeric = 1509, ///< A parameter value was non-numeric.
		kPacketInvalidChecksum = 1510, ///< Response failed checksum calculation.
		kPacketTooMuchData = 1512, ///< Response already contains number of bytes declared in header.
		kAmplifierTooMuchData = 2001, ///< Too much data passed with command.
		kAmplifierChecksumError = 2002, ///< Checksum error on receieved command.
		kAmplifierUnknownCommand = 2003, ///< Unknown command.
		kAmplifierNotEnoughData = 2004, ///< Not enough data was supplied with the command.
		kAmplifierTooMuchDataSupplied = 2005, ///< Too much data was supplied with the command.
		kAmplifierFlashEraseError = 2006, ///< Error erasing flash memory.
		kAmplifierFlashWritingError = 2007, ///< Error writing to flash memory.
		kAmplifierIllegalMemoryPage = 2008, ///< Illegal memory page specified with parameter.
		kAmplifierUnknownParameterId = 2009, ///< Unknown parameter ID.
		kAmplifierDataValueOutOfRange = 2010, ///< Data value out-of-range.
		kAmplifierAttemptToModifyReadOnly = 2011, ///< Attempt to modify a read-only parameter.
		kAmplifierInvalidTraceChannel = 2012, ///< Invalid trace channel specified in command.
		kAmplifierInvalidTraceVariable = 2013, ///< Invalid trace variable number specified.
		kAmplifierInvalidOperationMode = 2014, ///< Invalid mode of operation specified.
		kAmplifierParameterDoesNotExist = 2015, ///< Parameter does not exist on requested page.
		kAmplifierIllegalSerialPortForwarding = 2016, ///< Illegal serial port forwarding.
		kAmplifierFlashPageCrcError = 2017, ///< Data flash page CRC error.
		kAmplifierIllegalMoveWhileMoving = 2018, ///< Illegal attempt to start a move while currently moving.
		kAmplifierIllegalVelocityLimit = 2019, ///< Illegal velocity limit for move.
		kAmplifierIllegalAccelerationLimit = 2020, ///< Illegal acceleration limit for move.
		kAmplifierIllegalDecelerationLimit = 2021, ///< Illegal decelleration limit for move.
		kAmplifierIllegalJerkLimit = 2022, ///< Illegal jerk limit for move.
		kAmplifierTrajectoryBufferUnderflow = 2023, ///< Trajectory buffer underflowed during move.
		kAmplifierTrajectoryBufferOverflow = 2024, ///< Trajectory buffer overflowed when adding data.
		kAmplifierInvalidTrajectoryMode = 2025, ///< Invalid trajectory mode.
		kAmplifierCvmLocationNotAvailable = 2026, ///< That CVM program location is not available.
		kAmplifierCommandDuringCvm = 2027, ///< Command is not allowed while CVM is running.
		kAmplifierCvmProgramTooBig = 2028, ///< THe CVM program is too big to upload.
		kAmplifierFileSystemError = 2029, ///< File system internal error.
		kAmplifierProgramDoesntExist = 2030, ///< Specified program doesn't exist.
		kAmplifierInvalidNodeForForwarding = 2031, ///< Invalid node ID for serial port forwarding.
		kAmplifierCanCommunicationFailure = 2032, ///< CAN network communications failure.
		kAmplifierAsciiParsingError = 2033, ///< ASCII command parsing error.
		kAmplifierInternalError = 2034, ///< Internal Error.
		kAmplifierFileSystemChangeWhileCamming = 2035, ///< File system can't be changed while in camming mode.
		kAmplifierBadAxisLetter = 2036, ///< Bad axis letter specified.
		kAmplifierInvalidFpgaData = 2037, ///< Invalid FPGA data stored on amp.
		kAmplifierFpgaInitFailed = 2038, ///< Unable to initialize FPGA.
		kAmplifierFpgaConfigureFailed = 2039, ///< FPGA failed to configure.
		kAmplifierFileAlreadyExists = 2040, ///< File already exists.
		kAmplifierNoFreeFileEntries = 2041, ///< No free file entries in directory.
		kAmplifierFileDoesntExist = 2042, ///< File doesn't exist.
		kAmplifierNoFreeSpace = 2043, ///< No free space in file system.
		kAmplifierInvalidFileFormat = 2044, ///< Invalid file format.
		kAmplifierEndOfFile = 2045, ///< End of file hit while reading.
		kAmplifierSendEncoderCommandError = 2046, ///< Error sending command to encoder.
		kAmplifierIllegalOperationInSerial = 2047, ///< Operation is illegal in current serial port mode.
		kAmplifierUnableToCalculateFilter = 2048, ///< Unable to calculate filter.
		kAmplifierCvmCommandFailed = 2049, ///< Failed to perform protected CMV command because Indexer register 31 wasn't set.
		kAmplifierUndocumentedTimeout = 2050, ///< Amplier returned undocumented timeout error.
		kMotorShortCircuit = 3000, ///< Short circuit detected.
		kMotorDriveOverTemp = 3001, ///< Drive over temperature.
		kMotorOverVoltage = 3002, ///< Over voltage.
		kMotorUnderVoltage = 3003, ///< Under voltage.
		kMotorTempActive = 3004, ///< Motor temperature sensor active.
		kMotorEncoderError = 3005, ///< Encoder feedback error.
		kMotorPhaseError = 3006, ///< Motor phasing error.
		kMotorCurrentLimited = 3007, ///< Current output limited.
		kMotorVoltageLimited = 3008, ///< Voltage output limited.
		kMotorLimitPosActive = 3009, ///< Positive limit switch active.
		kMotorLimitNegActive = 3010, ///< Negative limit switch active.
		kMotorHardwareDisable = 3011, ///< Hardware disable (enable input not active).
		kMotorSoftwareDisable = 3012, ///< Drive is disabled by software.
		kMotorStopAttempt = 3013, ///< Trying to stop motor.
		kMotorBrakeActive = 3014, ///< Motor brake activated.
		kMotorPwmOutputDisabled = 3015, ///< PWM outputs disabled.
		kMotorSoftwarePosLimit = 3016, ///< Positive software limit condition.
		kMotorSoftwareNegLimit = 3017, ///< Negative software limit condition.
		kMotorTrackingError = 3018, ///< Tracking error.
		kMotorTrackingWarning = 3019, ///< Tracking warning.
		kMotorReset = 3020, ///< Drive is currently in a reset condition.
		kMotorPositionWrapped = 3021, ///< Position has wrapped.
		kMotorDriveFault = 3022, ///< Drive latching fault.
		kMotorVelocityLimit = 3023, ///< Velocity limit reached.
		kMotorAccelerationLimit = 3024, ///< Acceleration limit reached.
		kMotorTrackingWindow = 3025, ///< Tracking window - position loop error is outside limits.
		kMotorHomeSwitchActive = 3026, ///< Home switch is active.
		kMotorMoving = 3027, ///< In motion.
		kMotorVelocityWindow = 3028, ///< Velocity window - velocity loop error is outside limits.
		kMotorPhaseNotInit = 3029, ///< Phase not yet initialized.
		kMotorCommandFault = 3030, ///< Command fault.
		kMotorUndefined = 3031, ///< Undefined.
		kMotorHomeFail = 4000, ///< Trajectory homing error.
		kHomingWindowFail = 4001, ///< Error during homing window search.
		kDriveFlashCrcFail = 5000, ///< Data flash CRC failure. This fault cannot be cleared.
		kDriveInternalError = 5001, ///< Drive internal error. This fault cannot be cleared.
		kDriveFeedbackFault = 5002, ///< Drive feedback fault.
		kDriveFpgaErrorType1 = 5003, ///< FPGA error type 1.
		kDriveCommandFault = 5004, ///< Drive command fault.
		kDriveFpgaErrorType2 = 5005, ///< FPGA error type 2.
		kDriveSafetyCircuitFault = 5006, ///< Safety circuit fault.
		kDriveCurrentControlFail = 5007, ///< Unable to control current.
		kDriveMotorWiringDisconnected = 5008, ///< Motor wiring disconnected.
		kMotorEncoderStatus = 6000, ///< Motor encoder fault detected.
		kLoadEncoderStatus = 6001, ///< Load encoder fault detected.
	};

	/// Unique name of this context.
	/// @return context name.
	static std::string context(void);

	/// Formatted message for a code.
	/// @param [in] code The status code to format.
	/// @return formatted string.
	static std::string code_text(const Code code);

private:
	/// Name of this object.
	static constexpr const char _context[] = "Copley";
};

// Copley status types.
using CopleyStatus = Status<CopleyContext>;
extern template class Status<CopleyContext>;

} // namespace momentum
