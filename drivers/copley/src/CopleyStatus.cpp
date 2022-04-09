/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/copley/CopleyStatus.hpp>

namespace momentum {

// explicit instantiation of CopleyContext
template class Status<CopleyContext>;
constexpr const char CopleyContext::_context[];

std::string CopleyContext::context(void)
{
	return _context;
}

std::string CopleyContext::code_text(const Code code)
{
	switch(code) {
		case Code::kOk: return "OK."; break;
		case Code::kConfigFail: return "Driver configuration failed."; break;
		case Code::kInvalidInputData: return "Requested value is not valid."; break;
		case Code::kInvalidComProtocol: return "ASCII communication protocol not supported."; break;
		case Code::kComOpenFail: return "Unable to open the communicaton port."; break;
		case Code::kComAlreadyOpen: return "Attempted to open a port that had already been opened."; break;
		case Code::kComNotOpen: return "Attempted to access a port that had not yet been opened."; break;
		case Code::kComNotConnected: return "Communication is not established."; break;
		case Code::kComNotConfigured: return "Attempted to connect before motor parameters configured."; break;
		case Code::kComWriteFail: return "Unable to write to the communication port."; break;
		case Code::kComReadFail: return "Unable to read from the communication port."; break;
		case Code::kComTimeout: return "Communication timed out."; break;
		case Code::kComGarbage: return "Unexpected data."; break;
		case Code::kComChecksumFail: return "Checksum failed."; break;
		case Code::kComUnderrun: return "Less data received than expected."; break;
		case Code::kComIncorrectDevice: return "Incorrect device specified."; break;
		case Code::kComAckMismatch: return "An acknowledgement was received but did not match the request."; break;
		case Code::kComDataFlush: return "Data was discarded from a communication port."; break;
		case Code::kComNotAcknowledged: return "An acknowledgement was expected but not received."; break;
		case Code::kComBaudUnknown: return "Baud rate unknown."; break;
		case Code::kComNodeIdInUse: return "Network node ID is already in use."; break;
		case Code::kPacketInvalidMotorId: return "Invalid motor ID specified."; break;
		case Code::kPacketPayloadUndersize: return "Payload size is too small."; break;
		case Code::kPacketPayloadOutOfRange: return "Value of the payload is out-of-range."; break;
		case Code::kPacketUnknownCommand: return "An unknown command was transmitted."; break;
		case Code::kPacketUnknownResponse: return "An unknown response was received."; break;
		case Code::kPacketExceededRegisterRange: return "A register operation specified an out-of-range register."; break;
		case Code::kPacketRegisterBankInvalid: return "A register was read or written from an incorrect memory bank."; break;
		case Code::kPacketRegisterNotWritable: return "Attempted to write a register that is read-only."; break;
		case Code::kPacketParameterNonNumeric: return "A parameter value was non-numeric."; break;
		case Code::kPacketInvalidChecksum: return "Response failed checksum calculation."; break;
		case Code::kPacketTooMuchData: return "Response already contains number of bytes declared in header."; break;
		case Code::kAmplifierTooMuchData: return "Too much data passed with command."; break;
		case Code::kAmplifierChecksumError: return "Checksum error on receieved command."; break;
		case Code::kAmplifierUnknownCommand: return "Unknown command."; break;
		case Code::kAmplifierNotEnoughData: return "Not enough data was supplied with the command."; break;
		case Code::kAmplifierTooMuchDataSupplied: return "Too much data was supplied with the command."; break;
		case Code::kAmplifierFlashEraseError: return "Error erasing flash memory."; break;
		case Code::kAmplifierFlashWritingError: return "Error writing to flash memory."; break;
		case Code::kAmplifierIllegalMemoryPage: return "Illegal memory page specified with parameter."; break;
		case Code::kAmplifierUnknownParameterId: return "Unknown parameter ID."; break;
		case Code::kAmplifierDataValueOutOfRange: return "Data value out-of-range."; break;
		case Code::kAmplifierAttemptToModifyReadOnly: return "Attempt to modify a read-only parameter."; break;
		case Code::kAmplifierInvalidTraceChannel: return "Invalid trace channel specified in command."; break;
		case Code::kAmplifierInvalidTraceVariable: return "Invalid trace variable number specified."; break;
		case Code::kAmplifierInvalidOperationMode: return "Invalid mode of operation specified."; break;
		case Code::kAmplifierParameterDoesNotExist: return "Parameter does not exist on requested page."; break;
		case Code::kAmplifierIllegalSerialPortForwarding: return "Illegal serial port forwarding."; break;
		case Code::kAmplifierFlashPageCrcError: return "Data flash page CRC error."; break;
		case Code::kAmplifierIllegalMoveWhileMoving: return "Illegal attempt to start a move while currently moving."; break;
		case Code::kAmplifierIllegalVelocityLimit: return "Illegal velocity limit for move."; break;
		case Code::kAmplifierIllegalAccelerationLimit: return "Illegal acceleration limit for move."; break;
		case Code::kAmplifierIllegalDecelerationLimit: return "Illegal decelleration limit for move."; break;
		case Code::kAmplifierIllegalJerkLimit: return "Illegal jerk limit for move."; break;
		case Code::kAmplifierTrajectoryBufferUnderflow: return "Trajectory buffer underflowed during move."; break;
		case Code::kAmplifierTrajectoryBufferOverflow: return "Trajectory buffer overflowed when adding data."; break;
		case Code::kAmplifierInvalidTrajectoryMode: return "Invalid trajectory mode."; break;
		case Code::kAmplifierCvmLocationNotAvailable: return "That CVM program location is not available."; break;
		case Code::kAmplifierCommandDuringCvm: return "Command is not allowed while CVM is running."; break;
		case Code::kAmplifierCvmProgramTooBig: return "THe CVM program is too big to upload."; break;
		case Code::kAmplifierFileSystemError: return "File system internal error."; break;
		case Code::kAmplifierProgramDoesntExist: return "Specified program doesn't exist."; break;
		case Code::kAmplifierInvalidNodeForForwarding: return "Invalid node ID for serial port forwarding."; break;
		case Code::kAmplifierCanCommunicationFailure: return "CAN network communications failure."; break;
		case Code::kAmplifierAsciiParsingError: return "ASCII command parsing error."; break;
		case Code::kAmplifierInternalError: return "Internal Error."; break;
		case Code::kAmplifierFileSystemChangeWhileCamming: return "File system can't be changed while in camming mode."; break;
		case Code::kAmplifierBadAxisLetter: return "Bad axis letter specified."; break;
		case Code::kAmplifierInvalidFpgaData: return "Invalid FPGA data stored on amp."; break;
		case Code::kAmplifierFpgaInitFailed: return "Unable to initialize FPGA."; break;
		case Code::kAmplifierFpgaConfigureFailed: return "FPGA failed to configure."; break;
		case Code::kAmplifierFileAlreadyExists: return "File already exists."; break;
		case Code::kAmplifierNoFreeFileEntries: return "No free file entries in directory."; break;
		case Code::kAmplifierFileDoesntExist: return "File doesn't exist."; break;
		case Code::kAmplifierNoFreeSpace: return "No free space in file system."; break;
		case Code::kAmplifierInvalidFileFormat: return "Invalid file format."; break;
		case Code::kAmplifierEndOfFile: return "End of file hit while reading."; break;
		case Code::kAmplifierSendEncoderCommandError: return "Error sending command to encoder."; break;
		case Code::kAmplifierIllegalOperationInSerial: return "Operation is illegal in current serial port mode."; break;
		case Code::kAmplifierUnableToCalculateFilter: return "Unable to calculate filter."; break;
		case Code::kAmplifierCvmCommandFailed: return "Failed to perform protected CMV command because Indexer register 31 wasn't set."; break;
		case Code::kAmplifierUndocumentedTimeout: return "Amplier returned undocumented timeout error."; break;
		case Code::kMotorShortCircuit: return "Short circuit detected."; break;
		case Code::kMotorDriveOverTemp: return "Drive over temperature."; break;
		case Code::kMotorOverVoltage: return "Over voltage."; break;
		case Code::kMotorUnderVoltage: return "Under voltage."; break;
		case Code::kMotorTempActive: return "Motor temperature sensor active."; break;
		case Code::kMotorEncoderError: return "Encoder feedback error."; break;
		case Code::kMotorPhaseError: return "Motor phasing error."; break;
		case Code::kMotorCurrentLimited: return "Current output limited."; break;
		case Code::kMotorVoltageLimited: return "Voltage output limited."; break;
		case Code::kMotorLimitPosActive: return "Positive limit switch active."; break;
		case Code::kMotorLimitNegActive: return "Negative limit switch active."; break;
		case Code::kMotorHardwareDisable: return "Hardware disable (enable input not active)."; break;
		case Code::kMotorSoftwareDisable: return "Drive is disabled by software."; break;
		case Code::kMotorStopAttempt: return "Trying to stop motor."; break;
		case Code::kMotorBrakeActive: return "Motor brake activated."; break;
		case Code::kMotorPwmOutputDisabled: return "PWM outputs disabled."; break;
		case Code::kMotorSoftwarePosLimit: return "Positive software limit condition."; break;
		case Code::kMotorSoftwareNegLimit: return "Negative software limit condition."; break;
		case Code::kMotorTrackingError: return "Tracking error."; break;
		case Code::kMotorTrackingWarning: return "Tracking warning."; break;
		case Code::kMotorReset: return "Drive is currently in a reset condition."; break;
		case Code::kMotorPositionWrapped: return "Position has wrapped."; break;
		case Code::kMotorDriveFault: return "Drive latching fault."; break;
		case Code::kMotorVelocityLimit: return "Velocity limit reached."; break;
		case Code::kMotorAccelerationLimit: return "Acceleration limit reached."; break;
		case Code::kMotorTrackingWindow: return "Tracking window - position loop error is outside limits."; break;
		case Code::kMotorHomeSwitchActive: return "Home switch is active."; break;
		case Code::kMotorMoving: return "In motion."; break;
		case Code::kMotorVelocityWindow: return "Velocity window - velocity loop error is outside limits."; break;
		case Code::kMotorPhaseNotInit: return "Phase not yet initialized."; break;
		case Code::kMotorCommandFault: return "Command fault."; break;
		case Code::kMotorUndefined: return "Undefined."; break;
		case Code::kMotorHomeFail: return "Trajectory homing error."; break;
		case Code::kHomingWindowFail: return "Error during homing window search."; break;
		case Code::kDriveFlashCrcFail: return "Data flash CRC failure. This fault cannot be cleared."; break;
		case Code::kDriveInternalError: return "Drive internal error. This fault cannot be cleared."; break;
		case Code::kDriveFeedbackFault: return "Drive feedback fault."; break;
		case Code::kDriveFpgaErrorType1: return "FPGA error type 1."; break;
		case Code::kDriveCommandFault: return "Drive command fault."; break;
		case Code::kDriveFpgaErrorType2: return "FPGA error type 2."; break;
		case Code::kDriveSafetyCircuitFault: return "Safety circuit fault."; break;
		case Code::kDriveCurrentControlFail: return "Unable to control current."; break;
		case Code::kDriveMotorWiringDisconnected: return "Motor wiring disconnected."; break;
		case Code::kMotorEncoderStatus: return "Motor encoder fault detected."; break;
		case Code::kLoadEncoderStatus: return "Load encoder fault detected."; break;
		default: return "Unknown code."; break;
	}
}

}	// namespace momentum
