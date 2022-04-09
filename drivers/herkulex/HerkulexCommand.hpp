/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Herkulex command and acknowledgement types.
 **/

#pragma once

#include <stdint.h>

namespace momentum
{

/// Herkulex command.
enum class HerkulexCommand : uint8_t {
	EEP_WRITE = 0x01, ///< Write an EEP register.
	EEP_READ  = 0x02, ///< Read an EEP register.
	RAM_WRITE = 0x03, ///< Write a RAM register.
	RAM_READ  = 0x04, ///< Read a RAM register.
	I_JOG	 = 0x05,
	S_JOG	 = 0x06,
	STAT	  = 0x07, ///< Request a status packet.
	ROLLBACK  = 0x08, ///< Roll motor back to factory settings.
	REBOOT	= 0x09  ///< Reboot the motor.
};

/// Herkulex acknowledgment policy.
enum class HerkulexAckPolicy : uint8_t {
	kNoReply   = 0x00, ///< Never reply (with exception of STAT).
	kReplyRead = 0x01, ///< Reply only to register read commands.
	kReplyAll  = 0x02  ///< Reply to all packets.
};

/// Herkulex absolute position reset
enum class HerkulexPositionResetCommand : uint8_t {
	kPotRst		= 0x01, ///< r(Absolute Position) reset by Potentiometer (0602 datasheet, pg 21)
	kMinRst		= 0x02, ///< Set r(Absolute Position) to 0
	kNeg180Rst  = 0x03, ///< Set r(Absolute Position) to 9903 (-180 deg)
	kZeroDegRst = 0x04, ///< Set r(Absolute Position) to 16384 (0 deg)
	kPos180Rst  = 0x05, ///< Set r(Absolute Position) to 22865 (180 deg)
	kMaxRst		= 0x06  ///< Set r(Absolute Position) to 32767
};

} // namespace momentum
