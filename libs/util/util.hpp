/*
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Collection of utility functions common across multiple subsystems/classes.
 **/

#pragma once

#include <iostream>
#include <iomanip>
#include <sstream>
#include <termios.h>

namespace momentum
{

namespace util
{

/// Translate a byte to a formatted hexidecimal string 0x__.
/// @param [in] byte The byte to translate.
/// @return formatted hexidecimal string.
template <typename T> std::string int_to_hex(T i)
{
	std::stringstream stream;
	stream << "0x" << std::setfill('0') << std::setw(sizeof(T) * 2) << std::uppercase << std::hex << +i;
	return stream.str();
}

/// Cast an enum to its underlying type.
/// @param [in] e The enumerated value to cast.
/// @return e, cast to its underlying type.
template <typename E> constexpr auto to_underlying(E e) -> typename std::underlying_type<E>::type
{
	return static_cast<typename std::underlying_type<E>::type>(e);
}

} // namespace util

} // namespace momentum
