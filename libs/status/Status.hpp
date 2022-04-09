/*
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Status and error handling classes.
 **/

#pragma once

#include <libs/status/GenericStatus.hpp>

#include <memory>

namespace momentum
{

/// Status class for error and status handling.
/// This class is thread-safe.
/// @note Requires TContext::name() be a static member function.
/// @tparam TContext The status context.
template <typename TContext> class Status : public GenericStatus
{
public:
	/// Status codes.
	using Code = typename TContext::Code;

	/// Simple constructor.
	Status(void) : GenericStatus(TContext::context())
	{
	}

	/// Non-copyable. All client code should reference a single status object.
	Status(const Status &) = delete;

	/// Raise a warning.
	/// @param [in] code The code to raise.
	/// @param [in] detail Optional detailed message.
	/// @return The status raised.
	const Status<TContext> &raise_warning(const Code code, const std::string &detail = "")
	{
		GenericStatus::raise(static_cast<status_code_t>(code), Severity::kWarning, detail);
		return *this;
	}

	/// Raise an error.
	/// @param [in] code The code to raise.
	/// @param [in] detail Optional detailed message.
	/// @return The status raised.
	const Status<TContext> &raise_error(const Code code, const std::string &detail = "")
	{
		GenericStatus::raise(static_cast<status_code_t>(code), Severity::kError, detail);
		return *this;
	}

	/// Raise a fatal error.
	/// @param [in] code The code to raise.
	/// @param [in] detail Optional detailed message.
	/// @return The status raised.
	const Status<TContext> &raise_fatal(const Code code, const std::string &detail = "")
	{
		GenericStatus::raise(static_cast<status_code_t>(code), Severity::kFatal, detail);
		return *this;
	}

	/// Raise from another status object. Timestamp of the argument is ignored and the current time is used.
	/// Severity of the status object is ignored and replaced with the argument to this method.
	/// @param [in] status The status to raise.
	/// @param [in] severity Severity of the status.
	/// @param [in] append_detail Optional string to append to the detail field of the status tuple.
	/// @return The status raised.
	const Status<TContext> &
	raise(const Status<TContext> &status, const Severity severity, const std::string &append_detail = "")
	{
		GenericStatus::raise(status, severity, append_detail);
		return *this;
	}

	/// Merge another status into this one.
	/// See base GenericStatus::merge() for documentation.
	/// @param [in] src The source status to copy.
	/// @return Reference to this object.
	const Status<TContext> &merge(const Status<TContext> &src)
	{
		GenericStatus::merge(src);
		return *this;
	}

	/// Code of this status.
	/// @return code.
	Code code(void) const
	{
		return static_cast<Code>(GenericStatus::code());
	}

	// Forward base class clear().
	// Required since this class defines an override.
	using GenericStatus::clear;

	/// Clear a specific code.
	/// @param [in] code The code to clear.
	void clear(const Code code)
	{
		GenericStatus::clear(static_cast<status_code_t>(code));
	}

	/// Prevent broadcasting a specific status code.
	/// Adds the code to the suppression stack. Pop to restore.
	/// This does not prevent the raising and elevation of a code,
	/// it merely suppresses a broadcast upon raise.
	/// Use when an error is expected and may be cleared.
	/// @param [in] code The error code to suppress
	void suppress_push(const Code code)
	{
		GenericStatus::suppress_push(static_cast<status_code_t>(code));
	}

	/// Pop a code from the suppression stack.
	void suppress_pop(const Code code)
	{
		GenericStatus::suppress_pop(static_cast<status_code_t>(code));
	}

	/// Cache this status tuple of this object and clear.
	/// Use to temporarily reset the value of this object.
	/// @param [out] popped The popped status.
	void cache(Status<TContext> &popped)
	{
		popped.set_status_tuple(*this);
		GenericStatus::clear();
	}

	/// Restore a popped status tuple.
	/// Use to restore this status after being temporarily reset.
	/// @param [in] popped The previously popped value.
	/// @param [in] merge_into_popped True if the status tuple should be set by merging popped into the current value.
	void restore(const Status<TContext> &popped)
	{
		if (popped.severity() >= severity()) {
			set_status_tuple(popped);
		}
	}

	/// Compare against another status.
	/// @param [in] code The status code to compare.
	/// @return true if codes are equal.
	bool operator==(const Code code) const
	{
		return compare(static_cast<status_code_t>(code));
	}

	/// Compare against another status.
	/// @param [in] code The status code to compare.
	/// @return true if codes differ.
	bool operator!=(const Code code) const
	{
		return !operator==(code);
	}

	/// Compare against another status.
	/// @param [in] code The status code to compare.
	/// @return true if this severity is greater than the other severity.
	bool operator>(const Status<TContext> &status) const
	{
		return severity() > status.severity();
	}

	/// Compare two status objects and return the object with the highest severity.
	/// Base case for variadic template max.
	/// @param [in] status1 The first status object.
	/// @param [in] status2 The second status object.
	/// @return The status object with the highest severity. If severities are equal, the first object is returned.
	static const Status<TContext> &max(const Status<TContext> &status1, const Status<TContext> &status2)
	{
		if (status1.severity() >= status2.severity()) {
			return status1;
		} else {
			return status2;
		}
	}

	/// Compare any number of status objects and return the object with the highest severity.
	/// @tparam TStatus the status type.
	/// @param [in] first The first status object.
	/// @param [in] rest The remaining status objects.
	/// @return The status object with the highest severity. If severities are equal, the first object of the highest
	/// severity is returned.
	template <typename... TStatus>
	static const Status<TContext> &max(const Status<TContext> &first, const TStatus &... rest)
	{
		return Status::max(first, Status::max(rest...));
	}

private:
	/// Formatted message for a code.
	/// @note Requires TContext::code_text() be a static method.
	/// @param [in] code The status code to format.
	/// @return formatted string.
	std::string code_text(const status_code_t code) const override final
	{
		return TContext::code_text(static_cast<const Code>(code));
	}

	/// Allocate a new status object, initialize, and return a unique pointer to a constant value.
	/// @param [in] value The value to which the new object will be initialized.
	/// @return unique pointer to a const GenericStatus.
	std::unique_ptr<const GenericStatus> instance(const StatusTuple &value) const override final
	{
		Status<TContext> *const instance = new Status<TContext>();
		instance->set_status_tuple(value);
		return std::unique_ptr<const GenericStatus>(instance);
	}
};

} // namespace momentum
