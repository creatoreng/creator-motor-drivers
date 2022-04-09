/*
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 * @brief Status and error handling classes.
 **/

#pragma once

#include <atomic>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <map>
#include <set>
#include <string>

namespace momentum
{

/// Datatype for status and error codes.
using status_code_t = uint32_t;

/// Status suppression object which uses RAII to automatically suppress status messages
/// when created, and remove unsuppress when destroyed.
class StatusSuppression
{
public:
	/// Increment suppress_count on construction.
	/// @param [in] suppress_count shared pointer to the suppression count for the status object.
	explicit StatusSuppression(const std::shared_ptr<std::atomic<uint32_t>> suppress_count);

	StatusSuppression(const StatusSuppression &)  = delete;
	StatusSuppression(const StatusSuppression &&) = delete;

	/// Decrement suppress count on destruction.
	~StatusSuppression(void);

private:
	/// Shared pointer to the status object suppression count.
	const std::shared_ptr<std::atomic<uint32_t>> _suppress_count;
};

/// Base class for status codes and messages.
/// Derive context-specific status classes from this.
/// A status is a tuple (context, code, severity, timestamp, detail) where:
/// 	- context, a unique identifier for a driver, API, etc.
/// 	  that is not specific to an instance. This is implied by derived status types.
///		- code, a status code unique to each context
///		  that enumerates all status values.
///		- severity, a measurement of significance of the status code.
///		- timestamp, the timestamp at which the code was raised.
///		- detail, an optional message pertaining to the code.
///
/// This class is thread-safe.
class GenericStatus
{
public:
	/// Severity level of a status.
	/// Ordered by ascending severity.
	enum class Severity : uint32_t {
		kInfo = 0, ///< Nominal.
		kWarning,  ///< Warning severity.
		kError,	///< Error severity.
		kFatal	 ///< Fatal severity.
	};

	/// Non-copyable.
	GenericStatus(const GenericStatus &) = delete;

	/// Default destructor.
	virtual ~GenericStatus(void) = default;

	/// Severity of this status.
	/// @return severity.
	Severity severity(void) const;

	/// Timestamp of this status.
	/// @return timestamp
	boost::posix_time::ptime timestamp(void) const;

	/// Detail of this status.
	/// @return detail string.
	std::string detail(void) const;

	/// Context of this status.
	/// @return context string.
	std::string context(void) const;

	/// Is the current severity ok?
	/// @return true if status is ok or a warning.
	bool ok(void) const;

	/// Clear this status to its initial value.
	void clear(void);

	/// Convert a severity to a string.
	/// @return severity as a string.
	static std::string to_string(const Severity severity);

	/// Formatted error message.
	/// @return message.
	std::string to_string(void) const;

	/// RAII suppress push/pop object for automatic suppress popping when destroyed.
	/// @return status suppression object which increments status suppression on construction,
	/// and decrements on destruction.
	std::unique_ptr<StatusSuppression> suppress_all(void);

protected:
	/// Status tuple.
	struct StatusTuple {
		const std::string context;	 ///< Context.
		status_code_t code;			   ///< Status code.
		Severity severity;			   ///< Severity of this code.
		boost::posix_time::ptime time; ///< Timestamp of this code.
		std::string detail;			   ///< Detail.

		/// Initialization constructor.
		/// @param [in] _context The context of this status.
		/// @param [in] _code The status code.
		/// @param [in] _severity The severity of the code.
		/// @param [in] _time The timestamp of this code.
		/// @param [in] _detail The detail of this code.
		StatusTuple(const std::string &_context,
					const status_code_t _code			 = 0,
					const Severity _severity			 = Severity::kInfo,
					const boost::posix_time::ptime _time = boost::posix_time::microsec_clock::universal_time(),
					const std::string _detail			 = "");

		/// Clear this tuple to default values.
		void clear(void);
	};

	/// Default constructor.
	/// @param [in] context The context for this status.
	explicit GenericStatus(const std::string &context);

	/// Code of this status.
	/// @warning caller is responsible for contextual comparisons.
	/// @return raw code.
	status_code_t code(void) const;

	/// Compares this status.
	/// @warning Caller must ensure comparison contexts are the same.
	/// @param [in] code The status code to compare.
	/// @return true if status codes are equal.
	bool compare(const status_code_t code) const;

	/// Raise status code. If severity is elevated, code is stored and broadcast.
	/// @note Do not use this to clear; use clear() instead.
	/// @param [in] code The code to raise.
	/// @param [in] severity The severity of the code.
	/// @param [in] detail Optional detail message.
	void raise(const status_code_t code, const Severity severity, const std::string &detail = "");

	/// Raise from another status object. Timestamp of the argument is ignored and the current time is used.
	/// Severity of the status object is ignored and replaced with the argument to this method.
	/// @param [in] status The status to raise.
	/// @param [in] severity Severity of the status.
	/// @param [in] append_detail Optional string to append to the detail field of the status tuple.
	/// @return The status raised.
	void raise(const GenericStatus &status, const Severity severity, const std::string &append_detail = "");

	/// Merge a status into this one.
	/// The result is a status whose severity has remained
	/// the same, or has increased. Severity is never decreased.
	/// The status code and severity are copied, but raise() is not called
	/// as it is presumed to have been called by the source object.
	/// If severity is equal, no action is taken.
	/// If contexts are different, no action is taken.
	/// @param [in] src The source status to merge.
	/// @return Reference to this object.
	const GenericStatus &merge(const GenericStatus &src);

	/// Clear a specific code.
	/// @param [in] code The code to clear.
	void clear(const status_code_t code);

	/// Sets the status tuple. No broadcasting or severity checking.
	/// @param [in] status The status to set.
	void set_status_tuple(const GenericStatus &status);

	/// Sets the status tuple. No broadcasting or severity checking.
	/// @param [in] status The status tuple.
	void set_status_tuple(const StatusTuple &status);

	/// Prevent broadcasting a specific status code.
	/// Adds the code to the suppression stack. Pop to restore.
	/// This does not prevent the raising and elevation of a code,
	/// it merely suppresses a broadcast upon raise.
	/// Use when an error is expected and may be cleared.
	/// @param [in] code The error code to suppress
	void suppress_push(const status_code_t code);

	/// Pop a code from the suppression stack.
	void suppress_pop(const status_code_t code);

private:
	/// Formatted error message.
	/// @param [in] status The status to format.
	/// @return formatted message.
	std::string to_string(const StatusTuple &status) const;

	/// Formatted message for a code.
	/// @param [in] code The status code to format.
	/// @return formatted string.
	virtual std::string code_text(const status_code_t code) const = 0;

	/// Allocate a constant instance of a status object.
	/// @param [in] value The value to which the new status object will be initialized.
	/// @return unique pointer to a const GenericStatus object.
	virtual std::unique_ptr<const GenericStatus> instance(const StatusTuple &value) const = 0;

	/// Set of codes that should not be broadcast when raised.
	/// A multiset is used here to allow stack semantics.
	const std::shared_ptr<std::multiset<status_code_t>> _suppress_code;
	/// _suppress mutex
	boost::shared_mutex _suppress_code_mutex;

	/// Suppress all codes when raised.
	/// Implemented as a counter to allow stack semantics.
	const std::shared_ptr<std::atomic<uint32_t>> _suppress_all_count;

	/// Status.
	StatusTuple _status;
	/// Status tuple mutex.
	mutable boost::shared_mutex _status_mutex;
};

} // namespace momentum
