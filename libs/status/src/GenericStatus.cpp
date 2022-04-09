/*
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 **/

#include <libs/status/GenericStatus.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <functional>

namespace momentum
{

using namespace boost::posix_time;

StatusSuppression::StatusSuppression(const std::shared_ptr<std::atomic<uint32_t>> suppress_count)
  : _suppress_count(suppress_count)
{
	(*_suppress_count)++;
}

StatusSuppression::~StatusSuppression(void)
{
	if (*(_suppress_count) > 0)
		(*_suppress_count)--;
}

GenericStatus::StatusTuple::StatusTuple(const std::string &_context,
										const status_code_t _code,
										const Severity _severity,
										const ptime _time,
										const std::string _detail)
  : context(_context), code(_code), severity(_severity), time(_time), detail(_detail)
{
}

void GenericStatus::StatusTuple::clear(void)
{
	code	 = 0;
	severity = Severity::kInfo;
	time	 = microsec_clock::universal_time();
	detail.clear();
}

GenericStatus::GenericStatus(const std::string &context)
  : _suppress_code(new std::multiset<status_code_t>()),
	_suppress_code_mutex(),
	_suppress_all_count(new std::atomic<uint32_t>(0)),
	_status(context),
	_status_mutex()
{
}

GenericStatus::Severity GenericStatus::severity(void) const
{
	boost::shared_lock<boost::shared_mutex> guard(_status_mutex);
	return _status.severity;
}

boost::posix_time::ptime GenericStatus::timestamp(void) const
{
	boost::shared_lock<boost::shared_mutex> guard(_status_mutex);
	return _status.time;
}

std::string GenericStatus::detail(void) const
{
	boost::shared_lock<boost::shared_mutex> guard(_status_mutex);
	return _status.detail;
}

std::string GenericStatus::context(void) const
{
	boost::shared_lock<boost::shared_mutex> guard(_status_mutex);
	return _status.context;
}

void GenericStatus::clear(void)
{
	{
		boost::unique_lock<boost::shared_mutex> guard(_status_mutex);
		_status.clear();
		// add a "cleared" message for observers to see
		_status.detail = "cleared";
	}

	boost::unique_lock<boost::shared_mutex> guard(_status_mutex);
	_status.detail.clear();
}

void GenericStatus::clear(const status_code_t code)
{
	bool codes_match = false;
	{
		boost::shared_lock<boost::shared_mutex> gaurd(_status_mutex);
		codes_match = _status.code == code;
	}
	if (codes_match) {
		clear();
	}
}

void GenericStatus::set_status_tuple(const GenericStatus &status)
{
	boost::shared_lock<boost::shared_mutex> gaurd(status._status_mutex);
	set_status_tuple(status._status);
}

void GenericStatus::set_status_tuple(const StatusTuple &status)
{
	boost::unique_lock<boost::shared_mutex> guard(_status_mutex);
	_status.code	 = status.code;
	_status.severity = status.severity;
	_status.time	 = status.time;
	_status.detail   = status.detail;
}

bool GenericStatus::ok(void) const
{
	boost::shared_lock<boost::shared_mutex> guard(_status_mutex);
	return _status.severity < Severity::kError;
}

status_code_t GenericStatus::code(void) const
{
	return _status.code;
}

bool GenericStatus::compare(const status_code_t code) const
{
	boost::shared_lock<boost::shared_mutex> guard(_status_mutex);
	return _status.code == code;
}

void GenericStatus::raise(const status_code_t code, const Severity severity, const std::string &detail)
{
	const StatusTuple raised_status(_status.context, code, severity, microsec_clock::universal_time(), detail);

	// code is not suppressed ?
	bool is_suppressed = (*_suppress_all_count) > 0;
	if (!is_suppressed) {
		{
			boost::shared_lock<boost::shared_mutex> guard(_suppress_code_mutex);
			is_suppressed = _suppress_code->find(raised_status.code) != _suppress_code->end();
		}
	}

	// elevate severity and store code
	bool is_elevated = false;
	{
		boost::shared_lock<boost::shared_mutex> guard(_status_mutex);
		is_elevated = severity > _status.severity;
	}
	if (is_elevated)
		set_status_tuple(raised_status);
}

void GenericStatus::raise(const GenericStatus &status, const Severity severity, const std::string &append_detail)
{
	boost::shared_lock<boost::shared_mutex> caller_guard(status._status_mutex);
	raise(status._status.code, severity, status.detail() + append_detail);
}

const GenericStatus &GenericStatus::merge(const GenericStatus &src)
{
	boost::shared_lock<boost::shared_mutex> caller_guard(src._status_mutex);
	bool is_elevated = false;
	{
		boost::shared_lock<boost::shared_mutex> this_guard(_status_mutex);
		is_elevated = src._status.severity > _status.severity;
	}
	if (is_elevated) {
		// do not raise on merge - src object should have already done this
		set_status_tuple(src._status);
	}
	return *this;
}

void GenericStatus::suppress_push(const status_code_t code)
{
	boost::unique_lock<boost::shared_mutex> guard(_suppress_code_mutex);
	_suppress_code->insert(code);
}

void GenericStatus::suppress_pop(const status_code_t code)
{
	boost::upgrade_lock<boost::shared_mutex> guard(_suppress_code_mutex);
	auto element = _suppress_code->find(code);
	if (element != _suppress_code->end()) {
		boost::upgrade_to_unique_lock<boost::shared_mutex> upgraded_guard(guard);
		_suppress_code->erase(element);
	}
}

std::unique_ptr<StatusSuppression> GenericStatus::suppress_all(void)
{
	return std::unique_ptr<StatusSuppression>(new StatusSuppression(_suppress_all_count));
}

std::string GenericStatus::to_string(const Severity severity)
{
	switch (severity) {
	case Severity::kInfo:
		return "info";
		break;
	case Severity::kWarning:
		return "warning";
		break;
	case Severity::kError:
		return "error";
		break;
	case Severity::kFatal:
		return "fatal";
		break;
	default:
		return "unknown";
		break;
	}
}

std::string GenericStatus::to_string(void) const
{
	boost::shared_lock<boost::shared_mutex> gaurd(_status_mutex);
	return to_string(_status);
}

std::string GenericStatus::to_string(const StatusTuple &status) const
{
	std::string msg;
	msg = to_string(status.severity) + " " + std::to_string(status.code) + ": " + code_text(status.code);
	if (!status.detail.empty()) {
		msg += " " + status.detail;
	}
	return msg;
}


} // namespace momentum
