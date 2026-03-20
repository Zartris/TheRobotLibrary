#pragma once

#include <logging/i_logger.hpp>

#include <memory>
#include <string>

namespace robotlib {

/// Return a named logger instance.
/// If a logger with the given name has already been created it is returned from
/// the internal registry; otherwise a new SpdlogLogger is constructed.
/// Implementation lives in src/get_logger.cpp (added in M1).
std::shared_ptr<ILogger> getLogger(const std::string& name = "default");

}  // namespace robotlib
