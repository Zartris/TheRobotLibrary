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

/// Register a logger for a given name (primarily for testing).
/// If a logger with this name already exists, it is replaced.
void registerLogger(const std::string& name, std::shared_ptr<ILogger> logger);

/// Clear all registered loggers (test teardown).
void clearLoggerRegistry();

}  // namespace robotlib
