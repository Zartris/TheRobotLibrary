#pragma once

#include <logging/i_logger.hpp>

#include <memory>
#include <string>

// Forward-declare spdlog's logger to keep this header lightweight.
// The full spdlog include lives in the .cpp implementation.
namespace spdlog {
class logger;
}  // namespace spdlog

namespace robotlib {

/// Concrete ILogger backed by spdlog.
/// Implementation lives in src/spdlog_logger.cpp (added in M1).
class SpdlogLogger final : public ILogger {
public:
    /// Construct a named logger. If a logger with this name already exists in
    /// spdlog's registry it will be reused; otherwise a new stderr sink is created.
    explicit SpdlogLogger(const std::string& name);

    ~SpdlogLogger() override;

    void debug(std::string_view msg) override;
    void info(std::string_view msg) override;
    void warn(std::string_view msg) override;
    void error(std::string_view msg) override;

private:
    std::shared_ptr<spdlog::logger> m_logger;
};

}  // namespace robotlib
