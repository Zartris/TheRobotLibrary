#include <logging/spdlog_logger.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace robotlib {

SpdlogLogger::SpdlogLogger(const std::string& name) {
    m_logger = spdlog::get(name);
    if (!m_logger) {
        m_logger = spdlog::stderr_color_mt(name);
        m_logger->set_level(spdlog::level::trace);
    }
}

SpdlogLogger::~SpdlogLogger() = default;

void SpdlogLogger::debug(std::string_view msg) { m_logger->debug("{}", msg); }
void SpdlogLogger::info(std::string_view msg) { m_logger->info("{}", msg); }
void SpdlogLogger::warn(std::string_view msg) { m_logger->warn("{}", msg); }
void SpdlogLogger::error(std::string_view msg) { m_logger->error("{}", msg); }

}  // namespace robotlib
