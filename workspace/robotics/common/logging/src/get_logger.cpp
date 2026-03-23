#include <logging/get_logger.hpp>
#include <logging/spdlog_logger.hpp>

#include <mutex>
#include <unordered_map>

namespace robotlib {
namespace {
std::mutex g_loggerMutex;
std::unordered_map<std::string, std::shared_ptr<ILogger>> g_loggerRegistry;
}  // namespace

std::shared_ptr<ILogger> getLogger(const std::string& name) {
    std::lock_guard lock(g_loggerMutex);
    auto it = g_loggerRegistry.find(name);
    if (it != g_loggerRegistry.end()) {
        return it->second;
    }
    auto logger = std::make_shared<SpdlogLogger>(name);
    g_loggerRegistry[name] = logger;
    return logger;
}

void registerLogger(const std::string& name, std::shared_ptr<ILogger> logger) {
    std::lock_guard lock(g_loggerMutex);
    g_loggerRegistry[name] = std::move(logger);
}

void clearLoggerRegistry() {
    std::lock_guard lock(g_loggerMutex);
    g_loggerRegistry.clear();
}

}  // namespace robotlib
