#include <logging/get_logger.hpp>
#include <logging/spdlog_logger.hpp>

#include <mutex>
#include <unordered_map>

namespace robotlib {

std::shared_ptr<ILogger> getLogger(const std::string& name) {
    static std::mutex s_mutex;
    static std::unordered_map<std::string, std::shared_ptr<ILogger>> s_registry;

    std::lock_guard lock(s_mutex);
    auto it = s_registry.find(name);
    if (it != s_registry.end()) {
        return it->second;
    }
    auto logger = std::make_shared<SpdlogLogger>(name);
    s_registry[name] = logger;
    return logger;
}

}  // namespace robotlib
