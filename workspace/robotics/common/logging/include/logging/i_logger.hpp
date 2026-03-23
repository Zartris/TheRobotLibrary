#pragma once

#include <string_view>

namespace robotlib {

/// Abstract logging interface.
/// All modules log through this interface, decoupling them from any concrete
/// logging backend (spdlog, stdout, test doubles, etc.).
class ILogger {
public:
    virtual ~ILogger() = default;

    virtual void trace(std::string_view msg) = 0;
    virtual void debug(std::string_view msg) = 0;
    virtual void info(std::string_view msg) = 0;
    virtual void warn(std::string_view msg) = 0;
    virtual void error(std::string_view msg) = 0;
};

}  // namespace robotlib
