#pragma once
#include <logging/i_logger.hpp>
#include <mutex>
#include <string>
#include <vector>

namespace robotlib::testing {

/// A mock ILogger that records all log calls for test assertions.
/// Thread-safe: can be shared between module and test thread.
struct LogEntry {
    enum class Level { TRACE, DEBUG, INFO, WARN, ERROR };
    Level level;
    std::string message;
};

class RecordingLogger : public ILogger {
public:
    void trace(std::string_view msg) override {
        std::lock_guard lock(m_mutex);
        m_entries.push_back({LogEntry::Level::TRACE, std::string(msg)});
    }
    void debug(std::string_view msg) override {
        std::lock_guard lock(m_mutex);
        m_entries.push_back({LogEntry::Level::DEBUG, std::string(msg)});
    }
    void info(std::string_view msg) override {
        std::lock_guard lock(m_mutex);
        m_entries.push_back({LogEntry::Level::INFO, std::string(msg)});
    }
    void warn(std::string_view msg) override {
        std::lock_guard lock(m_mutex);
        m_entries.push_back({LogEntry::Level::WARN, std::string(msg)});
    }
    void error(std::string_view msg) override {
        std::lock_guard lock(m_mutex);
        m_entries.push_back({LogEntry::Level::ERROR, std::string(msg)});
    }

    /// Get all recorded entries (snapshot).
    std::vector<LogEntry> entries() const {
        std::lock_guard lock(m_mutex);
        return m_entries;
    }

    /// Check if any entry at the given level contains the substring.
    bool hasMessageContaining(LogEntry::Level level, const std::string& substr) const {
        std::lock_guard lock(m_mutex);
        for (const auto& e : m_entries) {
            if (e.level == level && e.message.find(substr) != std::string::npos) {
                return true;
            }
        }
        return false;
    }

    /// Check that no ERROR-level entries were recorded.
    bool hasNoErrors() const {
        std::lock_guard lock(m_mutex);
        for (const auto& e : m_entries) {
            if (e.level == LogEntry::Level::ERROR) return false;
        }
        return true;
    }

    /// Clear all entries.
    void clear() {
        std::lock_guard lock(m_mutex);
        m_entries.clear();
    }

private:
    mutable std::mutex m_mutex;
    std::vector<LogEntry> m_entries;
};

}  // namespace robotlib::testing
