#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <memory>

namespace CloudCore {

class Log {
public:
    static void init();
    static void shutdown();

    // Set log level at runtime
    static void setLevel(spdlog::level::level_enum level);

    // Convenience methods for common log levels
    static void setVerbose(bool verbose);
    static void setTrace() { setLevel(spdlog::level::trace); }
    static void setDebug() { setLevel(spdlog::level::debug); }
    static void setInfo() { setLevel(spdlog::level::info); }
    static void setWarn() { setLevel(spdlog::level::warn); }
    static void setError() { setLevel(spdlog::level::err); }

    inline static std::shared_ptr<spdlog::logger>& getCoreLogger() { return s_CoreLogger; }
    inline static std::shared_ptr<spdlog::logger>& getLogger() { return s_Logger; }

private:
    static std::shared_ptr<spdlog::logger> s_CoreLogger;
    static std::shared_ptr<spdlog::logger> s_Logger;
};

} // namespace CloudCore

// Core logging macros
#define CC_CORE_TRACE(...)    ::CloudCore::Log::getCoreLogger()->trace(__VA_ARGS__)
#define CC_CORE_DEBUG(...)    ::CloudCore::Log::getCoreLogger()->debug(__VA_ARGS__)
#define CC_CORE_INFO(...)     ::CloudCore::Log::getCoreLogger()->info(__VA_ARGS__)
#define CC_CORE_WARN(...)     ::CloudCore::Log::getCoreLogger()->warn(__VA_ARGS__)
#define CC_CORE_ERROR(...)    ::CloudCore::Log::getCoreLogger()->error(__VA_ARGS__)
#define CC_CORE_CRITICAL(...) ::CloudCore::Log::getCoreLogger()->critical(__VA_ARGS__)

#define CC_TRACE(...)    ::CloudCore::Log::getLogger()->trace(__VA_ARGS__)
#define CC_DEBUG(...)    ::CloudCore::Log::getLogger()->debug(__VA_ARGS__)
#define CC_INFO(...)     ::CloudCore::Log::getLogger()->info(__VA_ARGS__)
#define CC_WARN(...)     ::CloudCore::Log::getLogger()->warn(__VA_ARGS__)
#define CC_ERROR(...)    ::CloudCore::Log::getLogger()->error(__VA_ARGS__)
#define CC_CRITICAL(...) ::CloudCore::Log::getLogger()->critical(__VA_ARGS__)


#ifdef CC_ENABLE_ASSERTS
    #ifdef _MSC_VER
        #define CC_DEBUGBREAK() __debugbreak()
    #elif defined(__linux__) || defined(__APPLE__)
        #include <signal.h>
        #define CC_DEBUGBREAK() raise(SIGTRAP)
    #else
        #define CC_DEBUGBREAK()
    #endif

    #define CC_CORE_ASSERT(x, ...) \
        { if(!(x)) { CC_CORE_ERROR("Assertion Failed: {0}", __VA_ARGS__); CC_DEBUGBREAK(); } }
    
    #define CC_ASSERT(x, ...) \
        { if(!(x)) { CC_ERROR("Assertion Failed: {0}", __VA_ARGS__); CC_DEBUGBREAK(); } }
#else
    #define CC_CORE_ASSERT(x, ...)
    #define CC_ASSERT(x, ...)
#endif
