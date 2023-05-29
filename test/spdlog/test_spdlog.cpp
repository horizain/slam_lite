#include <memory>
#include <spdlog/logger.h>
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

int main() 
{
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_st>();
    console_sink->set_level(spdlog::level::debug);
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_st>("../log/log.txt", true);
    file_sink->set_level(spdlog::level::debug);
    spdlog::sinks_init_list sink_list = {console_sink, file_sink};
    // spdlog::logger logger("unity", sink_list.begin(), sink_list.end());
    std::shared_ptr<spdlog::logger> plogger = std::make_shared<spdlog::logger>("unity", sink_list.begin(), sink_list.end());
    plogger->set_level(spdlog::level::debug);
    spdlog::set_default_logger(plogger);
    plogger->info("test");
    SPDLOG_LOGGER_DEBUG(plogger, "macro test");

    // spdlog::warn("Easy padding in numbers like {:08d}", 12);
    // spdlog::critical("Support for int: {0:d};  hex: {0:x};  oct: {0:o}; bin: {0:b}", 42);
    // spdlog::info("Support for floats {:03.2f}", 1.23456);
    // spdlog::info("Positional args are {1} {0}..", "too", "supported");
    // spdlog::info("{:<30}", "left aligned");
    
    // spdlog::set_level(spdlog::level::debug); // Set global log level to debug
    // spdlog::debug("This message should be displayed..");    
    
    // // change log pattern
    // spdlog::set_pattern("[%H:%M:%S %z] [%n] [%^---%L---%$] [thread %t] %v");
    
    // // Compile time log levels
    // // define SPDLOG_ACTIVE_LEVEL to desired level
    // SPDLOG_TRACE("Some trace message with param {}", 42);
    // SPDLOG_DEBUG("Some debug message");
}