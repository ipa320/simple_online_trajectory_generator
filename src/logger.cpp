#include "sotg/logger.hpp"

using namespace SOTG;

void Logger::log(const std::string& message, MsgType type = INFO) const
{
    switch (type) {
    INFO:
        std::cout << "[INFO] " << message << std::endl;
        break;
    WARNING:
        std::cout << "[WARNING] " << message << std::endl;
        break;
    DEBUG:
        std::cout << "[DEBUG] " << message << std::endl;
        break;
    }
}
