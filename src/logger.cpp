#include "sotg/logger.hpp"

using namespace SOTG;

void Logger::log( [[maybe_unused]] const std::string& message, [[maybe_unused]] MsgType type) const
{
#ifdef VERBOSE
    switch (type) {
    case INFO:
        std::cout << "[ INFO] " << message << std::endl;
        break;
    case WARNING:
        std::cout << "[ WARN] " << message << std::endl;
        break;
    case DEBUG:
#ifdef DEBUG
        std::cout << "[DEBUG] " << message << std::endl;
#endif
        break;
    default:
        throw std::runtime_error("SOTG Logger received message with unknown Message Type, Content: " + message);
    }
#endif
}
