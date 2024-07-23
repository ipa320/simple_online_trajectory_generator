#pragma once

#include <iostream>

namespace SOTG {

class KinematicSolver;

/*The Logger class can be used in order to pass SOTG internal warnings and debugging info to the user instead
 * of printing them to directly to cout*/
class Logger {
public:
    enum MsgType { INFO, WARNING, DEBUG };

    virtual void log(const std::string& message, MsgType type = INFO) const;
};

}  // namespace SOTG