
#include "sotg/value_group.hpp"

#include <string>

using namespace SOTG;

void ValueGroup::operator=(std::initializer_list<double> list)
{
    if (symbols_.size() != list.size()) {
        throw std::runtime_error("Amount of symbols and values does not match");
    }

    for (size_t i = 0; i < list.size(); i++) {
        values_(i) = list.begin()[i];
    }
}

void ValueGroup::operator=(Eigen::Quaterniond quat)
{
    if (symbols_.size() != 4) {
        throw std::runtime_error("A quaternion requires 4 Symbols, " + std::to_string(symbols_.size())
                                 + " provided");
    }

    quat_values_ = quat;
    is_quaternion_ = true;
}
double& ValueGroup::operator[](std::string key)
{
    size_t index = symbols_.getIndex(key);
    if (!symbols_.is_quaternion) {
        return values_[index];
    } else {
        switch (index) {
        case 0:
            return quat_values_.w();
            break;
        case 1:
            return quat_values_.x();
            break;
        case 2:
            return quat_values_.y();
            break;
        case 3:
            return quat_values_.z();
            break;
        default:
            throw std::runtime_error("Requested symbol: \"" + key + "\" with index: " + std::to_string(index)
                                     + " but quaternions only have 4 components");
            break;
        }
    }
}

ValueGroup ValueGroup::operator+(ValueGroup& value_group)
{
    if (value_group.getSymbols() != symbols_) {
        throw std::runtime_error("Adding value group with symbols: \"" + value_group.getSymbols().str()
                                 + "\" to value group with different symbols: \"" + symbols_.str()
                                 + "\" is not allowed");
    }
}
// ValueGroup ValueGroup::operator-(ValueGroup& value_group) { }
// ValueGroup ValueGroup::operator*(double scalar) { }