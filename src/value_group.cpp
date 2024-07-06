
#include "sotg/value_group.hpp"

using namespace SOTG;

void ValueGroup::operator=(std::initializer_list<double> list)
{
    if (symbols_.size() != list.size()) {
        throw std::runtime_error("Amount of symbols and values does not match");
    }

    if (symbols_.is_quaternion != true) {
        for (size_t i = 0; i < list.size(); i++) {
            values_[i] = list.begin()[i];
        }
    } else {
        quat_value_ = Eigen::Quaterniond{list.begin()[0], list.begin()[1], list.begin()[2], list.begin()[3]};
    }
}

double& ValueGroup::operator[](std::string key)
{
    size_t index = symbols_.getIndex(key);
    if (!symbols_.is_quaternion) {
        return values_[index];
    } else {
        // ToDo implement this
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