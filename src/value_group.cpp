
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

// void ValueGroup::operator=(const ValueGroup& vg)
// {
//     is_quaternion_ = vg.is_quaternion_;
//     values_ = vg.values_;
//     quat_values_ = vg.quat_values_;
//     a_max_ = vg.a_max_;
//     v_max_ = vg.v_max_;
//     blend_radius_ = blend_radius_;
// }

void ValueGroup::operator=(Eigen::VectorXd vec) {
    values_ = vec;
    is_quaternion_ = false;
}

double& ValueGroup::operator[](std::string key)
{
    size_t index = symbols_.getIndex(key);
    if (!symbols_.isQuaternion()) {
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

ValueGroup ValueGroup::operator+(const ValueGroup& value_group) const
{
    if (value_group.getSymbols() != symbols_) {
        throw std::runtime_error("Adding value group with symbols: \"" + value_group.getSymbols().str()
                                 + "\" to value group with different symbols: \"" + symbols_.str()
                                 + "\" is not allowed");
    }

    ValueGroup out(symbols_);
    if (this->isQuaternion()) {
        // handle that
    } else {

        out = this->getCartesianValues() + value_group.getCartesianValues();
    }
    return out;

}

ValueGroup ValueGroup::operator*(double scalar) const
{
    ValueGroup out(symbols_);
    out = this->getCartesianValues() * scalar;
    return out;
}

// ValueGroup ValueGroup::operator-(ValueGroup& value_group) { }
// ValueGroup ValueGroup::operator*(double scalar) { }

/*

#include "sotg/value_group.hpp"

#include <string>

using namespace SOTG;

void CartesianValueGroup::operator=(std::initializer_list<double> list)
{
    if (symbols_.size() != list.size()) {
        throw std::runtime_error("Amount of symbols and values does not match");
    }

    for (size_t i = 0; i < list.size(); i++) {
        values_(i) = list.begin()[i];
    }
}

void QuaternionValueGroup::operator=(Eigen::Quaterniond quat)
{
    if (symbols_.size() != 4) {
        throw std::runtime_error("A quaternion requires 4 Symbols, " + std::to_string(symbols_.size())
                                 + " provided");
    }

    values_ = quat;
}
double& CartesianValueGroup::operator[](std::string key)
{
    size_t index = symbols_.getIndex(key);
    return values_[index];
}

double& QuaternionValueGroup::operator[](std::string key)
{
    size_t index = symbols_.getIndex(key);
    switch (index) {
    case 0:
        return values_.w();
        break;
    case 1:
        return values_.x();
        break;
    case 2:
        return values_.y();
        break;
    case 3:
        return values_.z();
        break;
    default:
        throw std::runtime_error("Requested symbol: \"" + key + "\" with index: " + std::to_string(index)
                                    + " but quaternions only have 4 components");
        break;
    }
}

CartesianValueGroup CartesianValueGroup::operator+(CartesianValueGroup& value_group)
{
    if (value_group.getSymbols() != symbols_) {
        throw std::runtime_error("Adding value group with symbols: \"" + value_group.getSymbols().str()
                                 + "\" to value group with different symbols: \"" + symbols_.str()
                                 + "\" is not allowed");
    } else {
        CartesianValueGroup new_group(symbols_);
        new_group = values_ + value_group.getValues();
    }
}

QuaternionValueGroup QuaternionValueGroup::operator+(QuaternionValueGroup& value_group)
{
        if (value_group.getSymbols() != symbols_) {
        throw std::runtime_error("Adding value group with symbols: \"" + value_group.getSymbols().str()
                                 + "\" to value group with different symbols: \"" + symbols_.str()
                                 + "\" is not allowed");
    }
}*/