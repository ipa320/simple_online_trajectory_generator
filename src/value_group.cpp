
#include "sotg/value_group.hpp"

#include <string>

using namespace SOTG;

void ValueGroup::operator=(std::initializer_list<double> list)
{
    if (symbol_group_.size() != list.size()) {
        throw std::runtime_error("Amount of symbols and values does not match");
    }

    if(is_quaternion_) {
        quat_values_ = Eigen::Quaterniond(list.begin()[0],list.begin()[1],list.begin()[2],list.begin()[3]);
    } else {
    for (size_t i = 0; i < list.size(); i++) {
        values_(i) = list.begin()[i];
    }
    }

}

void ValueGroup::operator=(Eigen::Quaterniond quat)
{
    if (symbol_group_.size() != 4) {
        throw std::runtime_error("A quaternion requires 4 Symbols, " + std::to_string(symbol_group_.size())
                                 + " provided");
    }

    quat_values_ = quat;
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

    if (is_quaternion_) {
        quat_values_ = Eigen::Quaterniond(vec[0], vec[1], vec[2], vec[3]);
    } else {
        values_ = vec;
    }
}

double& ValueGroup::operator[](std::string key)
{
    size_t index = symbol_group_.getIndex(key);
    if (!symbol_group_.isQuaternion()) {
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
    if (value_group.getSymbols() != symbol_group_) {
        throw std::runtime_error("Adding value group with symbols: \"" + value_group.getSymbols().str()
                                 + "\" to value group with different symbols: \"" + symbol_group_.str()
                                 + "\" is not allowed");
    }

    ValueGroup out(symbol_group_);
    if (this->isQuaternion()) {
        // handle that
    } else {

        out = this->getCartesianValues() + value_group.getCartesianValues();
    }
    return out;

}

ValueGroup ValueGroup::operator*(double scalar) const
{
    ValueGroup out(symbol_group_);
    out = this->getCartesianValues() * scalar;
    return out;
}