#pragma once

#include <initializer_list>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "sotg/symbol_group.hpp"

namespace SOTG {

// The Blueprint for are SymbolGroupValue object.
// SymbolGroups are used in a SymbolMap to define the structure of the Poses that the trajectorie should cover.
// A SymbolGroup defines part of a Pose that share the same unit, e.g. Position in metres or Euler Angles in
// radians Exemple: a SymbolGroup could define two Positions x,y that are jointly limited in accelration and
// velocity, together with another SymbolGroup that defines a single Euler Angle that has its own angular
// acceleration and velocity limits, forms the basis for a 2D trajectory.

class ValueGroup {
private:
    SymbolGroup symbol_group_;

    Eigen::VectorXd values_;
    Eigen::Quaterniond quat_values_;

    double a_max_{-1};
    double v_max_{-1};

    double blend_radius_{0};

    bool is_quaternion_{false};

public:
    ValueGroup(SymbolGroup symbol_group)
        : symbol_group_(symbol_group), is_quaternion_(symbol_group.isQuaternion())
    {
        values_ = Eigen::VectorXd(symbol_group_.size());
    }
    ValueGroup() = delete;

    // Todo initialize with Eigen::Vector and Eigen::Quaternion
    void operator=(std::initializer_list<double> list);
    void operator=(Eigen::VectorXd vec);
    void operator=(Eigen::Quaterniond quat);
    //void operator=(const ValueGroup& vg);
    double& operator[](std::string key);
    ValueGroup operator+(const ValueGroup& value_group) const;
    ValueGroup operator*(double scalar) const;

    const SymbolGroup& getSymbols() const { return symbol_group_; }
    const Eigen::VectorXd getCartesianValues() const { return values_; }
    const Eigen::Quaterniond getQuaternionValues() const { return quat_values_; }

    double getMaxAcc() const { return a_max_; }
    double getMaxVel() const { return v_max_; }
    double getBlendRadius() const { return blend_radius_; }

    void setConstraints(double a_max, double v_max, double blend_radius = 0.0) {
        a_max_ = a_max;
        v_max_ = v_max;
        blend_radius_ = blend_radius;
    }
    void setMaxAcc(double a_max) { a_max_ = a_max; }
    void setMaxVel(double v_max) { v_max_ = v_max; }
    void setBlendRadius(double blend_radius) { blend_radius_ = blend_radius; }
    bool isQuaternion() const { return is_quaternion_; }

    friend std::ostream& operator<<(std::ostream& out, const ValueGroup& vg)
    {
        auto& symbols = vg.getSymbols();
        out << "{";
        if (vg.isQuaternion()) {
            out << " " << symbols[0] << " : " << std::to_string(vg.quat_values_.w()) << ",";
            out << " " << symbols[1] << " : " << std::to_string(vg.quat_values_.x()) << ",";
            out << " " << symbols[2] << " : " << std::to_string(vg.quat_values_.y()) << ",";
            out << " " << symbols[3] << " : " << std::to_string(vg.quat_values_.z()) << ",";

        } else {
            for (size_t i = 0; i < vg.getSymbols().size(); i++) {
                    out << " " << symbols[i] << " : " << std::to_string(vg.values_[i]) << ",";
            }
        }
        out << "\b";
        out << " }";

        return out;
    }
};

using ValueGroupMap = std::unordered_map<std::string, ValueGroup>;

}  // namespace SOTG