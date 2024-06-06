
#pragma once

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

#include "sotg/utility_functions.hpp"

namespace SOTG {

// Wraps std::vector and allows vector arithmetic, stores information about the
// unit of a particular value
class Point {
private:
    std::vector<double> values_;
    int orientation_index_ = -1;
    int id_ = -1;

public:
    Point();
    explicit Point(std::vector<Eigen::VectorXd> vec_list);

    void addValue(double value) { values_.push_back(value); };
    void setOrientationIndex(int new_index) { orientation_index_ = new_index; };
    void zeros(size_t num_components);
    int getOrientationIndex() const { return orientation_index_; }

    size_t size() const { return values_.size(); };
    double getValue(int index) const { return values_[index]; };
    Point operator+(const Point& p2) const;
    Point operator-(const Point& p2) const;
    Point operator-() const;
    Point operator/(const double& scalar) const;
    Point operator*(const double& scalar) const;
    double operator[](size_t index) const;
    double norm();

    int getID() const { return id_; }
    void setID(int id) { id_ = id; }

    Point getLocation();
    Point getOrientation();

    std::vector<double>::iterator begin() { return values_.begin(); }
    std::vector<double>::iterator end() { return values_.end(); }

    friend std::ostream& operator<<(std::ostream& out, const Point& point)
    {
        if (point.values_.size() < 1) {
            return out << "[](" << point.orientation_index_ << ")";
        }
        out << "[ ";
        for (size_t i = 0; i < point.values_.size() - 1; i++) {
            out << point.values_[i] << ", ";
        }
        if (point.orientation_index_ == -1) {
            out << point.values_.back() << " ]";
        } else {
            out << point.values_.back() << " ](" << point.orientation_index_ << ")";
        }

        return out;
    }
};

}  // namespace SOTG