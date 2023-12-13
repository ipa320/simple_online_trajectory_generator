#include "sotg/point.hpp"

using namespace SOTG;

Point::Point() { }

Point::Point(std::vector<Eigen::VectorXd> vec_list)
{
    for (auto& vec : vec_list)
        for (auto i = 0; i < vec.size(); ++i)
            addValue(vec[i]);
    setOrientationIndex(vec_list[0].size());
}

Point Point::operator+(const Point& p2) const
{
    Point new_point;
    if (values_.size() != p2.size())
        throw std::runtime_error("Error: Adding two points of different size!");
    else {
        if (p2.getOrientationIndex() != orientation_index_) {
            if (p2.getOrientationIndex() != -1)
                new_point.setOrientationIndex(p2.getOrientationIndex());
            else if (orientation_index_ != -1)
                new_point.setOrientationIndex(orientation_index_);
        }
        for (size_t i = 0; i < values_.size(); i++) {
            new_point.addValue(values_[i] + p2.getValue(i));
        }
        new_point.setOrientationIndex(orientation_index_);
        return new_point;
    }
}

Point Point::operator-() const
{
    Point new_point;
    for (auto& value : values_)
        new_point.addValue(-value);
    new_point.setOrientationIndex(orientation_index_);
    return new_point;
}

Point Point::operator-(const Point& p2) const
{
    Point new_point;
    if (values_.size() != p2.size()) {
        std::ostringstream os1, os2;
        os1 << *this;
        os2 << p2;

        throw std::runtime_error("Trying to substract two points of different size!, left Point:" + os1.str()
                                 + ", right Point:" + os2.str());
    } else {
        if (p2.getOrientationIndex() != orientation_index_) {
            if (p2.getOrientationIndex() != -1)
                new_point.setOrientationIndex(p2.getOrientationIndex());
            else if (orientation_index_ != -1)
                new_point.setOrientationIndex(orientation_index_);
        }
        for (size_t i = 0; i < values_.size(); i++) {
            new_point.addValue(values_[i] - p2.getValue(i));
        }
        new_point.setOrientationIndex(orientation_index_);
        return new_point;
    }
}

double Point::operator[](size_t index) const
{
    if (index >= values_.size()) {
        throw std::runtime_error("Index out of bounds, trying to access " + std::to_string(index)
                                 + "# value from a point with " + std::to_string(values_.size())
                                 + " values total");
    }
    return values_[index];
}

double Point::norm()
{
    double sum = 0.0;
    for (auto& value : values_) {
        sum += std::pow(value, 2);
    }

    return std::sqrt(sum);
}

void Point::zeros(size_t num_components) { values_ = std::vector<double>(num_components); }

Point Point::operator/(const double& scalar) const
{
    if (utility::nearlyZero(scalar)) {
        std::ostringstream os;
        os << *this;
        throw std::runtime_error("Trying to divide the point " + os.str() + " by 0!");
    }

    Point new_point;
    for (double value : values_) {
        new_point.addValue(value / scalar);
    }
    new_point.setOrientationIndex(orientation_index_);
    return new_point;
}

Point Point::operator*(const double& scalar) const
{
    Point new_point;
    for (double value : values_) {
        new_point.addValue(value * scalar);
    }
    new_point.setOrientationIndex(orientation_index_);
    return new_point;
}

Point Point::getLocation()
{
    Point new_point;
    for (size_t i = 0; i < values_.size(); ++i) {
        if (int(i) < orientation_index_)
            new_point.addValue(values_[i]);
    }
    return new_point;
}

Point Point::getOrientation()
{
    Point new_point;
    for (size_t i = 0; i < values_.size(); ++i) {
        if (int(i) >= orientation_index_)
            new_point.addValue(values_[i]);
    }
    return new_point;
}