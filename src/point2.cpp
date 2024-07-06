#include "sotg/point2.hpp"

using namespace SOTG;

Point2::Point2(SymbolGroupMap symbol_map, std::string name)
{
    name_ = name;

    for (auto& [key, symbols] : symbol_map) {
        value_groups_[key].setSymbols(symbols);
    }
}

// Point2 Point2::operator+(const Point2& p2) const
// {
//     Point2 new_point;
//     if (values_.size() != p2.size())
//         throw std::runtime_error("Error: Adding two Point2s of different size!");
//     else {
//         if (p2.getOrientationIndex() != orientation_index_) {
//             if (p2.getOrientationIndex() != -1)
//                 new_Point2.setOrientationIndex(p2.getOrientationIndex());
//             else if (orientation_index_ != -1)
//                 new_Point2.setOrientationIndex(orientation_index_);
//         }
//         for (size_t i = 0; i < values_.size(); i++) {
//             new_Point2.addValue(values_[i] + p2.getValue(i));
//         }
//         new_Point2.setOrientationIndex(orientation_index_);
//         return new_Point2;
//     }
// }

// Point2 Point2::operator-() const
// {
//     Point2 new_Point2;
//     for (auto& value : values_)
//         new_Point2.addValue(-value);
//     new_Point2.setOrientationIndex(orientation_index_);
//     return new_Point2;
// }

// Point2 Point2::operator-(const Point2& p2) const
// {
//     Point2 new_Point2;
//     if (values_.size() != p2.size()) {
//         std::ostringstream os1, os2;
//         os1 << *this;
//         os2 << p2;

//         throw std::runtime_error("Trying to substract two Point2s of different size!, left Point2:" + os1.str()
//                                  + ", right Point2:" + os2.str());
//     } else {
//         if (p2.getOrientationIndex() != orientation_index_) {
//             if (p2.getOrientationIndex() != -1)
//                 new_Point2.setOrientationIndex(p2.getOrientationIndex());
//             else if (orientation_index_ != -1)
//                 new_Point2.setOrientationIndex(orientation_index_);
//         }
//         for (size_t i = 0; i < values_.size(); i++) {
//             new_Point2.addValue(values_[i] - p2.getValue(i));
//         }
//         new_Point2.setOrientationIndex(orientation_index_);
//         return new_Point2;
//     }
// }

// double Point2::norm()
// {
//     double sum = 0.0;
//     for (auto& value : values_) {
//         sum += std::pow(value, 2);
//     }

//     return std::sqrt(sum);
// }

// void Point2::zeros(size_t num_components) { values_ = std::vector<double>(num_components); }

// Point2 Point2::operator/(const double& scalar) const
// {
//     if (utility::nearlyZero(scalar)) {
//         std::ostringstream os;
//         os << *this;
//         throw std::runtime_error("Trying to divide the Point2 " + os.str() + " by 0!");
//     }

//     Point2 new_Point2;
//     for (double value : values_) {
//         new_Point2.addValue(value / scalar);
//     }
//     new_Point2.setOrientationIndex(orientation_index_);
//     return new_Point2;
// }

// Point2 Point2::operator*(const double& scalar) const
// {
//     Point2 new_Point2;
//     for (double value : values_) {
//         new_Point2.addValue(value * scalar);
//     }
//     new_Point2.setOrientationIndex(orientation_index_);
//     return new_Point2;
// }