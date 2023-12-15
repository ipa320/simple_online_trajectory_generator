#include "sotg/point.hpp"

using namespace SOTG;

Frame::Frame(Eigen::VectorXd& vec)
{
    for (Eigen::Index i = 0; i < vec.size(); ++i) {
        addValue(vec[i]);
    }
}

Frame::Frame(std::vector<Eigen::VectorXd> vec_list)
{
    for (auto& vec : vec_list)
        for (auto i = 0; i < vec.size(); ++i)
            addValue(vec[i]);
    setOrientationIndex(vec_list[0].size());
}

Frame::Frame(Eigen::VectorXd lin, Eigen::Quaterniond ang)
{
    location_ = lin;
    orientation_ = ang;
}

Frame Frame::diff(const Frame& f2) const
{
    Eigen::VectorXd lin = f2.getLocation() - location_;
    Eigen::Quaterniond ang = utility::quatDiff(orientation_, f2.getOrientation());
    Frame new_frame(lin, ang);
    return new_frame;
}