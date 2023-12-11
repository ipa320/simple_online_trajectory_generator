#include "sotg/section_constraint.hpp"

using namespace SOTG;

SectionConstraint::SectionConstraint(double acc_lin, double acc_ang, double vel_lin, double vel_ang)
{
    acceleration_magnitude_linear_ = acc_lin;
    acceleration_magnitude_angular_ = acc_ang;

    velocity_magnitude_linear_ = vel_lin;
    velocity_magnitude_angular_ = vel_ang;
}