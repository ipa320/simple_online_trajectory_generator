#pragma once

namespace SOTG {

// Contains the limiting factors for a section
class SectionConstraint {
private:
    double acceleration_magnitude_linear_;
    double acceleration_magnitude_angular_;

    double velocity_magnitude_linear_;
    double velocity_magnitude_angular_;

public:
    SectionConstraint(double acc_lin, double acc_ang, double vel_lin, double vel_ang);

    double getAccelerationMagnitudeLinear() const { return acceleration_magnitude_linear_; }
    double getAccelerationMagnitudeAngular() const { return acceleration_magnitude_angular_; }
    double getVelocityMagnitudeLinear() const { return velocity_magnitude_linear_; }
    double getVelocityMagnitudeAngular() const { return velocity_magnitude_angular_; }
};

}  // namespace SOTG