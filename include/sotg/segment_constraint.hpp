#pragma once

namespace SOTG {

// Contains limiting factors for segments
class SegmentConstraint {
private:
    double blending_distance_;

public:
    SegmentConstraint();
    explicit SegmentConstraint(double blend_dist);

    double getBlendDistance() const { return blending_distance_; }
};

}  // namespace SOTG