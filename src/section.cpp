#include "sotg/section.hpp"

using namespace SOTG;
using namespace detail;

Section::Section(Frame& p_start_ref, Frame& p_end_ref, SectionConstraint constraint_copy, size_t section_id)
    : start_point_(p_start_ref)
    , end_point_(p_end_ref)
    , constraint_(constraint_copy)
    , id_(section_id)
{
    Eigen::VectorXd diff_lin = end_point_.getLocation() - start_point_.getLocation();
    Eigen::Quaterniond diff_ang = utility::quatDiff(start_point_.getOrientation(), end_point_.getOrientation());
    Frame diff_frame(diff_lin, diff_ang);
    setDifference(diff_frame);

    setAngularDistance(start_point_.getOrientation().angularDistance(end_point_.getOrientation()));
}

const Phase& Section::getPhaseByTime(double time) const
{
    double previous_time = 0.0;
    for (auto& phase : phases_) {
        if (previous_time <= time && time < phase.duration + previous_time) {
            return phase;
        }
        previous_time += phase.duration;
    }
    if (utility::nearlyEqual(time, previous_time, 1e-6)) {
        return phases_.back();
    } else {
        throw std::runtime_error("In section Nr." + std::to_string(id_)
                                 + " a phase at time: " + std::to_string(time)
                                 + " was requested, which is outside the section. Total section time: "
                                 + std::to_string(previous_time));
    }
}

const Phase& Section::getPhaseByType(PhaseType type) const
{
    auto it = std::find_if(phases_.begin(), phases_.end(), [&](const Phase& phase) { return type == phase.type; });

    if (it != phases_.end()) {
        return *it;
    } else {
        throw std::runtime_error("In section Nr." + std::to_string(id_) + " a phase of type: " + ToString(type)
                                 + " was requested, but isn't available.");
    }
}

const Phase& Section::getPhaseByDistance(double distance) const
{
    // if (utility::nearlyZero(distance)) {
    //     return getPhaseByType(PhaseType::ConstantAcceleration);
    // }

    double last_distance = 0.0;
    for (auto& phase : getPhases()) {
        if (last_distance <= distance && distance < (phase.length + last_distance)) {
            return phase;
        }
        last_distance += phase.length;
    }
    if (utility::nearlyEqual(distance, last_distance, 1e-5)) {
        return phases_.back();
    } else {
        throw std::runtime_error(
            "In section Nr." + std::to_string(id_) + " a phase at distance: " + std::to_string(distance)
            + " was requested, which is outside the section. Section length: " + std::to_string(last_distance));
    }
}