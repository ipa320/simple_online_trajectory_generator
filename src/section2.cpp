#include "sotg/section2.hpp"

using namespace SOTG;
using namespace detail;

const Phase& Section2::getPhaseByTime(double time) const
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

const Phase& Section2::getPhaseByType(PhaseType type) const
{
    auto it = std::find_if(phases_.begin(), phases_.end(), [&](const Phase& phase) { return type == phase.type; });

    if (it != phases_.end()) {
        return *it;
    } else {
        throw std::runtime_error("In section Nr." + std::to_string(id_) + " a phase of type: " + ToString(type)
                                 + " was requested, but isn't available.");
    }
}

const Phase& Section2::getPhaseByDistance(double distance) const
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
            + " was requested, which is outside the section. Section2 length: " + std::to_string(last_distance));
    }
}