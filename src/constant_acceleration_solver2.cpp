#include "sotg/constant_acceleration_solver2.hpp"

using namespace SOTG;
using namespace detail;

int findIndexOfMax(const std::vector<double>& values);
double calcVecNorm(const std::vector<double>& vec);
double calcPhaseLength(const Phase& phase);

void ConstantAccelerationSolver2::calcAccAndVelPerDoF(const Section2& section, std::vector<double>& a_max_vec,
                                                      std::vector<double>& v_max_vec)
{
    Point2 p_start = section.getStartPoint();
    Point2 p_end = section.getEndPoint();
}

void ConstantAccelerationSolver2::calcPhaseTimeAndDistance(double& a_max, double& v_max, double L_total,
                                                           PhaseDoF& acc_phase_single_dof,
                                                           PhaseDoF& coast_phase_single_dof,
                                                           PhaseDoF& dec_phase_single_dof)
{
    double T_acc;
    double T_coast;
    double T_dec;
    double L_acc;
    double L_coast;
    double L_dec;

    if (utility::nearlyZero(a_max)) {
        T_acc = 0.0;
        L_acc = 0.0;
    } else {
        T_acc = v_max / a_max;
        L_acc = 0.5 * a_max * std::pow(T_acc, 2);
    }
    acc_phase_single_dof.duration = T_acc;
    acc_phase_single_dof.length = L_acc;
    acc_phase_single_dof.distance_p_start = 0.0;

    if (utility::nearlyZero(v_max)) {
        T_dec = 0.0;
    } else {
        T_dec = v_max / a_max;
    }

    L_dec = -0.5 * a_max * std::pow(T_dec, 2) + v_max * T_dec;
    dec_phase_single_dof.duration = T_dec;
    dec_phase_single_dof.length = L_dec;

    L_coast = L_total - L_acc - L_dec;
    if (utility::nearlyZero(v_max)) {
        T_coast = 0.0;
    } else {
        T_coast = L_coast / v_max;
    }
    coast_phase_single_dof.duration = T_coast;
    coast_phase_single_dof.length = L_coast;
    coast_phase_single_dof.distance_p_start = L_acc;

    dec_phase_single_dof.distance_p_start = L_acc + L_coast;

    if (L_coast < 0.0 && !(std::abs(L_coast) < 1e-4)) {
        std::cout << "Warning::KinematicSolver: velocity should not be decreased in this step!" << std::endl;
    }
}

void ConstantAccelerationSolver2::calcTotalTimeAndDistanceSingleDoF(double& a_max, double& v_max,
                                                                    double total_length, double& total_time)
{
    double T_acc;
    double T_coast;
    double T_dec;
    double L_acc;
    double L_coast;
    double L_dec;

    if (utility::nearlyZero(a_max)) {
        T_acc = 0.0;
        L_acc = 0.0;
    } else {
        T_acc = v_max / a_max;
        L_acc = 0.5 * a_max * std::pow(T_acc, 2);
    }

    if (utility::nearlyZero(v_max)) {
        T_dec = 0.0;
    } else {
        T_dec = v_max / a_max;
    }

    L_dec = -0.5 * a_max * std::pow(T_dec, 2) + v_max * T_dec;

    L_coast = total_length - L_acc - L_dec;
    if (utility::nearlyZero(v_max)) {
        T_coast = 0.0;
    } else {
        T_coast = L_coast / v_max;
    }

    total_time = T_acc + T_coast + T_dec;

    if (L_coast < 0.0 && !(std::abs(L_coast) < 1e-6)) {
        double v_max_reduced = std::sqrt(total_length * a_max);
#ifdef DEBUG
        std::cout << "Info::KinematicSolver: Decreasing maximum velocity" << std::endl;
#endif
        calcTotalTimeAndDistanceSingleDoF(a_max, v_max_reduced, total_length, total_time);
        v_max = v_max_reduced;
    }
}

void ConstantAccelerationSolver2::calcTimesAndLengthsMultiDoF(Phase& acc_phase, Phase& coast_phase,
                                                              Phase& dec_phase,
                                                              std::vector<double>& total_time_per_dof,
                                                              std::vector<double>& total_length_per_dof,
                                                              Point2 diff, std::vector<double>& a_max_vec,
                                                              std::vector<double>& v_max_vec)
{
    // for (size_t i = 0; i < diff.size(); i++) {
    //     double total_length_dof = std::abs(diff[i]);
    //     double total_time_dof = 0.0;

    //     calcTotalTimeAndDistanceSingleDoF(a_max_vec[i], v_max_vec[i], total_length_dof, total_time_dof);

    //     total_time_per_dof.push_back(total_time_dof);
    //     total_length_per_dof.push_back(total_length_dof);
    // }

    int index_slowest_dof = findIndexOfMax(total_time_per_dof);

    // phase sync
    // https://theses.hal.science/tel-01285383/document p.62 ff.

    // Find the first component duration that is not 0.0 because all of them have the same value or 0.0
    auto it_acc
        = std::find_if(acc_phase.components.begin(), acc_phase.components.end(),
                       [](const PhaseDoF& phase_dof) { return !(utility::nearlyZero(phase_dof.duration)); });
    if (it_acc != acc_phase.components.end()) {
        acc_phase.duration = it_acc->duration;
    }

    auto it_coast
        = std::find_if(coast_phase.components.begin(), coast_phase.components.end(),
                       [](const PhaseDoF& phase_dof) { return !(utility::nearlyZero(phase_dof.duration)); });
    if (it_coast != coast_phase.components.end()) {
        coast_phase.duration = it_coast->duration;
    }

    auto it_dec
        = std::find_if(dec_phase.components.begin(), dec_phase.components.end(),
                       [](const PhaseDoF& phase_dof) { return !(utility::nearlyZero(phase_dof.duration)); });
    if (it_dec != dec_phase.components.end()) {
        dec_phase.duration = it_dec->duration;
    }
}

Section2 ConstantAccelerationSolver2::calcSection(Point2& p_start_ref, Point2& p_end_ref, size_t section_id)
{
    Section2 section(p_start_ref, p_end_ref, section_id);

    for (auto& group : section.getValueGroups()) { }

    std::vector<double> reduced_acceleration_per_dof;
    std::vector<double> reduced_velocity_per_dof;

    calcAccAndVelPerDoF(section, reduced_acceleration_per_dof, reduced_velocity_per_dof);

    Point2 p_start = section.getStartPoint();
    Point2 p_end = section.getEndPoint();

    std::vector<double> total_time_per_dof;
    std::vector<double> total_length_per_dof;
    std::vector<Phase> phases;
    Phase acc_phase, coast_phase, dec_phase;

    calcTimesAndLengthsMultiDoF(acc_phase, coast_phase, dec_phase, total_time_per_dof, total_length_per_dof, diff,
                                reduced_acceleration_per_dof, reduced_velocity_per_dof);

    int index_slowest_dof = findIndexOfMax(total_time_per_dof);
    double T_total = total_time_per_dof[index_slowest_dof];
    section.setIndexSlowestDoF(index_slowest_dof);
    section.setDuration(T_total);

    acc_phase.type = PhaseType::ConstantAcceleration;
    acc_phase.length = calcPhaseLength(acc_phase);
    phases.push_back(acc_phase);

    coast_phase.type = PhaseType::ConstantVelocity;
    coast_phase.t_start = acc_phase.duration;
    coast_phase.length = calcPhaseLength(coast_phase);
    coast_phase.distance_p_start = acc_phase.length;
    phases.push_back(coast_phase);

    dec_phase.type = PhaseType::ConstantDeacceleration;
    dec_phase.t_start = coast_phase.duration + coast_phase.t_start;
    dec_phase.length = calcPhaseLength(dec_phase);
    dec_phase.distance_p_start = coast_phase.distance_p_start + coast_phase.length;
    phases.push_back(dec_phase);

    section.setPhases(phases);

    section.setAdaptedAcceleration(reduced_acceleration_per_dof);
    section.setAdaptedVelocity(reduced_velocity_per_dof);

    return section;
}

void ConstantAccelerationSolver2::calcPosAndVelSingleDoFLinear(double section_dof_length, const Phase& phase,
                                                               double phase_distance_to_p_start, double t_phase,
                                                               double a_max_reduced, double v_max_reduced,
                                                               double& pos, double& vel) const
{
    double p_i{0}, v_i{0};
    if (phase.type == PhaseType::ConstantAcceleration) {
        p_i = 0.5 * a_max_reduced * std::pow(t_phase, 2);
        v_i = a_max_reduced * t_phase;
    } else if (phase.type == PhaseType::ConstantVelocity) {
        p_i = v_max_reduced * t_phase + phase_distance_to_p_start;
        v_i = v_max_reduced;
    } else if (phase.type == PhaseType::ConstantDeacceleration) {
        p_i = -0.5 * a_max_reduced * std::pow(t_phase, 2) + v_max_reduced * t_phase + phase_distance_to_p_start;
        v_i = -a_max_reduced * t_phase + v_max_reduced;
    }
    if (utility::nearlyZero(section_dof_length)) {
        pos = 0;
    } else {
        pos = p_i / section_dof_length;
    }
    vel = v_i;
}

void ConstantAccelerationSolver2::calcPosAndVelSection(double t_section, const Section2& section, Point2& pos,
                                                       Point2& vel) const
{
    // Point2 p_start = section.getStartPoint();
    // Point2 p_end = section.getEndPoint();
    // Point2 diff = p_end - p_start;

    // const std::vector<double>& a_max_vec = section.getAdaptedAcceleration();
    // const std::vector<double>& v_max_vec = section.getAdaptedVelocity();

    // const Phase& phase = section.getPhaseByTime(t_section);
    // double t_phase = t_section - phase.t_start;
    // for (size_t i = 0; i < p_start.size(); i++) {
    //     double pos_relative_magnitude, vel_magnitude;
    //     calcPosAndVelSingleDoFLinear(std::abs(diff[i]), phase, phase.components[i].distance_p_start, t_phase,
    //                                  a_max_vec[i], v_max_vec[i], pos_relative_magnitude, vel_magnitude);

    //     double pos_component = p_start[i] + pos_relative_magnitude * diff[i];
    //     double vel_component = vel_magnitude * utility::sign(diff[i]);

    //     pos.addValue(pos_component);
    //     vel.addValue(vel_component);
    // }
    // pos.setOrientationIndex(p_start.getOrientationIndex());
    // vel.setOrientationIndex(p_start.getOrientationIndex());
}
