#include "sotg/constant_acceleration_solver2.hpp"

using namespace SOTG;
using namespace detail;

int findIndexOfMax(const std::vector<double>& values);
double calcVecNorm(const std::vector<double>& vec);
double calcPhaseLength(const Phase2& phase);

double calcPhaseLength(const Phase2& phase)
{
    double sum = 0.0;
    for (auto& component : phase.components) {
        sum += std::pow(component.second.length, 2);
    }
    return std::sqrt(sum);
}

void ConstantAccelerationSolver2::calcPhaseDurationAndDistance(double a_max, double v_max, double L_total,
                                                           Phase2Component& acc_phase_single_dof,
                                                           Phase2Component& coast_phase_single_dof,
                                                           Phase2Component& dec_phase_single_dof)
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
}

void ConstantAccelerationSolver2::calcDurationAndDistancePerGroup(double& a_max, double& v_max, double distance,
                                               double& duration)
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

    L_coast = distance - L_acc - L_dec;
    if (utility::nearlyZero(v_max)) {
        T_coast = 0.0;
    } else {
        T_coast = L_coast / v_max;
    }

    duration = T_acc + T_coast + T_dec;

    if (L_coast < 0.0 && !(std::abs(L_coast) < 1e-6)) {
        double v_max_reduced = std::sqrt(distance * a_max);
#ifdef DEBUG
        std::cout << "Info::KinematicSolver: Decreasing maximum velocity" << std::endl;
#endif
        calcDurationAndDistancePerGroup(a_max, v_max_reduced, distance, duration);
        v_max = v_max_reduced;
    }
}

void ConstantAccelerationSolver2::calcDurationAndDistance(Phase2& acc_phase, Phase2& coast_phase,
                                                              Phase2& dec_phase, Point2& start_point, Point2& end_point,
                                                              std::vector<double>& duration_per_group,
                                                              std::vector<double>& distance_per_group, std::map<std::string, double>& group_acc, std::map<std::string, double>& group_vel)
{
    std::vector<std::string> group_names;
    for (const auto [group_name, symbols] : symbol_map_)
    {
        double duration;

        if(symbols.isQuaternion()) {
            //handle that
        } else {
            double distance = (end_point[group_name].getCartesianValues() - start_point[group_name].getCartesianValues()).norm();
            distance_per_group.push_back(distance);

            double a_max = end_point[group_name].getMaxAcc();
            double v_max = end_point[group_name].getMaxVel();

            calcDurationAndDistancePerGroup(a_max,v_max, distance, duration);

            group_vel[group_name] = v_max;
            group_acc[group_name] = a_max;
            duration_per_group.push_back(duration);
            group_names.push_back(group_name);
        }
    }

    int index_slowest_group = findIndexOfMax(duration_per_group);

    for (size_t i = 0; i < distance_per_group.size(); ++i)
    {
        double lambda = 0.0;  // lambda is defined in theses
        if (!utility::nearlyZero(distance_per_group[index_slowest_group])) {
            lambda = distance_per_group[i] / distance_per_group[index_slowest_group];
        }

        std::string current_group = group_names[i];
        double a_max = group_acc[current_group];
        double v_max = group_vel[current_group];

        double reduced_a_max = lambda * a_max;
        double reduced_v_max = lambda * v_max;

        group_acc[current_group] = reduced_a_max;
        group_vel[current_group] = reduced_v_max;

        double section_length = distance_per_group[i];
        Phase2Component acc_phase_per_group, coast_phase_per_group, dec_phase_per_group;
        calcPhaseDurationAndDistance(reduced_a_max, reduced_v_max, section_length, acc_phase_per_group,
                                 coast_phase_per_group, dec_phase_per_group);

        acc_phase.components[current_group] = acc_phase_per_group;
        coast_phase.components[current_group] = coast_phase_per_group;
        dec_phase.components[current_group] = dec_phase_per_group;
    }

    // Find the first component duration that is not 0.0 because all of them have the same value or 0.0
    auto it_acc
        = std::find_if(acc_phase.components.begin(), acc_phase.components.end(),
                       [](const auto &it) { return !(utility::nearlyZero(it.second.duration)); });
    if (it_acc != acc_phase.components.end()) {
        acc_phase.duration = it_acc->second.duration;
    }

    auto it_coast
        = std::find_if(coast_phase.components.begin(), coast_phase.components.end(),
                       [](const auto &it) { return !(utility::nearlyZero(it.second.duration)); });
    if (it_coast != coast_phase.components.end()) {
        coast_phase.duration = it_coast->second.duration;
    }

    auto it_dec
        = std::find_if(dec_phase.components.begin(), dec_phase.components.end(),
                       [](const auto &it) { return !(utility::nearlyZero(it.second.duration)); });
    if (it_dec != dec_phase.components.end()) {
        dec_phase.duration = it_dec->second.duration;
    }
}

Section2 ConstantAccelerationSolver2::calcSection(Point2& p_start_ref, Point2& p_end_ref, size_t section_id)
{
    Section2 section(p_start_ref, p_end_ref, section_id);

    std::vector<double> duration_per_group;
    std::vector<double> distance_per_group;
    std::vector<Phase2> phases;
    Phase2 acc_phase, coast_phase, dec_phase;

    std::map<std::string, double>& v_max = section.getVelocities();
    std::map<std::string, double>& a_max = section.getAccelerations();


    calcDurationAndDistance(acc_phase, coast_phase, dec_phase, p_start_ref, p_end_ref, duration_per_group, distance_per_group, a_max, v_max);

    int index_slowest_dof = findIndexOfMax(duration_per_group);
    double T_total = duration_per_group[index_slowest_dof];
    section.setIndexSlowestDoF(index_slowest_dof);
    section.setDuration(T_total);

    acc_phase.type = PhaseType2::ConstantAcceleration2;
    acc_phase.length = calcPhaseLength(acc_phase);
    phases.push_back(acc_phase);

    coast_phase.type = PhaseType2::ConstantVelocity2;
    coast_phase.t_start = acc_phase.duration;
    coast_phase.length = calcPhaseLength(coast_phase);
    coast_phase.distance_p_start = acc_phase.length;
    phases.push_back(coast_phase);

    dec_phase.type = PhaseType2::ConstantDeacceleration2;
    dec_phase.t_start = coast_phase.duration + coast_phase.t_start;
    dec_phase.length = calcPhaseLength(dec_phase);
    dec_phase.distance_p_start = coast_phase.distance_p_start + coast_phase.length;
    phases.push_back(dec_phase);

    section.setPhases(phases);

    return section;
}

void ConstantAccelerationSolver2::calcPosAndVelSingleDoFLinear(double section_dof_length, const Phase2& phase,
                                                               double phase_distance_to_p_start, double t_phase,
                                                               double a_max_reduced, double v_max_reduced,
                                                               double& pos, double& vel) const
{
    double p_i{0}, v_i{0};
    if (phase.type == PhaseType2::ConstantAcceleration2) {
        p_i = 0.5 * a_max_reduced * std::pow(t_phase, 2);
        v_i = a_max_reduced * t_phase;
    } else if (phase.type == PhaseType2::ConstantVelocity2) {
        p_i = v_max_reduced * t_phase + phase_distance_to_p_start;
        v_i = v_max_reduced;
    } else if (phase.type == PhaseType2::ConstantDeacceleration2) {
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

void ConstantAccelerationSolver2::calcPosAndVelSection(double t_section, Section2& section, Result& result) const
{
    Point2 p_start = section.getStartPoint();
    Point2 p_end = section.getEndPoint();

    Point2 diff = p_end - p_start;

    Phase2& phase = section.getPhaseByTime(t_section);
    double t_phase = t_section - phase.t_start;

    for (const auto [group_name, symbols] : symbol_map_)
    {
        double a_max = section.getAccelerations()[group_name];
        double v_max = section.getVelocities()[group_name];

        if (symbols.isQuaternion()) {
            // handle that
        } else {
            ValueGroup diff_group = diff[group_name];
            double diff_magnitude = diff_group.getCartesianValues().norm();
            ValueGroup dir_group = diff_group * (1.0 / diff_magnitude);

            double pos_magnitude, vel_magnitude;
            calcPosAndVelSingleDoFLinear(diff_magnitude, phase, phase.components[group_name].distance_p_start, t_phase,
                                     a_max, v_max, pos_magnitude, vel_magnitude);

            ValueGroup& loc = result.getLocation()[group_name];
            ValueGroup& vel = result.getVelocity()[group_name];
            ValueGroup test = diff_group * pos_magnitude;
            loc = p_start[group_name] + test;
            vel = dir_group * vel_magnitude;
        }

    }
}
