#include "sotg/constant_acceleration_solver.hpp"

using namespace SOTG;
using namespace detail;

int findIndexOfMax(const std::vector<double>& values);
double calcVecNorm(const std::vector<double>& vec);
double calcPhaseLength(const Phase& phase);

double calcPhaseLength(const Phase& phase)
{
    double sum = 0.0;
    for (auto& component : phase.components) {
        sum += std::pow(component.length, 2);
    }
    return std::sqrt(sum);
}

void ConstantAccelerationSolver::calcAccAndVelPerDoF(const Section& section, std::vector<double>& a_max_vec,
                                                     std::vector<double>& v_max_vec)
{
    Point p_start = section.getStartPoint();
    Point p_end = section.getEndPoint();

    double a_max_lin = section.getAccMaxLinear();
    double a_max_ang = section.getAccMaxAngular();
    double v_max_lin = section.getVelMaxLinear();
    double v_max_ang = section.getVelMaxAngular();

    Point dir_lin;
    Point diff_lin;

    Point dir_ang;
    Point diff_ang;
    if (p_start.getOrientationIndex() != -1) {
        for (int i = 0; i < p_start.getOrientationIndex(); i++) {
            diff_lin.addValue(p_end[i] - p_start[i]);
        }
        double diff_lin_mag = diff_lin.norm();

        if (utility::nearlyZero(diff_lin_mag)) {
            dir_lin.zeros(diff_lin.size());
        } else {
            dir_lin = diff_lin / diff_lin_mag;
        }

        for (size_t i = p_start.getOrientationIndex(); i < p_start.size(); i++) {
            diff_ang.addValue(p_end[i] - p_start[i]);
        }
        double diff_ang_mag = diff_ang.norm();

        if (utility::nearlyZero(diff_ang_mag)) {
            dir_ang.zeros(diff_ang.size());
        } else {
            dir_ang = diff_ang / diff_ang_mag;
        }
    } else {
        for (size_t i = 0; i < p_start.size(); i++) {
            diff_lin.addValue(p_end[i] - p_start[i]);
        }
        double diff_lin_mag = diff_lin.norm();

        if (utility::nearlyZero(diff_lin_mag)) {
            dir_lin.zeros(diff_lin.size());
        } else {
            dir_lin = diff_lin / diff_lin_mag;
        }
    }

    for (auto& dir : dir_lin) {
        a_max_vec.push_back(std::abs(a_max_lin * dir));
        v_max_vec.push_back(std::abs(v_max_lin * dir));
    }
    for (auto& dir : dir_ang) {
        a_max_vec.push_back(std::abs(a_max_ang * dir));
        v_max_vec.push_back(std::abs(v_max_ang * dir));
    }
}

void ConstantAccelerationSolver::calcPhaseTimeAndDistance(double& a_max, double& v_max, double L_total,
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
}

int findIndexOfMax(const std::vector<double>& values)
{
    int index_max = 0;
    double max_value = values[0];
    for (size_t i = 1; i < values.size(); i++) {
        if (values[i] > max_value) {
            max_value = values[i];
            index_max = i;
        }
    }

    return index_max;
}

void ConstantAccelerationSolver::calcTotalTimeAndDistanceSingleDoF(double& a_max, double& v_max,
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

        logger_.log("[Section Nr." + std::to_string(current_section_id)
                        + "] KinematicSolver: Decreasing maximum velocity from " + std::to_string(v_max) + " to "
                        + std::to_string(v_max_reduced),
                    Logger::INFO);

        calcTotalTimeAndDistanceSingleDoF(a_max, v_max_reduced, total_length, total_time);
        v_max = v_max_reduced;
    }
}

void ConstantAccelerationSolver::calcTimesAndLengthsMultiDoF(Phase& acc_phase, Phase& coast_phase,
                                                             Phase& dec_phase,
                                                             std::vector<double>& total_time_per_dof,
                                                             std::vector<double>& total_length_per_dof, Point diff,
                                                             std::vector<double>& a_max_vec,
                                                             std::vector<double>& v_max_vec)
{
    for (size_t i = 0; i < diff.size(); i++) {
        double total_length_dof = std::abs(diff[i]);
        double total_time_dof = 0.0;

        calcTotalTimeAndDistanceSingleDoF(a_max_vec[i], v_max_vec[i], total_length_dof, total_time_dof);

        total_time_per_dof.push_back(total_time_dof);
        total_length_per_dof.push_back(total_length_dof);
    }

    int index_slowest_dof = findIndexOfMax(total_time_per_dof);

    // phase sync
    // https://theses.hal.science/tel-01285383/document p.62 ff.
    for (size_t i = 0; i < diff.size(); i++) {
        double lambda = 0.0;  // lambda is defined in theses
        if (!utility::nearlyZero(total_length_per_dof[index_slowest_dof])) {
            lambda = total_length_per_dof[i] / total_length_per_dof[index_slowest_dof];
        }

        double acc_scaling_factor = 0.0;
        double vel_scaling_factor = 0.0;
        if (!utility::nearlyZero(a_max_vec[i])) {
            acc_scaling_factor = a_max_vec[index_slowest_dof] / a_max_vec[i];
        }
        if (!utility::nearlyZero(v_max_vec[i])) {
            vel_scaling_factor = v_max_vec[index_slowest_dof] / v_max_vec[i];
        }

        a_max_vec[i] *= lambda * acc_scaling_factor;
        v_max_vec[i] *= lambda * vel_scaling_factor;

        double section_length = std::abs(diff[i]);
        PhaseDoF acc_phase_single_dof, coast_phase_single_dof, dec_phase_single_dof;
        calcPhaseTimeAndDistance(a_max_vec[i], v_max_vec[i], section_length, acc_phase_single_dof,
                                 coast_phase_single_dof, dec_phase_single_dof);

        acc_phase.components.push_back(acc_phase_single_dof);
        coast_phase.components.push_back(coast_phase_single_dof);
        dec_phase.components.push_back(dec_phase_single_dof);
    }

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

Section ConstantAccelerationSolver::calcSection(Point& p_start_ref, Point& p_end_ref,
                                                SectionConstraint constraint_copy, size_t section_id)
{
    current_section_id = section_id;

    Section section(p_start_ref, p_end_ref, constraint_copy, section_id);

    std::vector<double> reduced_acceleration_per_dof;
    std::vector<double> reduced_velocity_per_dof;

    calcAccAndVelPerDoF(section, reduced_acceleration_per_dof, reduced_velocity_per_dof);

    Point p_start = section.getStartPoint();
    Point p_end = section.getEndPoint();
    Point diff = p_end - p_start;

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

double calcVecNorm(const std::vector<double>& vec)  // into util
{
    double sum = 0.0;
    for (auto& component : vec) {
        sum += std::pow(component, 2);
    }
    return std::sqrt(sum);
}

void ConstantAccelerationSolver::calcPreBlendParams(double blending_dist_pre, const Section& pre_section,
                                                    Point& A_blend, double& T_blend,
                                                    double& vel_pre_blend_magnitude,
                                                    double& absolute_blend_start_time_with_shift)
{
    const Point& A = pre_section.getStartPoint();
    const Point& AB = pre_section.getDifference();
    double length_AB = pre_section.getLength();

    A_blend = A + AB * ((length_AB - blending_dist_pre) / length_AB);

    double absolute_blend_start_time_without_shift;
    Point vel_pre_blend;
    calcVelAndTimeByDistance(pre_section, length_AB - blending_dist_pre, vel_pre_blend,
                             absolute_blend_start_time_without_shift);
    vel_pre_blend_magnitude = vel_pre_blend.norm();

    if (!utility::nearlyZero(vel_pre_blend_magnitude)) {
        T_blend = 2 * blending_dist_pre / vel_pre_blend_magnitude;
    } else {
        T_blend = 0.0;
    }

    absolute_blend_start_time_with_shift = absolute_blend_start_time_without_shift - pre_section.getTimeShift();
}

void ConstantAccelerationSolver::calcPostBlendParams(double blending_dist_post, const Section& post_section,
                                                     Point& C_blend, double& T_blend,
                                                     double& vel_post_blend_magnitude,
                                                     double& absolute_blend_end_time_without_shift)
{
    const Point& B = post_section.getStartPoint();
    const Point& BC = post_section.getDifference();
    double length_BC = post_section.getLength();

    if (utility::nearlyZero(length_BC)) {
        C_blend = B;
    } else {
        C_blend = B + BC * (blending_dist_post / length_BC);
    }

    Point vel_post_blend;
    calcVelAndTimeByDistance(post_section, blending_dist_post, vel_post_blend,
                             absolute_blend_end_time_without_shift);
    vel_post_blend_magnitude = vel_post_blend.norm();

    if (!utility::nearlyZero(vel_post_blend_magnitude)) {
        T_blend = 2 * blending_dist_post / vel_post_blend_magnitude;
    } else {
        T_blend = 0.0;
    }
}

void ConstantAccelerationSolver::calcSecondBlendingDist(double T_blend, double T_acc_second,
                                                        double a_max_magnitude_second,
                                                        double vel_first_blend_magnitude,
                                                        double blending_dist_first, double& blending_dist_second,
                                                        [[maybe_unused]] size_t segment_id)
{
    if (utility::nearlyZero(vel_first_blend_magnitude)) {
        blending_dist_second = 0.0;
        return;
    }

    // If "second" values are post values, the "first" values will be pre values. And vice versa
    if (T_blend < T_acc_second) {
        logger_.log("[Segment Nr." + std::to_string(segment_id) + "] Blending into acceleration phase",
                    Logger::DEBUG);

        blending_dist_second = 2 * std::pow(blending_dist_first, 2) * a_max_magnitude_second
                               / std::pow(vel_first_blend_magnitude, 2);
    } else {
        logger_.log("[Segment Nr." + std::to_string(segment_id) + "] Blending into constant velocity phase",
                    Logger::DEBUG);

        blending_dist_second
            = blending_dist_first * a_max_magnitude_second * T_acc_second / vel_first_blend_magnitude;
    }
}

void ConstantAccelerationSolver::calcSegmentPreparations(const Section& pre_section, const Section& post_section,
                                                         std::vector<double>& a_max_post,
                                                         std::vector<double>& a_max_pre,
                                                         double& L_acc_magnitude_post, double& T_acc_post,
                                                         double& T_acc_pre)
{
    const Phase& acc_phase_post = post_section.getPhaseByType(PhaseType::ConstantAcceleration);
    T_acc_post = acc_phase_post.duration;
    a_max_post = post_section.getAdaptedAcceleration();

    const Phase& acc_phase_pre = pre_section.getPhaseByType(PhaseType::ConstantAcceleration);
    T_acc_pre = acc_phase_pre.duration;
    a_max_pre = pre_section.getAdaptedAcceleration();

    L_acc_magnitude_post = 0.0;
    for (auto& component : acc_phase_post.components) {
        L_acc_magnitude_post += std::pow(component.length, 2);
    }
    L_acc_magnitude_post = std::sqrt(L_acc_magnitude_post);
}

bool ConstantAccelerationSolver::isBlendAccelerationTooHigh(const std::vector<double>& a_max_blend,
                                                            const double& T_blend,
                                                            const double& vel_pre_blend_magnitude,
                                                            const double& vel_post_blend_magnitude,
                                                            const Section& pre_section,
                                                            const Section& post_section, size_t segment_id)
{
    const Point& dir_AB = pre_section.getDirection();
    const Point& dir_BC = post_section.getDirection();

    Point blend_acc, blend_acc_linear, blend_acc_angular;
    double blend_acc_linear_mag, blend_acc_angular_mag;
    double a_max_linear_mag, a_max_angular_mag;
    std::vector<double> a_max_linear, a_max_angular;
    if (!utility::nearlyZero(T_blend)) {
        blend_acc = (dir_BC * vel_post_blend_magnitude - dir_AB * vel_pre_blend_magnitude) / T_blend;
        size_t o_index = blend_acc.getOrientationIndex();

        for (size_t i = 0; i < blend_acc.size(); ++i) {
            if (i < o_index) {
                blend_acc_linear.addValue(blend_acc[i]);
                a_max_linear.push_back(a_max_blend[i]);
            } else {
                blend_acc_angular.addValue(blend_acc[i]);
                a_max_angular.push_back(a_max_blend[i]);
            }
        }
        blend_acc_linear_mag = blend_acc_linear.norm();
        blend_acc_angular_mag = blend_acc_angular.norm();

        a_max_linear_mag = calcVecNorm(a_max_linear);
        a_max_angular_mag = calcVecNorm(a_max_angular);

    } else {
        blend_acc_angular_mag = 0.0;
        blend_acc_linear_mag = 0.0;

        a_max_linear_mag = 0.0;
        a_max_angular_mag = 0.0;
    }

    if (blend_acc_linear_mag > a_max_linear_mag && !utility::nearlyZero(blend_acc_linear_mag)) {
        logger_.log("[Segment Nr." + std::to_string(segment_id) + "] Eceeding maximum linear acceleration!",
                    Logger::WARNING);
        logger_.log("[Segment Nr." + std::to_string(segment_id) + "] Linear Acceleration magnitude would be "
                        + std::to_string(blend_acc_linear_mag) + " m/s^2, but only "
                        + std::to_string(a_max_linear_mag) + " m/s^2 is allowed",
                    Logger::WARNING);
        logger_.log("[Segment Nr." + std::to_string(segment_id) + "] Deactivating blending in this segment",
                    Logger::WARNING);

        return true;
    } else if (blend_acc_angular_mag > a_max_angular_mag && !utility::nearlyZero(blend_acc_angular_mag)) {
        logger_.log("[Segment Nr." + std::to_string(segment_id) + "] Eceeding maximum angular acceleration!",
                    Logger::WARNING);
        logger_.log("[Segment Nr." + std::to_string(segment_id) + "] Angular Acceleration magnitude would be "
                        + std::to_string(blend_acc_angular_mag) + " 1/s^2, but only "
                        + std::to_string(a_max_angular_mag) + " 1/s^2 is allowed",
                    Logger::WARNING);
        logger_.log("[Segment Nr." + std::to_string(segment_id) + "] Deactivating blending in this segment",
                    Logger::WARNING);

        return true;
    } else
        return false;
}

void ConstantAccelerationSolver::setNoBlendingParams(const Section& pre_section, const Section& post_section,
                                                     double& T_blend, double& t_abs_start_blend_with_shift,
                                                     double& t_abs_end_blend_without_shift, Point& A_blend,
                                                     Point& C_blend, double& vel_pre_blend_magnitude,
                                                     double& vel_post_blend_magnitude)
{
    T_blend = 0.0;
    t_abs_start_blend_with_shift = pre_section.getEndTime() - pre_section.getTimeShift();
    t_abs_end_blend_without_shift = post_section.getStartTime();
    A_blend = post_section.getStartPoint();
    C_blend = A_blend;
    vel_pre_blend_magnitude = 0.0;
    vel_post_blend_magnitude = 0.0;
}

std::shared_ptr<BlendSegment>
ConstantAccelerationSolver::calcBlendSegment(Section& pre_section, Section& post_section,
                                             const SegmentConstraint& constraint, size_t segment_id,
                                             std::map<std::string, double>& debug_output)
{
    /* Blending from A' to C' across B with constant acceleration
       https://www.diag.uniroma1.it/~deluca/rob1_en/14_TrajectoryPlanningCartesian.pdf

              B
            /   \
          A'.-'-. C'
        /           \
      A               C

    */

    current_segment_id = segment_id;

    double length_AB = pre_section.getLength();
    double length_BC = post_section.getLength();
    double a_max_magnitude_post, a_max_magnitude_pre, L_acc_magnitude_post, T_acc_post, T_acc_pre;

    std::vector<double> a_max_post, a_max_pre, a_max_blend;

    calcSegmentPreparations(pre_section, post_section, a_max_post, a_max_pre, L_acc_magnitude_post, T_acc_post,
                            T_acc_pre);

    a_max_magnitude_post = calcVecNorm(a_max_post);
    a_max_magnitude_pre = calcVecNorm(a_max_pre);

    if (a_max_magnitude_post >= a_max_magnitude_pre) {
        a_max_blend = a_max_post;
    } else {
        a_max_blend = a_max_pre;
    }

    double blending_dist_pre = constraint.getBlendDistance();

    if (blending_dist_pre > length_AB / 2) {
        logger_.log("[Segment Nr." + std::to_string(segment_id) + "] Pre blending distance is cropped",
                    Logger::INFO);

        blending_dist_pre = length_AB / 2;
    }

    Point A_blend;
    double T_blend, vel_pre_blend_magnitude, t_abs_start_blend_with_shift;
    calcPreBlendParams(blending_dist_pre, pre_section, A_blend, T_blend, vel_pre_blend_magnitude,
                       t_abs_start_blend_with_shift);

    double blending_dist_post = 0.0;
    calcSecondBlendingDist(T_blend, T_acc_post, a_max_magnitude_post, vel_pre_blend_magnitude, blending_dist_pre,
                           blending_dist_post, segment_id);

    Point vel_post_blend;
    Point C_blend;
    double vel_post_blend_magnitude;
    double t_abs_end_blend_without_shift;

    if (blending_dist_post >= length_BC / 2) {
        blending_dist_post = length_BC / 2;

        calcPostBlendParams(blending_dist_post, post_section, C_blend, T_blend, vel_post_blend_magnitude,
                            t_abs_end_blend_without_shift);

        calcSecondBlendingDist(T_blend, T_acc_pre, a_max_magnitude_pre, vel_post_blend_magnitude,
                               blending_dist_post, blending_dist_pre, segment_id);

        calcPreBlendParams(blending_dist_pre, pre_section, A_blend, T_blend, vel_pre_blend_magnitude,
                           t_abs_start_blend_with_shift);

    } else {
        calcPosAndVelSection(T_blend, post_section, C_blend, vel_post_blend);
        vel_post_blend_magnitude = vel_post_blend.norm();

        calcVelAndTimeByDistance(post_section, blending_dist_post, vel_post_blend, t_abs_end_blend_without_shift);
    }

    if (isBlendAccelerationTooHigh(a_max_blend, T_blend, vel_pre_blend_magnitude, vel_post_blend_magnitude,
                                   pre_section, post_section, segment_id)) {
        setNoBlendingParams(pre_section, post_section, T_blend, t_abs_start_blend_with_shift,
                            t_abs_end_blend_without_shift, A_blend, C_blend, vel_pre_blend_magnitude,
                            vel_post_blend_magnitude);
    }

    // Absolute Time denotes a point in time relative to the first waypoint in the path
    // Time shift is the result of blending in between the sections, thereby reducing the time until the next
    // section is reached There are three other time frames to be aware of: Section, Segment and Phase relative
    // times denoted as t_section, t_segment and t_phase respectively
    double t_abs_end_blend_with_shift = t_abs_start_blend_with_shift + T_blend;
    double time_shift = t_abs_end_blend_without_shift - t_abs_end_blend_with_shift;

    post_section.setTimeShift(time_shift);

    std::shared_ptr<BlendSegment> segment(new BlendSegment(pre_section, post_section, constraint,
                                                           vel_pre_blend_magnitude, vel_post_blend_magnitude,
                                                           T_blend, t_abs_start_blend_with_shift));

    segment->setStartPoint(A_blend);
    segment->setEndPoint(C_blend);
    segment->setID(segment_id);

    debug_output["pre_blend_dist"] = blending_dist_pre;
    debug_output["post_blend_dist"] = blending_dist_post;
    debug_output["pre_blend_vel"] = vel_pre_blend_magnitude;
    debug_output["post_blend_vel"] = vel_post_blend_magnitude;

    return segment;
}

void ConstantAccelerationSolver::calcPosAndVelSingleDoFLinear(double section_dof_length, const Phase& phase,
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

void ConstantAccelerationSolver::calcVelAndTimeByDistance(const Section& section, double distance,
                                                          Point& velocity_per_dof, double& t_abs)
{
    const Point& dir = section.getDirection();

    std::vector<double> a_max_vec = section.getAdaptedAcceleration();
    std::vector<double> v_max_vec = section.getAdaptedVelocity();

    const Phase& phase = section.getPhaseByDistance(distance);

    // calculate the time it takes to cross "distance" in phase space
    double T_distance = 0;
    for (size_t i = 0; i < phase.components.size(); ++i) {
        double distance_dof = std::abs(dir[i]) * distance - phase.components[i].distance_p_start;

        if (phase.type == PhaseType::ConstantAcceleration) {
            if (!utility::nearlyZero(a_max_vec[i])) {
                T_distance = std::sqrt(2 * distance_dof / a_max_vec[i]);
            }
        } else if (phase.type == PhaseType::ConstantVelocity) {
            if (!utility::nearlyZero(v_max_vec[i])) {
                T_distance = distance_dof / v_max_vec[i];
            }
        } else if (phase.type == PhaseType::ConstantDeacceleration) {
            if (!utility::nearlyZero(a_max_vec[i])) {
                double radicand = std::pow(v_max_vec[i] / a_max_vec[i], 2) - 2.0 / a_max_vec[i] * distance_dof;
                if (utility::nearlyZero(radicand)) {
                    radicand = 0.0;
                }
                if (radicand >= 0.0) {
                    T_distance = v_max_vec[i] / a_max_vec[i] - std::sqrt(radicand);
                }
            }
        } else {
            throw std::runtime_error("Error::KinematicSolver: Unrecognized phase type");
        }

        if (!utility::nearlyZero(T_distance)) {
            break;
        }
    }

    double t_phase = T_distance;
    double t_section_blend_start = t_phase + phase.t_start;
    t_abs = t_section_blend_start + section.getStartTime();

    // calculate velocity per dof at time when distance is reached

    double section_length = section.getLength();
    for (size_t i = 0; i < dir.size(); ++i) {
        double section_dof_length = dir[i] * section_length;
        double _, velocity;
        calcPosAndVelSingleDoFLinear(section_dof_length, phase, phase.components[i].distance_p_start, t_phase,
                                     a_max_vec[i], v_max_vec[i], _, velocity);
        velocity_per_dof.addValue(velocity);
    }
}

void ConstantAccelerationSolver::calcPosAndVelLinearSegment(double t_section, const LinearSegment& segment,
                                                            Point& pos, Point& vel) const
{
    const Section& section = segment.getSection();

    calcPosAndVelSection(t_section, section, pos, vel);
}

void ConstantAccelerationSolver::calcPosAndVelBlendSegment(double t_segment, const BlendSegment& segment,
                                                           Point& pos, Point& vel) const
{
    Point dir_AB = segment.getPreBlendDirection();
    Point dir_BC = segment.getPostBlendDirection();

    double duration = segment.getDuration();

    Point A_blend = segment.getStartPoint();
    double vel_blend_pre_magnitude = segment.getPreBlendVelocityMagnitude();
    double vel_blend_post_magnitude = segment.getPostBlendVelocityMagnitude();

    pos = A_blend + dir_AB * vel_blend_pre_magnitude * t_segment
          + (dir_BC * vel_blend_post_magnitude - dir_AB * vel_blend_pre_magnitude) * std::pow(t_segment, 2)
                / (2 * duration);

    vel = dir_AB * vel_blend_pre_magnitude
          + (dir_BC * vel_blend_post_magnitude - dir_AB * vel_blend_pre_magnitude) * t_segment / duration;
    pos.setOrientationIndex(dir_AB.getOrientationIndex());
    vel.setOrientationIndex(dir_AB.getOrientationIndex());
}

void ConstantAccelerationSolver::calcPosAndVelSection(double t_section, const Section& section, Point& pos,
                                                      Point& vel) const
{
    Point p_start = section.getStartPoint();
    Point p_end = section.getEndPoint();
    Point diff = p_end - p_start;

    const std::vector<double>& a_max_vec = section.getAdaptedAcceleration();
    const std::vector<double>& v_max_vec = section.getAdaptedVelocity();

    const Phase& phase = section.getPhaseByTime(t_section);
    double t_phase = t_section - phase.t_start;
    for (size_t i = 0; i < p_start.size(); i++) {
        double pos_relative_magnitude, vel_magnitude;
        calcPosAndVelSingleDoFLinear(std::abs(diff[i]), phase, phase.components[i].distance_p_start, t_phase,
                                     a_max_vec[i], v_max_vec[i], pos_relative_magnitude, vel_magnitude);

        double pos_component = p_start[i] + pos_relative_magnitude * diff[i];
        double vel_component = vel_magnitude * utility::sign(diff[i]);

        pos.addValue(pos_component);
        vel.addValue(vel_component);
    }
    pos.setOrientationIndex(p_start.getOrientationIndex());
    vel.setOrientationIndex(p_start.getOrientationIndex());
}
