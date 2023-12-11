
#include "sotg/sotg.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

#include "termcolor/termcolor.hpp"

using namespace SOTG;

std::vector<double> create_range(double start, double stop, double step);
double gradient(double x_past, double x_present, double delta);
Point gradient_point(Point x_past, Point x_present, double delta);
size_t sotgTest();

bool velocities_are_similar(const std::vector<Point>& velocities, const std::vector<Point>& numeric_velocities);
bool blend_velocities_are_similar(Json::Value& expected_blending_velocities,
                                  std::vector<std::map<std::string, double>>& debug_info_vec);
bool blend_distances_are_similar(Json::Value& expected_blending_distances,
                                 std::vector<std::map<std::string, double>>& debug_info_vec);

Path json_to_path(const Json::Value& locations, const Json::Value& orientations);
std::vector<SectionConstraint> section_constraints_from_json(const Json::Value& test_case);
std::vector<SegmentConstraint> segment_constraints_from_json(const Json::Value& test_case);

size_t sotgTest()
{
    std::ifstream file("tests.json");
    Json::Reader reader;
    Json::Value test_cases;
    reader.parse(file, test_cases);

    size_t num_failed_tests = 0;
    int test_case_counter = 1;
    for (auto& test_case : test_cases["test_cases_positive"]) {
        Json::Value& test_id = test_case["id"];
        std::cout << "\n *** Starting test case Nr." << test_id << " *** \n" << std::endl;

        TrajectoryGenerator tg;
        Path generalised_path = json_to_path(test_case["points"], test_case["orientations"]);

        std::vector<SectionConstraint> section_constraints = section_constraints_from_json(test_case);
        std::vector<SegmentConstraint> blend_constraints = segment_constraints_from_json(test_case);

        tg.resetPath(generalised_path, section_constraints, blend_constraints);

        std::vector<std::map<std::string, double>> debug_info_vec;
        debug_info_vec = tg.getDebugInfo();

        double frequency = 40.0;
        double Ts = 1.0 / frequency;
        double total_time = tg.getDuration();

        std::vector<double> tvec = create_range(0.0, total_time, Ts);

        // Edge case handling, appears on single waypoints
        if (total_time < Ts) {
            ++test_case_counter;
            continue;
        }

#ifdef VERBOSE_TESTS
        std::cout << std::left << std::setw(5) << "ID" << std::setw(10) << "Time" << std::setw(30) << "Position"
                  << std::setw(30) << "Velocity" << std::setw(30) << "Numeric Velocity" << std::endl;
#endif

        std::vector<Point> positions, velocities, numeric_velocities;
        Point last_pos;
        for (size_t i = 0; i < tvec.size(); ++i) {
            Point position, velocity;
            int segment_id;
            tg.calcPositionAndVelocity(tvec[i], position, velocity, segment_id);
            Point numeric_vel;
            if (i > 0) {
                numeric_vel = gradient_point(last_pos, position, Ts);
            }
            last_pos = position;
            positions.push_back(position);
            velocities.push_back(velocity);
            numeric_velocities.push_back(numeric_vel);

            std::ostringstream oss_pos;
            oss_pos << position;
            std::ostringstream oss_vel;
            oss_vel << velocity;
            std::ostringstream oss_num_vel;
            oss_num_vel << numeric_vel;

#ifdef VERBOSE_TESTS
            std::cout << std::setw(5) << segment_id << std::setw(10) << tvec[i] << std::setw(30) << oss_pos.str()
                      << std::setw(30) << oss_vel.str() << std::setw(30) << oss_num_vel.str() << std::endl;
#endif
        }

        std::cout << "\n * Begin tests for case Nr. " << test_case_counter << " * " << std::endl;

        size_t num_failed_local_tests = 0;
        size_t num_local_tests = 3;

        if (velocities_are_similar(velocities, numeric_velocities)) {
            std::cout << termcolor::green << "Test: analytic and numeric velocities match." << termcolor::reset
                      << std::endl;
        } else {
            std::cout << termcolor::red << "Test: analytic and numeric velocities do not match."
                      << termcolor::reset << std::endl;
            num_failed_local_tests++;
        }

        if (blend_velocities_are_similar(test_case["expected_blending_velocity_magnitudes"], debug_info_vec)) {
            std::cout << termcolor::green << "Test: Blending velocities match." << termcolor::reset << std::endl;
        } else {
            std::cout << termcolor::red << "Test: Blending velocities do not match." << termcolor::reset
                      << std::endl;
            num_failed_local_tests++;
        }

        if (blend_distances_are_similar(test_case["expected_blending_dist"], debug_info_vec)) {
            std::cout << termcolor::green << "Test: Blending distances match." << termcolor::reset << std::endl;
        } else {
            std::cout << termcolor::red << "Test: Blending distances do not match." << termcolor::reset
                      << std::endl;
            num_failed_local_tests++;
        }

        if (num_failed_local_tests > 0) {
            num_failed_tests++;

            std::cout << termcolor::red << "### Local tests finished " << num_local_tests - num_failed_local_tests
                      << " / " << num_local_tests << " passed ###" << termcolor::reset << std::endl;
        } else
            std::cout << termcolor::green << "### All local tests finished successfuly ###" << termcolor::reset
                      << std::endl;

        // if (test_case_counter >= 1)
        //     break;
        test_case_counter++;
    }

    if (num_failed_tests > 0)
        std::cout << termcolor::red << "### All Tests finished " << test_case_counter - 1 - num_failed_tests
                  << " / " << test_case_counter - 1 << " passed ###" << termcolor::reset << std::endl;
    else
        std::cout << termcolor::green << "### All tests finished successfuly ###" << termcolor::reset << std::endl;

    return num_failed_tests;
}

std::vector<double> create_range(double start, double stop, double step)
{
    std::vector<double> output;
    double last_value = start;
    while (last_value <= stop) {
        output.push_back(last_value);
        last_value += step;
    }

    return output;
}

double gradient(double x_past, double x_present, double delta) { return (x_present - x_past) / (delta); }

Point gradient_point(Point x_past, Point x_present, double delta)
{
    Point output;
    for (size_t i = 0; i < x_past.size(); ++i) {
        output.addValue(gradient(x_past[i], x_present[i], delta));
    }
    return output;
}

bool velocities_are_similar(const std::vector<Point>& velocities, const std::vector<Point>& numeric_velocities)
{
    double eps = 0.1;

    for (size_t i = 1; i < velocities.size() - 1; ++i) {
        for (size_t j = 0; j < velocities[i].size(); ++j) {
            if (std::abs(velocities[i][j] - numeric_velocities[i][j]) >= eps)
                return false;
        }
    }
    return true;
}

bool blend_velocities_are_similar(Json::Value& expected_blending_velocities,
                                  std::vector<std::map<std::string, double>>& debug_info_vec)
{
    Json::Value& expected_pre_values = expected_blending_velocities["pre"];
    Json::Value& expected_post_values = expected_blending_velocities["post"];

    double max_deviation = 0.1;

    for (Json::Value::ArrayIndex i = 0; i < expected_pre_values.size(); ++i) {
        double expected_pre_value = expected_pre_values[i].asDouble();
        double expected_post_value = expected_post_values[i].asDouble();
        double pre_value = debug_info_vec[i]["pre_blend_vel"];
        double post_value = debug_info_vec[i]["post_blend_vel"];
        if (!utility::nearlyEqual(expected_pre_value, pre_value, max_deviation)
            || !utility::nearlyEqual(expected_post_value, post_value, max_deviation)) {
            std::cout << termcolor::red << "Test: Blend Distances in segment " << i
                      << " does not match the expected values" << std::endl;
            std::cout << "Expected pre blend distance: " << expected_pre_value << ", received: " << pre_value
                      << std::endl;
            std::cout << "Expected post blend distance: " << expected_post_value << ", received: " << post_value
                      << termcolor::reset << std::endl;
            return false;
        }
    }
    return true;
}

bool blend_distances_are_similar(Json::Value& expected_blending_distances,
                                 std::vector<std::map<std::string, double>>& debug_info_vec)
{
    Json::Value& expected_pre_values = expected_blending_distances["pre"];
    Json::Value& expected_post_values = expected_blending_distances["post"];

    double max_deviation = 0.1;

    for (Json::Value::ArrayIndex i = 0; i < expected_pre_values.size(); ++i) {
        double expected_pre_value = expected_pre_values[i].asDouble();
        double expected_post_value = expected_post_values[i].asDouble();
        double pre_value = debug_info_vec[i]["pre_blend_dist"];
        double post_value = debug_info_vec[i]["post_blend_dist"];
        if (!utility::nearlyEqual(expected_pre_value, pre_value, max_deviation)
            || !utility::nearlyEqual(expected_post_value, post_value, max_deviation)) {
            std::cout << termcolor::red << "Test: Blend Distances in segment " << i
                      << " does not match the expected values" << std::endl;
            std::cout << "Expected pre blend distance: " << expected_pre_value << ", received: " << pre_value
                      << std::endl;
            std::cout << "Expected post blend distance: " << expected_post_value << ", received: " << post_value
                      << termcolor::reset << std::endl;
            return false;
        }
    }
    return true;
}

Path json_to_path(const Json::Value& locations, const Json::Value& orientations)
{
    Json::Value::ArrayIndex size_larger_array = 0;
    if (locations.size() >= orientations.size()) {
        size_larger_array = locations.size();
    } else {
        size_larger_array = orientations.size();
    }

    Path path;

    for (Json::Value::ArrayIndex i = 0; i < size_larger_array; i++) {
        Point point;

        if (i < locations.size()) {
            for (Json::Value::ArrayIndex j = 0; j < locations[i].size(); j++) {
                point.addValue(locations[i][j].asDouble());
            }
        } else if (locations.size() > 0) {
            const Json::Value& lastValue = locations[locations.size() - 1];
            for (Json::Value::ArrayIndex j = 0; j < lastValue.size(); j++) {
                point.addValue(lastValue[j].asDouble());
            }
        }

        if (i <= orientations.size()) {
            for (Json::Value::ArrayIndex j = 0; j < orientations[i].size(); j++) {
                point.addValue(orientations[i][j].asDouble());
            }
        } else if (orientations.size() > 0) {
            const Json::Value& lastValue = orientations[orientations.size() - 1];

            for (Json::Value::ArrayIndex j = 0; j < lastValue.size(); j++) {
                point.addValue(lastValue[j].asDouble());
            }
        }
        point.setOrientationIndex(locations[i].size());
        path.addPoint(point);
    }
    return path;
}

std::vector<SectionConstraint> section_constraints_from_json(const Json::Value& test_case)
{
    // assume the length of all constraints is sufficiant and the same
    // TODO handle mismatch
    int size_largest_array = test_case["max_accelerations"]["linear"].size();
    std::vector<SectionConstraint> constraints;

    for (int i = 0; i < size_largest_array; i++) {
        SectionConstraint constraint(test_case["max_accelerations"]["linear"][i].asDouble(),
                                     test_case["max_accelerations"]["angular"][i].asDouble(),
                                     test_case["max_velocities"]["linear"][i].asDouble(),
                                     test_case["max_velocities"]["angular"][i].asDouble());

        constraints.push_back(constraint);
    }

    return constraints;
}

std::vector<SegmentConstraint> segment_constraints_from_json(const Json::Value& test_case)
{
    // assume the length of all constraints is sufficiant and the same
    // TODO handle mismatch
    int size_largest_array = test_case["max_accelerations"]["linear"].size();
    std::vector<SegmentConstraint> constraints;

    for (int i = 0; i < size_largest_array - 1; i++) {
        SegmentConstraint constraint(test_case["blending_dist"][i].asDouble());

        constraints.push_back(constraint);
    }

    return constraints;
}