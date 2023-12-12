[![url](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# SOTG - Simple Online Trajectory Genrator
Generate trajectories in realtime across multiple via points, constrained by velocity, acceleration and maximum path deviation.


## Requirments
The math library Eigen version 3.3 or higher is required
``` bash
apt install libeigen3-dev
```

## Build and Install
``` bash
git clone https://gitlab.cc-asp.fraunhofer.de/jsk/simple_online_trajectory_generator.git
cd simple_online_trajectory_generator

mkdir build
cd build

cmake ..
make
```

## Example
### Cartesian Trajectory
``` cpp
#include "sotg/sotg.hpp"

...

// Setup the path once

Eigen::VectorXd v1(6), v2(6), v3(6);
v1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//     x    y    z   roll yawn tilt  The order can be different but orientation values must always be placed after the locations, e.g. y,z,x,tilt,roll,yawn

v2 << 0.0, 1.0, 0.0, 0.0, 1.0, 0.0;
v3 << 1.0, 1.0, 0.0, 0.0, 1.0, 0.0;

SOTG::Point p1(v1);
p1.setOrientationIndex(3); // index of the first occuring angle value
SOTG::Point p2(v2);
p2.setOrientationIndex(3);
SOTG::Point p3(v3);
p3.setOrientationIndex(3);

SOTG::Path path;
path.addPoint(p1);
path.addPoint(p2);
path.addPoint(p3);

std::vector<SOTG::SectionConstraint> section_constraints;
SOTG::SectionConstraint s1(1.0, 2.0, 1.0, 1.0); // max_lin_acceleration, max_ang_acceleration, max_lin_velocity, max_ang_velocity
SOTG::SectionConstraint s2(1.0, 2.0, 1.0, 1.0);
section_constraints.push_back(s1);
section_constraints.push_back(s2); // must be section_constraints.size() == path.size() - 1

std::vector<SOTG::SegmentConstraint> segment_constraint;
SOTG::SegmentConstraint sc(0.5); // max blend distance
segment_constaints.push_back(sc); // must be segment_constraints.size() == path.size() - 2

SOTG::TrajectoryGenerator trajectory_generator
trajectory_generator.resetPath(path, section_constraints, segment_constraints);

double duration = trajectory_generator.getDuration();

...
// This gets called every tick, a tick is a point in time between 0 and the calculated duration of the trajectory

SOTG::Point position, velocity;
trajectory_generator.calcPositionAndVelocity(tick, position, velocity);

// position and velocity contain the values for this tick

```

## Debug
Uncomment the following lines in CMakeList.txt when more debug output is desired
``` cmake
add_definitions(-DDEBUG)
add_definitions(-DVERBOSE_TESTS)
```
