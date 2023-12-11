[![url](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# SOTG - Simple Online Trajectory Genrator
Generate trajectories in realtime across multiple via points, constrained by velocity, acceleration and maximum path deviation.

## Build
catkin build

## Debug
Uncomment the following lines in CMakeList.txt when more debug output is desired
```
#add_definitions(-DDEBUG) #uncomment for additional debug output
#add_definitions(-DVERBOSE_TESTS) # print all calculated values
```
Uncomment this line in order to be able to step into the library using a debugger
```
#set(CMAKE_BUILD_TYPE Debug)
```
