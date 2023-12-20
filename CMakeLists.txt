cmake_minimum_required(VERSION 3.0.2)
project(sotg)

if ( CMAKE_COMPILER_IS_GNUCXX )
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconversion -Wsign-conversion")  # too noisy for now
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wshadow -Wnon-virtual-dtor -pedantic -Wpedantic -Wnull-dereference")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wcast-align -Wformat-nonliteral -Wformat=2 -Winvalid-pch -Wmissing-declarations -Wmissing-format-attribute -Wredundant-decls -Wswitch-default -Wswitch-enum -Wno-comment")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wold-style-cast -Wunused -Woverloaded-virtual -Wuseless-cast")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wimplicit-fallthrough")
endif()

add_compile_options(-std=c++17)

###########
## Build ##
###########

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/trajectory_generator.cpp
  src/constant_acceleration_solver.cpp
  src/path_manager.cpp
  src/point.cpp
  src/path.cpp
  src/section_constraint.cpp
  src/section.cpp
  src/linear_segment.cpp
  src/blend_segment.cpp
  src/segment_constraint.cpp
  src/utility_functions.cpp
)

target_link_libraries (${PROJECT_NAME} Eigen3::Eigen)

install(TARGETS sotg DESTINATION lib)

  ###########
  ## Debug ##
  ###########

#add_definitions(-DDEBUG) #uncomment for additional debug output
#add_definitions(-DVERBOSE_TESTS) # print all calculated values