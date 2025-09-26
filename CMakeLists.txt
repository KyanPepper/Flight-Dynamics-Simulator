# This file tells CMake how to build our project
cmake_minimum_required(VERSION 3.20)
project(FlightSim6DOF CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 COMPONENTS Widgets REQUIRED)


add_executable(flightSim
  src/main_qt.cpp
  src/physics.cpp
  src/ui_horizon_qt.cpp
)

# Where to find header files
target_include_directories(flightSim PRIVATE include)

# Link with Qt and pthread (for threading)
target_link_libraries(flightSim Qt6::Widgets pthread)