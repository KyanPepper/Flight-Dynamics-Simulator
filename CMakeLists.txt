
cmake_minimum_required(VERSION 3.20)
project(FlightSim6DOF CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Main executable
find_package(Qt6 COMPONENTS Widgets REQUIRED)

add_executable(flightSim
  src/main_qt.cpp
  src/physics.cpp
  src/ui_horizon_qt.cpp
)
target_include_directories(flightSim PRIVATE include)
target_link_libraries(flightSim Qt6::Widgets pthread)

# Testing executables
option(BUILD_TESTS "Build test programs" ON)

if(BUILD_TESTS)
    # Unit tests
    add_executable(test_physics
        test/test_physics.cpp
        src/physics.cpp
    )
    target_include_directories(test_physics PRIVATE include)
    # Add test command
    enable_testing()
    add_test(NAME PhysicsTests COMMAND test_physics)
endif()