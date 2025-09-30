#pragma once
#include <array>
#include <atomic>

/**
 * All the fixed properties of our aircraft (like weight, size, etc.)
 * These don't change during flight
 */
struct AircraftParameters
{
    // Basic physical properties
    double mass = 1200.0;       // How heavy the aircraft is (kg)
    double gravity = 9.81;      // Earth's gravity (m/s²)
    double air_density = 1.225; // How thick the air is (kg/m³)

    // How hard it is to rotate the aircraft around each axis
    double roll_inertia = 1200.0;  // Resistance to rolling motion (kg·m²)
    double pitch_inertia = 1500.0; // Resistance to pitching motion (kg·m²)
    double yaw_inertia = 1800.0;   // Resistance to yawing motion (kg·m²)

    // Wing and aerodynamic properties
    double wing_area = 16.0;             // Total wing surface area (m²)
    double base_lift_coefficient = 0.2;  // Lift when wings are level
    double lift_slope = 5.5;             // How much lift increases with angle of attack
    double base_drag_coefficient = 0.03; // Minimum drag (parasitic drag)
    double induced_drag_factor = 0.07;   // Extra drag from making lift

    // How effective the controls are (bigger = more responsive)
    double aileron_power = 3000.0;  // Roll control strength (N·m per unit input)
    double elevator_power = 5000.0; // Pitch control strength (N·m per unit input)
    double rudder_power = 2000.0;   // Yaw control strength (N·m per unit input)

    double max_thrust = 4000.0; // Maximum engine thrust (N)

    // ADDED: Aerodynamic damping coefficients (critical for stability)
    double roll_damping = 2.0;  // Roll damping coefficient (INCREASED)
    double pitch_damping = 3.0; // Pitch damping coefficient (INCREASED)
    double yaw_damping = 1.5;   // Yaw damping coefficient (INCREASED)

    // Stall characteristics
    double stall_angle = 0.26;         // ~15 degrees stall AoA (radians)
    double max_lift_coefficient = 1.4; // Maximum CL before stall
};

/**
 * What the pilot is doing with the controls right now
 * All values are normalized: -1 to +1 for control surfaces, 0 to 1 for throttle
 */
struct PilotInputs
{
    double aileron = 0.0;   // Roll control stick position [-1 = left, +1 = right]
    double elevator = 0.05; // Pitch control stick position [-1 = nose down, +1 = nose up]
    double rudder = 0.0;    // Yaw pedal position [-1 = left, +1 = right]
    double throttle = 0.6;  // Engine power setting [0 = idle, 1 = full power]
};

/**
 * Complete description of where the aircraft is and what it's doing right now
 * This is everything we need to know about the aircraft's motion
 */
struct AircraftState
{
    // Where the aircraft is in the world (meters)
    double north_position = 0.0; // Distance north of starting point
    double east_position = 0.0;  // Distance east of starting point
    double down_position = 0.0;  // Distance below starting point (negative = altitude)

    // How fast the aircraft is moving in its own coordinate system (m/s)
    double forward_velocity = 55.0; // Speed forward (start with some airspeed)
    double side_velocity = 0.0;     // Speed sideways (usually small)
    double vertical_velocity = 0.0; // Speed up/down

    // Aircraft orientation in space (radians)
    double roll_angle = 0.0;   // Banking left/right (0 = wings level)
    double pitch_angle = 0.03; // Nose up/down (positive = nose up, slight climb)
    double yaw_angle = 0.0;    // Compass heading (0 = north)

    // How fast the aircraft is rotating around its center (rad/s)
    double roll_rate = 0.0;  // Rolling motion (around forward axis)
    double pitch_rate = 0.0; // Pitching motion (around side axis)
    double yaw_rate = 0.0;   // Yawing motion (around vertical axis)

    // Simulation time
    double time = 0.0;
};

/**
 * All the rates of change of the aircraft state
 * These are the "derivatives" - they tell us how fast everything is changing
 */
struct StateDerivatives
{
    // How fast position is changing (this is just velocity)
    double north_velocity = 0.0;
    double east_velocity = 0.0;
    double down_velocity = 0.0;

    // How fast velocity is changing (this is acceleration)
    double forward_acceleration = 0.0;
    double side_acceleration = 0.0;
    double vertical_acceleration = 0.0;

    // How fast orientation angles are changing
    double roll_rate = 0.0;
    double pitch_rate = 0.0;
    double yaw_rate = 0.0;

    // How fast rotation rates are changing (angular acceleration)
    double roll_acceleration = 0.0;
    double pitch_acceleration = 0.0;
    double yaw_acceleration = 0.0;

    // Default constructor zeros everything
    StateDerivatives() = default;
};

/**
 * Main physics calculator for aircraft simulation
 * This class knows how to calculate all the forces and update the aircraft state
 */
class AircraftPhysics
{
private:
    AircraftParameters params; // Aircraft specifications
    PilotInputs inputs;        // Current control inputs

public:
    AircraftPhysics() = default;

    // Set the aircraft's physical properties
    void setAircraftParameters(const AircraftParameters &new_params)
    {
        params = new_params;
    }

    // Update what the pilot is doing with the controls
    void setPilotInputs(const PilotInputs &new_inputs)
    {
        inputs = new_inputs;
    }

    // Calculate how fast everything is changing right now
    StateDerivatives calculateDerivatives(const AircraftState &current_state) const;

    // Move the simulation forward by one small time step
    AircraftState integrateOneStep(const AircraftState &current_state, double time_step) const;
};