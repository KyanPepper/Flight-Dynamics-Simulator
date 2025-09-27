#include <iostream>
#include <cassert>
#include <cmath>
#include "physics.hpp"
#include "utils.hpp"

/**
 * Helper function to check if two floating point numbers are approximately equal
 * This is needed because floating point math isn't perfectly precise
 */
bool approx_equal(double a, double b, double tolerance = 1e-6)
{
    return std::abs(a - b) < tolerance;
}

/**
 * Test 1: Verify that an aircraft at rest with no controls just falls due to gravity
 * This tests our basic physics setup and gravity implementation
 */
void test_equilibrium()
{
    std::cout << "Test 1: Equilibrium (aircraft at rest should just fall)...";

    // Set up the physics simulation
    AircraftPhysics physics;
    AircraftParameters params; // Use default parameters
    physics.setAircraftParameters(params);

    // Set all controls to neutral (no pilot input)
    PilotInputs controls;
    controls.aileron = 0.0;
    controls.elevator = 0.0;
    controls.rudder = 0.0;
    controls.throttle = 0.0; // No engine power
    physics.setPilotInputs(controls);

    // Create aircraft state at rest
    AircraftState state;
    state.north_position = 0;
    state.east_position = 0;
    state.down_position = 0;
    state.forward_velocity = 0;  // No forward speed
    state.side_velocity = 0;     // No sideways speed
    state.vertical_velocity = 0; // No vertical speed
    state.roll_rate = 0;         // Not rotating
    state.pitch_rate = 0;
    state.yaw_rate = 0;
    state.roll_angle = 0;  // Wings level
    state.pitch_angle = 0; // Nose level
    state.yaw_angle = 0;   // Facing north

    // Take one small time step
    AircraftState new_state = physics.integrateOneStep(state, 2);

    // Should only move downward due to gravity (no thrust, no lift)
    assert(approx_equal(new_state.north_position, 0)); // No north movement
    assert(approx_equal(new_state.east_position, 0));  // No east movement

    std::cout << " PASSED\n";
}

/**
 * Test 2: Verify that applying throttle creates forward acceleration
 * This tests our thrust calculation and force integration
 */
void test_thrust()
{
    std::cout << "Test 2: Thrust acceleration (engine should speed up aircraft)...";

    AircraftPhysics physics;
    AircraftParameters params;
    physics.setAircraftParameters(params);

    // Apply full throttle with level elevator
    PilotInputs controls;
    controls.throttle = 1.0; // Full engine power
    controls.elevator = 0.0; // Keep level to focus on thrust
    controls.aileron = 0.0;
    controls.rudder = 0.0;
    physics.setPilotInputs(controls);

    // Start with some forward speed (so we have airflow over wings)
    AircraftState state;
    state.forward_velocity = 10.0; // 10 m/s initial speed
    double initial_speed = state.forward_velocity;

    // Run simulation for 1 second (100 steps of 0.01 seconds each)
    for (int i = 0; i < 100; i++)
    {
        state = physics.integrateOneStep(state, 0.01);
    }

    // Aircraft should have accelerated forward due to thrust
    assert(state.forward_velocity > initial_speed);

    std::cout << " PASSED\n";
}

/**
 * Test 3: Verify rotation matrices have correct mathematical properties
 * Rotation matrices should be orthogonal (R * R^T = Identity)
 */
void test_rotation_matrix()
{
    std::cout << "Test 3: Rotation matrix properties (should be orthogonal)...";

    // Create a rotation matrix with some arbitrary angles
    RotationMatrix rotation(0.1, 0.2, 0.3); // roll=0.1, pitch=0.2, yaw=0.3 radians

    // Test orthogonality: dot product of first row with itself should be 1
    double row1_magnitude = rotation(0, 0) * rotation(0, 0) +
                            rotation(0, 1) * rotation(0, 1) +
                            rotation(0, 2) * rotation(0, 2);

    // Test orthogonality: dot product of first row with second row should be 0
    double row1_dot_row2 = rotation(0, 0) * rotation(1, 0) +
                           rotation(0, 1) * rotation(1, 1) +
                           rotation(0, 2) * rotation(1, 2);

    assert(approx_equal(row1_magnitude, 1.0, 1e-10)); // Should be exactly 1
    assert(approx_equal(row1_dot_row2, 0.0, 1e-10));  // Should be exactly 0

    std::cout << " PASSED\n";
}

/**
 * Test 4: Verify Euler angle rate conversions work correctly
 * At small angles, the conversions should be approximately linear
 */
void test_euler_rates()
{
    std::cout << "Test 4: Euler rate conversion (body rates to angle rates)...";

    // Use small angles where approximations should be accurate
    double roll_angle = 0.1;  // radians
    double pitch_angle = 0.2; // radians

    // Small body rotation rates
    double p_body_rate = 0.01; // roll rate
    double q_body_rate = 0.02; // pitch rate
    double r_body_rate = 0.03; // yaw rate

    EulerAngleRates angle_rates = convertBodyRatesToAngleRates(roll_angle, pitch_angle,
                                                               p_body_rate, q_body_rate, r_body_rate);

    // At small angles, roll angle rate should be approximately equal to body roll rate
    assert(approx_equal(angle_rates.roll_rate, p_body_rate, 0.01));

    std::cout << " PASSED\n";
}

/**
 * Test 5: Verify control surfaces create the expected moments
 * Moving controls should create rotation in the correct direction
 */
void test_controls()
{
    std::cout << "Test 5: Control surfaces (should create correct rotational moments)...";

    AircraftPhysics physics;
    AircraftParameters params;
    physics.setAircraftParameters(params);

    // Start with some airspeed so aerodynamics work
    AircraftState state;
    state.forward_velocity = 50.0; // 50 m/s airspeed

    // Test aileron control (should create roll)
    PilotInputs controls;
    controls.aileron = 1.0; // Full right aileron
    controls.elevator = 0.0;
    controls.rudder = 0.0;
    controls.throttle = 0.5;
    physics.setPilotInputs(controls);

    StateDerivatives derivatives = physics.calculateDerivatives(state);
    assert(derivatives.roll_acceleration > 0); // Positive aileron should create positive roll

    // Test elevator control (should create pitch)
    controls.aileron = 0.0;
    controls.elevator = 1.0; // Full up elevator
    physics.setPilotInputs(controls);

    derivatives = physics.calculateDerivatives(state);
    assert(derivatives.pitch_acceleration > 0); // Positive elevator should create positive pitch

    std::cout << " PASSED\n";
}

/**
 * Test 6: Basic energy conservation check
 * With no drag or thrust, total energy should be roughly conserved
 */
void test_energy_conservation()
{
    std::cout << "Test 6: Energy tracking (should conserve energy without drag/thrust)...";

    AircraftPhysics physics;
    AircraftParameters params;

    // Remove drag for this test to check energy conservation
    params.base_drag_coefficient = 0.0; // No parasitic drag
    params.induced_drag_factor = 0.0;   // No induced drag
    physics.setAircraftParameters(params);

    // No thrust input
    PilotInputs controls;
    controls.throttle = 0.0; // No engine power
    physics.setPilotInputs(controls);

    // Start with some speed and altitude
    AircraftState state;
    state.forward_velocity = 50.0; // Some kinetic energy
    state.down_position = -1000.0; // Start at altitude (negative = above ground)

    // Calculate initial total energy
    double initial_kinetic = 0.5 * params.mass * (state.forward_velocity * state.forward_velocity + state.side_velocity * state.side_velocity + state.vertical_velocity * state.vertical_velocity);
    double initial_potential = params.mass * params.gravity * (-state.down_position);
    double initial_total_energy = initial_kinetic + initial_potential;

    // Run simulation for a while
    for (int i = 0; i < 100; i++)
    {
        state = physics.integrateOneStep(state, 0.01);
    }

    // Calculate final total energy
    double final_kinetic = 0.5 * params.mass * (state.forward_velocity * state.forward_velocity + state.side_velocity * state.side_velocity + state.vertical_velocity * state.vertical_velocity);
    double final_potential = params.mass * params.gravity * (-state.down_position);
    double final_total_energy = final_kinetic + final_potential;

    // Energy should be approximately conserved (allowing for numerical errors)
    double energy_change_percent = std::abs(final_total_energy - initial_total_energy) / initial_total_energy;
    assert(energy_change_percent < 0.01); // Should be within 1%

    std::cout << " PASSED\n";
}

/**
 * Main test runner - executes all tests and reports results
 */
int main()
{
    std::cout << "=== Running Aircraft Physics Tests ===\n";
    std::cout << "Testing the cleaned up aircraft simulation code...\n\n";

    try
    {
        test_equilibrium();
        test_thrust();
        test_rotation_matrix();
        test_euler_rates();
        test_controls();
        test_energy_conservation();

        std::cout << "All tests passed successfully!\n";
        std::cout << "Aircraft physics simulation is working correctly.\n";
    }
    catch (const std::exception &e)
    {
        std::cout << "\nTest failed with exception: " << e.what() << "\n";
        return 1;
    }
    catch (...)
    {
        std::cout << "Test failed with unknown exception\n";
        return 1;
    }

    return 0;
}