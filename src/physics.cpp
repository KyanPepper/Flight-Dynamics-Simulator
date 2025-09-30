#include "physics.hpp"
#include "utils.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

StateDerivatives AircraftPhysics::calculateDerivatives(const AircraftState &state) const
{
    StateDerivatives derivatives;

    // Step 1: Create rotation matrix to convert between aircraft and world coordinates
    RotationMatrix rotation(state.roll_angle, state.pitch_angle, state.yaw_angle);

    // Step 2: Calculate how fast position is changing (velocity in world frame)
    derivatives.north_velocity = rotation(0, 0) * state.forward_velocity +
                                 rotation(0, 1) * state.side_velocity +
                                 rotation(0, 2) * state.vertical_velocity;

    derivatives.east_velocity = rotation(1, 0) * state.forward_velocity +
                                rotation(1, 1) * state.side_velocity +
                                rotation(1, 2) * state.vertical_velocity;

    derivatives.down_velocity = rotation(2, 0) * state.forward_velocity +
                                rotation(2, 1) * state.side_velocity +
                                rotation(2, 2) * state.vertical_velocity;

    // Step 3: Calculate airspeed and angle of attack
    double total_airspeed = std::sqrt(state.forward_velocity * state.forward_velocity +
                                      state.side_velocity * state.side_velocity +
                                      state.vertical_velocity * state.vertical_velocity) +
                            1e-6;

    double angle_of_attack = std::atan2(-state.vertical_velocity, state.forward_velocity);
    double sideslip = std::atan2(state.side_velocity, state.forward_velocity);
    double dynamic_pressure = 0.5 * params.air_density * total_airspeed * total_airspeed;

    // Step 4: Calculate aerodynamic forces
    double lift_coefficient = params.base_lift_coefficient + params.lift_slope * angle_of_attack;
    double lift_force = lift_coefficient * dynamic_pressure * params.wing_area;

    double drag_coefficient = params.base_drag_coefficient +
                              params.induced_drag_factor * lift_coefficient * lift_coefficient;
    double drag_force = drag_coefficient * dynamic_pressure * params.wing_area;

    // Side force from sideslip
    double side_force_coefficient = -2.0 * sideslip; // Simplified
    double side_force = side_force_coefficient * dynamic_pressure * params.wing_area;

    // Step 5: Calculate engine thrust
    double thrust_force = params.max_thrust * clamp(inputs.throttle, 0.0, 1.0);

    // Step 6: FIXED - Calculate weight components in aircraft body frame
    // Gravity vector in world frame is (0, 0, mass*g) pointing DOWN
    // Transform to body frame using TRANSPOSE of rotation matrix
    double weight = params.mass * params.gravity;
    double weight_forward = rotation(2, 0) * weight;  // Row 2, Col 0 of R^T
    double weight_side = rotation(2, 1) * weight;     // Row 2, Col 1 of R^T
    double weight_vertical = rotation(2, 2) * weight; // Row 2, Col 2 of R^T

    // Step 7: Calculate linear accelerations using Newton's second law
    // FIXED - Proper signs and Coriolis effects
    derivatives.forward_acceleration = (thrust_force - drag_force + weight_forward) / params.mass +
                                       (state.yaw_rate * state.side_velocity - state.pitch_rate * state.vertical_velocity);

    derivatives.side_acceleration = (side_force + weight_side) / params.mass +
                                    (state.roll_rate * state.vertical_velocity - state.yaw_rate * state.forward_velocity);

    derivatives.vertical_acceleration = (-lift_force + weight_vertical) / params.mass +
                                        (state.pitch_rate * state.forward_velocity - state.roll_rate * state.side_velocity);

    // Step 8: Calculate control moments with aerodynamic damping (CRITICAL FIX)
    double roll_damping = -params.roll_damping * state.roll_rate * dynamic_pressure * params.wing_area;
    double pitch_damping = -params.pitch_damping * state.pitch_rate * dynamic_pressure * params.wing_area;
    double yaw_damping = -params.yaw_damping * state.yaw_rate * dynamic_pressure * params.wing_area;

    double roll_moment = params.aileron_power * clamp(inputs.aileron, -1.0, 1.0) + roll_damping;
    double pitch_moment = params.elevator_power * clamp(inputs.elevator, -1.0, 1.0) + pitch_damping;
    double yaw_moment = params.rudder_power * clamp(inputs.rudder, -1.0, 1.0) + yaw_damping;

    // Step 9: Calculate angular accelerations using Euler's equations
    derivatives.roll_acceleration = (roll_moment + (params.pitch_inertia - params.yaw_inertia) *
                                                       state.pitch_rate * state.yaw_rate) /
                                    params.roll_inertia;

    derivatives.pitch_acceleration = (pitch_moment + (params.yaw_inertia - params.roll_inertia) *
                                                         state.roll_rate * state.yaw_rate) /
                                     params.pitch_inertia;

    derivatives.yaw_acceleration = (yaw_moment + (params.roll_inertia - params.pitch_inertia) *
                                                     state.roll_rate * state.pitch_rate) /
                                   params.yaw_inertia;

    // Step 10: Calculate how fast the orientation angles are changing
    EulerAngleRates angle_rates = convertBodyRatesToAngleRates(state.roll_angle, state.pitch_angle,
                                                               state.roll_rate, state.pitch_rate, state.yaw_rate);

    derivatives.roll_rate = angle_rates.roll_rate;
    derivatives.pitch_rate = angle_rates.pitch_rate;
    derivatives.yaw_rate = angle_rates.yaw_rate;

    return derivatives;
}

AircraftState AircraftPhysics::integrateOneStep(const AircraftState &current_state, double time_step) const
{
    // RK4 integration for better stability
    StateDerivatives k1 = calculateDerivatives(current_state);

    // Create intermediate state for k2
    AircraftState state_k2 = current_state;
    state_k2.north_position += 0.5 * k1.north_velocity * time_step;
    state_k2.east_position += 0.5 * k1.east_velocity * time_step;
    state_k2.down_position += 0.5 * k1.down_velocity * time_step;
    state_k2.forward_velocity += 0.5 * k1.forward_acceleration * time_step;
    state_k2.side_velocity += 0.5 * k1.side_acceleration * time_step;
    state_k2.vertical_velocity += 0.5 * k1.vertical_acceleration * time_step;
    state_k2.roll_angle += 0.5 * k1.roll_rate * time_step;
    state_k2.pitch_angle += 0.5 * k1.pitch_rate * time_step;
    state_k2.yaw_angle += 0.5 * k1.yaw_rate * time_step;
    state_k2.roll_rate += 0.5 * k1.roll_acceleration * time_step;
    state_k2.pitch_rate += 0.5 * k1.pitch_acceleration * time_step;
    state_k2.yaw_rate += 0.5 * k1.yaw_acceleration * time_step;

    StateDerivatives k2 = calculateDerivatives(state_k2);

    // Final state update using RK4
    AircraftState next_state = current_state;

    next_state.north_position += k2.north_velocity * time_step;
    next_state.east_position += k2.east_velocity * time_step;
    next_state.down_position += k2.down_velocity * time_step;
    next_state.forward_velocity += k2.forward_acceleration * time_step;
    next_state.side_velocity += k2.side_acceleration * time_step;
    next_state.vertical_velocity += k2.vertical_acceleration * time_step;
    next_state.roll_angle += k2.roll_rate * time_step;
    next_state.pitch_angle += k2.pitch_rate * time_step;
    next_state.yaw_angle += k2.yaw_rate * time_step;
    next_state.roll_rate += k2.roll_acceleration * time_step;
    next_state.pitch_rate += k2.pitch_acceleration * time_step;
    next_state.yaw_rate += k2.yaw_acceleration * time_step;

    next_state.time += time_step;

    return next_state;
}