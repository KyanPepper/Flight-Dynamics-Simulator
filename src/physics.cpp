#include "physics.hpp"
#include "utils.hpp"

StateDerivatives AircraftPhysics::calculateDerivatives(const AircraftState &state) const
{
    StateDerivatives derivatives;

    // Step 1: Create rotation matrix to convert between aircraft and world coordinates
    RotationMatrix rotation(state.roll_angle, state.pitch_angle, state.yaw_angle);

    // Step 2: Calculate how fast position is changing (velocity in world frame)
    // Transform aircraft velocity to world velocity using rotation matrix
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

    double angle_of_attack = std::atan2(state.vertical_velocity, state.forward_velocity);
    double dynamic_pressure = 0.5 * params.air_density * total_airspeed * total_airspeed;

    // Step 4: Calculate aerodynamic forces
    double lift_coefficient = params.base_lift_coefficient + params.lift_slope * angle_of_attack;
    double lift_force = lift_coefficient * dynamic_pressure * params.wing_area;

    double drag_coefficient = params.base_drag_coefficient +
                              params.induced_drag_factor * lift_coefficient * lift_coefficient;
    double drag_force = drag_coefficient * dynamic_pressure * params.wing_area;

    // Step 5: Calculate engine thrust
    double thrust_force = params.max_thrust * clamp(inputs.throttle, 0.0, 1.0);

    // Step 6: Calculate weight components in aircraft body frame
    // Weight always points down in world frame, but we need it in aircraft frame
    double weight_forward = rotation(0, 2) * params.mass * params.gravity;
    double weight_side = rotation(1, 2) * params.mass * params.gravity;
    double weight_vertical = rotation(2, 2) * params.mass * params.gravity;

    // Step 7: Calculate linear accelerations using Newton's second law (F = ma)
    // Include Coriolis effects from rotating reference frame
    derivatives.forward_acceleration = (thrust_force - drag_force + weight_forward) / params.mass -
                                       (state.pitch_rate * state.vertical_velocity - state.yaw_rate * state.side_velocity);

    derivatives.side_acceleration = (0.0 + weight_side) / params.mass -
                                    (state.yaw_rate * state.forward_velocity - state.roll_rate * state.vertical_velocity);

    derivatives.vertical_acceleration = (-lift_force + weight_vertical) / params.mass -
                                        (state.roll_rate * state.side_velocity - state.pitch_rate * state.forward_velocity);

    // Step 8: Calculate control moments (torques from moving control surfaces)
    double roll_moment = params.aileron_power * clamp(inputs.aileron, -1.0, 1.0);
    double pitch_moment = params.elevator_power * clamp(inputs.elevator, -1.0, 1.0);
    double yaw_moment = params.rudder_power * clamp(inputs.rudder, -1.0, 1.0);

    // Step 9: Calculate angular accelerations using Euler's equations for rotation
    // These account for gyroscopic effects when the aircraft spins
    derivatives.roll_acceleration = (roll_moment - (params.yaw_inertia - params.pitch_inertia) *
                                                       state.pitch_rate * state.yaw_rate) /
                                    params.roll_inertia;

    derivatives.pitch_acceleration = (pitch_moment - (params.roll_inertia - params.yaw_inertia) *
                                                         state.roll_rate * state.yaw_rate) /
                                     params.pitch_inertia;

    derivatives.yaw_acceleration = (yaw_moment - (params.pitch_inertia - params.roll_inertia) *
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
    // Simple Euler integration: new_value = old_value + rate_of_change * time_step
    StateDerivatives derivatives = calculateDerivatives(current_state);

    AircraftState next_state = current_state; // Copy current state

    // Update positions (integrate velocities)
    next_state.north_position += derivatives.north_velocity * time_step;
    next_state.east_position += derivatives.east_velocity * time_step;
    next_state.down_position += derivatives.down_velocity * time_step;

    // Update velocities (integrate accelerations)
    next_state.forward_velocity += derivatives.forward_acceleration * time_step;
    next_state.side_velocity += derivatives.side_acceleration * time_step;
    next_state.vertical_velocity += derivatives.vertical_acceleration * time_step;

    // Update orientation angles (integrate angle rates)
    next_state.roll_angle += derivatives.roll_rate * time_step;
    next_state.pitch_angle += derivatives.pitch_rate * time_step;
    next_state.yaw_angle += derivatives.yaw_rate * time_step;

    // Update rotation rates (integrate angular accelerations)
    next_state.roll_rate += derivatives.roll_acceleration * time_step;
    next_state.pitch_rate += derivatives.pitch_acceleration * time_step;
    next_state.yaw_rate += derivatives.yaw_acceleration * time_step;

    // Update simulation time
    next_state.time += time_step;

    return next_state;
}