#pragma once
#include <array>
#include <atomic>

// Aircraft parameters (constants)
struct Params
{
    // Physical properties
    double mass = 1200;    // Mass (kg)
    double gravity = 9.81; // Gravity (m/s²)
    double rho = 1.225;    // Air density (kg/m³)

    // Moments of inertia (resistance to rotation)
    double Ix = 1200; // Roll inertia (kg·m²)
    double Iy = 1500; // Pitch inertia
    double Iz = 1800; // Yaw inertia

    // Aerodynamic coefficients
    double S = 16.0;   // Wing area (m²)
    double CL0 = 0.2;  // Lift coefficient at zero angle of attack
    double CLa = 5.5;  // Lift slope (per radian)
    double CD0 = 0.03; // Parasitic drag coefficient
    double k = 0.07;   // Induced drag factor

    // Control effectiveness
    double Kl = 3000; // Aileron effectiveness (N·m per unit deflection)
    double Km = 5000; // Elevator effectiveness
    double Kn = 2000; // Rudder effectiveness

    double Tmax = 4000; // Maximum thrust (N)
};

// Pilot inputs (control stick/throttle positions)
struct Inputs
{
    double aileron = 0.0;   // Roll control [-1,1]
    double elevator = 0.05; // Pitch control [-1,1]
    double rudder = 0.0;    // Yaw control [-1,1]
    double throttle = 0.6;  // Engine power [0,1]
};

// Complete aircraft state at a moment in time
struct State
{
    // Position in world (NED) coordinates (meters)
    double N = 0, E = 0, D = 0;

    // Velocity in body frame (m/s)
    double u = 55; // Forward velocity (start with some speed)
    double v = 0;  // Sideways velocity
    double w = 0;  // Vertical velocity

    // Orientation (Euler angles in radians)
    double phi = 0;   // Roll angle
    double th = 0.03; // Pitch angle (slight climb)
    double psi = 0;   // Yaw angle

    // Angular rates in body frame (rad/s)
    double p = 0; // Roll rate
    double q = 0; // Pitch rate
    double r = 0; // Yaw rate

    // Simulation time
    double t = 0;
};

// Time derivatives of all state variables
struct Deriv
{
    double dN, dE, dD;      // Position derivatives (velocity)
    double du, dv, dw;      // Acceleration
    double dphi, dth, dpsi; // Euler angle rates
    double dp, dq, dr;      // Angular accelerations
};

class Physics
{
public:
    Physics() = default;

    // Set aircraft parameters
    void setParams(const Params &P_) { P = P_; }

    // Set control inputs
    void setInputs(const Inputs &U_) { U = U_; }

    // Calculate derivatives (right-hand side of differential equations)
    Deriv rhs(const State &s) const;

    // Integrate one time step using Euler method
    void step(State &s, double dt) const;

private:
    Params P; // Aircraft parameters
    Inputs U; // Control inputs
};
