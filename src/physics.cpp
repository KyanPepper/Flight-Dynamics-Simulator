#include "physics.hpp"
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

#include "physics.hpp"
#include "math.hpp"
#include <cmath>
#include <algorithm>

// This calculates all the derivatives (rates of change)
Deriv Physics::rhs(const State &s) const
{
    Deriv k{}; // Initialize all derivatives to zero

    // === Step 1: Get rotation matrix ===
    // This converts vectors from body frame to world frame
    auto R = R_ib_from_euler(s.phi, s.th, s.psi);

    // === Step 2: Calculate position derivatives ===
    // Velocity in world frame = Rotation * velocity in body frame
    k.dN = R[0] * s.u + R[1] * s.v + R[2] * s.w; // North velocity
    k.dE = R[3] * s.u + R[4] * s.v + R[5] * s.w; // East velocity
    k.dD = R[6] * s.u + R[7] * s.v + R[8] * s.w; // Down velocity

    // === Step 3: Calculate airspeed and angle of attack ===
    double V = std::sqrt(s.u * s.u + s.v * s.v + s.w * s.w) + 1e-6; // Total velocity
    double alpha = std::atan2(s.w, s.u);                            // Angle of attack
    double qdyn = 0.5 * P.rho * V * V;                              // Dynamic pressure

    // === Step 4: Calculate aerodynamic forces ===
    double CL = P.CL0 + P.CLa * alpha; // Lift coefficient
    double L = CL * qdyn * P.S;        // Lift force
    double CD = P.CD0 + P.k * CL * CL; // Drag coefficient
    double D = CD * qdyn * P.S;        // Drag force

    // === Step 5: Calculate thrust ===
    double T = P.Tmax * std::clamp(U.throttle, 0.0, 1.0);

    // === Step 6: Transform weight to body frame ===
    // Weight acts downward in world frame [0, 0, mg]
    // We need it in body frame for our equations
    double Wx = R[2] * P.mass * P.gravity; // Weight component along body X
    double Wy = R[5] * P.mass * P.gravity; // Weight component along body Y
    double Wz = R[8] * P.mass * P.gravity; // Weight component along body Z

    // === Step 7: Calculate accelerations (F = ma) ===
    // Including Coriolis terms from rotating reference frame
    k.du = (T - D + Wx) / P.mass - (s.q * s.w - s.r * s.v);
    k.dv = (0.0 + Wy) / P.mass - (s.r * s.u - s.p * s.w);
    k.dw = (-L + Wz) / P.mass - (s.p * s.v - s.q * s.u);

    // === Step 8: Calculate control moments ===
    double Lm = P.Kl * std::clamp(U.aileron, -1.0, 1.0);  // Roll moment
    double Mm = P.Km * std::clamp(U.elevator, -1.0, 1.0); // Pitch moment
    double Nm = P.Kn * std::clamp(U.rudder, -1.0, 1.0);   // Yaw moment

    // === Step 9: Calculate angular accelerations ===
    // Using Euler's equations for rigid body rotation
    k.dp = (Lm - (P.Iz - P.Iy) * s.q * s.r) / P.Ix;
    k.dq = (Mm - (P.Ix - P.Iz) * s.p * s.r) / P.Iy;
    k.dr = (Nm - (P.Iy - P.Ix) * s.p * s.q) / P.Iz;

    // === Step 10: Calculate Euler angle rates ===
    auto er = euler_rates(s.phi, s.th, s.p, s.q, s.r);

    k.dphi = er.dphi;
    k.dth = er.dth;
    k.dpsi = er.dpsi;

    return k;
}

// Integrate the equations of motion for one time step
void Physics::step(State &s, double dt) const
{
    // Simple Euler integration: new_value = old_value + derivative * time_step
    auto k = rhs(s); // Get all derivatives

    // Update positions
    s.N += k.dN * dt;
    s.E += k.dE * dt;
    s.D += k.dD * dt;

    // Update velocities
    s.u += k.du * dt;
    s.v += k.dv * dt;
    s.w += k.dw * dt;

    // Update angles
    s.phi += k.dphi * dt;
    s.th += k.dth * dt;
    s.psi += k.dpsi * dt;

    // Update angular rates
    s.p += k.dp * dt;
    s.q += k.dq * dt;
    s.r += k.dr * dt;

    // Update time
    s.t += dt;
}