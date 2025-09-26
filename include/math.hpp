#include <cmath>
#include <array>

inline double clamp(double x, double min, double max)
{
    if (x < min)
        return min;
    if (x > max)
        return max;
    return x;
}

// Convert Euler angles to rotation matrix
// This matrix transforms vectors from body frame to inertial frame
inline std::array<double, 9> R_ib_from_euler(double phi, double th, double psi)
{
    // Pre-calculate trig functions for efficiency
    double cphi = std::cos(phi), sphi = std::sin(phi);
    double cth = std::cos(th), sth = std::sin(th);
    double cpsi = std::cos(psi), spsi = std::sin(psi);

    // This is the rotation matrix C_IB (Inertial from Body)
    // It's a 3x3 matrix stored as array of 9 values
    return {
        // Row 1
        cth * cpsi,
        cth * spsi,
        -sth,
        // Row 2
        sphi * sth * cpsi - cphi * spsi,
        sphi * sth * spsi + cphi * cpsi,
        sphi * cth,
        // Row 3
        cphi * sth * cpsi + sphi * spsi,
        cphi * sth * spsi - sphi * cpsi,
        cphi * cth};
}
struct EulerRates
{
    double dphi; // Roll rate (rad/s)
    double dth;  // Pitch rate (rad/s)
    double dpsi; // Yaw rate (rad/s)
};

// Convert body rates (p, q, r) to Euler angle rates (φ̇, θ̇, ψ̇)
// phi: roll angle (rad)
// th: pitch angle (rad)
// p, q, r: body angular rates (rad/s)
// Returns: EulerRates struct containing dphi, dth, dpsi
inline EulerRates euler_rates(double phi, double th, double p, double q, double r)
{
    // Precompute trigonometric values for efficiency
    double cphi = std::cos(phi), sphi = std::sin(phi);
    double cth = std::cos(th), sth = std::sin(th);

    // Compute Euler angle rates using standard transformation
    // dphi: roll rate
    // dth: pitch rate
    // dpsi: yaw rate
    EulerRates rates;
    rates.dphi = p + std::tan(th) * (q * sphi + r * cphi);
    rates.dth = q * cphi - r * sphi;
    rates.dpsi = (q * sphi + r * cphi) / cth;

    return rates;
}
