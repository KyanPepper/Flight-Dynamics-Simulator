#include <cmath>
#include <array>

// Clamp value between min and max
inline double clamp(double x, double a, double b)
{
    return std::fmax(a, std::fmin(b, x));
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

// Convert body rates (p,q,r) to Euler angle rates (φ̇,θ̇,ψ̇)
inline void euler_rates(double phi, double th, double p, double q, double r,
                        double &dphi, double &dth, double &dpsi)
{
    double cphi = std::cos(phi), sphi = std::sin(phi);
    double cth = std::cos(th), sth = std::sin(th);

    // These equations relate body rates to Euler rates
    dphi = p + std::tan(th) * (q * sphi + r * cphi);
    dth = q * cphi - r * sphi;
    dpsi = (q * sphi + r * cphi) / cth;
}
