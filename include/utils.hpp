#pragma once
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * Clamp a value between min and max
 */
inline double clamp(double value, double min_val, double max_val)
{
    return std::max(min_val, std::min(max_val, value));
}

/**
 * Rotation matrix for converting between body and world coordinates
 * Implements a 3x3 rotation matrix using Euler angles (roll, pitch, yaw)
 */
class RotationMatrix
{
private:
    double m[3][3];

public:
    RotationMatrix(double roll, double pitch, double yaw)
    {
        // Pre-compute trig values
        double cr = std::cos(roll);
        double sr = std::sin(roll);
        double cp = std::cos(pitch);
        double sp = std::sin(pitch);
        double cy = std::cos(yaw);
        double sy = std::sin(yaw);

        // Build rotation matrix: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        // This transforms from body frame to world frame
        m[0][0] = cy * cp;
        m[0][1] = cy * sp * sr - sy * cr;
        m[0][2] = cy * sp * cr + sy * sr;

        m[1][0] = sy * cp;
        m[1][1] = sy * sp * sr + cy * cr;
        m[1][2] = sy * sp * cr - cy * sr;

        m[2][0] = -sp;
        m[2][1] = cp * sr;
        m[2][2] = cp * cr;
    }

    // Access matrix element
    double operator()(int row, int col) const
    {
        return m[row][col];
    }
};

struct EulerAngleRates
{
    double roll_rate;
    double pitch_rate;
    double yaw_rate;
};

/**
 * Convert body angular velocities (p, q, r) to Euler angle rates
 *
 * Body rates are how fast the aircraft is rotating around its own axes.
 * Euler rates are how fast the orientation angles (roll, pitch, yaw) are changing.
 *
 * The relationship is non-linear and depends on the current orientation.
 */
inline EulerAngleRates convertBodyRatesToAngleRates(
    double roll_angle,
    double pitch_angle,
    double roll_rate_body,  // p - body roll rate
    double pitch_rate_body, // q - body pitch rate
    double yaw_rate_body)   // r - body yaw rate
{
    EulerAngleRates rates;

    double cos_roll = std::cos(roll_angle);
    double sin_roll = std::sin(roll_angle);
    double cos_pitch = std::cos(pitch_angle);
    double sin_pitch = std::sin(pitch_angle);
    double tan_pitch = std::tan(pitch_angle);

    // Prevent division by zero at pitch = ±90°
    if (std::abs(cos_pitch) < 0.01)
    {
        // At gimbal lock, use simplified equations
        rates.roll_rate = roll_rate_body;
        rates.pitch_rate = pitch_rate_body * cos_roll;
        rates.yaw_rate = pitch_rate_body * sin_roll;
    }
    else
    {
        // Standard transformation matrix
        rates.roll_rate = roll_rate_body +
                          sin_roll * tan_pitch * pitch_rate_body +
                          cos_roll * tan_pitch * yaw_rate_body;

        rates.pitch_rate = cos_roll * pitch_rate_body -
                           sin_roll * yaw_rate_body;

        rates.yaw_rate = (sin_roll / cos_pitch) * pitch_rate_body +
                         (cos_roll / cos_pitch) * yaw_rate_body;
    }

    return rates;
}

/**
 * Normalize an angle to [-pi, pi]
 */
inline double normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

/**
 * Convert degrees to radians
 */
inline double degToRad(double degrees)
{
    return degrees * M_PI / 180.0;
}

/**
 * Convert radians to degrees
 */
inline double radToDeg(double radians)
{
    return radians * 180.0 / M_PI;
}