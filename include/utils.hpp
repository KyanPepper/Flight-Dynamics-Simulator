#pragma once
#include <cmath>
#include <array>

/**
 * Keep a number between minimum and maximum values
 * Example: clamp(5, 0, 10) returns 5, clamp(-2, 0, 10) returns 0
 */
inline double clamp(double value, double min_value, double max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

/**
 * A 3x3 rotation matrix that converts directions from aircraft body frame
 * to world frame. Think of it like a compass that tells you which way
 * "forward" points in world coordinates when the aircraft is tilted.
 */
struct RotationMatrix
{
    std::array<double, 9> data; // 3x3 matrix stored as 9 numbers

    // Constructor creates the matrix from aircraft orientation angles
    RotationMatrix(double roll, double pitch, double yaw)
    {
        // Pre-calculate sine and cosine values (more efficient)
        double cos_roll = std::cos(roll);
        double sin_roll = std::sin(roll);
        double cos_pitch = std::cos(pitch);
        double sin_pitch = std::sin(pitch);
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);

        // Fill in the 3x3 rotation matrix (stored row by row)
        data = {
            // First row: how much world X-axis points in each body direction
            cos_pitch * cos_yaw,
            cos_pitch * sin_yaw,
            -sin_pitch,

            // Second row: how much world Y-axis points in each body direction
            sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw,
            sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw,
            sin_roll * cos_pitch,

            // Third row: how much world Z-axis points in each body direction
            cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw,
            cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw,
            cos_roll * cos_pitch};
    }

    // Access matrix elements like matrix[row][column]
    double operator()(int row, int col) const
    {
        return data[row * 3 + col];
    }
};

/**
 * Rates of change of aircraft orientation angles
 * These tell us how fast the aircraft is rolling, pitching, and yawing
 */
struct EulerAngleRates
{
    double roll_rate;  // How fast roll angle is changing (rad/s)
    double pitch_rate; // How fast pitch angle is changing (rad/s)
    double yaw_rate;   // How fast yaw angle is changing (rad/s)

    // Default constructor sets everything to zero
    EulerAngleRates() : roll_rate(0), pitch_rate(0), yaw_rate(0) {}

    // Constructor with values
    EulerAngleRates(double droll, double dpitch, double dyaw)
        : roll_rate(droll), pitch_rate(dpitch), yaw_rate(dyaw) {}
};

/**
 * Convert body rotation rates to angle change rates
 * Body rates (p,q,r) are what gyroscopes measure - rotation around aircraft axes
 * Angle rates are how fast the aircraft's orientation angles change
 *
 * Think of it like this: if you spin in your chair (body rate), how fast
 * does your compass heading change (angle rate)? It depends on how you're tilted!
 */
inline EulerAngleRates convertBodyRatesToAngleRates(double roll_angle, double pitch_angle,
                                                    double p_body_rate, double q_body_rate, double r_body_rate)
{
    // Pre-calculate trigonometric functions for efficiency
    double cos_roll = std::cos(roll_angle);
    double sin_roll = std::sin(roll_angle);
    double cos_pitch = std::cos(pitch_angle);
    double sin_pitch = std::sin(pitch_angle);
    double tan_pitch = std::tan(pitch_angle);

    EulerAngleRates rates;

    // How fast roll angle changes (includes coupling from pitch/yaw rates)
    rates.roll_rate = p_body_rate + tan_pitch * (q_body_rate * sin_roll + r_body_rate * cos_roll);

    // How fast pitch angle changes
    rates.pitch_rate = q_body_rate * cos_roll - r_body_rate * sin_roll;

    // How fast yaw angle changes (affected by aircraft tilt)
    rates.yaw_rate = (q_body_rate * sin_roll + r_body_rate * cos_roll) / cos_pitch;

    return rates;
}