#include <QApplication>
#include <QTimer>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "physics.hpp"
#include "ring.hpp"
#include "horizon_widget.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Snapshot
{
    AircraftState s;
    PilotInputs inputs;
    double g_force;
    double aoa;
    double sideslip;
};

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

double calculateTotalVelocity(const AircraftState &state)
{
    return std::sqrt(
        state.forward_velocity * state.forward_velocity +
        state.side_velocity * state.side_velocity +
        state.vertical_velocity * state.vertical_velocity);
}

double calculateAngleOfAttack(const AircraftState &state)
{
    if (std::abs(state.forward_velocity) > 0.1)
    {
        return std::atan2(-state.vertical_velocity, state.forward_velocity);
    }
    return 0.0;
}

double calculateSideslip(const AircraftState &state)
{
    if (std::abs(state.forward_velocity) > 0.1)
    {
        return std::atan2(state.side_velocity, state.forward_velocity);
    }
    return 0.0;
}

double calculateGForce(const AircraftState &current, const AircraftState &previous, double dt)
{
    double forward_accel = (current.forward_velocity - previous.forward_velocity) / dt;
    double side_accel = (current.side_velocity - previous.side_velocity) / dt;
    double vertical_accel = (current.vertical_velocity - previous.vertical_velocity) / dt;

    forward_accel += current.yaw_rate * current.side_velocity - current.pitch_rate * current.vertical_velocity;
    side_accel += current.roll_rate * current.vertical_velocity - current.yaw_rate * current.forward_velocity;
    vertical_accel += current.pitch_rate * current.forward_velocity - current.roll_rate * current.side_velocity;

    double sin_pitch = std::sin(current.pitch_angle);
    double cos_pitch = std::cos(current.pitch_angle);
    double sin_roll = std::sin(current.roll_angle);
    double cos_roll = std::cos(current.roll_angle);

    double gravity_forward = -9.81 * sin_pitch;
    double gravity_side = 9.81 * cos_pitch * sin_roll;
    double gravity_vertical = 9.81 * cos_pitch * cos_roll;

    double felt_forward = forward_accel - gravity_forward;
    double felt_side = side_accel - gravity_side;
    double felt_vertical = vertical_accel - gravity_vertical;

    double total_felt_accel = std::sqrt(
        felt_forward * felt_forward +
        felt_side * felt_side +
        felt_vertical * felt_vertical);

    return total_felt_accel / 9.81;
}

// ============================================================================
// AUTOPILOT FUNCTIONS
// ============================================================================

void applyAltitudeControl(PilotInputs &inputs, const AircraftState &state, double target_altitude)
{
    double altitude_error = (target_altitude - (-state.down_position));
    double climb_rate_target = altitude_error * 0.05;                     // REDUCED gain from 0.1
    climb_rate_target = std::max(-2.0, std::min(2.0, climb_rate_target)); // Limit climb rate
    double climb_rate_error = climb_rate_target - state.vertical_velocity;
    inputs.elevator = 0.05 + climb_rate_error * 0.005;                // REDUCED gain from 0.01
    inputs.elevator = std::max(-0.3, std::min(0.3, inputs.elevator)); // REDUCED limits
}

void applySpeedControl(PilotInputs &inputs, const AircraftState &state, double target_speed)
{
    double speed = calculateTotalVelocity(state);
    double speed_error = target_speed - speed;
    inputs.throttle = 0.7 + speed_error * 0.005;                     // REDUCED gain from 0.01
    inputs.throttle = std::max(0.5, std::min(0.9, inputs.throttle)); // TIGHTER limits
}

void applyCoordinatedTurn(PilotInputs &inputs, const AircraftState &state)
{
    inputs.rudder = inputs.aileron * 0.5 + state.side_velocity * 0.1;
}

void applyGentleBanking(PilotInputs &inputs, double time)
{
    double phase = time * 0.02;              // VERY SLOW oscillation (50 second period)
    inputs.aileron = 0.08 * std::sin(phase); // VERY GENTLE (reduced from 0.15)
}

void applyManeuver(PilotInputs &inputs, const AircraftState &state, int maneuver_type)
{
    switch (maneuver_type)
    {
    case 0:
        inputs.aileron = 0.4;
        inputs.elevator = 0.15;
        inputs.throttle = 0.9;
        break;
    case 1:
        inputs.aileron = -0.2;
        inputs.elevator = -0.1;
        inputs.throttle = 0.4;
        break;
    case 2:
        inputs.aileron = 0;
        inputs.elevator = 0.05;
        inputs.throttle = 1.0;
        break;
    case 3:
        inputs.aileron = 0.5 * std::sin(state.time * 0.5);
        break;
    }
}

void updateAutopilot(PilotInputs &inputs, const AircraftState &state)
{
    // VERY GENTLE base control - reduce oscillation period
    applyGentleBanking(inputs, state.time);

    // Altitude hold
    applyAltitudeControl(inputs, state, 1000.0);

    // Coordinated turn
    applyCoordinatedTurn(inputs, state);

    // Speed control
    applySpeedControl(inputs, state, 60.0);

    // DISABLE aggressive maneuvers - let it fly straight and level
    // Comment out maneuvers for stable flight
    /*
    int maneuver = ((int)(state.time / 30)) % 4;
    if (state.time - std::floor(state.time / 30) * 30 < 10) {
        applyManeuver(inputs, state, maneuver);
    }
    */
}

// ============================================================================
// LOGGING FUNCTIONS
// ============================================================================

class FlightDataLogger
{
private:
    std::ofstream file_;
    bool first_write_ = true;

public:
    FlightDataLogger(const std::string &filename)
    {
        file_.open(filename, std::ios::out);
        if (file_.is_open())
        {
            std::cout << "[LOG] Opened flight data log: " << filename << std::endl;
            writeHeader();
        }
        else
        {
            std::cerr << "[ERROR] Failed to open log file: " << filename << std::endl;
        }
    }

    ~FlightDataLogger()
    {
        if (file_.is_open())
        {
            file_.close();
            std::cout << "[LOG] Flight data log closed." << std::endl;
        }
    }

    void writeHeader()
    {
        if (!file_.is_open())
            return;

        file_ << "time,cycle,"
              << "altitude,north_pos,east_pos,"
              << "forward_vel,side_vel,vertical_vel,total_vel,"
              << "roll_angle,pitch_angle,yaw_angle,"
              << "roll_rate,pitch_rate,yaw_rate,"
              << "aileron,elevator,rudder,throttle,"
              << "aoa,sideslip,g_force,"
              << "heading_deg,bank_deg,pitch_deg,aoa_deg,slip_deg\n";
        file_.flush();
    }

    void logState(const Snapshot &snap, int cycle)
    {
        if (!file_.is_open())
            return;

        const AircraftState &s = snap.s;

        // Convert to display units
        double heading_deg = s.yaw_angle * 180.0 / M_PI;
        double bank_deg = s.roll_angle * 180.0 / M_PI;
        double pitch_deg = s.pitch_angle * 180.0 / M_PI;
        double aoa_deg = snap.aoa * 180.0 / M_PI;
        double slip_deg = snap.sideslip * 180.0 / M_PI;
        double total_vel = std::sqrt(s.forward_velocity * s.forward_velocity +
                                     s.side_velocity * s.side_velocity +
                                     s.vertical_velocity * s.vertical_velocity);

        file_ << std::fixed << std::setprecision(6)
              << s.time << "," << cycle << ","
              << -s.down_position << "," << s.north_position << "," << s.east_position << ","
              << s.forward_velocity << "," << s.side_velocity << "," << s.vertical_velocity << "," << total_vel << ","
              << s.roll_angle << "," << s.pitch_angle << "," << s.yaw_angle << ","
              << s.roll_rate << "," << s.pitch_rate << "," << s.yaw_rate << ","
              << snap.inputs.aileron << "," << snap.inputs.elevator << "," << snap.inputs.rudder << "," << snap.inputs.throttle << ","
              << snap.aoa << "," << snap.sideslip << "," << snap.g_force << ","
              << heading_deg << "," << bank_deg << "," << pitch_deg << "," << aoa_deg << "," << slip_deg << "\n";

        // Flush every 100 cycles for safety
        if (cycle % 100 == 0)
        {
            file_.flush();
        }
    }

    bool isOpen() const { return file_.is_open(); }
};

void logFlightState(const AircraftState &state, double g_force, double total_velocity)
{
    std::cout << "[FLIGHT] t=" << state.time
              << " Alt=" << -state.down_position
              << "m Speed=" << total_velocity
              << "m/s Roll=" << (state.roll_angle * 180.0 / M_PI)
              << "° Pitch=" << (state.pitch_angle * 180.0 / M_PI)
              << "° G=" << g_force << "g" << std::endl;
}

void logUiUpdate(double altitude, double total_velocity, double g_force)
{
    std::cout << "[UI] Updated - Alt: " << altitude
              << "m, Speed: " << total_velocity
              << "m/s, G: " << g_force << "g" << std::endl;
}

// ============================================================================
// SNAPSHOT CREATION
// ============================================================================

Snapshot createSnapshot(const AircraftState &state, const PilotInputs &inputs,
                        const AircraftState &prev_state, double dt)
{
    Snapshot snap;
    snap.s = state;
    snap.inputs = inputs;
    snap.g_force = calculateGForce(state, prev_state, dt);
    snap.aoa = calculateAngleOfAttack(state);
    snap.sideslip = calculateSideslip(state);
    return snap;
}

// ============================================================================
// UI STATE CONVERSION
// ============================================================================

UiState snapshotToUiState(const Snapshot &snap)
{
    UiState u;

    u.roll = snap.s.roll_angle;
    u.pitch = snap.s.pitch_angle;
    u.yaw = snap.s.yaw_angle;

    u.forward_velocity = snap.s.forward_velocity;
    u.side_velocity = snap.s.side_velocity;
    u.vertical_velocity = snap.s.vertical_velocity;
    u.total_velocity = calculateTotalVelocity(snap.s);

    u.altitude = -snap.s.down_position;
    u.north_pos = snap.s.north_position;
    u.east_pos = snap.s.east_position;

    u.roll_rate = snap.s.roll_rate;
    u.pitch_rate = snap.s.pitch_rate;
    u.yaw_rate = snap.s.yaw_rate;

    u.aileron = snap.inputs.aileron;
    u.elevator = snap.inputs.elevator;
    u.rudder = snap.inputs.rudder;
    u.throttle = snap.inputs.throttle;

    u.angle_of_attack = snap.aoa;
    u.sideslip_angle = snap.sideslip;
    u.g_force = snap.g_force;

    u.time = snap.s.time;

    return u;
}

// ============================================================================
// PHYSICS THREAD
// ============================================================================

void runPhysicsThread(std::atomic<bool> &running, SpscRing<Snapshot, 1024> &ring,
                      AircraftPhysics &phys, FlightDataLogger &logger)
{
    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    std::cout << "[INFO] Physics thread started." << std::endl;

    AircraftState S;
    S.forward_velocity = 55.0;
    S.down_position = -1000.0;

    double dt = 1.0 / 200.0;

    PilotInputs U;
    U.elevator = 0.05;
    U.throttle = 0.7;

    AircraftState prev_state = S;
    int cycle = 0;

    double smoothed_g = 1.0;

    while (running.load())
    {
        cycle++;

        updateAutopilot(U, S);
        phys.setPilotInputs(U);

        S = phys.integrateOneStep(S, dt);

        // NaN DETECTION AND RECOVERY
        if (std::isnan(S.forward_velocity) || std::isnan(S.down_position) ||
            std::isnan(S.roll_angle) || std::abs(S.forward_velocity) > 500.0)
        {

            std::cerr << "[ERROR] NaN or extreme values detected at t=" << S.time
                      << " cycle=" << cycle << std::endl;
            std::cerr << "[ERROR] Resetting aircraft to stable flight..." << std::endl;

            // Reset to stable state
            S.forward_velocity = 55.0;
            S.side_velocity = 0.0;
            S.vertical_velocity = 0.0;
            S.down_position = -1000.0;
            S.roll_angle = 0.0;
            S.pitch_angle = 0.05;
            S.yaw_angle = prev_state.yaw_angle; // Keep heading
            S.roll_rate = 0.0;
            S.pitch_rate = 0.0;
            S.yaw_rate = 0.0;

            // Reset to gentle controls
            U.aileron = 0.0;
            U.elevator = 0.05;
            U.rudder = 0.0;
            U.throttle = 0.7;

            prev_state = S;
            continue;
        }

        Snapshot snap = createSnapshot(S, U, prev_state, dt);

        smoothed_g = 0.9 * smoothed_g + 0.1 * snap.g_force;
        snap.g_force = smoothed_g;

        // Log to CSV file every 10 cycles (50ms @ 200Hz = 20Hz logging)
        if (cycle % 10 == 0)
        {
            logger.logState(snap, cycle);
        }

        if (cycle % 400 == 0)
        {
            logFlightState(S, snap.g_force, calculateTotalVelocity(S));
        }

        ring.push(snap);

        prev_state = S;

        next += std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(dt));
        std::this_thread::sleep_until(next);
    }

    std::cout << "[INFO] Physics thread exiting." << std::endl;
}

// ============================================================================
// UI UPDATE HANDLER
// ============================================================================

void updateUiFromRing(SpscRing<Snapshot, 1024> &ring, HorizonWidget &horizon)
{
    Snapshot snap;
    bool have = false;

    while (auto s = ring.pop())
    {
        snap = *s;
        have = true;
    }

    if (have)
    {
        UiState u = snapshotToUiState(snap);
        horizon.setUiState(u);

        static int ui_count = 0;
        if (++ui_count % 600 == 0)
        {
            logUiUpdate(u.altitude, u.total_velocity, u.g_force);
        }
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char **argv)
{
    std::cout << "[INFO] Starting Enhanced Flight Dynamics Simulator..." << std::endl;

    AircraftPhysics phys;
    AircraftParameters P;
    phys.setAircraftParameters(P);

    std::cout << "[INFO] Physics initialized." << std::endl;

    // Create timestamped log filename
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "flight_log_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".csv";
    std::string log_filename = ss.str();

    FlightDataLogger logger(log_filename);
    if (!logger.isOpen())
    {
        std::cerr << "[WARNING] Logging disabled - failed to open log file." << std::endl;
    }

    std::atomic<bool> running{true};
    SpscRing<Snapshot, 1024> ring;

    std::thread physics_thread([&]()
                               { runPhysicsThread(running, ring, phys, logger); });

    QApplication app(argc, argv);

    HorizonWidget horizon;
    horizon.setWindowTitle("FlightSim Professional — Full Instrumentation");
    horizon.resize(1400, 900);
    horizon.show();

    std::cout << "[INFO] UI initialized with full instrumentation." << std::endl;
    std::cout << "[INFO] Logging flight data to: " << log_filename << std::endl;

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]()
                     { updateUiFromRing(ring, horizon); });

    timer.start(16);

    std::cout << "[INFO] Simulator running. Close window to exit." << std::endl;

    int rc = app.exec();

    running.store(false);
    if (physics_thread.joinable())
        physics_thread.join();

    std::cout << "[INFO] Simulator exiting cleanly." << std::endl;
    std::cout << "[INFO] Flight data saved to: " << log_filename << std::endl;
    return rc;
}