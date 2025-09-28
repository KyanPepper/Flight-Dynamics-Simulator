#include <QApplication>
#include <QTimer>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>
#include "physics.hpp"
#include "ring.hpp"
#include "horizon_widget.h"

struct Snapshot
{
    AircraftState s;    // Complete aircraft state
    PilotInputs inputs; // Control inputs
    double g_force;     // G-force
    double aoa;         // Angle of attack
    double sideslip;    // Sideslip angle
};

int main(int argc, char **argv)
{
    std::cout << "[INFO] Starting Enhanced Flight Dynamics Simulator..." << std::endl;

    // === Initialize Physics ===
    AircraftPhysics phys;
    AircraftParameters P;
    phys.setAircraftParameters(P);

    PilotInputs U;
    U.elevator = 0.05; // Slight up elevator for level flight
    U.throttle = 0.7;  // 70% power
    phys.setPilotInputs(U);

    std::cout << "[INFO] Physics initialized." << std::endl;

    // === Setup Threading ===
    std::atomic<bool> running{true}; // Flag to stop threads
    SpscRing<Snapshot, 1024> ring;   // Ring buffer (1024 slots)

    AircraftState S;           // Aircraft state
    S.forward_velocity = 55.0; // Start with good airspeed
    S.down_position = -1000.0; // Start at 1000m altitude

    double dt = 1.0 / 200.0; // 5ms time step (200 Hz)

    // Variables for tracking
    AircraftState prev_state = S;
    double prev_vertical_accel = 0;

    // === Physics Thread ===
    std::thread physics_thread([&]
                               {
        using clock = std::chrono::steady_clock;
        auto next = clock::now();
        std::cout << "[INFO] Physics thread started." << std::endl;
        
        int cycle = 0;
        
        while(running.load()){
            cycle++;
            
            // === Autopilot / Demo Control Logic ===
            // Create interesting flight patterns
            double phase = S.time * 0.1;  // Slow phase for demo
            
            // Gentle banking turns
            U.aileron = 0.3 * std::sin(phase);
            
            // Maintain altitude with pitch adjustments
            double altitude_error = (-1000.0 - S.down_position);  // Target 1000m
            double climb_rate_target = altitude_error * 0.1;      // Proportional control
            double climb_rate_error = climb_rate_target - S.vertical_velocity;
            U.elevator = 0.05 + climb_rate_error * 0.01;         // Pitch to control climb
            U.elevator = std::max(-0.5, std::min(0.5, U.elevator));
            
            // Coordinated turns with rudder
            U.rudder = U.aileron * 0.5 + S.side_velocity * 0.1;  // Coordinate turns
            
            // Throttle management based on speed
            double speed_error = 60.0 - S.forward_velocity;  // Target 60 m/s
            U.throttle = 0.7 + speed_error * 0.01;
            U.throttle = std::max(0.3, std::min(1.0, U.throttle));
            
            // Special maneuvers every 30 seconds
            int maneuver = ((int)(S.time / 30)) % 4;
            if (S.time - std::floor(S.time / 30) * 30 < 10) {  // First 10 seconds of each 30
                switch(maneuver) {
                    case 0:  // Climbing spiral
                        U.aileron = 0.4;
                        U.elevator = 0.15;
                        U.throttle = 0.9;
                        break;
                    case 1:  // Gentle dive
                        U.aileron = -0.2;
                        U.elevator = -0.1;
                        U.throttle = 0.4;
                        break;
                    case 2:  // Level acceleration
                        U.aileron = 0;
                        U.elevator = 0.05;
                        U.throttle = 1.0;
                        break;
                    case 3:  // S-turns
                        U.aileron = 0.5 * std::sin(S.time * 0.5);
                        break;
                }
            }
            
            phys.setPilotInputs(U);
            
            // Run physics for one time step
            S = phys.integrateOneStep(S, dt);
            
            // === Calculate derived values ===
            
            // Total velocity (airspeed)
            double total_velocity = std::sqrt(
                S.forward_velocity * S.forward_velocity +
                S.side_velocity * S.side_velocity +
                S.vertical_velocity * S.vertical_velocity
            );
            
            // Angle of attack (simplified)
            double aoa = 0;
            if (std::abs(S.forward_velocity) > 0.1) {
                aoa = std::atan2(-S.vertical_velocity, S.forward_velocity);
            }
            
            // Sideslip angle
            double sideslip = 0;
            if (std::abs(S.forward_velocity) > 0.1) {
                sideslip = std::atan2(S.side_velocity, S.forward_velocity);
            }
            
            // G-force calculation (simplified)
            double vertical_accel = (S.vertical_velocity - prev_state.vertical_velocity) / dt;
            double centripetal_accel = S.forward_velocity * S.yaw_rate;  // Simplified
            double total_accel = std::sqrt(
                centripetal_accel * centripetal_accel +
                (vertical_accel + 9.81) * (vertical_accel + 9.81)
            );
            double g_force = total_accel / 9.81;
            
            // Smooth G-force
            prev_vertical_accel = 0.9 * prev_vertical_accel + 0.1 * vertical_accel;
            
            // Logging state periodically
            if (cycle % 400 == 0) {  // Every 2 seconds at 200Hz
                std::cout << "[FLIGHT] t="  << S.time
                          << " Alt=" << -S.down_position 
                          << "m Speed=" << total_velocity
                          << "m/s Roll=" << (S.roll_angle * 180.0 / M_PI)
                          << "° Pitch=" << (S.pitch_angle * 180.0 / M_PI)
                          << "° G=" << g_force << "g" << std::endl;
            }
            
            // Create snapshot with all data
            Snapshot snap;
            snap.s = S;
            snap.inputs = U;
            snap.g_force = g_force;
            snap.aoa = aoa;
            snap.sideslip = sideslip;
            
            // Send state to UI (non-blocking)
            ring.push(snap);
            
            prev_state = S;
            
            // Sleep until next physics tick
            next += std::chrono::duration_cast<clock::duration>(
                std::chrono::duration<double>(dt));
            std::this_thread::sleep_until(next);
        }
        std::cout << "[INFO] Physics thread exiting." << std::endl; });

    // === Qt UI Thread (main thread) ===
    QApplication app(argc, argv);

    // Create horizon display window
    HorizonWidget horizon;
    horizon.setWindowTitle("FlightSim Professional — Full Instrumentation");
    horizon.resize(1400, 900); // Larger default size
    horizon.show();

    std::cout << "[INFO] UI initialized with full instrumentation." << std::endl;

    // Timer to update display at 60 Hz
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]()
                     {
        // Get latest state from physics thread
        Snapshot snap;
        bool have = false;
        
        // Drain ring buffer, keep only newest
        while (auto s = ring.pop()) { 
            snap = *s; 
            have = true; 
        }
        
        // Update display if we got new data
        if (have){
            UiState u;
            
            // Orientation
            u.roll = snap.s.roll_angle;
            u.pitch = snap.s.pitch_angle;
            u.yaw = snap.s.yaw_angle;
            
            // Velocities
            u.forward_velocity = snap.s.forward_velocity;
            u.side_velocity = snap.s.side_velocity;
            u.vertical_velocity = snap.s.vertical_velocity;
            u.total_velocity = std::sqrt(
                snap.s.forward_velocity * snap.s.forward_velocity +
                snap.s.side_velocity * snap.s.side_velocity +
                snap.s.vertical_velocity * snap.s.vertical_velocity
            );
            
            // Position
            u.altitude = -snap.s.down_position;
            u.north_pos = snap.s.north_position;
            u.east_pos = snap.s.east_position;
            
            // Rotation rates
            u.roll_rate = snap.s.roll_rate;
            u.pitch_rate = snap.s.pitch_rate;
            u.yaw_rate = snap.s.yaw_rate;
            
            // Control inputs
            u.aileron = snap.inputs.aileron;
            u.elevator = snap.inputs.elevator;
            u.rudder = snap.inputs.rudder;
            u.throttle = snap.inputs.throttle;
            
            // Derived values
            u.angle_of_attack = snap.aoa;
            u.sideslip_angle = snap.sideslip;
            u.g_force = snap.g_force;
            
            // Time
            u.time = snap.s.time;
            
            horizon.setUiState(u);
            
            // Log UI update occasionally
            static int ui_count = 0;
            if (++ui_count % 600 == 0) {  // Every 10 seconds at 60Hz
                std::cout << "[UI] Updated - Alt: " << u.altitude 
                          << "m, Speed: " << u.total_velocity 
                          << "m/s, G: " << u.g_force << "g" << std::endl;
            }
        } });

    timer.start(16); // ~60 Hz

    std::cout << "[INFO] Simulator running. Close window to exit." << std::endl;

    // Run Qt event loop (blocks until window closes)
    int rc = app.exec();

    running.store(false);
    if (physics_thread.joinable())
        physics_thread.join(); // Wait for thread to finish

    std::cout << "[INFO] Simulator exiting cleanly." << std::endl;
    return rc;
}