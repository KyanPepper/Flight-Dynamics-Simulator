#include <QApplication>
#include <QTimer>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>
#include <iostream> // <-- Add this for logging
#include "physics.hpp"
#include "ring.hpp"
#include "horizon_widget.h"

struct Snapshot
{
    AircraftState s; // Complete aircraft state
};

int main(int argc, char **argv)
{
    std::cout << "[INFO] Starting Flight Dynamics Simulator..." << std::endl;

    // === Initialize Physics ===
    AircraftPhysics phys;
    AircraftParameters P;
    phys.setAircraftParameters(P);

    PilotInputs U;
    U.elevator = 0.03; // Slight up elevator for level flight
    U.throttle = 0.6;  // 60% power
    phys.setPilotInputs(U);

    std::cout << "[INFO] Physics initialized." << std::endl;

    // === Setup Threading ===
    std::atomic<bool> running{true}; // Flag to stop threads
    SpscRing<Snapshot, 1024> ring;   // Ring buffer (1024 slots)
    AircraftState S;                 // Aircraft state

    double dt = 1.0 / 200.0; // 5ms time step (200 Hz)

    // === Physics Thread ===
    std::thread physics_thread([&]
                               {
        using clock = std::chrono::steady_clock;
        auto next = clock::now();
        std::cout << "[INFO] Physics thread started." << std::endl;
        
        while(running.load()){
            // Demo: Oscillate controls to show movement
            U.aileron = 0.2 * std::sin(S.time * 0.5);   // Gentle roll
            U.elevator = 0.03 + 0.02 * std::sin(S.time * 0.3);  // Gentle pitch
            phys.setPilotInputs(U);

            // Run physics for one time step
            phys.integrateOneStep(S, dt);

            // Logging state periodically
            if (static_cast<int>(S.time * 10) % 100 == 0) { // Every 10 seconds
                std::cout << "[DEBUG] t=" << S.time
                          << " roll=" << S.roll_angle
                          << " pitch=" << S.pitch_angle << std::endl;
            }
            
            // Send state to UI (non-blocking)
            ring.push(Snapshot{S});
            
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
    horizon.setWindowTitle("FlightSim — Artificial Horizon");
    horizon.show();

    std::cout << "[INFO] UI initialized." << std::endl;

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
            u.roll = snap.s.roll_angle;   // Roll angle
            u.pitch = snap.s.pitch_angle;   // Pitch angle
            horizon.setUiState(u);

            // Log UI update occasionally
            static int ui_count = 0;
            if (++ui_count % 600 == 0) { // Every 10 seconds at 60Hz
                std::cout << "[DEBUG] UI updated: roll=" << u.roll
                          << " pitch=" << u.pitch << std::endl;
            }
        } });
    timer.start(16); // ~60 Hz

    // Run Qt event loop (blocks until window closes)
    int rc = app.exec();

    running.store(false);
    if (physics_thread.joinable())
        physics_thread.join(); // Wait for thread to finish

    std::cout << "[INFO] Simulator exiting." << std::endl;
    return rc;
}