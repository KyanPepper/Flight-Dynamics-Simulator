#include <iostream>
#include <cassert>
#include <cmath>
#include "../include/physics.hpp"
#include "../include/math.hpp"

// Helper to check if two doubles are approximately equal
bool approx_equal(double a, double b, double eps = 1e-6)
{
    return std::abs(a - b) < eps;
}

// Test 1: Verify initial state remains stable with no inputs
void test_equilibrium()
{
    std::cout << "Test 1: Equilibrium...";

    Physics phys;
    Params P;
    phys.setParams(P);

    // Set neutral controls
    Inputs U;
    U.aileron = 0.0;
    U.elevator = 0.0;
    U.rudder = 0.0;
    U.throttle = 0.0;
    phys.setInputs(U);

    // Create state at rest
    State s;
    s.u = 0;
    s.v = 0;
    s.w = 0;
    s.p = 0;
    s.q = 0;
    s.r = 0;
    s.phi = 0;
    s.th = 0;
    s.psi = 0;

    // Step forward
    phys.step(s, 0.01);

    // Should only fall due to gravity
    assert(s.D > 0);              // Should be falling (D increases)
    assert(approx_equal(s.N, 0)); // No north movement
    assert(approx_equal(s.E, 0)); // No east movement

    std::cout << " PASSED\n";
}

// Test 2: Verify thrust produces forward acceleration
void test_thrust()
{
    std::cout << "Test 2: Thrust acceleration...";

    Physics phys;
    Params P;
    phys.setParams(P);

    Inputs U;
    U.throttle = 1.0; // Full throttle
    U.elevator = 0.0;
    phys.setInputs(U);

    State s;
    s.u = 10; // Some forward speed
    double initial_u = s.u;

    // Run for 1 second
    for (int i = 0; i < 100; i++)
    {
        phys.step(s, 0.01);
    }

    // Should have accelerated forward
    assert(s.u > initial_u);

    std::cout << " PASSED\n";
}

// Test 3: Verify rotation matrices are orthogonal
void test_rotation_matrix()
{
    std::cout << "Test 3: Rotation matrix properties...";

    auto R = R_ib_from_euler(0.1, 0.2, 0.3);

    // Check orthogonality: R * R^T should equal identity
    // Calculate R * R^T for first row
    double dot00 = R[0] * R[0] + R[1] * R[1] + R[2] * R[2];
    double dot01 = R[0] * R[3] + R[1] * R[4] + R[2] * R[5];

    assert(approx_equal(dot00, 1.0, 1e-10)); // Should be 1
    assert(approx_equal(dot01, 0.0, 1e-10)); // Should be 0

    std::cout << " PASSED\n";
}

// Test 4: Verify Euler rate conversions
void test_euler_rates()
{
    std::cout << "Test 4: Euler rate conversion...";

    double phi = 0.1, th = 0.2;
    double p = 0.01, q = 0.02, r = 0.03;
    double dphi, dth, dpsi;

    auto er = euler_rates(phi, th, p, q, r);

    // At small angles, dphi ≈ p
    assert(approx_equal(er.dphi, p, 0.01));

    std::cout << " PASSED\n";
}

// Test 5: Verify control surface effectiveness
void test_controls()
{
    std::cout << "Test 5: Control surfaces...";

    Physics phys;
    Params P;
    phys.setParams(P);

    State s;
    s.u = 50; // Need airspeed for controls to work

    // Test aileron (roll)
    Inputs U;
    U.aileron = 1.0;
    phys.setInputs(U);

    auto deriv = phys.rhs(s);
    assert(deriv.dp > 0); // Positive aileron -> positive roll rate

    // Test elevator (pitch)
    U.aileron = 0.0;
    U.elevator = 1.0;
    phys.setInputs(U);

    deriv = phys.rhs(s);
    assert(deriv.dq > 0); // Positive elevator -> positive pitch rate

    std::cout << " PASSED\n";
}

// Test 6: Energy conservation check
void test_energy_conservation()
{
    std::cout << "Test 6: Energy tracking...";

    Physics phys;
    Params P;
    P.CD0 = 0; // No drag for energy test
    P.k = 0;
    phys.setParams(P);

    Inputs U;
    U.throttle = 0; // No thrust
    phys.setInputs(U);

    State s;
    s.u = 50;
    s.D = -1000; // Start at altitude

    // Calculate initial energy
    double KE_initial = 0.5 * P.mass * (s.u * s.u + s.v * s.v + s.w * s.w);
    double PE_initial = P.mass * P.gravity * (-s.D); // Negative because D is down
    double E_initial = KE_initial + PE_initial;

    // Run simulation
    for (int i = 0; i < 100; i++)
    {
        phys.step(s, 0.01);
    }

    // Calculate final energy
    double KE_final = 0.5 * P.mass * (s.u * s.u + s.v * s.v + s.w * s.w);
    double PE_final = P.mass * P.gravity * (-s.D);
    double E_final = KE_final + PE_final;

    // Energy should be approximately conserved (within numerical errors)
    assert(approx_equal(E_initial, E_final, E_initial * 0.01)); // 1% tolerance

    std::cout << " PASSED\n";
}

// Main test runner
int main()
{
    std::cout << "=== Running Physics Tests ===\n";

    test_equilibrium();
    test_thrust();
    test_rotation_matrix();
    test_euler_rates();
    test_controls();
    test_energy_conservation();

    std::cout << "\n✅ All tests passed!\n";
    return 0;
}