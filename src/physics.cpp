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