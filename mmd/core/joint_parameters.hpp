#pragma once

#include "rigid_body_simulation.hpp"
#include <utility>

namespace mmd::core
{
// Reflect Spring6Dof PMX limits into Maya's right-handed joint frame
// (F = diag(1, 1, -1)): linear Z and angular X/Y intervals negate and swap.
// Linear X/Y, angular Z and the stiffness magnitudes are invariant.
inline void reflectJointParameters(RigidBodySimulation::JointDefinition& j)
{
    const auto flip = [](double& lo, double& hi) {
        std::swap(lo, hi);
        lo = -lo;
        hi = -hi;
    };
    flip(j.linearMin.z, j.linearMax.z);
    flip(j.angularMin.x, j.angularMax.x);
    flip(j.angularMin.y, j.angularMax.y);
}
} // namespace mmd::core
