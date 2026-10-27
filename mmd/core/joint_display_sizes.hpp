#pragma once

#include "rigid_body_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace mmd::core::joint_display
{
struct GuideSizes
{
    double radius;           // X dial radius (Y and Z dials are wider)
    double tick;             // end stops, zigzag depth, arrow heads
    double rulerOffset;      // gap between a one-axis translation ruler and its axis
    double attachmentMarker; // ring around the hub at attachment A
    double bodyMarker;       // marker at each body's centre
};

// Symbolic guides (rotation dials, springs, free-axis arrows, markers) scale
// with the hub so they keep its proportions at any zoom and in every
// viewport; physical translation limits never use this scale.  hubRadius is
// the rendered sphere radius, in the same units as the guides.
inline GuideSizes guideSizes(double hubRadius)
{
    const double radius = hubRadius * 2.5;
    return {radius, radius * .09, hubRadius * 1.6, hubRadius * 1.25, hubRadius * .6};
}

// Base hub radius as a fraction of a model's (clamped) median link; the one
// constant to tune the default size of every model's hubs.
constexpr double kHubSizeFactor = 0.6;
// Hub radius used before a model's size is known (no solver world yet).
constexpr double kFallbackHubRadius = 0.12 * kHubSizeFactor;

// One hub radius for every joint of a model, from the median REST distance
// between the bodies its joints link (about 0.5 units in real models), so
// the hubs read at any model scale, all look alike, and never depend on the
// simulation state.  The user's jointDisplayScale multiplies the result.
inline double modelHubRadius(const std::vector<RigidBodySimulation::BodyDefinition>& bodies,
                             const std::vector<RigidBodySimulation::JointDefinition>& joints)
{
    std::vector<double> links;
    links.reserve(joints.size());
    const auto valid = [&](long index)
    { return index >= 0 && index < static_cast<long>(bodies.size()); };
    for (const auto& joint : joints)
    {
        if (!valid(joint.bodyA) || !valid(joint.bodyB))
            continue;
        const Double3& a = bodies[joint.bodyA].restPos;
        const Double3& b = bodies[joint.bodyB].restPos;
        const double link = std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                                      (a.z - b.z) * (a.z - b.z));
        if (link > 1e-4)
            links.push_back(link);
    }
    if (links.empty())
        return kFallbackHubRadius;
    const auto middle = links.begin() + static_cast<std::ptrdiff_t>(links.size() / 2);
    std::nth_element(links.begin(), middle, links.end());
    // Keep the hub compact in dense rigs: a fraction of the median link.
    return std::clamp(*middle * 0.35, 0.05, 0.5) * kHubSizeFactor;
}
} // namespace mmd::core::joint_display
