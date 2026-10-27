/*
 * SPDX-License-Identifier: MIT
 *
 * joint_palette.hpp
 *
 * Every colour the joint visualization uses, in one place.  Each hue belongs
 * to ONE channel:
 *
 *   - joint STATE (hubs): what kind of motion the joint permits.  A joint
 *     that both rotates and translates shows BOTH hues, one per half of the
 *     hub, so this channel needs only two hues;
 *   - RELATIONSHIP (selected joint): body A / body B.  A body that is A for
 *     one selected joint and B for another is striped A/B, not a third hue;
 *   - AXES (selected joint's motion guides): local X / Y / Z, Maya's RGB;
 *   - guide neutral: boundaries, zero references, equilibrium marks.
 *
 * The four state and relationship hues sit about 90 degrees apart (rose,
 * gold, cyan, violet), clear of the axis colours and Maya's selection green:
 * every pair of them is at least 28 CIEDE2000 apart, also under simulated
 * deuteranopia and protanopia.  Selection itself uses Maya's own selection
 * colours (lead green, white).
 */

#pragma once

#include <maya/MColor.h>

namespace mmd::maya::joint_palette
{
// --- state (hub colour) ---
inline const MColor kWelded(0.70f, 0.70f, 0.73f);     // grey: no freedom
inline const MColor kRotates(0.35f, 0.95f, 0.95f);    // cyan
inline const MColor kTranslates(1.00f, 0.45f, 0.56f); // rose
// Inactive (constrains nothing): a hollow dashed hub.  The FORM, not the
// colour, tells it from a welded hub; the darker grey only lets it recede.
inline const MColor kInactive(0.52f, 0.52f, 0.55f);

// --- relationship (selected joint; also the bodies' highlight wires) ---
inline const MColor kBodyA(0.70f, 0.45f, 1.00f); // violet
inline const MColor kBodyB(1.00f, 0.82f, 0.00f); // gold
// Role bits: 1 = body A, 2 = body B, 3 = A for one selected joint, B for another.
constexpr unsigned int kRoleA = 1u;
constexpr unsigned int kRoleB = 2u;

// --- axes (selected joint's motion guides) ---
inline const MColor kAxis[3] = {MColor(1.0f, 0.25f, 0.22f), MColor(0.35f, 1.0f, 0.25f),
                                MColor(0.30f, 0.65f, 1.0f)};
inline const MColor kGuide(0.95f, 0.95f, 0.95f);

// Fallback when Maya's selection colour is near-black.
inline const MColor kSelectionFallback(1.0f, 0.60f, 0.0f);
} // namespace mmd::maya::joint_palette
