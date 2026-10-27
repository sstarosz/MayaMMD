#pragma once

#include "joint_palette.hpp"
#include "rigid_body_shape.hpp"
#include <maya/MMatrix.h>
#include <maya/MUIDrawManager.h>

namespace mmd::maya
{
// Shared native primitives: capsule height is the cylinder length, box sizes
// are half-extents. The optional matrix maps body space into the drawing space.
inline void drawColliderWire(MHWRender::MUIDrawManager& draw, const MPoint& size,
                             short type, const MMatrix& matrix = MMatrix::identity)
{
    const MPoint center = MPoint::origin * matrix;
    const MVector up = MVector(0, 1, 0) * matrix;
    const MVector right = MVector(1, 0, 0) * matrix;
    const MVector front = MVector(0, 0, 1) * matrix;
    switch (type)
    {
    case RigidBodyShape::kColliderBox:
        draw.box(center, up.normal(), right.normal(), size.x * right.length(),
                 size.y * up.length(), size.z * front.length(), false);
        break;
    case RigidBodyShape::kColliderSphere:
        draw.sphere(center, size.x * right.length(), false);
        break;
    default:
        draw.capsule(center, up.normal(), size.x * right.length(), size.y * up.length(),
                     12, 8, false);
        break;
    }
}

// A connected body's highlight in its A / B role colour (joint_palette.hpp).
// A body with both roles is striped: the B colour dashed over the A colour.
inline void drawRoleColliderWire(MHWRender::MUIDrawManager& draw, const MPoint& size, short type,
                                 unsigned int roles, const MMatrix& matrix = MMatrix::identity)
{
    namespace palette = joint_palette;
    draw.setLineStyle(MHWRender::MUIDrawManager::kSolid);
    draw.setColor((roles & palette::kRoleA) ? palette::kBodyA : palette::kBodyB);
    drawColliderWire(draw, size, type, matrix);
    if (roles == (palette::kRoleA | palette::kRoleB))
    {
        draw.setLineStyle(MHWRender::MUIDrawManager::kDashed);
        draw.setColor(palette::kBodyB);
        drawColliderWire(draw, size, type, matrix);
        draw.setLineStyle(MHWRender::MUIDrawManager::kSolid);
    }
}
} // namespace mmd::maya
