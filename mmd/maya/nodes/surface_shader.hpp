/*
 * SPDX-License-Identifier: MIT
 *
 * surface_shader.hpp
 *
 * The lit stock surface shader shared by the physics viewport overrides
 * (colliders and joint hubs): OpenPBR, Maya 2026+'s default material, which
 * matches the imported meshes' openPBRSurface look.  Older SDKs have no
 * OpenPBR stock shader, so they fall back to the stock Blinn.
 */

#pragma once

#include <maya/MColor.h>
#include <maya/MFloatVector.h>
#include <maya/MShaderManager.h>
#include <maya/MTypes.h>

namespace mmd::maya
{
// A new shader instance; the caller releases it with MShaderManager::releaseShader.
inline MHWRender::MShaderInstance* createSurfaceShader(const MHWRender::MShaderManager& manager)
{
#if MAYA_API_VERSION >= 20260000
    return manager.getStockShader(MHWRender::MShaderManager::k3dIsotropicOpenPBRSurfaceShader);
#else
    return manager.getStockShader(MHWRender::MShaderManager::k3dBlinnShader);
#endif
}

inline void setSurfaceColor(MHWRender::MShaderInstance& shader, const MColor& color)
{
#if MAYA_API_VERSION >= 20260000
    shader.setParameter("baseColor", MFloatVector(color.r, color.g, color.b)); // float3
#else
    const float diffuse[4] = {color.r, color.g, color.b, 1.0f};
    shader.setParameter("diffuseColor", diffuse); // float4
#endif
}
} // namespace mmd::maya
