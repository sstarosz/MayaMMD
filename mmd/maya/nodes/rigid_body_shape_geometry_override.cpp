/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_shape_geometry_override.cpp
 *
 * RigidBodyShapeGeometryOverride — lit viewport renderer for one
 * pmxRigidBodyShape (see the header for the design summary).
 *
 * The collider's SOLID surface is drawn with the lit stock surface shader
 * (rigid_body_display.hpp) so it responds to the viewport lights; the WIRE
 * outline is drawn with MUIDrawManager.  Both
 * are in the guide's LOCAL space (the
 * guide transform is the body's current pose), so the render-item matrix is
 * the identity and the collider follows the animated guide automatically.
 *
 * Lifecycle follows the devkit footPrintNode_GeometryOverride sample: the
 * render item lives in VP2's persistent list (never Destroy'd), cleanUp() has
 * an empty body (it is pure virtual in the base class), and the lazily-created
 * stock shader is released in the destructor.
 */

#include "rigid_body_shape_geometry_override.hpp"
#include "maya_utils.hpp"
#include "rigid_body_display.hpp"

#include "rigid_body_shape.hpp"

#include <maya/MColor.h>
#include <maya/MFn.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGeometry.h>
#include <maya/MPlug.h>
#include <maya/MPoint.h>
#include <maya/MShaderManager.h>
#include <maya/MStatus.h>
#include <maya/MVector.h>
#include <maya/MViewport2Renderer.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <vector>

namespace
{
// ---------------------------------------------------------------------------
// PMX collision-group colour palette (group id -> RGB) — moved here from the
// old draw override so the solid + wire agree.
// ---------------------------------------------------------------------------
constexpr std::array<std::array<float, 3>, 16> kGroupColors = {
    {
        {1.00F, 0.25F, 0.25F}, // 0 red
        {1.00F, 0.55F, 0.15F}, // 1 orange
        {1.00F, 0.85F, 0.15F}, // 2 yellow
        {0.45F, 1.00F, 0.25F}, // 3 lime
        // 4: PURPLE/ORCHID — a saturated hue chosen as the complement of the
        // (green) selection highlight, so it can never be confused with it.
        {0.70F, 0.25F, 0.70F}, // 4 orchid
        {0.20F, 0.90F, 1.00F}, // 5 cyan
        {0.30F, 0.45F, 1.00F}, // 6 blue
        {0.65F, 0.30F, 1.00F}, // 7 violet
        {1.00F, 0.30F, 0.85F}, // 8 magenta
        {0.90F, 0.20F, 0.50F}, // 9 pink
        {0.60F, 0.40F, 0.25F}, // 10 brown
        {0.55F, 0.55F, 0.55F}, // 11 grey
        {0.30F, 0.70F, 0.50F}, // 12 teal
        {0.90F, 0.75F, 0.40F}, // 13 tan
        {0.40F, 0.60F, 0.80F}, // 14 steel
        {0.80F, 0.50F, 0.60F}, // 15 rose
    },
};

// Render-item name (matched in updateRenderItems / populateGeometry).
constexpr const char* kSolidItemName = "mmd_rigid_body_solid";

// ---------------------------------------------------------------------------
// Collider tessellation — produces position + normal + triangle-index data.
// The mesh is in the guide's LOCAL space with the collider at the origin.
// ---------------------------------------------------------------------------
struct ColliderMesh
{
    std::vector<float> positions; // xyz interleaved
    std::vector<float> normals;   // xyz interleaved
    std::vector<unsigned int> indices;
};

void pushQuad(ColliderMesh& mesh, const std::array<float, 3>& n, const std::array<float, 3>& a,
              const std::array<float, 3>& b, const std::array<float, 3>& c,
              const std::array<float, 3>& d)
{
    const auto base = static_cast<unsigned int>(mesh.positions.size() / 3);
    for (const auto& v : {a, b, c, d})
    {
        mesh.positions.insert(mesh.positions.end(), {v.at(0), v.at(1), v.at(2)});
        mesh.normals.insert(mesh.normals.end(), {n.at(0), n.at(1), n.at(2)});
    }
    mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
}

// Box: PMX shape_size VERBATIM as the half-extent (full span = 2 × size,
// matching MMD/btBoxShape).  Flat-shaded: 6 quads with per-face normals.
void buildBox(float hx, float hy, float hz, ColliderMesh& mesh)
{
    // Face corners are wound CCW when viewed from outside.
    const std::array<float, 3> xp{1, 0, 0};
    const std::array<float, 3> xn{-1, 0, 0};
    const std::array<float, 3> yp{0, 1, 0};
    const std::array<float, 3> yn{0, -1, 0};
    const std::array<float, 3> zp{0, 0, 1};
    const std::array<float, 3> zn{0, 0, -1};
    const std::array<std::array<float, 3>, 8> p{
        {
            {hx, -hy, -hz},
            {hx, hy, -hz},
            {hx, hy, hz},
            {hx, -hy, hz},
            {-hx, -hy, hz},
            {-hx, hy, hz},
            {-hx, hy, -hz},
            {-hx, -hy, -hz},
        },
    };
    pushQuad(mesh, xp, p.at(0), p.at(1), p.at(2), p.at(3)); // +X
    pushQuad(mesh, xn, p.at(4), p.at(5), p.at(6), p.at(7)); // -X
    pushQuad(mesh, yp, p.at(1), p.at(6), p.at(5), p.at(2)); // +Y
    pushQuad(mesh, yn, p.at(0), p.at(3), p.at(4), p.at(7)); // -Y
    pushQuad(mesh, zp, p.at(3), p.at(2), p.at(5), p.at(4)); // +Z
    pushQuad(mesh, zn, p.at(0), p.at(7), p.at(6), p.at(1)); // -Z
}

// Sphere: radius = size.x, smooth normals (position normalized).
// Wound outward (CCW seen from outside), matching buildBox / buildCapsule.
void buildSphere(float r, int stacks, int slices, ColliderMesh& mesh)
{
    mmd::maya::rigid_body_display::appendSphereMesh(r, slices, stacks, mesh.positions, mesh.normals,
                                                    mesh.indices);
}

// Capsule: radius = size.x, cylindrical length = size.y (matches the
// engine's applyShapeSize).  Smooth normals; rings describe the profile.
void buildCapsule(float r, float len, int slices, int capStacks, ColliderMesh& mesh)
{
    struct Ring
    {
        float y;
        float rad;
        float ny; // Y component of the ring's unit normal
    };
    std::vector<Ring> rings;
    const float half = len * 0.5F;

    rings.push_back({-half - r, 0.0F, -1.0F}); // bottom tip
    for (int k = 1; k < capStacks; ++k)
    {
        const auto phi = static_cast<float>(M_PI * k / (2.0 * capStacks));
        rings.push_back({-half - (r * std::cos(phi)), r * std::sin(phi), -std::cos(phi)});
    }
    constexpr int kCylRings = 2; // intermediate cylinder rings
    rings.push_back({-half, r, 0.0F});
    for (int t = 1; t <= kCylRings; ++t)
    {
        rings.push_back(
            {-half + (static_cast<float>(t) * len / static_cast<float>(kCylRings + 1)), r, 0.0F});
    }
    rings.push_back({half, r, 0.0F});
    for (int k = capStacks - 1; k >= 1; --k)
    {
        const auto phi = static_cast<float>(M_PI * k / (2.0 * capStacks));
        rings.push_back({half + (r * std::cos(phi)), r * std::sin(phi), std::cos(phi)});
    }
    rings.push_back({half + r, 0.0F, 1.0F}); // top tip

    for (const Ring& ring : rings)
    {
        const float nr = std::sqrt(std::max(0.0F, 1.0F - (ring.ny * ring.ny)));
        for (int j = 0; j <= slices; ++j)
        {
            const auto theta = static_cast<float>(2.0 * M_PI * j / slices);
            const float x = ring.rad * std::cos(theta);
            const float z = ring.rad * std::sin(theta);
            mesh.positions.insert(mesh.positions.end(), {x, ring.y, z});
            mesh.normals.insert(mesh.normals.end(),
                                {std::cos(theta) * nr, ring.ny, std::sin(theta) * nr});
        }
    }

    const int ringCount = static_cast<int>(rings.size());
    for (int i = 0; i < ringCount - 1; ++i)
    {
        for (int j = 0; j < slices; ++j)
        {
            const auto a = static_cast<unsigned int>((i * (slices + 1)) + j);
            const auto b = static_cast<unsigned int>(((i + 1) * (slices + 1)) + j);
            const auto c = static_cast<unsigned int>(((i + 1) * (slices + 1)) + j + 1);
            const auto d = static_cast<unsigned int>((i * (slices + 1)) + j + 1);
            mesh.indices.insert(mesh.indices.end(), {a, b, d, b, c, d});
        }
    }
}

void buildCollider(short colliderType, const MPoint& size, ColliderMesh& mesh)
{
    switch (colliderType)
    {
    case RigidBodyShape::kColliderBox:
        buildBox(static_cast<float>(size.x), static_cast<float>(size.y), static_cast<float>(size.z),
                 mesh);
        break;
    case RigidBodyShape::kColliderSphere:
        buildSphere(static_cast<float>(size.x), 12, 16, mesh);
        break;
    default: // kColliderCapsule
        buildCapsule(static_cast<float>(size.x), static_cast<float>(size.y), 16, 6, mesh);
        break;
    }
}

// Read a k3Double plug into an MPoint (x, y, z).
[[nodiscard]] MPoint readPoint3(const MPlug& plug)
{
    const mmd::core::Double3 v = mmd::maya::readPlugDouble3(plug);
    return {v.x, v.y, v.z};
}

} // namespace

// ===========================================================================
// Lifecycle / registration
// ===========================================================================
MHWRender::MPxGeometryOverride* RigidBodyShapeGeometryOverride::creator(const MObject& obj)
{
    return new RigidBodyShapeGeometryOverride(obj);
}

RigidBodyShapeGeometryOverride::RigidBodyShapeGeometryOverride(const MObject& obj)
    : MHWRender::MPxGeometryOverride(obj), fShape(obj)
{
    // Selecting a joint does not dirty its connected bodies (selection is not
    // a DG dependency): rigid_body_display.cpp dirties exactly the bodies whose
    // A/B role changes.
}

RigidBodyShapeGeometryOverride::~RigidBodyShapeGeometryOverride()
{
    // Release the lazily-created stock shader.  Safe here: the destructor
    // runs while Maya is alive (node deleted / plugin unloaded) — releasing
    // at DLL-exit time would crash, so we never touch the shader manager
    // from a global destructor.
    if (fSolidShader != nullptr)
    {
        MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
        if (renderer != nullptr)
        {
            const MHWRender::MShaderManager* shaderMgr = renderer->getShaderManager();
            if (shaderMgr != nullptr)
            {
                shaderMgr->releaseShader(fSolidShader);
            }
        }
        fSolidShader = nullptr;
    }
}

MHWRender::DrawAPI RigidBodyShapeGeometryOverride::supportedDrawAPIs() const
{
    return MHWRender::kOpenGL | MHWRender::kDirectX11 | MHWRender::kOpenGLCoreProfile;
}

bool RigidBodyShapeGeometryOverride::requiresGeometryUpdate() const
{
    return true;
}

bool RigidBodyShapeGeometryOverride::requiresUpdateRenderItems(const MDagPath& /*dagPath*/) const
{
    return true;
}

bool RigidBodyShapeGeometryOverride::hasUIDrawables() const
{
    return true;
}

// ===========================================================================
// Data preparation
// ===========================================================================
void RigidBodyShapeGeometryOverride::updateDG()
{
    MStatus stat;
    MFnDependencyNode fn(fShape);
    const MPlug typePlug = fn.findPlug(RigidBodyShape::aBodyColliderType, true, &stat);
    const MPlug sizePlug = fn.findPlug(RigidBodyShape::aBodyShapeSize, true, &stat);
    const MPlug groupPlug = fn.findPlug(RigidBodyShape::aBodyGroupId, true, &stat);
    const MPlug modePlug = fn.findPlug(RigidBodyShape::aDrawMode, true, &stat);
    const MPlug pmPlug = fn.findPlug(RigidBodyShape::aBodyPhysicsMode, true, &stat);
    const MPlug enabledPlug = fn.findPlug(RigidBodyShape::aBodyEnabled, true, &stat);

    if (sizePlug.isNull() || modePlug.isNull())
    {
        return;
    }

    fState.drawMode = modePlug.asShort();
    fState.colliderType = typePlug.asShort();
    fState.size = readPoint3(sizePlug);
    fState.groupId = groupPlug.isNull() ? 0 : groupPlug.asShort();
    fState.physicsMode = pmPlug.isNull() ? 0 : pmPlug.asShort();
    fState.enabled = enabledPlug.isNull() ? true : enabledPlug.asBool();
    fState.jointRoles = mmd::maya::rigid_body_display::bodyRoles(fShape);
}

void RigidBodyShapeGeometryOverride::groupColor(float& cr, float& cg, float& cb) const
{
    const int g = (fState.groupId >= 0 && fState.groupId < 16) ? fState.groupId : 0;
    const bool kinematic = (fState.physicsMode == 0); // FollowBone
    cr = kGroupColors.at(g)[0];
    cg = kGroupColors.at(g)[1];
    cb = kGroupColors.at(g)[2];
    if (!fState.enabled)
    {
        cr = 0.45F;
        cg = 0.45F;
        cb = 0.45F;
    }
    else if (kinematic)
    {
        cr *= 0.6F;
        cg *= 0.6F;
        cb *= 0.6F;
    }
}

void RigidBodyShapeGeometryOverride::updateRenderItems(const MDagPath& /*dagPath*/,
                                                       MRenderItemList& renderItems)
{
    const bool drawSolid = (fState.drawMode == RigidBodyShape::kDrawSolid ||
                            fState.drawMode == RigidBodyShape::kDrawWireSolid);

    // The render item persists in the list (VP2 owns it) — find-or-create so
    // we never append a duplicate across frames.
    MRenderItem* fSolidItem = nullptr;
    int const index = renderItems.indexOf(kSolidItemName);
    if (index < 0)
    {
        fSolidItem = MRenderItem::Create(kSolidItemName, MRenderItem::NonMaterialSceneItem,
                                         MGeometry::kTriangles);
        if (fSolidItem != nullptr)
        {
            const MMatrix identity;
            fSolidItem->setMatrix(&identity);
            // Must match the viewport's display modes or the item never draws
            // (per the devkit footPrint sample: kShaded | kTextured).
            fSolidItem->setDrawMode(
                static_cast<MGeometry::DrawMode>(static_cast<unsigned int>(MGeometry::kShaded) |
                                                 static_cast<unsigned int>(MGeometry::kTextured)));
            // NonMaterialSceneItem + wire depth priority = lit by the viewport
            // lights AND drawn on top of the character mesh (the colliders live
            // inside the body, so an opaque material item would be occluded).
            fSolidItem->depthPriority(MRenderItem::sDormantWireDepthPriority);
            renderItems.append(fSolidItem);
        }
    }
    else
    {
        fSolidItem = renderItems.itemAt(index);
    }

    if (fSolidItem == nullptr)
    {
        return;
    }

    if (fSolidShader == nullptr)
    {
        MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
        if (renderer != nullptr)
        {
            const MHWRender::MShaderManager* shaderMgr = renderer->getShaderManager();
            if (shaderMgr != nullptr)
            {
                fSolidShader = mmd::maya::createSurfaceShader(*shaderMgr);
            }
        }
    }
    // VP2 may replace a render item while this override retains its shader.
    // Bind to the current item even when no new shader had to be allocated.
    if (fSolidShader != nullptr)
    {
        fSolidItem->setShader(fSolidShader);
    }

    float cr = 1.0F;
    float cg = 0.25F;
    float cb = 0.25F;
    groupColor(cr, cg, cb); // solid keeps its group colour even when selected
    fSolidItem->enable(drawSolid);
    if (drawSolid && fSolidShader != nullptr)
    {
        mmd::maya::setSurfaceColor(*fSolidShader, MColor(cr, cg, cb));
    }
}

void RigidBodyShapeGeometryOverride::addUIDrawables(
    const MDagPath& objPath, MHWRender::MUIDrawManager& drawMgr,
    const MHWRender::MFrameContext& /*frameContext*/)
{
    // Selected bodies ALWAYS show a native selection wireframe (on top, in
    // Maya's selection colour) regardless of drawMode — like any mesh.
    // Unselected bodies show their group-colour wire only in wire modes.
    const bool selected = mmd::maya::rigid_body_display::isSelected(objPath);
    const bool related = fState.jointRoles != 0;
    const bool drawWire = selected || related ||
                          (fState.drawMode == RigidBodyShape::kDrawWire ||
                           fState.drawMode == RigidBodyShape::kDrawWireSolid);
    if (!drawWire)
    {
        return;
    }

    drawMgr.beginDrawable();
    if (related && !selected)
    {
        // A / B role across the selected joints, striped for both.
        drawMgr.setLineWidth(4.0F);
        mmd::maya::drawRoleColliderWire(drawMgr, fState.size, fState.colliderType,
                                        fState.jointRoles);
    }
    else
    {
        float cr = 1.0F;
        float cg = 0.25F;
        float cb = 0.25F;
        if (selected)
        {
            const MColor sel = mmd::maya::rigid_body_display::selectionColor(objPath);
            cr = sel.r;
            cg = sel.g;
            cb = sel.b;
        }
        else
        {
            groupColor(cr, cg, cb);
        }
        drawMgr.setLineWidth(selected ? 1.75F : 1.25F);
        drawMgr.setColor(MColor(cr, cg, cb));
        mmd::maya::drawColliderWire(drawMgr, fState.size, fState.colliderType);
    }
    drawMgr.endDrawable();
}

void RigidBodyShapeGeometryOverride::populateGeometry(const MGeometryRequirements& requirements,
                                                      const MRenderItemList& renderItems,
                                                      MGeometry& data)
{
    // Build the collider mesh from the cached draw state.
    ColliderMesh mesh;
    buildCollider(fState.colliderType, fState.size, mesh);
    if (mesh.positions.empty() || mesh.indices.empty())
    {
        return;
    }

    mmd::maya::rigid_body_display::fillMeshBuffers(requirements, mesh.positions, mesh.normals,
                                                   data);

    // One index buffer per matching render item.
    const auto numIndices = static_cast<unsigned int>(mesh.indices.size());
    const unsigned int idxBytes = numIndices * sizeof(unsigned int);
    for (int i = 0; i < renderItems.length(); ++i)
    {
        const MRenderItem* item = renderItems.itemAt(i);
        if (item == nullptr || item->name() != kSolidItemName)
        {
            continue;
        }
        MIndexBuffer* indexBuf = data.createIndexBuffer(MGeometry::kUnsignedInt32);
        if (indexBuf == nullptr)
        {
            continue;
        }
        void* idxPtr = indexBuf->acquire(numIndices, /*writeOnly=*/true);
        if (idxPtr == nullptr)
        {
            continue;
        }
        std::memcpy(idxPtr, mesh.indices.data(), idxBytes);
        indexBuf->commit(idxPtr);
        item->associateWithIndexBuffer(indexBuf);
    }
}

// MPxGeometryOverride::cleanUp() is pure virtual, so it must be overridden even
// though it does nothing: render items live in VP2's persistent list and are
// owned by VP2 — there is no per-frame cache to release.
void RigidBodyShapeGeometryOverride::cleanUp() {}

// ===========================================================================
// Selection — object-level picking of the solid render item.
// ===========================================================================
bool RigidBodyShapeGeometryOverride::refineSelectionPath(const MSelectionInfo& /*selectInfo*/,
                                                         const MRenderItem& /*hitItem*/,
                                                         MDagPath& dagPath,
                                                         MObject& /*geomComponents*/,
                                                         MSelectionMask& /*objectMask*/)
{
    MDagPath shapePath;
    if (!MDagPath::getAPathTo(fShape, shapePath))
    {
        return false;
    }
    dagPath = shapePath;
    return true;
}

void RigidBodyShapeGeometryOverride::updateSelectionGranularity(const MDagPath& /*dagPath*/,
                                                                MSelectionContext& selectionContext)
{
    selectionContext.setSelectionLevel(MSelectionContext::kObject);
}
