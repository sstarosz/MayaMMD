/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_shape_geometry_override.cpp
 *
 * RigidBodyShapeGeometryOverride — lit viewport renderer for one
 * pmxRigidBodyShape (see the header for the design summary).
 *
 * The collider's SOLID surface is drawn with the stock OpenPBR surface
 * shader (k3dIsotropicOpenPBRSurfaceShader) so it responds to the viewport
 * lights; the WIRE outline is drawn with MUIDrawManager (as before).  Both
 * are in the guide's LOCAL space (the
 * guide transform is the body's current pose), so the render-item matrix is
 * the identity and the collider follows the animated guide automatically.
 *
 * Lifecycle follows the devkit footPrintNode_GeometryOverride sample: the
 * render item lives in VP2's persistent list (never Destroy'd), cleanUp() is
 * empty, and the lazily-created stock shader is released in the destructor.
 */

#include "rigid_body_shape_geometry_override.hpp"

#include "rigid_body_shape.hpp"

#include <maya/MColor.h>
#include <maya/MFn.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGeometry.h>
#include <maya/MHWGeometryUtilities.h>
#include <maya/MPlug.h>
#include <maya/MPoint.h>
#include <maya/MShaderManager.h>
#include <maya/MStatus.h>
#include <maya/MVector.h>
#include <maya/MViewport2Renderer.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace
{
// ---------------------------------------------------------------------------
// PMX collision-group colour palette (group id -> RGB) — moved here from the
// old draw override so the solid + wire agree.
// ---------------------------------------------------------------------------
constexpr std::array<std::array<float, 3>, 16> kGroupColors = {{
    {1.00f, 0.25f, 0.25f}, // 0 red
    {1.00f, 0.55f, 0.15f}, // 1 orange
    {1.00f, 0.85f, 0.15f}, // 2 yellow
    {0.45f, 1.00f, 0.25f}, // 3 lime
    // 4: PURPLE/ORCHID — a saturated hue chosen as the complement of the
    // (green) selection highlight, so it can never be confused with it.
    {0.70f, 0.25f, 0.70f}, // 4 orchid
    {0.20f, 0.90f, 1.00f}, // 5 cyan
    {0.30f, 0.45f, 1.00f}, // 6 blue
    {0.65f, 0.30f, 1.00f}, // 7 violet
    {1.00f, 0.30f, 0.85f}, // 8 magenta
    {0.90f, 0.20f, 0.50f}, // 9 pink
    {0.60f, 0.40f, 0.25f}, // 10 brown
    {0.55f, 0.55f, 0.55f}, // 11 grey
    {0.30f, 0.70f, 0.50f}, // 12 teal
    {0.90f, 0.75f, 0.40f}, // 13 tan
    {0.40f, 0.60f, 0.80f}, // 14 steel
    {0.80f, 0.50f, 0.60f}, // 15 rose
}};

// Fallback selection colour (only used if Maya's selection colour queries as
// near-black) — a bright orange-yellow that cannot collide with the palette.
constexpr float kSelectionR = 1.00f;
constexpr float kSelectionG = 0.60f;
constexpr float kSelectionB = 0.00f;

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

void pushQuad(ColliderMesh& mesh, const float n[3], const float a[3], const float b[3],
              const float c[3], const float d[3])
{
    const unsigned int base = static_cast<unsigned int>(mesh.positions.size() / 3);
    for (const float* v : {a, b, c, d})
    {
        mesh.positions.insert(mesh.positions.end(), {v[0], v[1], v[2]});
        mesh.normals.insert(mesh.normals.end(), {n[0], n[1], n[2]});
    }
    mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
}

// Box: PMX shape_size VERBATIM as the half-extent (full span = 2 × size,
// matching MMD/btBoxShape).  Flat-shaded: 6 quads with per-face normals.
void buildBox(float hx, float hy, float hz, ColliderMesh& mesh)
{
    // Face corners are wound CCW when viewed from outside.
    const float xp[3] = {1, 0, 0};
    const float xn[3] = {-1, 0, 0};
    const float yp[3] = {0, 1, 0};
    const float yn[3] = {0, -1, 0};
    const float zp[3] = {0, 0, 1};
    const float zn[3] = {0, 0, -1};
    const float p[8][3] = {
        {hx, -hy, -hz}, {hx, hy, -hz}, {hx, hy, hz},   {hx, -hy, hz},
        {-hx, -hy, hz}, {-hx, hy, hz}, {-hx, hy, -hz}, {-hx, -hy, -hz},
    };
    pushQuad(mesh, xp, p[0], p[1], p[2], p[3]); // +X
    pushQuad(mesh, xn, p[4], p[5], p[6], p[7]); // -X
    pushQuad(mesh, yp, p[1], p[6], p[5], p[2]); // +Y
    pushQuad(mesh, yn, p[0], p[3], p[4], p[7]); // -Y
    pushQuad(mesh, zp, p[3], p[2], p[5], p[4]); // +Z
    pushQuad(mesh, zn, p[0], p[7], p[6], p[1]); // -Z
}

// Sphere: radius = size.x, smooth normals (position normalized).
void buildSphere(float r, int stacks, int slices, ColliderMesh& mesh)
{
    const auto at = [&](int i, int j) { return i * (slices + 1) + j; };
    for (int i = 0; i <= stacks; ++i)
    {
        const float phi = static_cast<float>(M_PI * i / stacks);
        const float y = r * std::cos(phi);
        const float ringR = r * std::sin(phi);
        for (int j = 0; j <= slices; ++j)
        {
            const float theta = static_cast<float>(2.0 * M_PI * j / slices);
            const float x = ringR * std::cos(theta);
            const float z = ringR * std::sin(theta);
            mesh.positions.insert(mesh.positions.end(), {x, y, z});
            const float inv = 1.0f / r;
            mesh.normals.insert(mesh.normals.end(), {x * inv, y * inv, z * inv});
        }
    }
    for (int i = 0; i < stacks; ++i)
    {
        for (int j = 0; j < slices; ++j)
        {
            const unsigned int a = at(i, j);
            const unsigned int b = at(i + 1, j);
            const unsigned int c = at(i + 1, j + 1);
            const unsigned int d = at(i, j + 1);
            mesh.indices.insert(mesh.indices.end(), {a, b, d, b, c, d});
        }
    }
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
    const float half = len * 0.5f;

    rings.push_back({-half - r, 0.0f, -1.0f}); // bottom tip
    for (int k = 1; k < capStacks; ++k)
    {
        const float phi = static_cast<float>(M_PI * k / (2.0 * capStacks));
        rings.push_back({-half - r * std::cos(phi), r * std::sin(phi), -std::cos(phi)});
    }
    constexpr int kCylRings = 2; // intermediate cylinder rings
    rings.push_back({-half, r, 0.0f});
    for (int t = 1; t <= kCylRings; ++t)
    {
        rings.push_back({-half + t * len / (kCylRings + 1), r, 0.0f});
    }
    rings.push_back({half, r, 0.0f});
    for (int k = capStacks - 1; k >= 1; --k)
    {
        const float phi = static_cast<float>(M_PI * k / (2.0 * capStacks));
        rings.push_back({half + r * std::cos(phi), r * std::sin(phi), std::cos(phi)});
    }
    rings.push_back({half + r, 0.0f, 1.0f}); // top tip

    for (const Ring& ring : rings)
    {
        const float nr = std::sqrt(std::max(0.0f, 1.0f - ring.ny * ring.ny));
        for (int j = 0; j <= slices; ++j)
        {
            const float theta = static_cast<float>(2.0 * M_PI * j / slices);
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
            const unsigned int a = static_cast<unsigned int>(i * (slices + 1) + j);
            const unsigned int b = static_cast<unsigned int>((i + 1) * (slices + 1) + j);
            const unsigned int c = static_cast<unsigned int>((i + 1) * (slices + 1) + j + 1);
            const unsigned int d = static_cast<unsigned int>(i * (slices + 1) + j + 1);
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
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const double* v = plug.asMDataHandle().asDouble3();
    return MPoint(v[0], v[1], v[2]);
}

// Draw one collider's WIRE outline in the guide's local space (MUIDrawManager
// primitives, same as the old draw override).
void drawColliderWire(MHWRender::MUIDrawManager& drawMgr, const MPoint& size, short colliderType)
{
    switch (colliderType)
    {
    case RigidBodyShape::kColliderBox:
        drawMgr.box(MPoint(0.0, 0.0, 0.0), MVector(0.0, 1.0, 0.0), MVector(1.0, 0.0, 0.0), size.x,
                    size.y, size.z, /*solid=*/false);
        break;
    case RigidBodyShape::kColliderSphere:
        drawMgr.sphere(MPoint(0.0, 0.0, 0.0), size.x, /*solid=*/false);
        break;
    default: // kColliderCapsule
        drawMgr.capsule(MPoint(0.0, 0.0, 0.0), MVector(0.0, 1.0, 0.0), size.x, size.y, 12, 8,
                        /*solid=*/false);
        break;
    }
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
                shaderMgr->releaseShader(fSolidShader);
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
        return;

    fState.drawMode = static_cast<short>(modePlug.asShort());
    fState.colliderType = static_cast<short>(typePlug.asShort());
    fState.size = readPoint3(sizePlug);
    fState.groupId = groupPlug.isNull() ? 0 : static_cast<short>(groupPlug.asShort());
    fState.physicsMode = pmPlug.isNull() ? 0 : static_cast<short>(pmPlug.asShort());
    fState.enabled = enabledPlug.isNull() ? true : enabledPlug.asBool();
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
        cr = 0.45f;
        cg = 0.45f;
        cb = 0.45f;
    }
    else if (kinematic)
    {
        cr *= 0.6f;
        cg *= 0.6f;
        cb *= 0.6f;
    }
}

bool RigidBodyShapeGeometryOverride::isSelected(const MDagPath& path)
{
    const MHWRender::DisplayStatus status = MHWRender::MGeometryUtilities::displayStatus(path);
    return status == MHWRender::kLead || status == MHWRender::kActive;
}

void RigidBodyShapeGeometryOverride::selectionColor(const MDagPath& path, float& cr, float& cg,
                                                    float& cb)
{
    const MColor sel = MHWRender::MGeometryUtilities::wireframeColor(path);
    cr = sel.r;
    cg = sel.g;
    cb = sel.b;
    // Guard: a near-black selection colour would hide the wire — fall back
    // to a bright orange that cannot match the palette.
    if (cr + cg + cb < 0.01f)
    {
        cr = kSelectionR;
        cg = kSelectionG;
        cb = kSelectionB;
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
    int index = renderItems.indexOf(kSolidItemName);
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
                static_cast<MGeometry::DrawMode>(MGeometry::kShaded | MGeometry::kTextured));
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
        return;

    if (fSolidShader == nullptr)
    {
        MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
        if (renderer != nullptr)
        {
            const MHWRender::MShaderManager* shaderMgr = renderer->getShaderManager();
            if (shaderMgr != nullptr)
                // OpenPBR — Maya 2026+'s default material (matches the imported
                // meshes' openPBRSurface look); baseColor param is float3.
                fSolidShader = shaderMgr->getStockShader(
                    MHWRender::MShaderManager::k3dIsotropicOpenPBRSurfaceShader);
        }
        if (fSolidShader != nullptr)
            fSolidItem->setShader(fSolidShader);
    }

    float cr = 1.0f, cg = 0.25f, cb = 0.25f;
    groupColor(cr, cg, cb); // solid keeps its group colour even when selected
    fSolidItem->enable(drawSolid);
    if (drawSolid && fSolidShader != nullptr)
    {
        fSolidShader->setParameter("baseColor", MFloatVector(cr, cg, cb));
    }
}

void RigidBodyShapeGeometryOverride::addUIDrawables(
    const MDagPath& objPath, MHWRender::MUIDrawManager& drawMgr,
    const MHWRender::MFrameContext& /*frameContext*/)
{
    // Selected bodies ALWAYS show a native selection wireframe (on top, in
    // Maya's selection colour) regardless of drawMode — like any mesh.
    // Unselected bodies show their group-colour wire only in wire modes.
    const bool selected = isSelected(objPath);
    const bool drawWire = selected || (fState.drawMode == RigidBodyShape::kDrawWire ||
                                       fState.drawMode == RigidBodyShape::kDrawWireSolid);
    if (!drawWire)
        return;

    float cr = 1.0f, cg = 0.25f, cb = 0.25f;
    if (selected)
        selectionColor(objPath, cr, cg, cb);
    else
        groupColor(cr, cg, cb);
    drawMgr.beginDrawable();
    drawMgr.setColor(MColor(cr, cg, cb));
    drawColliderWire(drawMgr, fState.size, fState.colliderType);
    drawMgr.endDrawable();
}

// Fill one vertex buffer (`dimension` floats per vertex) with `numVerts`
// elements.  NOTE: acquire()'s size is the ELEMENT count, not bytes.
void fillVertexBuffer(MVertexBuffer* buffer, const std::vector<float>& data, unsigned int numVerts,
                      int dimension, const MVertexBufferDescriptor& desc)
{
    if (buffer == nullptr || numVerts == 0 || data.size() < numVerts * dimension)
        return;
    if (desc.dataType() != MGeometry::kFloat || desc.dimension() != dimension)
        return;
    void* ptr = buffer->acquire(numVerts, /*writeOnly=*/true);
    if (ptr == nullptr)
        return;
    std::memcpy(ptr, data.data(), numVerts * dimension * sizeof(float));
    buffer->commit(ptr);
}

void RigidBodyShapeGeometryOverride::populateGeometry(const MGeometryRequirements& requirements,
                                                      const MRenderItemList& renderItems,
                                                      MGeometry& data)
{
    // Build the collider mesh from the cached draw state.
    ColliderMesh mesh;
    buildCollider(fState.colliderType, fState.size, mesh);
    if (mesh.positions.empty() || mesh.indices.empty())
        return;

    const unsigned int numVerts = static_cast<unsigned int>(mesh.positions.size() / 3);

    // Create vertex buffers from the shader's OWN requirements (position +
    // normal [+ uv] for the stock OpenPBR surface) so the stream
    // names/semantics always match what the item's shader expects.  Fall back
    // to manual position + normal + uv descriptors if the requirements come
    // back empty (can happen for custom render items).
    const MVertexBufferDescriptorList& descList = requirements.vertexRequirements();
    if (descList.length() > 0)
    {
        MVertexBufferDescriptor desc;
        for (int i = 0; i < descList.length(); ++i)
        {
            if (!descList.getDescriptor(i, desc))
                continue;
            switch (desc.semantic())
            {
            case MGeometry::kPosition:
                fillVertexBuffer(data.createVertexBuffer(desc), mesh.positions, numVerts, 3, desc);
                break;
            case MGeometry::kNormal:
                fillVertexBuffer(data.createVertexBuffer(desc), mesh.normals, numVerts, 3, desc);
                break;
            case MGeometry::kTexture:
            {
                // Solid colour needs no real UVs — provide a zeroed float2
                // stream so the OpenPBR shader has everything it declares.
                const std::vector<float> dummyUvs(numVerts * 2, 0.0f);
                fillVertexBuffer(data.createVertexBuffer(desc), dummyUvs, numVerts, 2, desc);
                break;
            }
            default:
                break;
            }
        }
    }
    else
    {
        const MVertexBufferDescriptor posDesc("position", MGeometry::kPosition, MGeometry::kFloat,
                                              3);
        const MVertexBufferDescriptor nrmDesc("normal", MGeometry::kNormal, MGeometry::kFloat, 3);
        const MVertexBufferDescriptor uvDesc("uv", MGeometry::kTexture, MGeometry::kFloat, 2);
        fillVertexBuffer(data.createVertexBuffer(posDesc), mesh.positions, numVerts, 3, posDesc);
        fillVertexBuffer(data.createVertexBuffer(nrmDesc), mesh.normals, numVerts, 3, nrmDesc);
        const std::vector<float> dummyUvs(numVerts * 2, 0.0f);
        fillVertexBuffer(data.createVertexBuffer(uvDesc), dummyUvs, numVerts, 2, uvDesc);
    }

    // One index buffer per matching render item.
    const unsigned int numIndices = static_cast<unsigned int>(mesh.indices.size());
    const unsigned int idxBytes = numIndices * sizeof(unsigned int);
    for (int i = 0; i < renderItems.length(); ++i)
    {
        const MRenderItem* item = renderItems.itemAt(i);
        if (item == nullptr || item->name() != kSolidItemName)
            continue;
        MIndexBuffer* indexBuf = data.createIndexBuffer(MGeometry::kUnsignedInt32);
        if (indexBuf == nullptr)
            continue;
        void* idxPtr = indexBuf->acquire(numIndices, /*writeOnly=*/true);
        if (idxPtr == nullptr)
            continue;
        std::memcpy(idxPtr, mesh.indices.data(), idxBytes);
        indexBuf->commit(idxPtr);
        item->associateWithIndexBuffer(indexBuf);
    }
}

void RigidBodyShapeGeometryOverride::cleanUp()
{
    // Render items live in VP2's persistent list and are owned by VP2 —
    // nothing to release per frame.
}

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
        return false;
    dagPath = shapePath;
    return true;
}

void RigidBodyShapeGeometryOverride::updateSelectionGranularity(const MDagPath& /*dagPath*/,
                                                                MSelectionContext& selectionContext)
{
    selectionContext.setSelectionLevel(MSelectionContext::kObject);
}
