/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_joint_geometry_override.cpp
 *
 * RigidBodyJointGeometryOverride — lit viewport renderer for one
 * pmxRigidBodyJoint (see the header for the design summary).
 */

#include "rigid_body_joint_geometry_override.hpp"

#include "rigid_body_joint.hpp"

#include <maya/MColor.h>
#include <maya/MFn.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGeometry.h>
#include <maya/MHWGeometry.h>
#include <maya/MHWGeometryUtilities.h>
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
// Solid render-item name — the populateGeometry pass matches on it.
constexpr const char* kSolidItemName = "mmd_rigid_body_joint_solid";
// Selection-colour fallback (near-black wireframe colours would hide the wire).
constexpr float kSelectionR = 1.0f;
constexpr float kSelectionG = 0.60f;
constexpr float kSelectionB = 0.0f;

// ---------------------------------------------------------------------------
// PMX joint-type colour palette (JointType value -> RGB) — moved here from the
// solver-level joint draw so the solid + wire agree.
// ---------------------------------------------------------------------------
constexpr std::array<std::array<float, 3>, 6> kJointColors = {{
    {0.30f, 0.85f, 1.00f}, // 0 Spring6DOF — cyan
    {0.35f, 0.55f, 1.00f}, // 1 6DOF       — blue
    {1.00f, 0.60f, 0.20f}, // 2 P2P        — orange
    {1.00f, 0.85f, 0.20f}, // 3 ConeTwist  — yellow
    {1.00f, 0.35f, 0.85f}, // 4 Slider     — magenta
    {0.45f, 0.90f, 0.35f}, // 5 Hinge      — green
}};

// A flat-shaded box mesh spanning [min, max] (degenerate axes get a sliver).
struct BoxMesh
{
    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<unsigned int> indices;
};

void pushQuad(BoxMesh& mesh, const float n[3], const float a[3], const float b[3],
              const float c[3], const float d[3], unsigned int& base)
{
    mesh.positions.insert(mesh.positions.end(), {a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1],
                                                 c[2], d[0], d[1], d[2]});
    mesh.normals.insert(mesh.normals.end(), {n[0], n[1], n[2], n[0], n[1], n[2], n[0], n[1],
                                             n[2], n[0], n[1], n[2]});
    mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    base += 4;
}

// The linear-limit box: centered at (min+max)/2 with half-extents |max-min|/2
// (flat-shaded 6 quads, like the collider box builder).  Axes with no range
// get a thin sliver so sliders still show a visible box.
void buildLimitBox(const MPoint& lmin, const MPoint& lmax, double sliver, BoxMesh& mesh)
{
    const auto axis = [&](double lo, double hi) -> std::pair<double, double>
    {
        const double spread = std::fabs(hi - lo);
        const double half = spread > 1e-4 ? 0.5 * spread : sliver;
        const double center = 0.5 * (lo + hi);
        return {center, half};
    };
    const auto [cx, hx] = axis(lmin.x, lmax.x);
    const auto [cy, hy] = axis(lmin.y, lmax.y);
    const auto [cz, hz] = axis(lmin.z, lmax.z);

    const float xp[3] = {1, 0, 0};
    const float xn[3] = {-1, 0, 0};
    const float yp[3] = {0, 1, 0};
    const float yn[3] = {0, -1, 0};
    const float zp[3] = {0, 0, 1};
    const float zn[3] = {0, 0, -1};
    const float hx0 = static_cast<float>(hx), hy0 = static_cast<float>(hy),
                hz0 = static_cast<float>(hz);
    const float cxf = static_cast<float>(cx), cyf = static_cast<float>(cy),
                czf = static_cast<float>(cz);
    const float p[8][3] = {
        {cxf + hx0, cyf - hy0, czf - hz0}, {cxf + hx0, cyf + hy0, czf - hz0},
        {cxf + hx0, cyf + hy0, czf + hz0}, {cxf + hx0, cyf - hy0, czf + hz0},
        {cxf - hx0, cyf - hy0, czf + hz0}, {cxf - hx0, cyf + hy0, czf + hz0},
        {cxf - hx0, cyf + hy0, czf - hz0}, {cxf - hx0, cyf - hy0, czf - hz0},
    };
    // Face corners are wound CCW when viewed from outside.
    unsigned int base = 0;
    pushQuad(mesh, xp, p[0], p[1], p[2], p[3], base); // +X
    pushQuad(mesh, xn, p[4], p[5], p[6], p[7], base); // -X
    pushQuad(mesh, yp, p[1], p[6], p[5], p[2], base); // +Y
    pushQuad(mesh, yn, p[0], p[3], p[4], p[7], base); // -Y
    pushQuad(mesh, zp, p[3], p[2], p[5], p[4], base); // +Z
    pushQuad(mesh, zn, p[0], p[7], p[6], p[1], base); // -Z
}

// Read a k3Double plug into an MPoint (x, y, z).
[[nodiscard]] MPoint readPoint3(const MPlug& plug)
{
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const double* v = plug.asMDataHandle().asDouble3();
    return MPoint(v[0], v[1], v[2]);
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
} // namespace

// ===========================================================================
// Lifecycle / registration
// ===========================================================================
MHWRender::MPxGeometryOverride* RigidBodyJointGeometryOverride::creator(const MObject& obj)
{
    return new RigidBodyJointGeometryOverride(obj);
}

RigidBodyJointGeometryOverride::RigidBodyJointGeometryOverride(const MObject& obj)
    : MHWRender::MPxGeometryOverride(obj), fJoint(obj)
{
}

RigidBodyJointGeometryOverride::~RigidBodyJointGeometryOverride()
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

MHWRender::DrawAPI RigidBodyJointGeometryOverride::supportedDrawAPIs() const
{
    return MHWRender::kOpenGL | MHWRender::kDirectX11 | MHWRender::kOpenGLCoreProfile;
}

bool RigidBodyJointGeometryOverride::requiresGeometryUpdate() const
{
    return true;
}

bool RigidBodyJointGeometryOverride::requiresUpdateRenderItems(const MDagPath& /*dagPath*/) const
{
    return true;
}

bool RigidBodyJointGeometryOverride::hasUIDrawables() const
{
    return true;
}

// ===========================================================================
// Data preparation
// ===========================================================================
void RigidBodyJointGeometryOverride::updateDG()
{
    MStatus stat;
    MFnDependencyNode fn(fJoint);
    const MPlug modePlug = fn.findPlug(RigidBodyJoint::aDrawMode, true, &stat);
    const MPlug typePlug = fn.findPlug(RigidBodyJoint::aJointType, true, &stat);
    const MPlug lminPlug = fn.findPlug(RigidBodyJoint::aJointLinearMin, true, &stat);
    const MPlug lmaxPlug = fn.findPlug(RigidBodyJoint::aJointLinearMax, true, &stat);

    if (modePlug.isNull())
        return;

    fState.drawMode = static_cast<short>(modePlug.asShort());
    fState.type = typePlug.isNull() ? 0 : static_cast<short>(typePlug.asShort());
    fState.linearMin = readPoint3(lminPlug);
    fState.linearMax = readPoint3(lmaxPlug);
}

void RigidBodyJointGeometryOverride::jointColor(float& cr, float& cg, float& cb) const
{
    const int t = (fState.type >= 0 && fState.type < 6) ? fState.type : 0;
    cr = kJointColors[t][0];
    cg = kJointColors[t][1];
    cb = kJointColors[t][2];
}

bool RigidBodyJointGeometryOverride::isSelected(const MDagPath& path)
{
    const MHWRender::DisplayStatus status = MHWRender::MGeometryUtilities::displayStatus(path);
    return status == MHWRender::kLead || status == MHWRender::kActive;
}

void RigidBodyJointGeometryOverride::selectionColor(const MDagPath& path, float& cr, float& cg,
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

void RigidBodyJointGeometryOverride::updateRenderItems(const MDagPath& /*dagPath*/,
                                                       MRenderItemList& renderItems)
{
    const bool drawSolid = (fState.drawMode == RigidBodyJoint::kDrawSolid ||
                            fState.drawMode == RigidBodyJoint::kDrawWireSolid);

    // The render item persists in the list (VP2 owns it) — find-or-create so
    // we never append a duplicate across frames.
    MRenderItem* solidItem = nullptr;
    const int index = renderItems.indexOf(kSolidItemName);
    if (index < 0)
    {
        solidItem = MRenderItem::Create(kSolidItemName, MRenderItem::NonMaterialSceneItem,
                                        MGeometry::kTriangles);
        if (solidItem != nullptr)
        {
            const MMatrix identity;
            solidItem->setMatrix(&identity);
            // Must match the viewport's display modes or the item never draws.
            solidItem->setDrawMode(
                static_cast<MGeometry::DrawMode>(MGeometry::kShaded | MGeometry::kTextured));
            // NonMaterialSceneItem + wire depth priority = lit by the viewport
            // lights AND drawn on top of the character mesh.
            solidItem->depthPriority(MRenderItem::sDormantWireDepthPriority);
            renderItems.append(solidItem);
        }
    }
    else
    {
        solidItem = renderItems.itemAt(index);
    }

    if (solidItem == nullptr)
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
            solidItem->setShader(fSolidShader);
    }

    float cr = 0.3f, cg = 0.85f, cb = 1.0f;
    jointColor(cr, cg, cb); // solid keeps its joint colour even when selected
    solidItem->enable(drawSolid);
    if (drawSolid && fSolidShader != nullptr)
    {
        fSolidShader->setParameter("baseColor", MFloatVector(cr, cg, cb));
    }
}

void RigidBodyJointGeometryOverride::populateGeometry(const MGeometryRequirements& requirements,
                                                      const MRenderItemList& renderItems,
                                                      MGeometry& data)
{
    // Build the limit-box mesh from the cached draw state (degenerate axes
    // get a sliver sized relative to the box's own extent).
    double spread = std::fabs(fState.linearMax.x - fState.linearMin.x);
    spread = std::max(spread, std::fabs(fState.linearMax.y - fState.linearMin.y));
    spread = std::max(spread, std::fabs(fState.linearMax.z - fState.linearMin.z));
    const double sliver = spread > 1e-4 ? spread * 0.12 : 0.05;

    BoxMesh mesh;
    buildLimitBox(fState.linearMin, fState.linearMax, sliver, mesh);
    if (mesh.positions.empty() || mesh.indices.empty())
        return;

    const unsigned int numVerts = static_cast<unsigned int>(mesh.positions.size() / 3);

    // Create vertex buffers from the shader's OWN requirements (position +
    // normal [+ uv] for the stock OpenPBR surface).  Fall back to manual
    // descriptors if the requirements come back empty.
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

void RigidBodyJointGeometryOverride::addUIDrawables(const MDagPath& objPath,
                                                    MUIDrawManager& drawMgr,
                                                    const MFrameContext& /*frameContext*/)
{
    // Selected joints ALWAYS show a native selection wireframe (on top, in
    // Maya's selection colour) regardless of drawMode — like any mesh.
    // Unselected joints show their type-colour wire only in wire modes.
    const bool selected = isSelected(objPath);
    const bool drawWire = selected || (fState.drawMode == RigidBodyJoint::kDrawWire ||
                                       fState.drawMode == RigidBodyJoint::kDrawWireSolid);
    if (!drawWire)
        return;

    double spread = std::fabs(fState.linearMax.x - fState.linearMin.x);
    spread = std::max(spread, std::fabs(fState.linearMax.y - fState.linearMin.y));
    spread = std::max(spread, std::fabs(fState.linearMax.z - fState.linearMin.z));
    const double sliver = spread > 1e-4 ? spread * 0.12 : 0.05;

    float cr = 1.0f, cg = 0.25f, cb = 0.25f;
    if (selected)
        selectionColor(objPath, cr, cg, cb);
    else
        jointColor(cr, cg, cb);

    BoxMesh mesh;
    buildLimitBox(fState.linearMin, fState.linearMax, sliver, mesh);

    drawMgr.beginDrawable();
    drawMgr.setColor(MColor(cr, cg, cb));
    // Draw the 12 box edges (each face quad contributes 4 edges; skip shared).
    const float v[8][3] = {
        {static_cast<float>(fState.linearMin.x), static_cast<float>(fState.linearMin.y),
         static_cast<float>(fState.linearMin.z)},
        {static_cast<float>(fState.linearMax.x), static_cast<float>(fState.linearMin.y),
         static_cast<float>(fState.linearMin.z)},
        {static_cast<float>(fState.linearMax.x), static_cast<float>(fState.linearMax.y),
         static_cast<float>(fState.linearMin.z)},
        {static_cast<float>(fState.linearMin.x), static_cast<float>(fState.linearMax.y),
         static_cast<float>(fState.linearMin.z)},
        {static_cast<float>(fState.linearMin.x), static_cast<float>(fState.linearMin.y),
         static_cast<float>(fState.linearMax.z)},
        {static_cast<float>(fState.linearMax.x), static_cast<float>(fState.linearMin.y),
         static_cast<float>(fState.linearMax.z)},
        {static_cast<float>(fState.linearMax.x), static_cast<float>(fState.linearMax.y),
         static_cast<float>(fState.linearMax.z)},
        {static_cast<float>(fState.linearMin.x), static_cast<float>(fState.linearMax.y),
         static_cast<float>(fState.linearMax.z)},
    };
    const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                              {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const auto& e : edges)
        drawMgr.line(MPoint(v[e[0]][0], v[e[0]][1], v[e[0]][2]),
                     MPoint(v[e[1]][0], v[e[1]][1], v[e[1]][2]));
    drawMgr.endDrawable();
}

void RigidBodyJointGeometryOverride::cleanUp()
{
    // Render items live in VP2's persistent list and are owned by VP2 —
    // nothing to release per frame.
}

// ===========================================================================
// Selection — object-level picking of the solid render item.
// ===========================================================================
bool RigidBodyJointGeometryOverride::refineSelectionPath(const MSelectionInfo& /*selectInfo*/,
                                                         const MRenderItem& /*hitItem*/,
                                                         MDagPath& dagPath,
                                                         MObject& /*geomComponents*/,
                                                         MSelectionMask& /*objectMask*/)
{
    MDagPath jointPath;
    if (!MDagPath::getAPathTo(fJoint, jointPath))
        return false;
    dagPath = jointPath;
    return true;
}

void RigidBodyJointGeometryOverride::updateSelectionGranularity(
    const MDagPath& /*dagPath*/, MSelectionContext& selectionContext)
{
    selectionContext.setSelectionLevel(MSelectionContext::kObject);
}
