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
#include "rigid_body_node.hpp"

#include <maya/MColor.h>
#include <maya/MDagPath.h>
#include <maya/MDistance.h>
#include <maya/MEulerRotation.h>
#include <maya/MFn.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnTransform.h>
#include <maya/MGeometry.h>
#include <maya/MHWGeometry.h>
#include <maya/MHWGeometryUtilities.h>
#include <maya/MMatrix.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MPoint.h>
#include <maya/MShaderManager.h>
#include <maya/MStatus.h>
#include <maya/MTransformationMatrix.h>
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

void pushQuad(BoxMesh& mesh, const float n[3], const float a[3], const float b[3], const float c[3],
              const float d[3], unsigned int& base)
{
    mesh.positions.insert(mesh.positions.end(),
                          {a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2]});
    mesh.normals.insert(mesh.normals.end(),
                        {n[0], n[1], n[2], n[0], n[1], n[2], n[0], n[1], n[2], n[0], n[1], n[2]});
    mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    base += 4;
}

// Read a k3Double plug into an MPoint (x, y, z).
[[nodiscard]] MPoint readPoint3(const MPlug& plug)
{
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const double* v = plug.asMDataHandle().asDouble3();
    return MPoint(v[0], v[1], v[2]);
}

// Read body `index`'s CURRENT pose (RigidBodies GROUP space) from the
// solver's outGuideTranslate array output.  Returns false when the element is
// missing (out of range / not yet evaluated).
[[nodiscard]] bool readBodyGuidePose(const MObject& solver, int index, MPoint& pos)
{
    if (solver.isNull() || index < 0)
        return false;
    MPlug tPlug(solver, RigidBodyNode::aOutGuideTranslate);
    const unsigned int count = tPlug.evaluateNumElements();
    if (static_cast<unsigned int>(index) >= count)
        return false;
    const MPlug el = tPlug.elementByLogicalIndex(static_cast<unsigned int>(index));
    if (el.isNull())
        return false;
    pos.x = el.child(RigidBodyNode::aOutGuideTranslateX).asMDistance().asCentimeters();
    pos.y = el.child(RigidBodyNode::aOutGuideTranslateY).asMDistance().asCentimeters();
    pos.z = el.child(RigidBodyNode::aOutGuideTranslateZ).asMDistance().asCentimeters();
    return true;
}

// The world matrix of the solver's DAG parent — the {model}_RigidBodies group
// whose space the solver's body-pose outputs (outGuideTranslate/Rotate) are
// expressed in.  Identity when it cannot be resolved.
[[nodiscard]] MMatrix solverGroupWorld(const MObject& solver)
{
    if (solver.isNull() || !solver.hasFn(MFn::kDagNode))
        return MMatrix();
    MStatus stat;
    MFnDagNode solverFn(solver, &stat);
    MObject group = stat ? solverFn.parent(0, &stat) : MObject::kNullObj;
    if (!stat || group.isNull())
        return MMatrix();
    MDagPath groupPath;
    if (MDagPath::getAPathTo(group, groupPath) != MS::kSuccess)
        return MMatrix();
    return groupPath.inclusiveMatrix();
}

// Draw a wireframe box's 12 edges spanning [lo, hi] (axis-aligned).
void drawBoxEdges(MHWRender::MUIDrawManager& drawMgr, const MPoint& lo, const MPoint& hi)
{
    const float v[8][3] = {
        {static_cast<float>(lo.x), static_cast<float>(lo.y), static_cast<float>(lo.z)},
        {static_cast<float>(hi.x), static_cast<float>(lo.y), static_cast<float>(lo.z)},
        {static_cast<float>(hi.x), static_cast<float>(hi.y), static_cast<float>(lo.z)},
        {static_cast<float>(lo.x), static_cast<float>(hi.y), static_cast<float>(lo.z)},
        {static_cast<float>(lo.x), static_cast<float>(lo.y), static_cast<float>(hi.z)},
        {static_cast<float>(hi.x), static_cast<float>(lo.y), static_cast<float>(hi.z)},
        {static_cast<float>(hi.x), static_cast<float>(hi.y), static_cast<float>(hi.z)},
        {static_cast<float>(lo.x), static_cast<float>(hi.y), static_cast<float>(hi.z)},
    };
    const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                              {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const auto& e : edges)
        drawMgr.line(MPoint(v[e[0]][0], v[e[0]][1], v[e[0]][2]),
                     MPoint(v[e[1]][0], v[e[1]][1], v[e[1]][2]));
}

// A flat-shaded unit cube of half-extent `h` centred at the origin — the
// joint's anchor marker (small, so it is the connector + limits that carry
// the information, not the solid).
void buildAnchorCube(double h, BoxMesh& mesh)
{
    const float hf = static_cast<float>(h);
    const float xp[3] = {1, 0, 0};
    const float xn[3] = {-1, 0, 0};
    const float yp[3] = {0, 1, 0};
    const float yn[3] = {0, -1, 0};
    const float zp[3] = {0, 0, 1};
    const float zn[3] = {0, 0, -1};
    const float p[8][3] = {
        {hf, -hf, -hf}, {hf, hf, -hf}, {hf, hf, hf},   {hf, -hf, hf},
        {-hf, -hf, hf}, {-hf, hf, hf}, {-hf, hf, -hf}, {-hf, -hf, -hf},
    };
    unsigned int base = 0;
    pushQuad(mesh, xp, p[0], p[1], p[2], p[3], base); // +X
    pushQuad(mesh, xn, p[4], p[5], p[6], p[7], base); // -X
    pushQuad(mesh, yp, p[1], p[6], p[5], p[2], base); // +Y
    pushQuad(mesh, yn, p[0], p[3], p[4], p[7], base); // -Y
    pushQuad(mesh, zp, p[3], p[2], p[5], p[4], base); // +Z
    pushQuad(mesh, zn, p[0], p[7], p[6], p[1], base); // -Z
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
    const MPlug bodyAPlug = fn.findPlug(RigidBodyJoint::aJointBodyA, true, &stat);
    const MPlug bodyBPlug = fn.findPlug(RigidBodyJoint::aJointBodyB, true, &stat);
    if (modePlug.isNull())
        return;

    fState.drawMode = static_cast<short>(modePlug.asShort());
    fState.type = typePlug.isNull() ? 0 : static_cast<short>(typePlug.asShort());
    fState.linearMin = readPoint3(lminPlug);
    fState.linearMax = readPoint3(lmaxPlug);

    // The connector links the two constrained bodies: resolve the owning
    // solver (joint.solver -> solver) and read bodyA/bodyB's CURRENT poses
    // (RigidBodies GROUP space) from its outGuideTranslate outputs.
    fState.hasBodyA = false;
    fState.hasBodyB = false;
    m_solver = MObject::kNullObj;
    MPlug solverPlug = fn.findPlug(RigidBodyJoint::aSolver, true, &stat);
    MPlugArray solverSrcs;
    if (!solverPlug.isNull())
        solverPlug.connectedTo(solverSrcs, true, false);
    MPoint frameGroup;
    if (solverSrcs.length() > 0)
    {
        m_solver = solverSrcs[0].node();
        const int bodyA = bodyAPlug.isNull() ? -1 : bodyAPlug.asInt();
        const int bodyB = bodyBPlug.isNull() ? -1 : bodyBPlug.asInt();
        fState.hasBodyA = readBodyGuidePose(m_solver, bodyA, fState.bodyAPos);
        fState.hasBodyB = readBodyGuidePose(m_solver, bodyB, fState.bodyBPos);
        // The frame in the SOLVER GROUP's space (the space the body poses
        // above are read in).  The joint may sit in a {model}_Joints
        // sub-group, so go through the world matrices instead of assuming
        // the guide is a direct child of the group.  Used only to size the
        // anchor.
        MFnDagNode dag(fJoint, &stat);
        MObject guide = stat ? dag.parent(0, &stat) : MObject::kNullObj;
        MDagPath guidePath;
        if (stat && !guide.isNull() && guide.hasFn(MFn::kTransform) &&
            MDagPath::getAPathTo(guide, guidePath) == MS::kSuccess)
        {
            const MMatrix groupInv = solverGroupWorld(m_solver).inverse();
            const MTransformationMatrix gtm(groupInv * guidePath.inclusiveMatrix());
            const MVector t = gtm.getTranslation(MSpace::kTransform);
            frameGroup = MPoint(t.x, t.y, t.z);
        }
    }

    // Adaptive anchor-cube size: relative to the connector extent (the limit
    // box and the frame-to-bodies distances), so it reads at any scene scale.
    double extent = std::fabs(fState.linearMax.x - fState.linearMin.x);
    extent = std::max(extent, std::fabs(fState.linearMax.y - fState.linearMin.y));
    extent = std::max(extent, std::fabs(fState.linearMax.z - fState.linearMin.z));
    if (fState.hasBodyA)
        extent = std::max(extent, (fState.bodyAPos - frameGroup).length());
    if (fState.hasBodyB)
        extent = std::max(extent, (fState.bodyBPos - frameGroup).length());
    fState.anchorHalf = extent > 1e-4 ? std::clamp(extent * 0.08, 0.03, 0.4) : 0.04;
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

void RigidBodyJointGeometryOverride::updateRenderItems(const MDagPath& dagPath,
                                                       MRenderItemList& renderItems)
{
    const bool drawSolid = (fState.drawMode == RigidBodyJoint::kDrawSolid ||
                            fState.drawMode == RigidBodyJoint::kDrawWireSolid);

    // Map the group-space body poses into the shape's LOCAL space so the
    // connector lines (drawn in addUIDrawables) land where the bodies are.
    // m_toLocal = shapeWorld^-1 * groupWorld (the solver's DAG parent is the
    // RigidBodies group).
    const MMatrix groupWorld = solverGroupWorld(m_solver);
    m_toLocal = dagPath.inclusiveMatrix().inverse() * groupWorld;
    fState.bodyAPos = fState.bodyAPos * m_toLocal;
    fState.bodyBPos = fState.bodyBPos * m_toLocal;

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
    // Build the small anchor-cube mesh (the joint marker at the frame — the
    // connector lines + limit wire carry the constraint information).
    BoxMesh mesh;
    buildAnchorCube(fState.anchorHalf, mesh);
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

    float cr, cg, cb;
    if (selected)
        selectionColor(objPath, cr, cg, cb);
    else
        jointColor(cr, cg, cb);
    const MColor color(cr, cg, cb);
    const MPoint frame(0.0, 0.0, 0.0); // the shape's local origin IS the frame

    drawMgr.beginDrawable();
    // Xray so the constraint stays visible even inside a collider or behind
    // the character mesh.
    drawMgr.beginDrawInXray();

    // 1. Connectors — the PRIMARY information: which two bodies this joint
    //    constrains, tracked to their CURRENT simulated pose (the group-space
    //    poses were mapped to local in updateRenderItems).
    drawMgr.setLineWidth(2.0f);
    drawMgr.setColor(color);
    if (fState.hasBodyA)
        drawMgr.line(fState.bodyAPos, frame);
    if (fState.hasBodyB)
        drawMgr.line(frame, fState.bodyBPos);

    // 2. Limit range as a thin wire box (the allowed translation range),
    //    degenerate axes get a sliver so sliders still show a visible box.
    drawMgr.setLineWidth(1.0f);
    double spread = std::fabs(fState.linearMax.x - fState.linearMin.x);
    spread = std::max(spread, std::fabs(fState.linearMax.y - fState.linearMin.y));
    spread = std::max(spread, std::fabs(fState.linearMax.z - fState.linearMin.z));
    const double sliver = spread > 1e-4 ? spread * 0.12 : fState.anchorHalf * 0.6;
    const auto axisBox = [&](double lo, double hi, double spr) -> std::pair<double, double>
    {
        const double half = spr > 1e-4 ? 0.5 * std::fabs(hi - lo) : sliver;
        return {0.5 * (lo + hi), half};
    };
    const auto [cx, hx] = axisBox(fState.linearMin.x, fState.linearMax.x, spread);
    const auto [cy, hy] = axisBox(fState.linearMin.y, fState.linearMax.y, spread);
    const auto [cz, hz] = axisBox(fState.linearMin.z, fState.linearMax.z, spread);
    drawMgr.setColor(color);
    drawBoxEdges(drawMgr, MPoint(cx - hx, cy - hy, cz - hz), MPoint(cx + hx, cy + hy, cz + hz));

    // 3. The anchor cube's wire outline (the solid is the render item).
    //    When selected this doubles as the native selection wireframe.
    const double h = fState.anchorHalf;
    drawMgr.setColor(color);
    drawBoxEdges(drawMgr, MPoint(-h, -h, -h), MPoint(h, h, h));

    // 4. Frame orientation triad (X red, Y green, Z blue), sized relative to
    //    the anchor so it reads at any scene scale.
    const double len = fState.anchorHalf * 3.0;
    drawMgr.setColor(MColor(1.0f, 0.2f, 0.2f));
    drawMgr.line(frame, frame + MVector(len, 0.0, 0.0));
    drawMgr.setColor(MColor(0.2f, 1.0f, 0.2f));
    drawMgr.line(frame, frame + MVector(0.0, len, 0.0));
    drawMgr.setColor(MColor(0.2f, 0.4f, 1.0f));
    drawMgr.line(frame, frame + MVector(0.0, 0.0, len));

    drawMgr.setLineWidth(1.0f);
    drawMgr.endDrawInXray();
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

void RigidBodyJointGeometryOverride::updateSelectionGranularity(const MDagPath& /*dagPath*/,
                                                                MSelectionContext& selectionContext)
{
    selectionContext.setSelectionLevel(MSelectionContext::kObject);
}
