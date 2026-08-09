/*
 * SPDX-License-Identifier: MIT
 *
 * simulation.cpp
 *
 * mmd::core::Simulation implementation — see simulation.hpp.  Maya-free
 * Bullet engine (no Maya headers).  Compiled into the plugin AND into the
 * unit-test target (tests/core/test_simulation.cpp), which proves it is
 * Maya-free.
 *
 * The simulation runs in the physics group's LOCAL space; the node supplies
 * and consumes plain poses (pos+quat).  All matrix/unit conversion is the
 * node's job — the pure math helpers come from physics_math.hpp.
 */

#include "simulation.hpp"
#include "physics_math.hpp"

#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <algorithm>

using namespace mmd::core::physics_math;

namespace
{
// PMX JointType -> Bullet constraint selection
constexpr int kJointSpring6Dof = 0;
constexpr int kJointSixDof = 1;
constexpr int kJointP2P = 2;
constexpr int kJointConeTwist = 3;
constexpr int kJointSlider = 4;
constexpr int kJointHinge = 5;

// Simulation stepping constants (see initialize()/step()).
constexpr int kSolverIterations = 30;       // > Bullet's default 10 — long rigid chains need it
constexpr int kMaxSubSteps = 8;             // max internal steps per step()
constexpr double kMaxStepTime = 0.5;        // clamp for huge time jumps (scrub/tab)
constexpr double kMinShapeSize = 1e-4;      // floor for shape sizes — 0 would make Bullet assert
constexpr float kConstraintSoftness = 0.3F; // cone/hinge limit softness (Bullet tuning)
constexpr double kAnchorMoveEps2 = 1e-6;    // squared-distance threshold for anchor movement
constexpr double kAnchorMoveEpsRot = 1e-5;  // column-dot threshold for anchor rotation
} // namespace

namespace mmd
{
namespace core
{

Simulation::Simulation() = default;

Simulation::~Simulation()
{
    clear();
}

void Simulation::clear()
{
    // CRITICAL teardown order: destroy the WORLD first while every body and
    // constraint is still alive.  btCollisionWorld's destructor iterates
    // m_collisionObjects and calls getBroadphase()->destroyProxy() on each —
    // if the bodies were already freed that is a use-after-free (access
    // violation, seen as intermittent Maya crashes during scene teardown).
    // (Member destruction would do this automatically — mWorld is declared
    // last — but clear() is also called on re-initialize, so it is explicit.)
    mWorld.reset(); // base dtor cleans up broadphase proxies on live bodies
    mConstraints.clear();
    mRigidBodies.clear();
    mShapes.clear();
    mConstraintSolver.reset();
    mBroadphase.reset();
    mDispatcher.reset();
    mCollisionConfig.reset();
    mBodies.clear();
    mKinematicBodyIndices.clear();
    mAnchorRest.clear();
    mAnchorCurrent.clear();
    mWorldBuilt = false;
}

bool Simulation::initialize(const Definition& definition)
{
    clear();
    if (definition.bodies.empty())
    {
        return false;
    }

    // ---- Runtime body state (body-indexed) + kinematic anchor order ----
    mBodies.clear();
    mBodies.reserve(definition.bodies.size());
    mKinematicBodyIndices.clear();
    for (const BodyDefinition& bd : definition.bodies)
    {
        Body b;
        b.restPos = bd.restPos;
        b.restRot = bd.restRot;
        b.mass = bd.mass;
        b.linearDamping = bd.linearDamping;
        b.angularDamping = bd.angularDamping;
        b.friction = bd.friction;
        b.restitution = bd.restitution;
        b.colliderType = bd.colliderType;
        b.radius = bd.radius;
        b.extents = bd.extents;
        b.length = bd.length;
        b.mask = bd.mask;
        b.groupId = bd.groupId;
        b.kinematic = bd.isKinematic();
        b.enabled = bd.enabled;
        b.resetAnchorIndex = bd.resetAnchorIndex;
        mBodies.push_back(b);
        if (b.kinematic && b.enabled)
        {
            mKinematicBodyIndices.push_back(mBodies.size() - 1);
        }
    }

    createWorld(definition.gravity);
    createBodies();

    for (const JointDefinition& j : definition.joints)
    {
        createJoint(j);
    }

    mWorldBuilt = true;
    return true;
}

void Simulation::createWorld(const Double3& gravity)
{
    // The world does NOT own the dispatcher / broadphase / collision config /
    // solver — held BY VALUE (std::optional) so nothing here is heap-allocated;
    // clear() frees them in reverse order (world first).
    mCollisionConfig.emplace();
    mDispatcher.emplace(&*mCollisionConfig);
    mBroadphase.emplace();
    mConstraintSolver.emplace();
    mWorld.emplace(&*mDispatcher, &*mBroadphase, &*mConstraintSolver, &*mCollisionConfig);
    mWorld->setGravity(btVector3(static_cast<btScalar>(gravity.x), static_cast<btScalar>(gravity.y),
                                 static_cast<btScalar>(gravity.z)));
    // Long rigid chains (MMD skirt/hair/ponytail strands are 10-30 links) need
    // more constraint iterations than Bullet's default 10, or the tension never
    // propagates and the chain detaches from its kinematic anchor (free-falls).
    mWorld->getSolverInfo().m_numIterations = kSolverIterations;
}

void Simulation::createBodies()
{
    if (!mWorld)
    {
        return;
    }
    mRigidBodies.clear();
    mRigidBodies.resize(mBodies.size()); // null unique_ptrs = disabled placeholders
    mShapes.clear();
    mAnchorRest.clear();
    mAnchorCurrent.clear();
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        Body& b = mBodies[i];
        if (!b.enabled)
        {
            // Disabled (removed): keep the body index ALIGNED so bodyPose() and
            // the draw data stay body-indexed — store a null placeholder and
            // never add it to the world (no collision, no simulation).  A
            // disabled kinematic body also gets no anchor entry.
            mRigidBodies[i] = nullptr;
            continue;
        }
        const btTransform start = transformFromRest(b.restPos, b.restRot);

        btCollisionShape* shape = nullptr;
        if (b.colliderType == ColliderType::eSphere)
        {
            shape = new btSphereShape(static_cast<btScalar>(std::max(b.radius, kMinShapeSize)));
            mShapes.emplace_back(shape);
        }
        else if (b.colliderType == ColliderType::eBox)
        {
            shape = new btBoxShape(
                btVector3(static_cast<btScalar>(std::max(b.extents.x, kMinShapeSize)),
                          static_cast<btScalar>(std::max(b.extents.y, kMinShapeSize)),
                          static_cast<btScalar>(std::max(b.extents.z, kMinShapeSize))));
            mShapes.emplace_back(shape);
        }
        else // capsule — btCapsuleShape is ALREADY Y-axis (m_upAxis = 1), which
             // matches MMD's vertical capsule and the polyCylinder guide mesh.
             // (An earlier "Bullet capsule axis is Z" rotation was WRONG — it
             // turned every capsule sideways, e.g. the torso capsule pointed its
             // hemispherical cap at the skirt and pushed it ~1 unit out, making
             // the skirt float with a visible gap from the body.)
        {
            auto* capsule =
                new btCapsuleShape(static_cast<btScalar>(std::max(b.radius, kMinShapeSize)),
                                   static_cast<btScalar>(std::max(b.length, kMinShapeSize)));
            mShapes.emplace_back(capsule);
            shape = capsule;
        }

        const btScalar mass =
            b.kinematic ? static_cast<btScalar>(0.0) : static_cast<btScalar>(std::max(b.mass, 0.0));
        btVector3 localInertia(0, 0, 0);
        if (mass > 0.0)
        {
            shape->calculateLocalInertia(mass, localInertia);
        }

        auto* motionState = new btDefaultMotionState(start);
        btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState, shape, localInertia);
        ci.m_linearDamping = static_cast<btScalar>(b.linearDamping);
        ci.m_angularDamping = static_cast<btScalar>(b.angularDamping);
        ci.m_friction = static_cast<btScalar>(b.friction);
        ci.m_restitution = static_cast<btScalar>(b.restitution);
        auto* body = new btRigidBody(ci);

        if (b.kinematic)
        {
            body->setCollisionFlags(body->getCollisionFlags() |
                                    btCollisionObject::CF_KINEMATIC_OBJECT);
            body->setActivationState(DISABLE_DEACTIVATION);
            body->setGravity(btVector3(0, 0, 0));
            // Record the anchor's REST pose (group-local) for scrub-back.
            mAnchorRest.emplace_back(AnchorPose());
            storeAnchorPose(mAnchorRest.back().pos, mAnchorRest.back().quat, start);
            // mAnchorCurrent must be the SAME size as mAnchorRest — it is
            // refreshed every frame in setKinematicPose and drives the
            // scrub-back reset (empty would silently disable the rewind).
            mAnchorCurrent.emplace_back(AnchorPose());
            storeAnchorPose(mAnchorCurrent.back().pos, mAnchorCurrent.back().quat, start);
        }
        else
        {
            body->setActivationState(ISLAND_SLEEPING); // wake on first step
            body->activate();
        }

        // Scrub-back reset: capture the constant offset bodyRest = anchorRest *
        // offset, where the anchor is the kinematic body whose bone is this
        // body's nearest kinematic ancestor (mapped by the node's Python).  On
        // rewind the body is teleported to anchorCurrent * offset — i.e. its
        // rest pose transformed by the CURRENT skeleton pose, instead of
        // rebuilding at the PMX rest pose while the skeleton is at another frame.
        if (!b.kinematic && b.resetAnchorIndex >= 0 &&
            b.resetAnchorIndex < static_cast<int>(mAnchorRest.size()))
        {
            const btTransform anchorRest = anchorPoseToTransform(
                mAnchorRest[b.resetAnchorIndex].pos, mAnchorRest[b.resetAnchorIndex].quat);
            const btTransform offset = anchorRest.inverse() * start;
            b.hasBoneReset = true;
            const btVector3& o = offset.getOrigin();
            const btQuaternion& q = offset.getRotation();
            b.resetOffsetPos.x = o.x();
            b.resetOffsetPos.y = o.y();
            b.resetOffsetPos.z = o.z();
            b.resetOffsetQuat.x = q.x();
            b.resetOffsetQuat.y = q.y();
            b.resetOffsetQuat.z = q.z();
            b.resetOffsetQuat.w = q.w();
        }

        // Bullet group bit from the raw PMX group id (legacy scenes without
        // it keep the default group 0); b.mask is the PMX non_collision_group
        // bools stored verbatim.
        const long group = 1L << ((b.groupId >= 0 ? b.groupId : 0) & 0x0F);
        mWorld->addRigidBody(body, group, b.mask);
        mRigidBodies[i].reset(body);
    }
}

void Simulation::createJoint(const JointDefinition& j)
{
    if (!mWorld)
    {
        return;
    }
    if (j.bodyA < 0 || j.bodyB < 0 || j.bodyA >= static_cast<long>(mRigidBodies.size()) ||
        j.bodyB >= static_cast<long>(mRigidBodies.size()))
    {
        return;
    }
    // Skip joints that reference a disabled (removed) body.
    if (!mBodies[j.bodyA].enabled || !mBodies[j.bodyB].enabled)
    {
        return;
    }

    btRigidBody* rbA = mRigidBodies[j.bodyA].get();
    btRigidBody* rbB = mRigidBodies[j.bodyB].get();
    const btTransform frameWorld = transformFromRest(j.frameT, j.frameR);
    btTransform frameInA = rbA->getWorldTransform().inverse() * frameWorld;
    btTransform frameInB = rbB->getWorldTransform().inverse() * frameWorld;

    btTypedConstraint* con = nullptr;
    switch (j.type)
    {
    case kJointSpring6Dof:
    {
        // MMD maps EVERY SPRING_6DOF joint to btGeneric6DofSpring2Constraint —
        // that is exactly what its physics engine creates.  The spring-2
        // limit motor treats upper==lower as LOCKED (see
        // btTranslationalLimitMotor2), so:
        //   * zero springs + zero limits -> proper RIGID WELD (locked),
        //     without btFixedConstraint's infinite-stiffness spring creep;
        //   * zero springs + real limits -> flexible 6DOF (limited);
        //   * nonzero springs            -> springy (PMX stiffness).
        // This is the single mapping that matches MMD's behaviour for the
        // whole joints data (rigid hair/cape chains, springy skirt, etc.).
        auto* g6 = new btGeneric6DofSpring2Constraint(*rbA, *rbB, frameInA, frameInB);
        g6->setLinearLowerLimit(btVector3(static_cast<btScalar>(j.linearMin.x),
                                          static_cast<btScalar>(j.linearMin.y),
                                          static_cast<btScalar>(j.linearMin.z)));
        g6->setLinearUpperLimit(btVector3(static_cast<btScalar>(j.linearMax.x),
                                          static_cast<btScalar>(j.linearMax.y),
                                          static_cast<btScalar>(j.linearMax.z)));
        g6->setAngularLowerLimit(btVector3(static_cast<btScalar>(j.angularMin.x),
                                           static_cast<btScalar>(j.angularMin.y),
                                           static_cast<btScalar>(j.angularMin.z)));
        g6->setAngularUpperLimit(btVector3(static_cast<btScalar>(j.angularMax.x),
                                           static_cast<btScalar>(j.angularMax.y),
                                           static_cast<btScalar>(j.angularMax.z)));
        for (int ax = 0; ax < 3; ++ax)
        {
            if (j.linearSpring[ax] != 0)
            {
                g6->enableSpring(ax, true);
                g6->setStiffness(ax, static_cast<btScalar>(j.linearSpring[ax]));
            }
            if (j.angularSpring[ax] != 0)
            {
                g6->enableSpring(ax + 3, true);
                g6->setStiffness(ax + 3, static_cast<btScalar>(j.angularSpring[ax]));
            }
        }
        con = g6;
        break;
    }
    case kJointSixDof:
    {
        auto* g6 = new btGeneric6DofConstraint(*rbA, *rbB, frameInA, frameInB, true);
        g6->setLinearLowerLimit(btVector3(static_cast<btScalar>(j.linearMin.x),
                                          static_cast<btScalar>(j.linearMin.y),
                                          static_cast<btScalar>(j.linearMin.z)));
        g6->setLinearUpperLimit(btVector3(static_cast<btScalar>(j.linearMax.x),
                                          static_cast<btScalar>(j.linearMax.y),
                                          static_cast<btScalar>(j.linearMax.z)));
        g6->setAngularLowerLimit(btVector3(static_cast<btScalar>(j.angularMin.x),
                                           static_cast<btScalar>(j.angularMin.y),
                                           static_cast<btScalar>(j.angularMin.z)));
        g6->setAngularUpperLimit(btVector3(static_cast<btScalar>(j.angularMax.x),
                                           static_cast<btScalar>(j.angularMax.y),
                                           static_cast<btScalar>(j.angularMax.z)));
        con = g6;
        break;
    }
    case kJointP2P:
    {
        const btVector3 pivotInA = frameInA.getOrigin();
        const btVector3 pivotInB = frameInB.getOrigin();
        con = new btPoint2PointConstraint(*rbA, *rbB, pivotInA, pivotInB);
        break;
    }
    case kJointConeTwist:
    {
        auto* ct = new btConeTwistConstraint(*rbA, *rbB, frameInA, frameInB);
        ct->setLimit(static_cast<btScalar>(j.angularMin.y), static_cast<btScalar>(j.angularMax.y),
                     0.0F, kConstraintSoftness, 0.0F, 1.0F);
        con = ct;
        break;
    }
    case kJointSlider:
    {
        auto* sl = new btSliderConstraint(*rbA, *rbB, frameInA, frameInB, true);
        sl->setLowerLinLimit(static_cast<btScalar>(j.linearMin.y));
        sl->setUpperLinLimit(static_cast<btScalar>(j.linearMax.y));
        sl->setLowerAngLimit(static_cast<btScalar>(j.angularMin.y));
        sl->setUpperAngLimit(static_cast<btScalar>(j.angularMax.y));
        con = sl;
        break;
    }
    case kJointHinge:
    {
        auto* hi = new btHingeConstraint(*rbA, *rbB, frameInA, frameInB, true);
        hi->setLimit(static_cast<btScalar>(j.angularMin.y), static_cast<btScalar>(j.angularMax.y),
                     kConstraintSoftness, 0.0F, 1.0F);
        con = hi;
        break;
    }
    default:
        break;
    }

    if (con != nullptr)
    {
        mWorld->addConstraint(con, /*disableCollisionsBetweenLinkedBodies=*/true);
        mConstraints.emplace_back(con);
    }
}

bool Simulation::initialized() const
{
    return mWorldBuilt;
}

bool Simulation::setKinematicPose(size_t anchorIndex, const Pose& pose)
{
    if (anchorIndex >= mKinematicBodyIndices.size())
    {
        return false;
    }
    const size_t bodyIdx = mKinematicBodyIndices[anchorIndex];
    btRigidBody* body = mRigidBodies[bodyIdx].get();
    if (body == nullptr)
    {
        return false;
    }

    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(static_cast<btScalar>(pose.pos.x), static_cast<btScalar>(pose.pos.y),
                          static_cast<btScalar>(pose.pos.z)));
    t.setRotation(
        btQuaternion(static_cast<btScalar>(pose.quat.x), static_cast<btScalar>(pose.quat.y),
                     static_cast<btScalar>(pose.quat.z), static_cast<btScalar>(pose.quat.w)));
    body->setWorldTransform(t);
    body->getMotionState()->setWorldTransform(t);

    // Detect movement (e.g. a bone dragged in the viewport at the current
    // frame): if an anchor moved but time did not advance, the sim still needs
    // to step so attached chains follow the bone immediately (MMD reacts to
    // bone changes instantly — not on the next frame).
    bool moved = false;
    if (anchorIndex < mAnchorCurrent.size())
    {
        const btTransform prev = anchorPoseToTransform(mAnchorCurrent[anchorIndex].pos,
                                                       mAnchorCurrent[anchorIndex].quat);
        const btVector3 d = t.getOrigin() - prev.getOrigin();
        const btVector3 c0 = t.getBasis().getColumn(0);
        const btVector3 p0 = prev.getBasis().getColumn(0);
        const btVector3 c1 = t.getBasis().getColumn(1);
        const btVector3 p1 = prev.getBasis().getColumn(1);
        if (d.length2() > btScalar(kAnchorMoveEps2) ||
            c0.dot(p0) < btScalar(1.0) - btScalar(kAnchorMoveEpsRot) ||
            c1.dot(p1) < btScalar(1.0) - btScalar(kAnchorMoveEpsRot))
        {
            moved = true;
        }
    }
    if (anchorIndex < mAnchorCurrent.size())
    {
        storeAnchorPose(mAnchorCurrent[anchorIndex].pos, mAnchorCurrent[anchorIndex].quat, t);
    }
    return moved;
}

void Simulation::step(double dt)
{
    if (!mWorld)
    {
        return;
    }
    dt = std::min(dt, kMaxStepTime); // guard against huge jumps
    mWorld->stepSimulation(btScalar(dt), kMaxSubSteps, btScalar(kFixedDt));
}

void Simulation::resetDynamicBodies()
{
    if (!mWorld)
    {
        return;
    }
    // Teleport every dynamic body (that has a reset anchor) to its rest pose
    // transformed by the CURRENT skeleton pose, zeroing velocities.  Uses the
    // anchor CURRENT poses captured by setKinematicPose.
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        const Body& b = mBodies[i];
        if (b.kinematic || !b.enabled || !b.hasBoneReset)
        {
            continue;
        }
        const int aIdx = b.resetAnchorIndex;
        if (aIdx < 0 || aIdx >= static_cast<int>(mAnchorCurrent.size()))
        {
            continue;
        }

        const btTransform anchorCurrent =
            anchorPoseToTransform(mAnchorCurrent[aIdx].pos, mAnchorCurrent[aIdx].quat);
        btTransform offset;
        offset.setIdentity();
        offset.setOrigin(btVector3(btScalar(b.resetOffsetPos.x), btScalar(b.resetOffsetPos.y),
                                   btScalar(b.resetOffsetPos.z)));
        offset.setRotation(
            btQuaternion(btScalar(b.resetOffsetQuat.x), btScalar(b.resetOffsetQuat.y),
                         btScalar(b.resetOffsetQuat.z), btScalar(b.resetOffsetQuat.w)));
        const btTransform target = anchorCurrent * offset;

        btRigidBody* body = mRigidBodies[i].get();
        if (body == nullptr)
        {
            continue;
        }
        body->setWorldTransform(target);
        body->getMotionState()->setWorldTransform(target);
        body->setLinearVelocity(btVector3(0, 0, 0));
        body->setAngularVelocity(btVector3(0, 0, 0));
        body->setActivationState(DISABLE_DEACTIVATION);
        body->activate();
    }
}

Simulation::Pose Simulation::bodyPose(size_t bodyIndex) const
{
    Pose p;
    if (mWorld && bodyIndex < mRigidBodies.size() && mRigidBodies[bodyIndex] != nullptr)
    {
        // Solved pose — what the simulation actually has right now.
        const btTransform& t = mRigidBodies[bodyIndex]->getWorldTransform();
        storeAnchorPose(p.pos, p.quat, t);
    }
    else if (bodyIndex < mBodies.size())
    {
        // Disabled/missing body (or world not initialized): rest pose.
        const Body& b = mBodies[bodyIndex];
        p.pos = b.restPos;
        const btQuaternion q = eulerDegreesToQuat(b.restRot.x, b.restRot.y, b.restRot.z);
        p.quat.x = q.x();
        p.quat.y = q.y();
        p.quat.z = q.z();
        p.quat.w = q.w();
    }
    return p;
}

} // namespace core
} // namespace mmd
