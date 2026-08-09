/*
 * SPDX-License-Identifier: MIT
 *
 * simulation.cpp
 *
 * mmd::core::Simulation implementation — see simulation.hpp.  All Bullet
 * state lives in the private Impl (PIMPL), so simulation.hpp stays
 * Bullet-free; the public Simulation methods are thin delegates.
 *
 * The simulation runs in the physics group's LOCAL space; the node supplies
 * and consumes plain poses (pos+quat).  All matrix/unit conversion is the
 * node's job — the pure math helpers come from physics_math.hpp and the
 * Bullet conversions from bullet_bridge.hpp.
 */

#include "simulation.hpp"
#include "bullet_bridge.hpp"
#include "physics_math.hpp"

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

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
constexpr int kSolverIterations = 30;         // > Bullet's default 10 — long rigid chains need it
constexpr int kMaxSubSteps = 8;               // max internal steps per step()
constexpr double kMaxStepTime = 0.5;          // clamp for huge time jumps (scrub/tab)
constexpr double kMinShapeSize = 1e-4;        // floor for shape sizes — 0 would make Bullet assert
constexpr float kConstraintSoftness = 0.3F;   // cone/hinge limit softness (Bullet tuning)
constexpr double kAnchorMoveDistEpsSq = 1e-6; // squared-distance threshold for anchor movement
constexpr double kAnchorMoveRotEps = 1e-5;    // column-dot threshold for anchor rotation

// Shared 6DOF linear/angular limit setup — identical for the SPRING_6DOF
// (btGeneric6DofSpring2Constraint) and 6DOF (btGeneric6DofConstraint) cases.
template <typename ConstraintT>
void applySixDofLimits(ConstraintT& constraint, const mmd::core::Simulation::JointDefinition& j)
{
    constraint.setLinearLowerLimit(btVector3(static_cast<btScalar>(j.linearMin.x),
                                             static_cast<btScalar>(j.linearMin.y),
                                             static_cast<btScalar>(j.linearMin.z)));
    constraint.setLinearUpperLimit(btVector3(static_cast<btScalar>(j.linearMax.x),
                                             static_cast<btScalar>(j.linearMax.y),
                                             static_cast<btScalar>(j.linearMax.z)));
    constraint.setAngularLowerLimit(btVector3(static_cast<btScalar>(j.angularMin.x),
                                              static_cast<btScalar>(j.angularMin.y),
                                              static_cast<btScalar>(j.angularMin.z)));
    constraint.setAngularUpperLimit(btVector3(static_cast<btScalar>(j.angularMax.x),
                                              static_cast<btScalar>(j.angularMax.y),
                                              static_cast<btScalar>(j.angularMax.z)));
}
} // namespace

namespace mmd::core
{

// =========================================================================
// Impl — every Bullet object + the core runtime state (PIMPL).
//
// The world does NOT own its dispatcher / broadphase / collision config /
// solver, and rigid bodies do NOT own their collision shapes — the Impl keeps
// them alive and tears them down in clear().  The world + its support objects
// are held BY VALUE in std::optional (no heap allocation) and mWorld is
// declared LAST so that when members are destroyed (reverse declaration order)
// the world goes down FIRST, while every body / shape / constraint it
// references is still alive — exactly what btCollisionWorld's destructor
// needs (it walks m_collisionObjects and calls destroyProxy() on each live
// body).
// =========================================================================
struct Simulation::SimulationImpl
{
    std::optional<btDefaultCollisionConfiguration> mCollisionConfig;
    std::optional<btCollisionDispatcher> mDispatcher;
    std::optional<btDbvtBroadphase> mBroadphase;
    std::optional<btSequentialImpulseConstraintSolver> mConstraintSolver;
    // Bodies / shapes / constraints are polymorphic Bullet types, so they stay
    // heap-allocated (std::unique_ptr); the explicit reset order in clear()
    // keeps them alive until after the world is gone.
    std::vector<std::unique_ptr<btRigidBody>> mRigidBodies; // body-indexed (null = disabled)
    std::vector<std::unique_ptr<btTypedConstraint>> mConstraints;
    std::vector<std::unique_ptr<btCollisionShape>> mShapes;
    std::vector<Body> mBodies; // body-indexed runtime state (Body embeds BodyDefinition)
    std::vector<size_t> mKinematicBodyIndices; // kinematic body indices, in anchor order
    std::vector<Pose> mAnchorRest;             // anchor rest poses (kinematic order)
    std::vector<Pose> mAnchorCurrent;          // anchor current poses (kinematic order)
    bool mWorldBuilt = false;
    std::optional<btDiscreteDynamicsWorld> mWorld; // declared last → destroyed first

    bool initialize(const Definition& definition);
    void clear();
    bool initialized() const;
    bool setKinematicPose(size_t anchorIndex, const Pose& pose);
    void step(double dt);
    void resetDynamicBodies();
    Pose bodyPose(size_t bodyIndex) const;

  private:
    void createWorld(const Double3& gravity);
    void createBodies();
    void createJoint(const JointDefinition& joint);
};

void Simulation::SimulationImpl::clear()
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

bool Simulation::SimulationImpl::initialize(const Definition& definition)
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
        b.def = bd; // BodyDefinition carries every rest/physics field
        mBodies.push_back(b);
        if (bd.isKinematic() && bd.enabled)
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

void Simulation::SimulationImpl::createWorld(const Double3& gravity)
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

void Simulation::SimulationImpl::createBodies()
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
        if (!b.def.enabled)
        {
            // Disabled (removed): keep the body index ALIGNED so bodyPose() and
            // the draw data stay body-indexed — store a null placeholder and
            // never add it to the world (no collision, no simulation).  A
            // disabled kinematic body also gets no anchor entry.
            mRigidBodies[i] = nullptr;
            continue;
        }
        const btTransform start = transformFromRest(b.def.restPos, b.def.restRot);

        btCollisionShape* shape = nullptr;
        if (b.def.colliderType == ColliderType::eSphere)
        {
            mShapes.push_back(std::make_unique<btSphereShape>(
                static_cast<btScalar>(std::max(b.def.radius, kMinShapeSize))));
            shape = mShapes.back().get();
        }
        else if (b.def.colliderType == ColliderType::eBox)
        {
            mShapes.push_back(std::make_unique<btBoxShape>(
                btVector3(static_cast<btScalar>(std::max(b.def.extents.x, kMinShapeSize)),
                          static_cast<btScalar>(std::max(b.def.extents.y, kMinShapeSize)),
                          static_cast<btScalar>(std::max(b.def.extents.z, kMinShapeSize)))));
            shape = mShapes.back().get();
        }
        else // capsule — btCapsuleShape is ALREADY Y-axis (m_upAxis = 1), which
             // matches MMD's vertical capsule and the polyCylinder guide mesh.
             // (An earlier "Bullet capsule axis is Z" rotation was WRONG — it
             // turned every capsule sideways, e.g. the torso capsule pointed its
             // hemispherical cap at the skirt and pushed it ~1 unit out, making
             // the skirt float with a visible gap from the body.)
        {
            mShapes.push_back(std::make_unique<btCapsuleShape>(
                static_cast<btScalar>(std::max(b.def.radius, kMinShapeSize)),
                static_cast<btScalar>(std::max(b.def.length, kMinShapeSize))));
            shape = mShapes.back().get();
        }

        const btScalar mass = b.def.isKinematic()
                                  ? static_cast<btScalar>(0.0)
                                  : static_cast<btScalar>(std::max(b.def.mass, 0.0));
        btVector3 localInertia(0, 0, 0);
        if (mass > 0.0)
        {
            shape->calculateLocalInertia(mass, localInertia);
        }

        // btRigidBody takes ownership of the motion state (deletes it in its
        // dtor), so the unique_ptr is released once the body owns it.
        std::unique_ptr<btDefaultMotionState> motionState =
            std::make_unique<btDefaultMotionState>(start);
        btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState.get(), shape, localInertia);
        ci.m_linearDamping = static_cast<btScalar>(b.def.linearDamping);
        ci.m_angularDamping = static_cast<btScalar>(b.def.angularDamping);
        ci.m_friction = static_cast<btScalar>(b.def.friction);
        ci.m_restitution = static_cast<btScalar>(b.def.restitution);
        auto body = std::make_unique<btRigidBody>(ci);
        (void) motionState.release(); // btRigidBody now owns the motion state

        if (b.def.isKinematic())
        {
            body->setCollisionFlags(body->getCollisionFlags() |
                                    btCollisionObject::CF_KINEMATIC_OBJECT);
            body->setActivationState(DISABLE_DEACTIVATION);
            body->setGravity(btVector3(0, 0, 0));
            // Record the anchor's REST pose (group-local) for scrub-back.
            mAnchorRest.emplace_back();
            storePose(mAnchorRest.back().pos, mAnchorRest.back().quat, start);
            // mAnchorCurrent must be the SAME size as mAnchorRest — it is
            // refreshed every frame in setKinematicPose and drives the
            // scrub-back reset (empty would silently disable the rewind).
            mAnchorCurrent.emplace_back();
            storePose(mAnchorCurrent.back().pos, mAnchorCurrent.back().quat, start);
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
        if (!b.def.isKinematic() && b.def.resetAnchorIndex >= 0 &&
            b.def.resetAnchorIndex < static_cast<int>(mAnchorRest.size()))
        {
            const btTransform anchorRest = poseToTransform(
                mAnchorRest[b.def.resetAnchorIndex].pos, mAnchorRest[b.def.resetAnchorIndex].quat);
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
        // it keep the default group 0); b.def.mask is the collision filter
        // mask passed verbatim to Bullet.  addRigidBody takes shorts; group 15
        // (1 << 15 = 0x8000) does not fit a signed short and truncates to
        // -32768 — the same limitation MMD itself has, noted so a future
        // "fix" doesn't change it silently.
        const short group =
            static_cast<short>(1 << ((b.def.groupId >= 0 ? b.def.groupId : 0) & 0x0F));
        mWorld->addRigidBody(body.get(), group, static_cast<short>(b.def.mask));
        mRigidBodies[i] = std::move(body);
    }
}

void Simulation::SimulationImpl::createJoint(const JointDefinition& j)
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
    if (!mBodies[j.bodyA].def.enabled || !mBodies[j.bodyB].def.enabled)
    {
        return;
    }

    btRigidBody* rbA = mRigidBodies[j.bodyA].get();
    btRigidBody* rbB = mRigidBodies[j.bodyB].get();
    const btTransform frameWorld = transformFromRest(j.frameT, j.frameR);
    const btTransform frameInA = rbA->getWorldTransform().inverse() * frameWorld;
    const btTransform frameInB = rbB->getWorldTransform().inverse() * frameWorld;

    std::unique_ptr<btTypedConstraint> con;
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
        auto g6 = std::make_unique<btGeneric6DofSpring2Constraint>(*rbA, *rbB, frameInA, frameInB);
        applySixDofLimits(*g6, j);
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
        con = std::move(g6);
        break;
    }
    case kJointSixDof:
    {
        auto g6 = std::make_unique<btGeneric6DofConstraint>(*rbA, *rbB, frameInA, frameInB, true);
        applySixDofLimits(*g6, j);
        con = std::move(g6);
        break;
    }
    case kJointP2P:
    {
        const btVector3 pivotInA = frameInA.getOrigin();
        const btVector3 pivotInB = frameInB.getOrigin();
        con = std::make_unique<btPoint2PointConstraint>(*rbA, *rbB, pivotInA, pivotInB);
        break;
    }
    case kJointConeTwist:
    {
        auto ct = std::make_unique<btConeTwistConstraint>(*rbA, *rbB, frameInA, frameInB);
        ct->setLimit(static_cast<btScalar>(j.angularMin.y), static_cast<btScalar>(j.angularMax.y),
                     0.0F, kConstraintSoftness, 0.0F, 1.0F);
        con = std::move(ct);
        break;
    }
    case kJointSlider:
    {
        auto sl = std::make_unique<btSliderConstraint>(*rbA, *rbB, frameInA, frameInB, true);
        sl->setLowerLinLimit(static_cast<btScalar>(j.linearMin.y));
        sl->setUpperLinLimit(static_cast<btScalar>(j.linearMax.y));
        sl->setLowerAngLimit(static_cast<btScalar>(j.angularMin.y));
        sl->setUpperAngLimit(static_cast<btScalar>(j.angularMax.y));
        con = std::move(sl);
        break;
    }
    case kJointHinge:
    {
        auto hi = std::make_unique<btHingeConstraint>(*rbA, *rbB, frameInA, frameInB, true);
        hi->setLimit(static_cast<btScalar>(j.angularMin.y), static_cast<btScalar>(j.angularMax.y),
                     kConstraintSoftness, 0.0F, 1.0F);
        con = std::move(hi);
        break;
    }
    default:
        break;
    }

    if (con)
    {
        mWorld->addConstraint(con.get(), /*disableCollisionsBetweenLinkedBodies=*/true);
        mConstraints.push_back(std::move(con));
    }
}

bool Simulation::SimulationImpl::initialized() const
{
    return mWorldBuilt;
}

bool Simulation::SimulationImpl::setKinematicPose(size_t anchorIndex, const Pose& pose)
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
    // The contract says pose.quat is a unit quaternion, but a sloppy adapter
    // must not be able to corrupt the body basis — normalize defensively.
    btQuaternion q(static_cast<btScalar>(pose.quat.x), static_cast<btScalar>(pose.quat.y),
                   static_cast<btScalar>(pose.quat.z), static_cast<btScalar>(pose.quat.w));
    q.normalize();
    t.setRotation(q);
    body->setWorldTransform(t);
    body->getMotionState()->setWorldTransform(t);

    // Detect movement (e.g. a bone dragged in the viewport at the current
    // frame): if an anchor moved but time did not advance, the sim still needs
    // to step so attached chains follow the bone immediately (MMD reacts to
    // bone changes instantly — not on the next frame).
    bool moved = false;
    if (anchorIndex < mAnchorCurrent.size())
    {
        const btTransform prev =
            poseToTransform(mAnchorCurrent[anchorIndex].pos, mAnchorCurrent[anchorIndex].quat);
        const btVector3 d = t.getOrigin() - prev.getOrigin();
        const btVector3 c0 = t.getBasis().getColumn(0);
        const btVector3 p0 = prev.getBasis().getColumn(0);
        const btVector3 c1 = t.getBasis().getColumn(1);
        const btVector3 p1 = prev.getBasis().getColumn(1);
        if (d.length2() > btScalar(kAnchorMoveDistEpsSq) ||
            c0.dot(p0) < btScalar(1.0) - btScalar(kAnchorMoveRotEps) ||
            c1.dot(p1) < btScalar(1.0) - btScalar(kAnchorMoveRotEps))
        {
            moved = true;
        }
    }
    if (anchorIndex < mAnchorCurrent.size())
    {
        storePose(mAnchorCurrent[anchorIndex].pos, mAnchorCurrent[anchorIndex].quat, t);
    }
    return moved;
}

void Simulation::SimulationImpl::step(double dt)
{
    if (!mWorld)
    {
        return;
    }
    dt = std::clamp(dt, 0.0, kMaxStepTime); // guard against huge jumps / negative dt
    mWorld->stepSimulation(btScalar(dt), kMaxSubSteps, btScalar(kFixedDt));
}

void Simulation::SimulationImpl::resetDynamicBodies()
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
        if (b.def.isKinematic() || !b.def.enabled || !b.hasBoneReset)
        {
            continue;
        }
        const int anchorIdx = b.def.resetAnchorIndex;
        if (anchorIdx < 0 || anchorIdx >= static_cast<int>(mAnchorCurrent.size()))
        {
            continue;
        }

        const btTransform anchorCurrent =
            poseToTransform(mAnchorCurrent[anchorIdx].pos, mAnchorCurrent[anchorIdx].quat);
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

Simulation::Pose Simulation::SimulationImpl::bodyPose(size_t bodyIndex) const
{
    Pose p;
    if (mWorld && bodyIndex < mRigidBodies.size() && mRigidBodies[bodyIndex] != nullptr)
    {
        // Solved pose — what the simulation actually has right now.
        const btTransform& t = mRigidBodies[bodyIndex]->getWorldTransform();
        storePose(p.pos, p.quat, t);
    }
    else if (bodyIndex < mBodies.size())
    {
        // Disabled/missing body (or world not initialized): rest pose.
        const Body& b = mBodies[bodyIndex];
        p.pos = b.def.restPos;
        p.quat = eulerDegreesToQuat(b.def.restRot.x, b.def.restRot.y, b.def.restRot.z);
    }
    return p;
}

// =========================================================================
// Simulation — thin PIMPL delegates
// =========================================================================
Simulation::Simulation() : mImpl(std::make_unique<SimulationImpl>()) {}

Simulation::~Simulation() = default;

bool Simulation::initialize(const Definition& definition)
{
    return mImpl->initialize(definition);
}

void Simulation::clear()
{
    mImpl->clear();
}

bool Simulation::initialized() const
{
    return mImpl->initialized();
}

bool Simulation::setKinematicPose(size_t anchorIndex, const Pose& pose)
{
    return mImpl->setKinematicPose(anchorIndex, pose);
}

void Simulation::step(double dt)
{
    mImpl->step(dt);
}

void Simulation::resetDynamicBodies()
{
    mImpl->resetDynamicBodies();
}

Simulation::Pose Simulation::bodyPose(size_t bodyIndex) const
{
    return mImpl->bodyPose(bodyIndex);
}

} // namespace mmd::core
