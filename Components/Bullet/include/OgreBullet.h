// This file is part of the OGRE project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at https://www.ogre3d.org/licensing.
// SPDX-License-Identifier: MIT

#ifndef COMPONENTS_BULLET_H_
#define COMPONENTS_BULLET_H_

#include "OgreBulletExports.h"

#include "btBulletDynamicsCommon.h"
#include "Ogre.h"

namespace Ogre
{
namespace Bullet
{

/** \addtogroup Optional
*  @{
*/
/** \defgroup Bullet Bullet
 * [Bullet-Physics](https://pybullet.org) to %Ogre connection
 * @{
 */

enum ColliderType
{
    CT_BOX,
    CT_SPHERE,
    CT_CYLINDER,
    CT_CAPSULE,
    CT_TRIMESH,
    CT_HULL,
    CT_COMPOUND
};

inline btQuaternion convert(const Quaternion& q) { return btQuaternion(q.x, q.y, q.z, q.w); }
inline btVector3 convert(const Vector3& v) { return btVector3(v.x, v.y, v.z); }

inline Quaternion convert(const btQuaternion& q) { return Quaternion(q.w(), q.x(), q.y(), q.z()); }
inline Vector3 convert(const btVector3& v) { return Vector3(v.x(), v.y(), v.z()); }

inline void unscaleBtBasis(btTransform &scaledBasis)
{
    btMatrix3x3 &basis(scaledBasis.getBasis());
    btVector3 column0 = basis.getColumn(0);
    btVector3 column1 = basis.getColumn(1);
    btVector3 column2 = basis.getColumn(2);

	// Check for zero scaling.
    if (column0.fuzzyZero()) {
        if (column1.fuzzyZero()) {
            if (column2.fuzzyZero()) {
                // All dimensions are fuzzy zero. Create a default basis.
                column0 = btVector3(1, 0, 0);
                column1 = btVector3(0, 1, 0);
                column2 = btVector3(0, 0, 1);
            } else { // Column 2 scale not fuzzy zero.
                     // Create two vectors orthogonal to row 2.
                     // Ensure that a default basis is created if row 2 = <0, 0, 1>
                     column1 = btVector3(0, column2[2], -column2[1]);
                     column0 = column1.cross(column2);
            }
        } else { // Column 1 scale not fuzzy zero.
            if (column2.fuzzyZero()) {
                // Create two vectors othogonal to column 1.
                // Ensure that a default basis is created if column 1 = <0, 1, 0>
                column0 = btVector3(column1[1], -column1[0], 0);
                column2 = column0.cross(column1);
            } else { // Column 1 and column 2 scales not fuzzy zero.
                // Create column 0 orthogonal to column 1 and column 2.
                column0 = column1.cross(column2);
            }
        }
    } else { // Column 0 scale not fuzzy zero.
        if (column1.fuzzyZero()) {
            if (column2.fuzzyZero()) {
            // Create two vectors orthogonal to column 0.
            // Ensure that a default basis is created if column 0 = <1, 0, 0>
                column2 = btVector3(-column0[2], 0, column0[0]);
                column1 = column2.cross(column0);
            } else { // Column 0 and column 2 scales not fuzzy zero.
                     // Create column 1 orthogonal to column 0 and column 2.
                column1 = column2.cross(column0);
            }
        } else { // Column 0 and column 1 scales not fuzzy zero.
            if (column2.fuzzyZero()) {
                // Create column 2 orthogonal to column 0 and column 1.
                column2 = column0.cross(column1);
            }
        }
    }

    // Normalize
    column0.normalize();
    column1.normalize();
    column2.normalize();

    basis.setValue(column0[0], column1[0], column2[0],
        column0[1], column1[1], column2[1],
        column0[2], column1[2], column2[2]);
}

/** A MotionState is Bullet's way of informing you about updates to an object.
 * Pass this MotionState to a btRigidBody to have your SceneNode updated automaticaly.
 */
class _OgreBulletExport RigidBodyState : public btMotionState
{
    Node* mNode;

public:
    RigidBodyState(Node* node) : mNode(node) {}

    void getWorldTransform(btTransform& ret) const override
    {
        ret = btTransform(convert(mNode->getOrientation()), convert(mNode->getPosition()));
    }

    void setWorldTransform(const btTransform& in) override
    {
        btQuaternion rot = in.getRotation();
        btVector3 pos = in.getOrigin();
        mNode->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
        mNode->setPosition(pos.x(), pos.y(), pos.z());
    }
};

/// create sphere collider using ogre provided data
_OgreBulletExport btSphereShape* createSphereCollider(const MovableObject* mo);
/// create box collider using ogre provided data
_OgreBulletExport btBoxShape* createBoxCollider(const MovableObject* mo);
/// create capsule collider using ogre provided data
_OgreBulletExport btCapsuleShape* createCapsuleCollider(const MovableObject* mo);
/// create capsule collider using ogre provided data
_OgreBulletExport btCylinderShape* createCylinderCollider(const MovableObject* mo);
/// create triMesh collider
_OgreBulletExport btBvhTriangleMeshShape* createTrimeshCollider(const Entity* ent);
/// create convex hull collider
_OgreBulletExport btConvexHullShape* createConvexHullCollider(const Entity* ent);
/// create compound shape
_OgreBulletExport btCompoundShape* createCompoundShape();

struct _OgreBulletExport CollisionListener
{
    virtual ~CollisionListener() {}
    /** Called when two objects collide
    * @param other the other object
    * @param manifoldPoint the collision point
    */
    virtual void contact(const MovableObject* other, const btManifoldPoint& manifoldPoint) = 0;
};

struct _OgreBulletExport RayResultCallback
{
    virtual ~RayResultCallback() {}
    /** Called for each object hit by the ray
    * @param other the other object
    * @param distance the distance from ray origin to hit point
    */
    virtual void addSingleResult(const MovableObject* other, float distance) = 0;
};

/// simplified wrapper with automatic memory management
class _OgreBulletExport CollisionWorld
{
protected:
    std::unique_ptr<btCollisionConfiguration> mCollisionConfig;
    std::unique_ptr<btCollisionDispatcher> mDispatcher;
    std::unique_ptr<btBroadphaseInterface> mBroadphase;

    btCollisionWorld* mBtWorld;

public:
    CollisionWorld(btCollisionWorld* btWorld) : mBtWorld(btWorld) {}
    virtual ~CollisionWorld();

    btCollisionObject* addCollisionObject(Entity* ent, ColliderType ct, int group = 1, int mask = -1);

    void rayTest(const Ray& ray, RayResultCallback* callback, float maxDist = 1000);
    void attachCollisionObject(btCollisionObject *collisionObject, Entity *ent, int group = 1, int mask = -1);
};

/// simplified wrapper with automatic memory management
class _OgreBulletExport DynamicsWorld : public CollisionWorld
{
    std::unique_ptr<btConstraintSolver> mSolver;

public:
    explicit DynamicsWorld(const Vector3& gravity);
    DynamicsWorld(btDynamicsWorld* btWorld) : CollisionWorld(btWorld) {}

    /** Add an Entity as a rigid body to the DynamicsWorld
    * @param mass the mass of the object
    * @param ent the entity to control
    * @param ct the collider type
    * @param listener a listener to call on collision with other objects
    * @param group the collision group
    * @param mask the collision mask
    */
    btRigidBody* addRigidBody(float mass, Entity* ent, ColliderType ct, CollisionListener* listener = nullptr,
                              int group = 1, int mask = -1);
    btRigidBody* addKinematicRigidBody(Entity* ent, ColliderType ct, int group = 1, int mask = -1);
    void attachRigidBody(btRigidBody *rigidBody, Entity *ent, CollisionListener* listener = nullptr,
                              int group = 1, int mask = -1);
    btDynamicsWorld* getBtWorld() const { return static_cast<btDynamicsWorld*>(mBtWorld); }
#if 0
    struct MotionResult {};
    bool test_body_motion(btRigidBody *body, const btTransform &from, const btVector3 motion,
                          bool infinite_inertia, MotionResult *result, bool exclude_raycast_shapes,
			  const std::set<btCollisionObject *> &exclude);
#endif
};

class _OgreBulletExport DebugDrawer : public btIDebugDraw
{
    SceneNode* mNode;
    btCollisionWorld* mWorld;

    ManualObject mLines;
    int mDebugMode;

public:
    DebugDrawer(SceneNode* node, btCollisionWorld* world)
        : mNode(node), mWorld(world), mLines(""), mDebugMode(DBG_DrawWireframe)
    {
        mLines.setCastShadows(false);
        mNode->attachObject(&mLines);
        mWorld->setDebugDrawer(this);
    }

    void update()
    {
        mWorld->debugDrawWorld();
        if (!mLines.getSections().empty()) // begin was called
            mLines.end();
    }

    void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override;

    void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime,
                          const btVector3& color) override
    {
        drawLine(PointOnB, PointOnB + normalOnB * distance * 20, color);
    }

    void reportErrorWarning(const char* warningString) override { LogManager::getSingleton().logWarning(warningString); }

    void draw3dText(const btVector3& location, const char* textString) override {}

    void setDebugMode(int mode) override
    {
        mDebugMode = mode;

        if (mDebugMode == DBG_NoDebug)
            clear();
    }

    void clear() { mLines.clear(); }

    int getDebugMode() const override { return mDebugMode; }
};
/** @} */
/** @} */
} // namespace Bullet
} // namespace Ogre

#endif
