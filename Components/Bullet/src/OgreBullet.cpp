// This file is part of the OGRE project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at https://www.ogre3d.org/licensing.
// SPDX-License-Identifier: MIT

#include "OgreBullet.h"

namespace Ogre
{
namespace Bullet
{

typedef std::vector<Vector3> Vector3Array;
typedef std::pair<unsigned short, Vector3Array*> BoneKeyIndex;

btSphereShape* createSphereCollider(const MovableObject* mo)
{
    OgreAssert(mo->getParentSceneNode(), "MovableObject must be attached");

    auto shape = new btSphereShape(mo->getBoundingRadius());
    shape->setLocalScaling(convert(mo->getParentSceneNode()->getScale()));

    return shape;
}
btBoxShape* createBoxCollider(const MovableObject* mo)
{
    OgreAssert(mo->getParentSceneNode(), "MovableObject must be attached");

    auto shape = new btBoxShape(convert(mo->getBoundingBox().getHalfSize()));
    shape->setLocalScaling(convert(mo->getParentSceneNode()->getScale()));

    return shape;
}

btCapsuleShape* createCapsuleCollider(const MovableObject* mo)
{
    OgreAssert(mo->getParentSceneNode(), "MovableObject must be attached");

    auto sz = mo->getBoundingBox().getHalfSize();

    btScalar height = std::max(sz.x, std::max(sz.y, sz.z));
    btScalar radius;
    btCapsuleShape* shape;
    // Orient the capsule such that its height is aligned with the largest dimension.
    if (height == sz.y)
    {
        radius = std::max(sz.x, sz.z);
        shape = new btCapsuleShape(radius, 2 * height - 2 * radius);
    }
    else if (height == sz.x)
    {
        radius = std::max(sz.y, sz.z);
        shape = new btCapsuleShapeX(radius, 2 * height - 2 * radius);
    }
    else
    {
        radius = std::max(sz.x, sz.y);
        shape = new btCapsuleShapeZ(radius, 2 * height - 2 * radius);
    }

    shape->setLocalScaling(convert(mo->getParentSceneNode()->getScale()));

    return shape;
}

/// create capsule collider using ogre provided data
btCylinderShape* createCylinderCollider(const MovableObject* mo)
{
    OgreAssert(mo->getParentSceneNode(), "MovableObject must be attached");

    auto sz = convert(mo->getBoundingBox().getHalfSize());

    btScalar height = std::max(sz.x(), std::max(sz.y(), sz.z()));
    btCylinderShape* shape;
    // Orient the capsule such that its height is aligned with the largest dimension.
    if (height == sz.y())
    {
        shape = new btCylinderShape(sz);
    }
    else if (height == sz.x())
    {
        shape = new btCylinderShapeX(sz);
    }
    else
    {
        shape = new btCylinderShapeZ(sz);
    }

    shape->setLocalScaling(convert(mo->getParentSceneNode()->getScale()));

    return shape;
}

/// create compound shape because we can
btCompoundShape* createCompoundShape()
{
	return new btCompoundShape;
}

struct EntityCollisionListener
{
    const MovableObject* entity;
    CollisionListener* listener;
};

static void onTick(btDynamicsWorld* world, btScalar timeStep)
{
    int numManifolds = world->getDispatcher()->getNumManifolds();
    auto manifolds = world->getDispatcher()->getInternalManifoldPointer();
    for (int i = 0; i < numManifolds; i++)
    {
        btPersistentManifold* manifold = manifolds[i];

        for (int j = 0; j < manifold->getNumContacts(); j++)
        {
            const btManifoldPoint& mp = manifold->getContactPoint(j);
            auto body0 = static_cast<EntityCollisionListener*>(manifold->getBody0()->getUserPointer());
            auto body1 = static_cast<EntityCollisionListener*>(manifold->getBody1()->getUserPointer());
            if (body0->listener)
                body0->listener->contact(body1->entity, mp);
            if (body1->listener)
                body1->listener->contact(body0->entity, mp);
        }
    }
}

class VertexIndexToShape
{
public:
    VertexIndexToShape(const Affine3& transform = Affine3::IDENTITY);
    VertexIndexToShape(Renderable* rend, const Affine3& transform = Affine3::IDENTITY);
    VertexIndexToShape(const Entity* entity, const Affine3& transform = Affine3::IDENTITY);
    ~VertexIndexToShape();

    Real getRadius();
    Vector3 getSize();

    btBvhTriangleMeshShape* createTrimesh();
    btConvexHullShape* createConvex();

    void addEntity(const Entity* entity, const Affine3& transform = Affine3::IDENTITY);
    void addMesh(const MeshPtr& mesh, const Affine3& transform = Affine3::IDENTITY);

    const Vector3* getVertices() { return mVertexBuffer; }
    unsigned int getVertexCount() { return mVertexCount; };

private:
    void addStaticVertexData(const VertexData* vertex_data);

    void addAnimatedVertexData(const VertexData* vertex_data, const VertexData* blended_data,
                               const Mesh::IndexMap* indexMap);

    void addIndexData(IndexData* data, const unsigned int offset = 0);

    Vector3* mVertexBuffer;
    unsigned int* mIndexBuffer;
    unsigned int mVertexCount;
    unsigned int mIndexCount;

    Vector3 mBounds;
    Real mBoundRadius;

    typedef std::map<unsigned char, std::vector<Vector3>*> BoneIndex;
    BoneIndex* mBoneIndex;

    Affine3 mTransform;

    Vector3 mScale;
};

/// create trimesh collider using ogre provided data
btBvhTriangleMeshShape* createTrimeshCollider(const Entity* ent)
{
        return VertexIndexToShape(ent).createTrimesh();
}

/// create convex hull collider using ogre provided data
btConvexHullShape* createConvexHullCollider(const Entity* ent)
{
	return VertexIndexToShape(ent).createConvex();
}

/// wrapper with automatic memory management
class CollisionObject
{
protected:
    btCollisionObject* mBtBody;
    btCollisionWorld* mBtWorld;

public:
    CollisionObject(btCollisionObject* btBody, btCollisionWorld* btWorld) : mBtBody(btBody), mBtWorld(btWorld) {}
    virtual ~CollisionObject()
    {
        mBtWorld->removeCollisionObject(mBtBody);
        delete mBtBody->getCollisionShape();
        delete mBtBody;
    }
};
class RigidBody : public CollisionObject
{
public:
    using CollisionObject::CollisionObject;

    ~RigidBody()
    {
        delete (EntityCollisionListener*)(mBtBody)->getUserPointer();
        delete ((btRigidBody*)mBtBody)->getMotionState();
    }
};

DynamicsWorld::DynamicsWorld(const Vector3& gravity) : CollisionWorld(NULL) // prevent CollisionWorld from creating a world
{
    // Bullet initialisation.
    mCollisionConfig.reset(new btDefaultCollisionConfiguration());
    mDispatcher.reset(new btCollisionDispatcher(mCollisionConfig.get()));
    mSolver.reset(new btSequentialImpulseConstraintSolver());
    mBroadphase.reset(new btDbvtBroadphase());

    auto btworld = new btDiscreteDynamicsWorld(mDispatcher.get(), mBroadphase.get(), mSolver.get(), mCollisionConfig.get());
    btworld->setGravity(convert(gravity));
    btworld->setInternalTickCallback(onTick);
    mBtWorld = btworld;
}

static btCollisionShape* getCollisionShape(Entity* ent, ColliderType ct)
{
    if (ent->hasSkeleton())
    {
        ent->addSoftwareAnimationRequest(false);
        ent->_updateAnimation();
        ent->setUpdateBoundingBoxFromSkeleton(true);
    }

    btCollisionShape* cs = NULL;
    switch (ct)
    {
    case CT_BOX:
        cs = createBoxCollider(ent);
        break;
    case CT_SPHERE:
        cs = createSphereCollider(ent);
        break;
    case CT_CYLINDER:
        cs = createCylinderCollider(ent);
        break;
    case CT_CAPSULE:
        cs = createCapsuleCollider(ent);
        break;
    case CT_TRIMESH:
        cs = createTrimeshCollider(ent);
        break;
    case CT_HULL:
        cs = createConvexHullCollider(ent);
        break;
    case CT_COMPOUND:
	cs = createCompoundShape();
	break;
    }

    if (ent->hasSkeleton())
        ent->removeSoftwareAnimationRequest(false);

    return cs;
}

btRigidBody* DynamicsWorld::addRigidBody(float mass, Entity* ent, ColliderType ct, CollisionListener* listener,
                                         int group, int mask)
{
    auto node = ent->getParentSceneNode();
    OgreAssert(node, "entity must be attached");
    RigidBodyState* state = new RigidBodyState(node);

    btCollisionShape* cs = getCollisionShape(ent, ct);

    btVector3 inertia(0, 0, 0);
    if (mass != 0) // mass = 0 -> static
        cs->calculateLocalInertia(mass, inertia);

    auto rb = new btRigidBody(mass, state, cs, inertia);
    getBtWorld()->addRigidBody(rb, group, mask);
    rb->setUserPointer(new EntityCollisionListener{ent, listener});

    // transfer ownership to node
    auto objWrapper = std::make_shared<RigidBody>(rb, mBtWorld);
    node->getUserObjectBindings().setUserAny("BtCollisionObject", objWrapper);

    return rb;
}

btRigidBody* DynamicsWorld::addKinematicRigidBody(Entity* ent, ColliderType ct, int group, int mask)
{
    btRigidBody* rb = addRigidBody(0, ent, ct, nullptr, group, mask);
    rb->setCollisionFlags(rb->getCollisionFlags()
                    | btCollisionObject::CF_KINEMATIC_OBJECT
                    | btCollisionObject::CF_NO_CONTACT_RESPONSE
                    );
    rb->setActivationState(DISABLE_DEACTIVATION);
    return rb;
}

btCollisionObject* CollisionWorld::addCollisionObject(Entity* ent, ColliderType ct, int group, int mask)
{
    auto node = ent->getParentSceneNode();
    OgreAssert(node, "entity must be attached");

    btCollisionShape* cs = getCollisionShape(ent, ct);

    auto co = new btCollisionObject();
    co->setCollisionShape(cs);
    mBtWorld->addCollisionObject(co, group, mask);

    // transfer ownership to node
    auto objWrapper = std::make_shared<CollisionObject>(co, mBtWorld);
    node->getUserObjectBindings().setUserAny("BtCollisionObject", objWrapper);

    return co;
}

void DynamicsWorld::attachRigidBody(btRigidBody *rigidBody, Entity* ent, CollisionListener* listener,
                                         int group, int mask)
{
    auto node = ent->getParentSceneNode();
    OgreAssert(node, "entity must be attached");
    /* If the body has incorrect btMotionState and is in world
     * we will crash or corrupt some memory. Hope the user
     * will know what he/she is doing */
    if (!rigidBody->isInWorld()) {
        RigidBodyState* state = new RigidBodyState(node);
	rigidBody->setMotionState(state);
        getBtWorld()->addRigidBody(rigidBody, group, mask);
    }
    rigidBody->setUserPointer(new EntityCollisionListener{ent, listener});
    // transfer ownership to node
    auto objWrapper = std::make_shared<RigidBody>(rigidBody, mBtWorld);
    node->getUserObjectBindings().setUserAny("BtCollisionObject", objWrapper);
}
void CollisionWorld::attachCollisionObject(btCollisionObject *collisionObject, Entity* ent, int group, int mask)
{
    auto node = ent->getParentSceneNode();
    OgreAssert(node, "entity must be attached");
    if (collisionObject->getWorldArrayIndex() == -1)
        mBtWorld->addCollisionObject(collisionObject, group, mask);

    // transfer ownership to node
    auto objWrapper = std::make_shared<CollisionObject>(collisionObject, mBtWorld);
    node->getUserObjectBindings().setUserAny("BtCollisionObject", objWrapper);
}

struct RayResultCallbackWrapper : public btCollisionWorld::RayResultCallback
{
    Bullet::RayResultCallback* mCallback;
    float mMaxDistance;
    RayResultCallbackWrapper(Bullet::RayResultCallback* callback, float maxDist)
        : mCallback(callback), mMaxDistance(maxDist)
    {
    }
    btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace) override
    {
        auto body0 = static_cast<const EntityCollisionListener*>(rayResult.m_collisionObject->getUserPointer());
        mCallback->addSingleResult(body0->entity, rayResult.m_hitFraction * mMaxDistance);
        return rayResult.m_hitFraction;
    }
};

void CollisionWorld::rayTest(const Ray& ray, RayResultCallback* callback, float maxDist)
{
    RayResultCallbackWrapper wrapper(callback, maxDist);
    btVector3 from = convert(ray.getOrigin());
    btVector3 to = convert(ray.getPoint(maxDist));
    mBtWorld->rayTest(from, to, wrapper);
}
#if 0
bool DynamicsWorld::recover_from_penetration(btRigidBody *body, const btTransform &body_position, btScalar recover_movement_scale, bool infinite_inertia, btVector3 &delta_recover_movement, RecoverResult *r_recover_result, const std::set &exclude) {
	// Calculate the cumulative AABB of all shapes of the kinematic body
	btVector3 aabb_min, aabb_max;
	bool shapes_found = false;

	for (int kinIndex = p_body->get_kinematic_utilities()->shapes.size() - 1; 0 <= kinIndex; --kinIndex) {
		const RigidBodyBullet::KinematicShape &kin_shape(p_body->get_kinematic_utilities()->shapes[kinIndex]);
		if (!kin_shape.is_active()) {
			continue;
		}

		if (kin_shape.shape->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE) {
			// Skip rayshape in order to implement custom separation process
			continue;
		}

		btTransform shape_transform = p_body_position * kin_shape.transform;
		shape_transform.getOrigin() += r_delta_recover_movement;

		btVector3 shape_aabb_min, shape_aabb_max;
		kin_shape.shape->getAabb(shape_transform, shape_aabb_min, shape_aabb_max);

		if (!shapes_found) {
			aabb_min = shape_aabb_min;
			aabb_max = shape_aabb_max;
			shapes_found = true;
		} else {
			aabb_min.setX((aabb_min.x() < shape_aabb_min.x()) ? aabb_min.x() : shape_aabb_min.x());
			aabb_min.setY((aabb_min.y() < shape_aabb_min.y()) ? aabb_min.y() : shape_aabb_min.y());
			aabb_min.setZ((aabb_min.z() < shape_aabb_min.z()) ? aabb_min.z() : shape_aabb_min.z());

			aabb_max.setX((aabb_max.x() > shape_aabb_max.x()) ? aabb_max.x() : shape_aabb_max.x());
			aabb_max.setY((aabb_max.y() > shape_aabb_max.y()) ? aabb_max.y() : shape_aabb_max.y());
			aabb_max.setZ((aabb_max.z() > shape_aabb_max.z()) ? aabb_max.z() : shape_aabb_max.z());
		}
	}

	// If there are no shapes then there is no penetration either
	if (!shapes_found) {
		return false;
	}

	// Perform broadphase test
	RecoverPenetrationBroadPhaseCallback recover_broad_result(p_body->get_bt_collision_object(), p_body->get_collision_layer(), p_body->get_collision_mask(), aabb_min, aabb_max);
	dynamicsWorld->getBroadphase()->aabbTest(aabb_min, aabb_max, recover_broad_result);

	bool penetration = false;

	// Perform narrowphase per shape
	for (int kinIndex = p_body->get_kinematic_utilities()->shapes.size() - 1; 0 <= kinIndex; --kinIndex) {
		const RigidBodyBullet::KinematicShape &kin_shape(p_body->get_kinematic_utilities()->shapes[kinIndex]);
		if (!kin_shape.is_active()) {
			continue;
		}

		if (kin_shape.shape->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE) {
			// Skip rayshape in order to implement custom separation process
			continue;
		}

		if (kin_shape.shape->getShapeType() == EMPTY_SHAPE_PROXYTYPE) {
			continue;
		}

		btTransform shape_transform = p_body_position * kin_shape.transform;
		shape_transform.getOrigin() += r_delta_recover_movement;

		for (int i = recover_broad_result.results.size() - 1; 0 <= i; --i) {
			btCollisionObject *otherObject = recover_broad_result.results[i].collision_object;

			CollisionObjectBullet *gObj = static_cast<CollisionObjectBullet *>(otherObject->getUserPointer());
			if (p_exclude.has(gObj->get_self())) {
				continue;
			}

			if (p_infinite_inertia && !otherObject->isStaticOrKinematicObject()) {
				otherObject->activate(); // Force activation of hitten rigid, soft body
				continue;
			} else if (!p_body->get_bt_collision_object()->checkCollideWith(otherObject) || !otherObject->checkCollideWith(p_body->get_bt_collision_object())) {
				continue;
			}

			if (otherObject->getCollisionShape()->isCompound()) {
				const btCompoundShape *cs = static_cast<const btCompoundShape *>(otherObject->getCollisionShape());
				if (cs->getNumChildShapes() == 0) {
					continue; // No shapes to depenetrate from.
				}
				int shape_idx = recover_broad_result.results[i].compound_child_index;
				ERR_FAIL_COND_V(shape_idx < 0 || shape_idx >= cs->getNumChildShapes(), false);

				if (cs->getChildShape(shape_idx)->isConvex()) {
					if (RFP_convex_convex_test(kin_shape.shape, static_cast<const btConvexShape *>(cs->getChildShape(shape_idx)), otherObject, kinIndex, shape_idx, shape_transform, otherObject->getWorldTransform() * cs->getChildTransform(shape_idx), p_recover_movement_scale, r_delta_recover_movement, r_recover_result)) {
						penetration = true;
					}
				} else {
					if (RFP_convex_world_test(kin_shape.shape, cs->getChildShape(shape_idx), p_body->get_bt_collision_object(), otherObject, kinIndex, shape_idx, shape_transform, otherObject->getWorldTransform() * cs->getChildTransform(shape_idx), p_recover_movement_scale, r_delta_recover_movement, r_recover_result)) {
						penetration = true;
					}
				}
			} else if (otherObject->getCollisionShape()->isConvex()) { /// Execute GJK test against object shape
				if (RFP_convex_convex_test(kin_shape.shape, static_cast<const btConvexShape *>(otherObject->getCollisionShape()), otherObject, kinIndex, 0, shape_transform, otherObject->getWorldTransform(), p_recover_movement_scale, r_delta_recover_movement, r_recover_result)) {
					penetration = true;
				}
			} else {
				if (RFP_convex_world_test(kin_shape.shape, otherObject->getCollisionShape(), p_body->get_bt_collision_object(), otherObject, kinIndex, 0, shape_transform, otherObject->getWorldTransform(), p_recover_movement_scale, r_delta_recover_movement, r_recover_result)) {
					penetration = true;
				}
			}
		}
	}

	return penetration;
}
#endif

struct MotionResult {};

struct TestBodyMotionStateParams {
    btRigidBody *mBody;
    btTransform bodyTransform;
    bool infiniteInertia;
    MotionResult result;
    std::vector<btCollisionShape *> mCollisionShapes;
    std::vector<btTransform> mCompoundShapesTransforms;
    void getCompoundShapeList(btCollisionShape *shape, const btTransform &xform,
                              std::list<std::pair<btCollisionShape *, btTransform> > &convex_list,
                              std::list<std::pair<btCollisionShape *, btTransform> > &compound_list)
    {
        int i;
	compound_list.clear();
	convex_list.clear();
	if (shape->isCompound()) {
            btCompoundShape* cshape = static_cast<btCompoundShape*>(shape);
            for (i = 0; i < cshape->getNumChildShapes(); i++) {
                btCollisionShape *subshape = cshape->getChildShape(i);
		btTransform subxform = cshape->getChildTransform(i);
		std::pair<btCollisionShape *, btTransform> s = {subshape, xform * subxform};
		if (subshape->isCompound())
                    compound_list.push_back(s);
		else
                    convex_list.push_back(s);
            }
        } else if (shape->isConvex())
            convex_list.push_back({shape, xform});
    }
    void buildShapeList(btRigidBody *body)
    {
        std::list<std::pair<btCollisionShape *, btTransform> > convex_list, compound_list;
	btCollisionShape* shape = body->getCollisionShape();
	getCompoundShapeList(shape, btTransform(), convex_list, compound_list);
	while (!compound_list.empty()) {
            std::pair<btCollisionShape *, btTransform> s = compound_list.front();
	    compound_list.pop_front();
            std::list<std::pair<btCollisionShape *, btTransform> > tmp_convex_list, tmp_compound_list;
	    getCompoundShapeList(s.first, s.second, tmp_convex_list, tmp_compound_list);
	    convex_list.splice(convex_list.end(), tmp_convex_list);
	    compound_list.splice(compound_list.end(), tmp_compound_list);
	    OgreAssert(tmp_convex_list.size() == 0, "Something is wrong processing convex collisions");
	    OgreAssert(tmp_compound_list.size() == 0, "Something is wrong processing compound collisions");
	}
    }
#if 0
#if 0
    TestBodyMotionStateParams(btRigidBody *body,
                        const Quaternion &fromOrientation,
                        const Vector3 &fromPosition,
			bool infiniteInertia = true)
        : body(body)
        , bodyTransform(fromOrientation, fromPosition)
        , infiniteInertia(infiniteInertia)
    {
        btCollisionShape *collision = body->getCollisionShape();
	std::list<std::pair<btCollisionShape *, btTransform> > queue;
	if (collision->isCompound())
            queue.push_back({collision, btTransform());
        else {
            mCollisionShapes.push_back(collision);
            mCollisionShapesTransforms.push_back(btTransform);
	}
	while (!queue.empty()) {
		std::pair<btCollisionShape *, btTransform> item = *queue.front();
		btCollisionShape *shape = item.first;
		btTransform xform = item.second;
		if (shape->isCompound()) {
                    int i;
                    btCompoundShape* compound = static_cast<btCollisionShape*>(compound);
                    for (i = 0; i < compound->getNumChildShapes(); i++) {
                        btCollisionShapes *subshape = compound->getChildShape(i);
			btTransform subxform = compound->getChildTransform(i);
                        if (subshape->isConvex()) {
                            mCollisionShapes.push_back(subshape);
                            mCollisionShapesTransforms.push_back(subxform);
                    }
		}
		queue.pop_front();
	}
#endif
#if 0
	if (collision->isCompound()) {
            int i;
            btCompoundShape* compound = static_cast<btCollisionShape*>(compound);
            for (i = 0; i < compound->getNumChildShapes(); i++) {
                btCollisionShapes *subshape = compound->getChildShape(i);
		if (subshape->isConvex())
                    mCollisionShapes.push_back(subshape);
            }
	}
#endif
    }
#endif
};

#if 0
bool DynamicsWorld::test_body_motion(btRigidBody *body, const btTransform &from, const btVector3 motion,
                          bool infinite_inertia, MotionResult *result, bool exclude_raycast_shapes,
			  const std::set<btCollisionObject *> &exclude)
{
	btTransform body_transform = from;
//	unscaleBtBasis(body_transform);
	btVector3 initial_recover_motion(0, 0, 0);
	{ /// Phase one - multi shapes depenetration using margin
		int t;
		for (t = 0; t < RECOVERING_MOVEMENT_CYCLES; t++) {
			if (!recover_from_penetration(body, body_transform, RECOVERING_MOVEMENT_SCALE,
						infinite_inertia, initial_recover_motion, nullptr, exclude))
				break;
		}
		// Add recover movement in order to make it safe
		body_transform.getOrigin() += initial_recover_motion;
	}
	Real total_length = motion.length();
	Real unsafe_fraction = 1.0;
	Real safe_fraction = 1.0;
	{
		// Phase two - sweep test, from a secure position without margin

		const int shape_count(p_body->get_shape_count());
		for (int shIndex = 0; shIndex < shape_count; ++shIndex) {
			if (p_body->is_shape_disabled(shIndex)) {
				continue;
			}

			if (!p_body->get_bt_shape(shIndex)->isConvex()) {
				// Skip no convex shape
				continue;
			}

			if (p_exclude_raycast_shapes && p_body->get_bt_shape(shIndex)->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE) {
				// Skip rayshape in order to implement custom separation process
				continue;
			}

			btConvexShape *convex_shape_test(static_cast<btConvexShape *>(p_body->get_bt_shape(shIndex)));

			btTransform shape_world_from = body_transform * p_body->get_kinematic_utilities()->shapes[shIndex].transform;

			btTransform shape_world_to(shape_world_from);
			shape_world_to.getOrigin() += motion;

			if ((shape_world_to.getOrigin() - shape_world_from.getOrigin()).fuzzyZero()) {
				motion = btVector3(0, 0, 0);
				break;
			}

			GodotKinClosestConvexResultCallback btResult(shape_world_from.getOrigin(), shape_world_to.getOrigin(), p_body, p_infinite_inertia, &p_exclude);
			btResult.m_collisionFilterGroup = p_body->get_collision_layer();
			btResult.m_collisionFilterMask = p_body->get_collision_mask();

			dynamicsWorld->convexSweepTest(convex_shape_test, shape_world_from, shape_world_to, btResult, dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration);

			if (btResult.hasHit()) {
				if (total_length > CMP_EPSILON) {
					real_t hit_fraction = btResult.m_closestHitFraction * motion.length() / total_length;
					if (hit_fraction < unsafe_fraction) {
						unsafe_fraction = hit_fraction;
						real_t margin = p_body->get_kinematic_utilities()->safe_margin;
						safe_fraction = MAX(hit_fraction - (1 - ((total_length - margin) / total_length)), 0);
					}
				}

				/// Since for each sweep test I fix the motion of new shapes in base the recover result,
				/// if another shape will hit something it means that has a deepest penetration respect the previous shape
				motion *= btResult.m_closestHitFraction;
			}
		}

		body_transform.getOrigin() += motion;
	}

	bool has_penetration = false;

	{ /// Phase three - contact test with margin

		btVector3 __rec(0, 0, 0);
		RecoverResult r_recover_result;

		has_penetration = recover_from_penetration(p_body, body_transform, 1, p_infinite_inertia, __rec, &r_recover_result, p_exclude);

		// Parse results
		if (r_result) {
			B_TO_G(motion + initial_recover_motion + __rec, r_result->motion);

			if (has_penetration) {
				const btRigidBody *btRigid = static_cast<const btRigidBody *>(r_recover_result.other_collision_object);
				CollisionObjectBullet *collisionObject = static_cast<CollisionObjectBullet *>(btRigid->getUserPointer());

				B_TO_G(motion, r_result->remainder); // is the remaining movements
				r_result->remainder = p_motion - r_result->remainder;

				B_TO_G(r_recover_result.pointWorld, r_result->collision_point);
				B_TO_G(r_recover_result.normal, r_result->collision_normal);
				B_TO_G(btRigid->getVelocityInLocalPoint(r_recover_result.pointWorld - btRigid->getWorldTransform().getOrigin()), r_result->collider_velocity); // It calculates velocity at point and assign it using special function Bullet_to_Godot
				r_result->collider = collisionObject->get_self();
				r_result->collider_id = collisionObject->get_instance_id();
				r_result->collider_shape = r_recover_result.other_compound_shape_index;
				r_result->collision_local_shape = r_recover_result.local_shape_most_recovered;
				r_result->collision_depth = Math::abs(r_recover_result.penetration_distance);
				r_result->collision_safe_fraction = safe_fraction;
				r_result->collision_unsafe_fraction = unsafe_fraction;

#if debug_test_motion
				Vector3 sup_line2;
				B_TO_G(motion, sup_line2);
				normalLine->clear();
				normalLine->begin(Mesh::PRIMITIVE_LINES, NULL);
				normalLine->add_vertex(r_result->collision_point);
				normalLine->add_vertex(r_result->collision_point + r_result->collision_normal * 10);
				normalLine->end();
#endif
			} else {
				r_result->remainder = Vector3();
			}
		}
	}

	return false;
}
#endif

CollisionWorld::~CollisionWorld() { delete mBtWorld; }

/*
 * =============================================================================================
 * BtVertexIndexToShape
 * =============================================================================================
 */

void VertexIndexToShape::addStaticVertexData(const VertexData* vertex_data)
{
    if (!vertex_data)
        return;

    const VertexData* data = vertex_data;

    const unsigned int prev_size = mVertexCount;
    mVertexCount += (unsigned int)data->vertexCount;

    Vector3* tmp_vert = new Vector3[mVertexCount];
    if (mVertexBuffer)
    {
        memcpy(tmp_vert, mVertexBuffer, sizeof(Vector3) * prev_size);
        delete[] mVertexBuffer;
    }
    mVertexBuffer = tmp_vert;

    // Get the positional buffer element
    {
        const VertexElement* posElem = data->vertexDeclaration->findElementBySemantic(VES_POSITION);
        HardwareVertexBufferSharedPtr vbuf = data->vertexBufferBinding->getBuffer(posElem->getSource());
        const unsigned int vSize = (unsigned int)vbuf->getVertexSize();

        unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(HardwareBuffer::HBL_READ_ONLY));
        float* pReal;
        Vector3* curVertices = &mVertexBuffer[prev_size];
        const unsigned int vertexCount = (unsigned int)data->vertexCount;
        for (unsigned int j = 0; j < vertexCount; ++j)
        {
            posElem->baseVertexPointerToElement(vertex, &pReal);
            vertex += vSize;

            curVertices->x = (*pReal++);
            curVertices->y = (*pReal++);
            curVertices->z = (*pReal++);

            *curVertices = mTransform * (*curVertices);

            curVertices++;
        }
        vbuf->unlock();
    }
}
//------------------------------------------------------------------------------------------------
void VertexIndexToShape::addAnimatedVertexData(const VertexData* vertex_data, const VertexData* blend_data,
                                               const Mesh::IndexMap* indexMap)
{
    // Get the bone index element
    assert(vertex_data);

    const VertexData* data = blend_data;
    const unsigned int prev_size = mVertexCount;
    mVertexCount += (unsigned int)data->vertexCount;
    Vector3* tmp_vert = new Vector3[mVertexCount];
    if (mVertexBuffer)
    {
        memcpy(tmp_vert, mVertexBuffer, sizeof(Vector3) * prev_size);
        delete[] mVertexBuffer;
    }
    mVertexBuffer = tmp_vert;

    // Get the positional buffer element
    {
        const VertexElement* posElem = data->vertexDeclaration->findElementBySemantic(VES_POSITION);
        assert(posElem);
        HardwareVertexBufferSharedPtr vbuf = data->vertexBufferBinding->getBuffer(posElem->getSource());
        const unsigned int vSize = (unsigned int)vbuf->getVertexSize();

        unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(HardwareBuffer::HBL_READ_ONLY));
        float* pReal;
        Vector3* curVertices = &mVertexBuffer[prev_size];
        const unsigned int vertexCount = (unsigned int)data->vertexCount;
        for (unsigned int j = 0; j < vertexCount; ++j)
        {
            posElem->baseVertexPointerToElement(vertex, &pReal);
            vertex += vSize;

            curVertices->x = (*pReal++);
            curVertices->y = (*pReal++);
            curVertices->z = (*pReal++);

            *curVertices = mTransform * (*curVertices);

            curVertices++;
        }
        vbuf->unlock();
    }
    {
        const VertexElement* bneElem = vertex_data->vertexDeclaration->findElementBySemantic(VES_BLEND_INDICES);
        assert(bneElem);

        HardwareVertexBufferSharedPtr vbuf = vertex_data->vertexBufferBinding->getBuffer(bneElem->getSource());
        const unsigned int vSize = (unsigned int)vbuf->getVertexSize();
        unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(HardwareBuffer::HBL_READ_ONLY));

        unsigned char* pBone;

        if (!mBoneIndex)
            mBoneIndex = new BoneIndex();
        BoneIndex::iterator i;

        Vector3* curVertices = &mVertexBuffer[prev_size];

        const unsigned int vertexCount = (unsigned int)vertex_data->vertexCount;
        for (unsigned int j = 0; j < vertexCount; ++j)
        {
            bneElem->baseVertexPointerToElement(vertex, &pBone);
            vertex += vSize;

            const unsigned char currBone = (indexMap) ? (*indexMap)[*pBone] : *pBone;
            i = mBoneIndex->find(currBone);
            Vector3Array* l = 0;
            if (i == mBoneIndex->end())
            {
                l = new Vector3Array;
                mBoneIndex->emplace(currBone, l);
            }
            else
            {
                l = i->second;
            }

            l->push_back(*curVertices);

            curVertices++;
        }
        vbuf->unlock();
    }
}
//------------------------------------------------------------------------------------------------
void VertexIndexToShape::addIndexData(IndexData* data, const unsigned int offset)
{
    const unsigned int prev_size = mIndexCount;
    mIndexCount += (unsigned int)data->indexCount;

    unsigned int* tmp_ind = new unsigned int[mIndexCount];
    if (mIndexBuffer)
    {
        memcpy(tmp_ind, mIndexBuffer, sizeof(unsigned int) * prev_size);
        delete[] mIndexBuffer;
    }
    mIndexBuffer = tmp_ind;

    const unsigned int numTris = (unsigned int)data->indexCount / 3;
    HardwareIndexBufferSharedPtr ibuf = data->indexBuffer;
    const bool use32bitindexes = (ibuf->getType() == HardwareIndexBuffer::IT_32BIT);
    unsigned int index_offset = prev_size;

    if (use32bitindexes)
    {
        const unsigned int* pInt = static_cast<unsigned int*>(ibuf->lock(HardwareBuffer::HBL_READ_ONLY));
        for (unsigned int k = 0; k < numTris; ++k)
        {
            mIndexBuffer[index_offset++] = offset + *pInt++;
            mIndexBuffer[index_offset++] = offset + *pInt++;
            mIndexBuffer[index_offset++] = offset + *pInt++;
        }
        ibuf->unlock();
    }
    else
    {
        const unsigned short* pShort = static_cast<unsigned short*>(ibuf->lock(HardwareBuffer::HBL_READ_ONLY));
        for (unsigned int k = 0; k < numTris; ++k)
        {
            mIndexBuffer[index_offset++] = offset + static_cast<unsigned int>(*pShort++);
            mIndexBuffer[index_offset++] = offset + static_cast<unsigned int>(*pShort++);
            mIndexBuffer[index_offset++] = offset + static_cast<unsigned int>(*pShort++);
        }
        ibuf->unlock();
    }
}
//------------------------------------------------------------------------------------------------
Real VertexIndexToShape::getRadius()
{
    if (mBoundRadius == (-1))
    {
        getSize();
        mBoundRadius = (std::max(mBounds.x, std::max(mBounds.y, mBounds.z)) * 0.5);
    }
    return mBoundRadius;
}
//------------------------------------------------------------------------------------------------
Vector3 VertexIndexToShape::getSize()
{
    const unsigned int vCount = getVertexCount();
    if (mBounds == Vector3(-1, -1, -1) && vCount > 0)
    {

        const Vector3* const v = getVertices();

        Vector3 vmin(v[0]);
        Vector3 vmax(v[0]);

        for (unsigned int j = 1; j < vCount; j++)
        {
            vmin.x = std::min(vmin.x, v[j].x);
            vmin.y = std::min(vmin.y, v[j].y);
            vmin.z = std::min(vmin.z, v[j].z);

            vmax.x = std::max(vmax.x, v[j].x);
            vmax.y = std::max(vmax.y, v[j].y);
            vmax.z = std::max(vmax.z, v[j].z);
        }

        mBounds.x = vmax.x - vmin.x;
        mBounds.y = vmax.y - vmin.y;
        mBounds.z = vmax.z - vmin.z;
    }

    return mBounds;
}
//------------------------------------------------------------------------------------------------
btConvexHullShape* VertexIndexToShape::createConvex()
{
    assert(mVertexCount && (mIndexCount >= 6) && ("Mesh must have some vertices and at least 6 indices (2 triangles)"));

    btConvexHullShape* shape = new btConvexHullShape((btScalar*)&mVertexBuffer[0].x, mVertexCount, sizeof(Vector3));

    shape->setLocalScaling(convert(mScale));

    return shape;
}
//------------------------------------------------------------------------------------------------
btBvhTriangleMeshShape* VertexIndexToShape::createTrimesh()
{
    assert(mVertexCount && (mIndexCount >= 6) && ("Mesh must have some vertices and at least 6 indices (2 triangles)"));

    unsigned int numFaces = mIndexCount / 3;

    btTriangleMesh* trimesh = new btTriangleMesh();
    unsigned int* indices = mIndexBuffer;
    Vector3* vertices = mVertexBuffer;

    btVector3 vertexPos[3];
    for (unsigned int n = 0; n < numFaces; ++n)
    {
        {
            const Vector3& vec = vertices[*indices];
            vertexPos[0][0] = vec.x;
            vertexPos[0][1] = vec.y;
            vertexPos[0][2] = vec.z;
        }
        {
            const Vector3& vec = vertices[*(indices + 1)];
            vertexPos[1][0] = vec.x;
            vertexPos[1][1] = vec.y;
            vertexPos[1][2] = vec.z;
        }
        {
            const Vector3& vec = vertices[*(indices + 2)];
            vertexPos[2][0] = vec.x;
            vertexPos[2][1] = vec.y;
            vertexPos[2][2] = vec.z;
        }

        indices += 3;

        trimesh->addTriangle(vertexPos[0], vertexPos[1], vertexPos[2]);
    }

    const bool useQuantizedAABB = true;
    btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape(trimesh, useQuantizedAABB);

    shape->setLocalScaling(convert(mScale));

    return shape;
}
//------------------------------------------------------------------------------------------------
VertexIndexToShape::~VertexIndexToShape()
{
    delete[] mVertexBuffer;
    delete[] mIndexBuffer;

    if (mBoneIndex)
    {
        for (auto & i : *mBoneIndex)
        {
            delete i.second;
        }
        delete mBoneIndex;
    }
}
//------------------------------------------------------------------------------------------------
VertexIndexToShape::VertexIndexToShape(const Affine3& transform)
    : mVertexBuffer(0), mIndexBuffer(0), mVertexCount(0), mIndexCount(0), mBounds(Vector3(-1, -1, -1)),
      mBoundRadius(-1), mBoneIndex(0), mTransform(transform), mScale(1)
{
}
//------------------------------------------------------------------------------------------------
VertexIndexToShape::VertexIndexToShape(const Entity* entity, const Affine3& transform) : VertexIndexToShape(transform)
{
    addEntity(entity, transform);
}
//------------------------------------------------------------------------------------------------
VertexIndexToShape::VertexIndexToShape(Renderable* rend, const Affine3& transform) : VertexIndexToShape(transform)
{
    RenderOperation op;
    rend->getRenderOperation(op);
    addStaticVertexData(op.vertexData);
    if (op.useIndexes)
        addIndexData(op.indexData);
}
//------------------------------------------------------------------------------------------------
void VertexIndexToShape::addEntity(const Entity* entity, const Affine3& transform)
{
    // Each entity added need to reset size and radius
    // next time getRadius and getSize are asked, they're computed.
    mBounds = Vector3(-1, -1, -1);
    mBoundRadius = -1;

    auto node = entity->getParentSceneNode();
    mTransform = transform;
    mScale = node ? node->getScale() : Vector3(1, 1, 1);

    bool hasSkeleton = entity->hasSkeleton();

    if (entity->getMesh()->sharedVertexData)
    {
        if (hasSkeleton)
            addAnimatedVertexData(entity->getMesh()->sharedVertexData, entity->_getSkelAnimVertexData(),
                                  &entity->getMesh()->sharedBlendIndexToBoneIndexMap);
        else
            addStaticVertexData(entity->getMesh()->sharedVertexData);
    }

    for (unsigned int i = 0; i < entity->getNumSubEntities(); ++i)
    {
        SubMesh* sub_mesh = entity->getSubEntity(i)->getSubMesh();

        if (!sub_mesh->useSharedVertices)
        {
            addIndexData(sub_mesh->indexData, mVertexCount);

            if (hasSkeleton)
                addAnimatedVertexData(sub_mesh->vertexData, entity->getSubEntity(i)->_getSkelAnimVertexData(),
                                      &sub_mesh->blendIndexToBoneIndexMap);
            else
                addStaticVertexData(sub_mesh->vertexData);
        }
        else
        {
            addIndexData(sub_mesh->indexData);
        }
    }
}
//------------------------------------------------------------------------------------------------
void VertexIndexToShape::addMesh(const MeshPtr& mesh, const Affine3& transform)
{
    // Each entity added need to reset size and radius
    // next time getRadius and getSize are asked, they're computed.
    mBounds = Vector3(-1, -1, -1);
    mBoundRadius = -1;

    mTransform = transform;

    if (mesh->hasSkeleton())
        LogManager::getSingleton().logWarning("Mesh " + mesh->getName() + " has a skeleton but added non animated");

    if (mesh->sharedVertexData)
    {
        VertexIndexToShape::addStaticVertexData(mesh->sharedVertexData);
    }

    for (unsigned int i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        SubMesh* sub_mesh = mesh->getSubMesh(i);

        if (!sub_mesh->useSharedVertices)
        {
            VertexIndexToShape::addIndexData(sub_mesh->indexData, mVertexCount);
            VertexIndexToShape::addStaticVertexData(sub_mesh->vertexData);
        }
        else
        {
            VertexIndexToShape::addIndexData(sub_mesh->indexData);
        }
    }
}

/*
 * =============================================================================================
 * BtDebugDrawer
 * =============================================================================================
 */
//------------------------------------------------------------------------------------------------
void DebugDrawer::drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
{
    if (mLines.getSections().empty())
    {
        const char* matName = "Ogre/Debug/LinesMat";
        auto mat = MaterialManager::getSingleton().getByName(matName, RGN_INTERNAL);
        if (!mat)
        {
            mat = MaterialManager::getSingleton().create(matName, RGN_INTERNAL);
            auto p = mat->getTechnique(0)->getPass(0);
            p->setLightingEnabled(false);
            p->setVertexColourTracking(TVC_AMBIENT);
        }
        mLines.setBufferUsage(HBU_CPU_TO_GPU);
        mLines.begin(mat, RenderOperation::OT_LINE_LIST);
    }
    else if (mLines.getCurrentVertexCount() == 0)
        mLines.beginUpdate(0);

    ColourValue col(color.x(), color.y(), color.z());
    mLines.position(convert(from));
    mLines.colour(col);
    mLines.position(convert(to));
    mLines.colour(col);
}
} // namespace Bullet
} // namespace Ogre
