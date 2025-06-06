// This file is part of the OGRE project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at https://www.ogre3d.org/licensing.
// SPDX-License-Identifier: MIT

#include "OgreBullet.h"
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <iostream>

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
btCompoundShape* createCompoundShape() { return new btCompoundShape; }

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
btBvhTriangleMeshShape* createTrimeshCollider(const Entity* ent) { return VertexIndexToShape(ent).createTrimesh(); }

/// create convex hull collider using ogre provided data
btConvexHullShape* createConvexHullCollider(const Entity* ent) { return VertexIndexToShape(ent).createConvex(); }

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

DynamicsWorld::DynamicsWorld(const Vector3& gravity)
    : CollisionWorld(NULL) // prevent CollisionWorld from creating a world
{
    // Bullet initialisation.
    mCollisionConfig.reset(new btDefaultCollisionConfiguration());
    mDispatcher.reset(new btCollisionDispatcher(mCollisionConfig.get()));
    mSolver.reset(new btSequentialImpulseConstraintSolver());
    mBroadphase.reset(new btDbvtBroadphase());

    auto btworld =
        new btDiscreteDynamicsWorld(mDispatcher.get(), mBroadphase.get(), mSolver.get(), mCollisionConfig.get());
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
    rb->setCollisionFlags(rb->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT |
                          btCollisionObject::CF_NO_CONTACT_RESPONSE);
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

void DynamicsWorld::attachRigidBody(btRigidBody* rigidBody, Entity* ent, CollisionListener* listener, int group,
                                    int mask)
{
    auto node = ent->getParentSceneNode();
    OgreAssert(node, "entity must be attached");
    /* If the body has incorrect btMotionState and is in world
     * we will crash or corrupt some memory. Hope the user
     * will know what he/she is doing */
    if (!rigidBody->isInWorld())
    {
        RigidBodyState* state = new RigidBodyState(node);
        rigidBody->setMotionState(state);
        getBtWorld()->addRigidBody(rigidBody, group, mask);
    }
    rigidBody->setUserPointer(new EntityCollisionListener{ent, listener});
    // transfer ownership to node
    auto objWrapper = std::make_shared<RigidBody>(rigidBody, mBtWorld);
    node->getUserObjectBindings().setUserAny("BtCollisionObject", objWrapper);
}
void CollisionWorld::attachCollisionObject(btCollisionObject* collisionObject, Entity* ent, int group, int mask)
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
        for (auto& i : *mBoneIndex)
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
#if 0
void KinematicMotion::setupCollisionShapes(btRigidBody* body)
{
    std::list<std::pair<btCompoundShape*, btTransform>> shape_list;
    btCollisionShape* root_shape = body->getCollisionShape();
    btTransform root_xform;
    root_xform.setIdentity();
    if (root_shape->isCompound())
    {
        btCompoundShape* cshape = static_cast<btCompoundShape*>(root_shape);
        shape_list.push_back({cshape, root_xform});
    }
    else
    {
        mCollisionShapes.push_back(root_shape);
        mCollisionTransforms.push_back(root_xform);
    }
    while (!shape_list.empty())
    {
        int i;
        std::pair<btCompoundShape*, btTransform> s = shape_list.front();
        shape_list.pop_front();
        for (i = 0; i < s.first->getNumChildShapes(); i++)
        {
            btCollisionShape* shape = s.first->getChildShape(i);
            btTransform xform = s.second * s.first->getChildTransform(i);
            if (shape->isConvex())
            {
                mCollisionShapes.push_back(shape);
                mCollisionTransforms.push_back(xform);
            }
            else if (shape->isCompound())
            {
                btCompoundShape* cshape = static_cast<btCompoundShape*>(shape);
                shape_list.push_back({cshape, xform});
            }
        }
    }
}
bool KinematicMotion::RFP_convex_convex_test(const btConvexShape* p_shapeA, const btConvexShape* p_shapeB,
                                             btCollisionObject* p_objectB, int p_shapeId_A, int p_shapeId_B,
                                             const btTransform& p_transformA, const btTransform& p_transformB,
                                             btScalar p_recover_movement_scale, btVector3& r_delta_recover_movement,
                                             RecoverResult* r_recover_result)
{
    // Initialize GJK input
    btGjkPairDetector::ClosestPointInput gjk_input;
    gjk_input.m_transformA = p_transformA;
    // Avoid repeat penetrations
    gjk_input.m_transformA.getOrigin() += r_delta_recover_movement;
    gjk_input.m_transformB = p_transformB;

    // Perform GJK test
    btPointCollector result;
    btGjkPairDetector gjk_pair_detector(p_shapeA, p_shapeB, gjk_simplex_solver, gjk_epa_pen_solver);
    gjk_pair_detector.getClosestPoints(gjk_input, result, nullptr);
    if (0 > result.m_distance)
    {
        // Has penetration
        r_delta_recover_movement += result.m_normalOnBInWorld * (result.m_distance * -1 * p_recover_movement_scale);

        if (r_recover_result)
        {
            if (result.m_distance < r_recover_result->mPenetrationDistance)
            {
                r_recover_result->mHasPenetration = true;
                r_recover_result->mLocalShapeMostRecovered = p_shapeId_A;
                r_recover_result->mOtherCollisionObject = p_objectB;
                r_recover_result->mOtherCompoundShapeIndex = p_shapeId_B;
                r_recover_result->mPenetrationDistance = result.m_distance;
                r_recover_result->mPointWorld = result.m_pointInWorld;
                r_recover_result->mNormal = result.m_normalOnBInWorld;
            }
        }
        return true;
    }
    return false;
}

bool KinematicMotion::RFP_convex_world_test(btDynamicsWorld* dynamicsWorld, const btConvexShape* p_shapeA,
                                            const btCollisionShape* p_shapeB, btCollisionObject* p_objectA,
                                            btCollisionObject* p_objectB, int p_shapeId_A, int p_shapeId_B,
                                            const btTransform& p_transformA, const btTransform& p_transformB,
                                            btScalar p_recover_movement_scale, btVector3& r_delta_recover_movement,
                                            RecoverResult* r_recover_result)
{
    /// Contact test

    btTransform tA(p_transformA);
    // Avoid repeat penetrations
    tA.getOrigin() += r_delta_recover_movement;

    btCollisionObjectWrapper obA(nullptr, p_shapeA, p_objectA, tA, -1, p_shapeId_A);
    btCollisionObjectWrapper obB(nullptr, p_shapeB, p_objectB, p_transformB, -1, p_shapeId_B);

    btCollisionAlgorithm* algorithm = dispatcher->findAlgorithm(&obA, &obB, nullptr, BT_CONTACT_POINT_ALGORITHMS);
    if (algorithm)
    {
        GodotDeepPenetrationContactResultCallback contactPointResult(&obA, &obB);
        // discrete collision detection query
        algorithm->processCollision(&obA, &obB, dynamicsWorld->getDispatchInfo(), &contactPointResult);

        algorithm->~btCollisionAlgorithm();
        dispatcher->freeCollisionAlgorithm(algorithm);

        if (contactPointResult.hasHit())
        {
            r_delta_recover_movement += contactPointResult.m_pointNormalWorld *
                                        (contactPointResult.m_penetration_distance * -1 * p_recover_movement_scale);
            if (r_recover_result)
            {
                if (contactPointResult.m_penetration_distance < r_recover_result->mPenetrationDistance)
                {
                    r_recover_result->mHasPenetration = true;
                    r_recover_result->mLocalShapeMostRecovered = p_shapeId_A;
                    r_recover_result->mOtherCollisionObject = p_objectB;
                    r_recover_result->mOtherCompoundShapeIndex = p_shapeId_B;
                    r_recover_result->mPenetrationDistance = contactPointResult.m_penetration_distance;
                    r_recover_result->mPointWorld = contactPointResult.m_pointWorld;
                    r_recover_result->mNormal = contactPointResult.m_pointNormalWorld;
                }
            }
            return true;
        }
    }
    return false;
}

bool KinematicMotion::recoverFromPenetration(btCollisionWorld* collisionWorld, const btTransform& bodyPosition,
                                             RecoverResult& rresult)
{
    // Here we must refresh the overlapping paircache as the penetrating movement itself or the
    // previous recovery iteration might have used setWorldTransform and pushed us into an object
    // that is not in the previous cache contents from the last timestep, as will happen if we
    // are pushed into a new AABB overlap. Unhandled this means the next convex sweep gets stuck.
    //
    // Do this by calling the broadphase's setAabb with the moved AABB, this will update the broadphase
    // paircache and the ghostobject's internal paircache at the same time.    /BW

    btVector3 minAabb, maxAabb;
    /*
    mCollisionShape->getAabb(mGhostObject->getWorldTransform(), minAabb, maxAabb);
    collisionWorld->getBroadphase()->setAabb(m_ghostObject->getBroadphaseHandle(), minAabb, maxAabb,
                                             collisionWorld->getDispatcher());
    */
    bool shapes_found = false;

    for (int kinIndex = 0; kinIndex < (int)mCollisionShapes.size(); kinIndex++)
    {

        btTransform shapeTransform = bodyPosition * mCollisionTransforms[kinIndex];
        shapeTransform.getOrigin() += mDeltaRecoverMovement;

        btVector3 shapeAabbMin, shapeAabbMax;
        mCollisionShapes[kinIndex]->getAabb(shapeTransform, shapeAabbMin, shapeAabbMax);

        if (!shapes_found)
        {
            minAabb = shapeAabbMin;
            maxAabb = shapeAabbMax;
            shapes_found = true;
        }
        else
        {
            minAabb.setX((minAabb.x() < shapeAabbMin.x()) ? minAabb.x() : shapeAabbMin.x());
            minAabb.setY((minAabb.y() < shapeAabbMin.y()) ? minAabb.y() : shapeAabbMin.y());
            minAabb.setZ((minAabb.z() < shapeAabbMin.z()) ? minAabb.z() : shapeAabbMin.z());

            maxAabb.setX((maxAabb.x() > shapeAabbMax.x()) ? maxAabb.x() : shapeAabbMax.x());
            maxAabb.setY((maxAabb.y() > shapeAabbMax.y()) ? maxAabb.y() : shapeAabbMax.y());
            maxAabb.setZ((maxAabb.z() > shapeAabbMax.z()) ? maxAabb.z() : shapeAabbMax.z());
        }
    }

    // If there are no shapes then there is no penetration either
    if (!shapes_found)
    {
        return false;
    }
    // Perform broadphase test
    struct RecoverBroadPhaseCallback : public btBroadphaseAabbCallback
    {
    private:
        btDbvtVolume mBounds;

        const btCollisionObject* mSelfCollisionObject;
        /*
        uint32_t mCollisionLayer;
        uint32_t mCollisionMask;
        */

        struct CompoundLeafCallback : btDbvt::ICollide
        {
        private:
            RecoverBroadPhaseCallback* mParentCallback;
            btCollisionObject* mCollisionObject;

        public:
            CompoundLeafCallback(RecoverBroadPhaseCallback* parentCallback, btCollisionObject* collisionObject)
                : mParentCallback(parentCallback), mCollisionObject(collisionObject)
            {
            }

            void Process(const btDbvtNode* leaf)
            {
                BroadphaseResult result = {mCollisionObject, leaf->dataAsInt};
                mParentCallback->mResults.push_back(result);
            }
        };

    public:
        struct BroadphaseResult
        {
            btCollisionObject* mCollisionObject;
            int mCompoundChildIndex;
        };

        std::vector<BroadphaseResult> mResults;

    public:
        RecoverBroadPhaseCallback(const btCollisionObject* selfCollisionObject, uint32_t collisionLayer,
                                  uint32_t collisionMask, btVector3 minAabb, btVector3 maxAabb)
            : mSelfCollisionObject(
                  selfCollisionObject) /*, mCollisionLayer(collisionLayer), mCollisionMask(collisionMask) */
        {
            mBounds = btDbvtVolume::FromMM(minAabb, maxAabb);
        }

        virtual ~RecoverBroadPhaseCallback() {}

        virtual bool process(const btBroadphaseProxy* proxy)
        {
            btCollisionObject* co = static_cast<btCollisionObject*>(proxy->m_clientObject);
            if (co->getInternalType() <= btCollisionObject::CO_RIGID_BODY)
            {
                if (mSelfCollisionObject != proxy->m_clientObject /* && GodotFilterCallback::test_collision_filters(
                        collision_layer, collision_mask, proxy->m_collisionFilterGroup, proxy->m_collisionFilterMask) */)
                {
                    if (co->getCollisionShape()->isCompound())
                    {
                        const btCompoundShape* cs = static_cast<btCompoundShape*>(co->getCollisionShape());

                        if (cs->getNumChildShapes() > 1)
                        {
                            const btDbvt* tree = cs->getDynamicAabbTree();
                            if (!tree)
                                return true;

                            // Transform bounds into compound shape local space
                            const btTransform otherInCompoundSpace = co->getWorldTransform().inverse();
                            const btMatrix3x3 absB = otherInCompoundSpace.getBasis().absolute();
                            const btVector3 localCenter = otherInCompoundSpace(mBounds.Center());
                            const btVector3 localExtent = mBounds.Extents().dot3(absB[0], absB[1], absB[2]);
                            const btVector3 localMinAabb = localCenter - localExtent;
                            const btVector3 localMaxAabb = localCenter + localExtent;
                            const btDbvtVolume localBounds = btDbvtVolume::FromMM(localMinAabb, localMaxAabb);

                            // Test collision against compound child shapes using its AABB tree
                            CompoundLeafCallback compoundLeafCallback(this, co);
                            tree->collideTV(tree->m_root, localBounds, compoundLeafCallback);
                        }
                        else
                        {
                            // If there's only a single child shape then there's no need to search any more, we know
                            // which child overlaps
                            BroadphaseResult result = {co, 0};
                            mResults.push_back(result);
                        }
                    }
                    else
                    {
                        BroadphaseResult result = {co, -1};
                        mResults.push_back(result);
                    }
                    return true;
                }
            }
            return false;
        }
    };

    RecoverBroadPhaseCallback recoverBroadResult(mRigidBody, 1 /* collision layer */, 0xFFFF /* collision mask */,
                                                 minAabb, maxAabb);
    collisionWorld->getBroadphase()->aabbTest(minAabb, maxAabb, recoverBroadResult);

    bool penetration = false;

    /*    collisionWorld->getDispatcher()->dispatchAllCollisionPairs(
            m_ghostObject->getOverlappingPairCache(), collisionWorld->getDispatchInfo(),
       collisionWorld->getDispatcher());

        m_currentPosition = m_ghostObject->getWorldTransform().getOrigin();

        //	btScalar maxPen = btScalar(0.0);
        for (int i = 0; i < m_ghostObject->getOverlappingPairCache()->getNumOverlappingPairs(); i++)
        {
            m_manifoldArray.resize(0);

            btBroadphasePair* collisionPair = &m_ghostObject->getOverlappingPairCache()->getOverlappingPairArray()[i];

            btCollisionObject* obj0 = static_cast<btCollisionObject*>(collisionPair->m_pProxy0->m_clientObject);
            btCollisionObject* obj1 = static_cast<btCollisionObject*>(collisionPair->m_pProxy1->m_clientObject);

            if ((obj0 && !obj0->hasContactResponse()) || (obj1 && !obj1->hasContactResponse()))
                continue;

            if (!needsCollision(obj0, obj1))
                continue;

            if (collisionPair->m_algorithm)
                collisionPair->m_algorithm->getAllContactManifolds(m_manifoldArray);

            for (int j = 0; j < m_manifoldArray.size(); j++)
            {
                btPersistentManifold* manifold = m_manifoldArray[j];
                btScalar directionSign = manifold->getBody0() == m_ghostObject ? btScalar(-1.0) : btScalar(1.0);
                for (int p = 0; p < manifold->getNumContacts(); p++)
                {
                    const btManifoldPoint& pt = manifold->getContactPoint(p);

                    btScalar dist = pt.getDistance();

                    if (dist < -m_maxPenetrationDepth)
                    {
                        // TODO: cause problems on slopes, not sure if it is needed
                        // if (dist < maxPen)
                        //{
                        //	maxPen = dist;
                        //	m_touchingNormal = pt.m_normalWorldOnB * directionSign;//??

                        //}
                        m_currentPosition += pt.m_normalWorldOnB * directionSign * dist * btScalar(0.2);
                        penetration = true;
                    }
                    else
                    {
                        // printf("touching %f\n", dist);
                    }
                }

                // manifold->clearManifold();
            }
        }
        btTransform newTrans = m_ghostObject->getWorldTransform();
        newTrans.setOrigin(m_currentPosition);
        m_ghostObject->setWorldTransform(newTrans);
        //	printf("m_touchingNormal = %f,%f,%f\n",m_touchingNormal[0],m_touchingNormal[1],m_touchingNormal[2]);
    */
    // Perform narrowphase per shape
    // godot: for (int kinIndex = p_body->get_kinematic_utilities()->shapes.size() - 1; 0 <= kinIndex; --kinIndex)
    for (int kinIndex = 0; kinIndex < (int)mCollisionShapes.size(); kinIndex++)
    {
        if (mCollisionShapes[kinIndex]->getShapeType() == EMPTY_SHAPE_PROXYTYPE)
        {
            continue;
        }

        btTransform shapeTransform = bodyPosition * mCollisionTransforms[kinIndex];
        shapeTransform.getOrigin() += mDeltaRecoverMovement;

        for (int i = recoverBroadResult.mResults.size() - 1; 0 <= i; --i)
        {
            btCollisionObject* otherObject = recoverBroadResult.mResults[i].mCollisionObject;

            /*
            CollisionObjectBullet* gObj = static_cast<CollisionObjectBullet*>(otherObject->getUserPointer());
            if (p_exclude.has(gObj->get_self()))
            {
                continue;
            }
            */

            if (mInfiniteInertia && !otherObject->isStaticOrKinematicObject())
            {
                otherObject->activate(); // Force activation of hitten rigid, soft body
                continue;
            }
            else if (!mRigidBody->checkCollideWith(otherObject) || !otherObject->checkCollideWith(mRigidBody))
            {
                continue;
            }

            if (otherObject->getCollisionShape()->isCompound())
            {
                const btCompoundShape* cs = static_cast<const btCompoundShape*>(otherObject->getCollisionShape());
                if (cs->getNumChildShapes() == 0)
                {
                    continue; // No shapes to depenetrate from.
                }
                int shapeIdx = recoverBroadResult.mResults[i].mCompoundChildIndex;
                if (shapeIdx < 0 || shapeIdx > cs->getNumChildShapes())
                    return false;

                if (cs->getChildShape(shapeIdx)->isConvex())
                {
                    if (RFP_convex_convex_test(mCollisionShapes[kinIndex],
                                               static_cast<const btConvexShape*>(cs->getChildShape(shapeIdx)),
                                               otherObject, kinIndex, shapeIdx, shapeTransform,
                                               otherObject->getWorldTransform() * cs->getChildTransform(shapeIdx),
                                               mRecoverMovementScale, mDeltaRecoverMovement, rresult))
                    {
                        penetration = true;
                    }
                }
                else
                {
                    if (RFP_convex_world_test(mCollisionShapes[kinIndex], cs->getChildShape(shapeIdx), mRigidBody,
                                              otherObject, kinIndex, shapeIdx, shapeTransform,
                                              otherObject->getWorldTransform() * cs->getChildTransform(shapeIdx),
                                              mRecoverMovementScale, mDeltaRecoverMovement, rresult))
                    {
                        penetration = true;
                    }
                }
            }
            else if (otherObject->getCollisionShape()->isConvex())
            { /// Execute GJK test against object shape
                if (RFP_convex_convex_test(mCollisionShapes[kinIndex],
                                           static_cast<const btConvexShape*>(otherObject->getCollisionShape()),
                                           otherObject, kinIndex, 0, shapeTransform, otherObject->getWorldTransform(),
                                           mRecoverMovementScale, mDeltaRecoverMovement, rresult))
                {
                    penetration = true;
                }
            }
            else
            {
                if (RFP_convex_world_test(mCollisionShapes[kinIndex], otherObject->getCollisionShape(), mRigidBody,
                                          otherObject, kinIndex, 0, shapeTransform, otherObject->getWorldTransform(),
                                          mRecoverMovementScale, mDeltaRecoverMovement, rresult))
                {
                    penetration = true;
                }
            }
        }
    }

    return penetration;
}
#endif
bool KinematicMotionSimple::recoverFromPenetration(btCollisionWorld* collisionWorld)
{
    // Here we must refresh the overlapping paircache as the penetrating movement itself or the
    // previous recovery iteration might have used setWorldTransform and pushed us into an object
    // that is not in the previous cache contents from the last timestep, as will happen if we
    // are pushed into a new AABB overlap. Unhandled this means the next convex sweep gets stuck.
    //
    // Do this by calling the broadphase's setAabb with the moved AABB, this will update the broadphase
    // paircache and the ghostobject's internal paircache at the same time.    /BW

    btVector3 minAabb, maxAabb;
    bool shapes_found = false;
    btTransform bodyPosition = mGhostObject->getWorldTransform();

    for (int kinIndex = 0; kinIndex < (int)mCollisionShapes.size(); kinIndex++)
    {

        btTransform shapeTransform = bodyPosition * mCollisionTransforms[kinIndex];

        btVector3 shapeAabbMin, shapeAabbMax;
        mCollisionShapes[kinIndex]->getAabb(shapeTransform, shapeAabbMin, shapeAabbMax);

        if (!shapes_found)
        {
            minAabb = shapeAabbMin;
            maxAabb = shapeAabbMax;
            shapes_found = true;
        }
        else
        {
            minAabb.setX((minAabb.x() < shapeAabbMin.x()) ? minAabb.x() : shapeAabbMin.x());
            minAabb.setY((minAabb.y() < shapeAabbMin.y()) ? minAabb.y() : shapeAabbMin.y());
            minAabb.setZ((minAabb.z() < shapeAabbMin.z()) ? minAabb.z() : shapeAabbMin.z());

            maxAabb.setX((maxAabb.x() > shapeAabbMax.x()) ? maxAabb.x() : shapeAabbMax.x());
            maxAabb.setY((maxAabb.y() > shapeAabbMax.y()) ? maxAabb.y() : shapeAabbMax.y());
            maxAabb.setZ((maxAabb.z() > shapeAabbMax.z()) ? maxAabb.z() : shapeAabbMax.z());
        }
    }

    // If there are no shapes then there is no penetration either
    if (!shapes_found)
    {
        return false;
    }
    collisionWorld->getBroadphase()->setAabb(mGhostObject->getBroadphaseHandle(), minAabb, maxAabb,
                                             collisionWorld->getDispatcher());

    bool penetration = false;

    collisionWorld->getDispatcher()->dispatchAllCollisionPairs(
        mGhostObject->getOverlappingPairCache(), collisionWorld->getDispatchInfo(), collisionWorld->getDispatcher());

    mCurrentPosition = mGhostObject->getWorldTransform().getOrigin();

    //	btScalar maxPen = btScalar(0.0);
    for (int i = 0; i < mGhostObject->getOverlappingPairCache()->getNumOverlappingPairs(); i++)
    {
        mManifoldArray.resize(0);

        btBroadphasePair* collisionPair = &mGhostObject->getOverlappingPairCache()->getOverlappingPairArray()[i];

        btCollisionObject* obj0 = static_cast<btCollisionObject*>(collisionPair->m_pProxy0->m_clientObject);
        btCollisionObject* obj1 = static_cast<btCollisionObject*>(collisionPair->m_pProxy1->m_clientObject);

        if ((obj0 && !obj0->hasContactResponse()) || (obj1 && !obj1->hasContactResponse()))
            continue;

        if (!needsCollision(obj0, obj1))
            continue;

        if (collisionPair->m_algorithm)
            collisionPair->m_algorithm->getAllContactManifolds(mManifoldArray);

        for (int j = 0; j < mManifoldArray.size(); j++)
        {
            btPersistentManifold* manifold = mManifoldArray[j];
            btScalar directionSign = manifold->getBody0() == mGhostObject ? btScalar(-1.0) : btScalar(1.0);
            for (int p = 0; p < manifold->getNumContacts(); p++)
            {
                const btManifoldPoint& pt = manifold->getContactPoint(p);

                btScalar dist = pt.getDistance();

                if (dist < -mMaxPenetrationDepth)
                {
                    // TODO: cause problems on slopes, not sure if it is needed
                    // if (dist < maxPen)
                    //{
                    //	maxPen = dist;
                    //	m_touchingNormal = pt.m_normalWorldOnB * directionSign;//??

                    //}
                    mCurrentPosition += pt.m_normalWorldOnB * directionSign * dist * btScalar(0.2);
                    penetration = true;
                }
                else
                {
                    // printf("touching %f\n", dist);
                }
            }

            // manifold->clearManifold();
        }
    }
    btTransform newTrans = mGhostObject->getWorldTransform();
    newTrans.setOrigin(mCurrentPosition);
    mGhostObject->setWorldTransform(newTrans);
    //	printf("m_touchingNormal = %f,%f,%f\n",m_touchingNormal[0],m_touchingNormal[1],m_touchingNormal[2]);
    return penetration;
}
bool KinematicMotionSimple::needsCollision(const btCollisionObject* body0, const btCollisionObject* body1)
{
    bool collides = (body0->getBroadphaseHandle()->m_collisionFilterGroup &
                     body1->getBroadphaseHandle()->m_collisionFilterMask) != 0;
    collides = collides && (body1->getBroadphaseHandle()->m_collisionFilterGroup &
                            body0->getBroadphaseHandle()->m_collisionFilterMask);
    return collides;
}
void KinematicMotionSimple::preStep(btCollisionWorld* collisionWorld)
{
    mCurrentPosition = mGhostObject->getWorldTransform().getOrigin();
    //	m_targetPosition = m_currentPosition;

    mCurrentOrientation = mGhostObject->getWorldTransform().getRotation();
    //	m_targetOrientation = m_currentOrientation;
    //	printf("m_targetPosition=%f,%f,%f\n",m_targetPosition[0],m_targetPosition[1],m_targetPosition[2]);
}
void KinematicMotionSimple::playerStep(btCollisionWorld* collisionWorld, btScalar dt)
{

    int numPenetrationLoops = 0;
    //	m_touchingContact = false;
    while (recoverFromPenetration(collisionWorld))
    {
        numPenetrationLoops++;
        //		m_touchingContact = true;
        if (numPenetrationLoops > 4)
        {
            // printf("character could not recover from penetration = %d\n", numPenetrationLoops);
            break;
        }
    }
}

void KinematicMotionSimple::updateAction(btCollisionWorld* collisionWorld, btScalar deltaTimeStep)
{
    preStep(collisionWorld);
    playerStep(collisionWorld, deltaTimeStep);
}
void KinematicMotionSimple::debugDraw(btIDebugDraw* debugDrawer) {}
void KinematicMotionSimple::setupCollisionShapes(btCollisionObject* body)
{
    std::list<std::pair<btCompoundShape*, btTransform>> shape_list;
    btCollisionShape* root_shape = body->getCollisionShape();
    btTransform root_xform;
    root_xform.setIdentity();
    if (root_shape->isCompound())
    {
        btCompoundShape* cshape = static_cast<btCompoundShape*>(root_shape);
        shape_list.push_back({cshape, root_xform});
    }
    else
    {
        mCollisionShapes.push_back(root_shape);
        mCollisionTransforms.push_back(root_xform);
    }
    while (!shape_list.empty())
    {
        int i;
        std::pair<btCompoundShape*, btTransform> s = shape_list.front();
        shape_list.pop_front();
        for (i = 0; i < s.first->getNumChildShapes(); i++)
        {
            btCollisionShape* shape = s.first->getChildShape(i);
            btTransform xform = s.second * s.first->getChildTransform(i);
            if (shape->isConvex())
            {
                mCollisionShapes.push_back(shape);
                mCollisionTransforms.push_back(xform);
            }
            else if (shape->isCompound())
            {
                btCompoundShape* cshape = static_cast<btCompoundShape*>(shape);
                shape_list.push_back({cshape, xform});
            }
        }
    }
}
KinematicMotionSimple::KinematicMotionSimple(btPairCachingGhostObject* ghostObject)
    : btActionInterface(), mGhostObject(ghostObject)
{
    setupCollisionShapes(ghostObject);
}

} // namespace Bullet
} // namespace Ogre
