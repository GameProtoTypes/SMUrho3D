
#include "NewtonCollisionShape.h"
#include "UrhoNewtonPhysicsWorld.h"
#include "NewtonRigidBody.h"
#include "NewtonMeshObject.h"

#include "../Core/Context.h"
#include "../Scene/Component.h"
#include "../Scene/Node.h"
#include "../Scene/Scene.h"
#include "../Graphics/Model.h"
#include "../IO/Log.h"

#include "Newton.h"
#include "dMatrix.h"

#include "UrhoNewtonConversions.h"
#include "Graphics/Geometry.h"
#include "Graphics/VertexBuffer.h"
#include "Graphics/GraphicsDefs.h"
#include "Graphics/IndexBuffer.h"
#include "Graphics/VisualDebugger.h"


namespace Urho3D {



    NewtonCollisionShape::NewtonCollisionShape(Context* context) : Component(context)
    {
    }

    NewtonCollisionShape::~NewtonCollisionShape()
    {
        freeInternalCollision();
    }

    void NewtonCollisionShape::RegisterObject(Context* context)
    {
        bool s = context->RegisterFactory<NewtonCollisionShape>(DEF_PHYSICS_CATEGORY.CString());
    }

    void NewtonCollisionShape::SetBox(const Vector3& size, const Vector3& position, const Quaternion& rotation)
    {
        shapeType_ = SHAPE_BOX;
        size_ = size;
        position_ = position;
        rotation_ = rotation;

        reEvaluateCollision();
        notifyRigidBody();
    }


    void NewtonCollisionShape::SetSphere(float diameter, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    {
        shapeType_ = SHAPE_SPHERE;
        size_ = Vector3(diameter, 0, 0);
        position_ = position;
        rotation_ = rotation;

        reEvaluateCollision();
        notifyRigidBody();
    }

    void NewtonCollisionShape::SetCylinder(float diameter, float height, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    {
        shapeType_ = SHAPE_CYLINDER;
        size_ = Vector3(diameter, height, 0);
        position_ = position;
        rotation_ = rotation;

        reEvaluateCollision();
        notifyRigidBody();
    }

    void NewtonCollisionShape::SetCapsule(float diameter, float height, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    {
        shapeType_ = SHAPE_CAPSULE;
        size_ = Vector3(diameter, height, 0);
        position_ = position;
        rotation_ = rotation;

        reEvaluateCollision();
        notifyRigidBody();
    }

    void NewtonCollisionShape::SetCone(float diameter, float height, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    {
        shapeType_ = SHAPE_CONE;
        size_ = Vector3(diameter, height, 0);
        position_ = position;
        rotation_ = rotation;

        reEvaluateCollision();
        notifyRigidBody();
    }

    void NewtonCollisionShape::SetTriangleMesh(Model* model, unsigned lodLevel /*= 0*/, const Vector3& scale /*= Vector3::ONE*/, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    {
        if (!model)
            return;

        shapeType_ = SHAPE_TRIANGLEMESH;
        size_ = scale;
        position_ = position;
        rotation_ = rotation;
        model_ = model;
        modelLodLevel_ = lodLevel;



        reEvaluateCollision();
        notifyRigidBody();


    }

    void NewtonCollisionShape::SetConvexHull(Model* model, unsigned lodLevel /*= 0*/, float tolerance /*= 0.0f*/, const Vector3& scale /*= Vector3::ONE*/, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    {
        if (!model)
            return;

        shapeType_ = SHAPE_CONVEXHULL;
        size_ = scale;
        position_ = position;
        rotation_ = rotation;
        model_ = model;
        modelLodLevel_ = lodLevel;
        hullTolerance_ = tolerance;


        reEvaluateCollision();
        notifyRigidBody();

    }

    void NewtonCollisionShape::SetCompound(Model* model, unsigned lodLevel /*= 0*/, float tolerance /*= 0.0f*/, const Vector3& scale /*= Vector3::ONE*/, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    {
        if (!model)
            return;

        shapeType_ = SHAPE_COMPOUND;
        size_ = scale;
        position_ = position;
        rotation_ = rotation;
        model_ = model;
        modelLodLevel_ = lodLevel;
        hullTolerance_ = tolerance;


        reEvaluateCollision();
        notifyRigidBody();
    }

    void NewtonCollisionShape::reEvaluateCollision()
{
        // first free any reference to an existing collision.
        freeInternalCollision();

        NewtonWorld * world = physicsWorld_->GetNewtonWorld();




        if (shapeType_ == SHAPE_BOX) {

            // get a newton collision object (note: the same NewtonCollision could be shared between multiple component so this is not nessecarily unique)
            newtonCollision_ = NewtonCreateBox(world, size_.x_,
                size_.y_,
                size_.z_, 0, nullptr);

        }
        else if (shapeType_ == SHAPE_SPHERE) {
            newtonCollision_ = NewtonCreateSphere(world, size_.x_*0.5f, 0, nullptr);
        }
        else if (shapeType_ == SHAPE_CONE) {
            newtonCollision_ = NewtonCreateCone(world, size_.x_*0.5f, size_.z_, 0, nullptr);
        }
        else if (shapeType_ == SHAPE_CYLINDER) {
            newtonCollision_ == NewtonCreateCylinder(world, size_.x_*0.5f, size_.y_*0.5f, size_.z_, 0, nullptr);
        }
        else if (shapeType_ == SHAPE_CAPSULE) {
            newtonCollision_ == NewtonCreateCapsule(world, size_.x_*0.5f, size_.y_*0.5f, size_.z_, 0, nullptr);
        }
        else if (shapeType_ == SHAPE_CHAMFERCYLINDER) {
            newtonCollision_ == NewtonCreateChamferCylinder(world, size_.x_*0.5f, size_.z_, 0, nullptr);
        }
        else if (shapeType_ == SHAPE_CONVEXHULL) {
            formConvexHullCollision();
        }
        else if (shapeType_ == SHAPE_COMPOUND) {
            formCompoundCollision();
        }
        ///.....
        else if (shapeType_ == SHAPE_TRIANGLEMESH)
        {
            formTriangleMeshCollision();
        }

        
        updateVolume();
    }

    void NewtonCollisionShape::freeInternalCollision()
    {
        if (newtonCollision_) {
            NewtonDestroyCollision(newtonCollision_);//decrement the reference count of the collision.
            newtonCollision_ = nullptr;
        }
    }

    void NewtonCollisionShape::notifyRigidBody()
    {
        if(!rigidBody_)
            rigidBody_ = node_->GetComponent<NewtonRigidBody>();

        if (rigidBody_) {
            rigidBody_->reEvaluateBody();
        }
    }

    NewtonCollision* NewtonCollisionShape::GetNewtonCollision()
    {
        if (newtonCollision_ == nullptr)
        {
            reEvaluateCollision();
        }
        return newtonCollision_;
    }




    void NewtonCollisionShape::updateReferenceToRigidBody()
    {
        rigidBody_ = node_->GetComponent<NewtonRigidBody>();
    }

    float NewtonCollisionShape::updateVolume()
{
        float vol = 0.0f;
        if (newtonCollision_)
            vol = NewtonConvexCollisionCalculateVolume(newtonCollision_);

        if (vol <= 0.0f) {
            volume_ = 0.0f;
            return 0.0f;
        }
        else
        {
            volume_ = vol;
            return vol;
        }
    }

    void NewtonCollisionShape::formTriangleMeshCollision()
    {
        NewtonWorld* world = physicsWorld_->GetNewtonWorld();

        if (formTriangleMesh()) {
            newtonCollision_ = NewtonCreateCompoundCollisionFromMesh(world, newtonMesh_->mesh, 0.0f, 0, 0);//not working yet..
        }
        else
        {
            URHO3D_LOGWARNING("Unable To Create Triangle Mesh For Model.");
        }
    }

    bool NewtonCollisionShape::formConvexHullCollision()
{
        NewtonWorld* world = physicsWorld_->GetNewtonWorld();

        const unsigned char* vertexData;
        const unsigned char* indexData;
        unsigned elementSize, indexSize;
        const PODVector<VertexElement>* elements;
        Geometry* geo = model_->GetGeometry(modelGeomIndx_, modelLodLevel_);
        geo->GetRawData(vertexData, elementSize, indexData, indexSize, elements);

        bool hasPosition = VertexBuffer::HasElement(*elements, TYPE_VECTOR3, SEM_POSITION);

        if (vertexData && indexData && hasPosition) {

            unsigned vertexStart = geo->GetVertexStart();
            unsigned vertexCount = geo->GetVertexCount();
            unsigned indexStart = geo->GetIndexStart();
            unsigned indexCount = geo->GetIndexCount();

            unsigned positionOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);

            newtonCollision_ = NewtonCreateConvexHull(world, vertexCount, (float*)vertexData, elementSize, hullTolerance_, 0, nullptr);
            return true;
        }
        else
        {
            URHO3D_LOGWARNING("Unable To Create Convex Hull For Model: " + model_->GetName());
            return false;
        }
    }

    bool NewtonCollisionShape::formCompoundCollision()
{
        NewtonWorld* world = physicsWorld_->GetNewtonWorld();

        if (formTriangleMesh())
        {
            newtonCollision_ = NewtonCreateCompoundCollisionFromMesh(world, newtonMesh_->mesh, hullTolerance_, 0, 0);
            return true;
        }
        return false;
    }




    bool NewtonCollisionShape::formTriangleMesh()
{
        NewtonWorld* world = physicsWorld_->GetNewtonWorld();

        StringHash meshKey = UrhoNewtonPhysicsWorld::NewtonMeshKey(model_->GetName(), modelLodLevel_, hullTolerance_);
        NewtonMeshObject* cachedMesh = physicsWorld_->GetNewtonMesh(meshKey);
        if (cachedMesh)
        {
            newtonMesh_ = cachedMesh;
            return true;
        }


        const unsigned char* vertexData;
        const unsigned char* indexData;
        unsigned elementSize, indexSize;
        const PODVector<VertexElement>* elements;
        Geometry* geo = model_->GetGeometry(modelGeomIndx_, modelLodLevel_);
        geo->GetRawData(vertexData, elementSize, indexData, indexSize, elements);

        bool hasPosition = VertexBuffer::HasElement(*elements, TYPE_VECTOR3, SEM_POSITION);

        if (vertexData && indexData && hasPosition)
        {
            unsigned vertexStart = geo->GetVertexStart();
            unsigned vertexCount = geo->GetVertexCount();
            unsigned indexStart = geo->GetIndexStart();
            unsigned indexCount = geo->GetIndexCount();

            unsigned positionOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);


            newtonMesh_ = physicsWorld_->GetCreateNewtonMesh(meshKey);
            NewtonMeshBeginBuild(newtonMesh_->mesh);

            for (unsigned curIdx = indexStart; curIdx < indexStart + indexCount; curIdx += 3)
            {
                //get indexes
                unsigned i1, i2, i3;
                if (indexSize == 2) {
                    i1 = *reinterpret_cast<const unsigned short*>(indexData + curIdx * indexSize);
                    i2 = *reinterpret_cast<const unsigned short*>(indexData + (curIdx + 1) * indexSize);
                    i3 = *reinterpret_cast<const unsigned short*>(indexData + (curIdx + 2) * indexSize);
                }
                else if (indexSize == 4)
                {
                    i1 = *reinterpret_cast<const unsigned *>(indexData + curIdx * indexSize);
                    i2 = *reinterpret_cast<const unsigned *>(indexData + (curIdx + 1) * indexSize);
                    i3 = *reinterpret_cast<const unsigned *>(indexData + (curIdx + 2) * indexSize);
                }

                //lookup triangle using indexes.
                const Vector3& v1 = *reinterpret_cast<const Vector3*>(vertexData + i1 * elementSize + positionOffset);
                const Vector3& v2 = *reinterpret_cast<const Vector3*>(vertexData + i2 * elementSize + positionOffset);
                const Vector3& v3 = *reinterpret_cast<const Vector3*>(vertexData + i3 * elementSize + positionOffset);


                NewtonMeshBeginFace(newtonMesh_->mesh);
                    NewtonMeshAddPoint(newtonMesh_->mesh, v1.x_, v1.y_, v1.z_);
                    NewtonMeshAddPoint(newtonMesh_->mesh, v2.x_, v2.y_, v2.z_);
                    NewtonMeshAddPoint(newtonMesh_->mesh, v3.x_, v3.y_, v3.z_);
                NewtonMeshEndFace(newtonMesh_->mesh);
            }

            NewtonMeshEndBuild(newtonMesh_->mesh);

            return true;
        }
        else
        {
            URHO3D_LOGWARNING("Unable To Create NewtonMesh For Model: " +  model_->GetName());
            return false;
        }
    }

    void NewtonCollisionShape::OnNodeSet(Node* node)
    {

        if (node)
        {
            if (GetScene() == node_)
                URHO3D_LOGWARNING(GetTypeName() + " should not be created to the root scene node");

            physicsWorld_ = WeakPtr<UrhoNewtonPhysicsWorld>(GetScene()->GetOrCreateComponent<UrhoNewtonPhysicsWorld>());

            reEvaluateCollision();


            physicsWorld_->addCollisionShape(this);

        }
        else
        {
            freeInternalCollision();

            if (physicsWorld_)
                physicsWorld_->removeCollisionShape(this);

        }


    }

    void NewtonCollisionShape::OnSceneSet(Scene* scene)
    {
       
    }

    void NewtonCollisionShape::OnMarkedDirty(Node* node)
    {
       // throw std::logic_error("The method or operation is not implemented.");
    }

}
