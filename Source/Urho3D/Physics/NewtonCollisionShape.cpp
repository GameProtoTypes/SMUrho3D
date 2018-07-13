
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
        bool s = context->RegisterFactory<NewtonCollisionShape>();
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

        Matrix4 mat;
        //mat.SetScale(size_);
        mat.SetTranslation(position_);
        mat.SetRotation(rotation_.RotationMatrix());
        dMatrix nMat = UrhoToNewton(mat);


        if (shapeType_ == SHAPE_BOX) {

            // get a newton collision object (note: the same NewtonCollision could be shared between multiple component so this is not nessecarily unique)
            newtonCollision_ = NewtonCreateBox(world, size_.x_*node_->GetScale().x_,
                size_.y_*node_->GetScale().y_,
                size_.z_*node_->GetScale().z_, 0, &nMat[0][0]);

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
        rigidBody_ = node_->GetComponent<NewtonRigidBody>();
        if (rigidBody_) {
            rigidBody_->reEvaluateBody();
        }
    }

    NewtonCollision* NewtonCollisionShape::GetNewtonCollision()
    {
        return newtonCollision_;
    }




    void NewtonCollisionShape::updateReferenceToRigidBody()
{
        rigidBody_ = node_->GetComponent<NewtonRigidBody>();
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
       
    }

    void NewtonCollisionShape::OnSceneSet(Scene* scene)
    {
        if (scene)
        {
            if (scene == node_)
                URHO3D_LOGWARNING(GetTypeName() + " should not be created to the root scene node");

            physicsWorld_ = WeakPtr<UrhoNewtonPhysicsWorld>(scene->GetOrCreateComponent<UrhoNewtonPhysicsWorld>());

            reEvaluateCollision();


            physicsWorld_->addCollisionShape(this);


            notifyRigidBody();
        }
        else
        {
            freeInternalCollision();

            if (physicsWorld_)
                physicsWorld_->removeCollisionShape(this);


            notifyRigidBody();
        }
    }

    void NewtonCollisionShape::OnMarkedDirty(Node* node)
    {
       // throw std::logic_error("The method or operation is not implemented.");
    }

}