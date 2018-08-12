
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
#include "Graphics/StaticModel.h"
#include "Scene/SceneEvents.h"


namespace Urho3D {



    NewtonCollisionShape::NewtonCollisionShape(Context* context) : Component(context)
    {
        SubscribeToEvent(E_NODEADDED, URHO3D_HANDLER(NewtonCollisionShape, HandleNodeAdded));
        SubscribeToEvent(E_NODEREMOVED, URHO3D_HANDLER(NewtonCollisionShape, HandleNodeRemoved));
    }

    NewtonCollisionShape::~NewtonCollisionShape()
    {
        freeInternalCollision();
    }

    void NewtonCollisionShape::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape>(DEF_PHYSICS_CATEGORY.CString());




        //URHO3D_ACCESSOR_ATTRIBUTE("Is Enabled", IsEnabled, SetEnabled, bool, true, AM_DEFAULT);
        //URHO3D_ENUM_ATTRIBUTE_EX("Shape Type", shapeType_, MarkShapeDirty, typeNames, SHAPE_BOX, AM_DEFAULT);
        //URHO3D_ATTRIBUTE_EX("Size", Vector3, size_, MarkShapeDirty, Vector3::ONE, AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("Offset Position", GetPosition, SetPosition, Vector3, Vector3::ZERO, AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("Offset Rotation", GetRotation, SetRotation, Quaternion, Quaternion::IDENTITY, AM_DEFAULT);
        //URHO3D_MIXED_ACCESSOR_ATTRIBUTE("Model", GetModelAttr, SetModelAttr, ResourceRef, ResourceRef(Model::GetTypeStatic()), AM_DEFAULT);
        //URHO3D_ATTRIBUTE_EX("LOD Level", int, lodLevel_, MarkShapeDirty, 0, AM_DEFAULT);
        //URHO3D_ATTRIBUTE_EX("Collision Margin", float, margin_, MarkShapeDirty, DEFAULT_COLLISION_MARGIN, AM_DEFAULT);
        //URHO3D_ATTRIBUTE_EX("CustomGeometry ComponentID", unsigned, customGeometryID_, MarkShapeDirty, 0, AM_DEFAULT | AM_COMPONENTID);


    }

    //void NewtonCollisionShape::SetBox(const Vector3& size, const Vector3& position, const Quaternion& rotation)
    //{
    //    shapeType_ = SHAPE_BOX;
    //    size_ = size;
    //    position_ = position;
    //    rotation_ = rotation;

    //    reEvaluateCollision();
    //    notifyRigidBody();
    //}


    //void NewtonCollisionShape::SetSphere(float diameter, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    //{
    //    shapeType_ = SHAPE_SPHERE;
    //    size_ = Vector3(diameter, 0, 0);
    //    position_ = position;
    //    rotation_ = rotation;

    //    reEvaluateCollision();
    //    notifyRigidBody();
    //}

    //void NewtonCollisionShape::SetCylinder(float diameter, float height, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    //{
    //    shapeType_ = SHAPE_CYLINDER;
    //    size_ = Vector3(diameter, height, 0);
    //    position_ = position;
    //    rotation_ = rotation;

    //    reEvaluateCollision();
    //    notifyRigidBody();
    //}

    //void NewtonCollisionShape::SetCapsule(float diameter, float height, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    //{
    //    shapeType_ = SHAPE_CAPSULE;
    //    size_ = Vector3(diameter, height, 0);
    //    position_ = position;
    //    rotation_ = rotation;

    //    reEvaluateCollision();
    //    notifyRigidBody();
    //}

    //void NewtonCollisionShape::SetCone(float diameter, float height, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    //{
    //    shapeType_ = SHAPE_CONE;
    //    size_ = Vector3(diameter, height, 0);
    //    position_ = position;
    //    rotation_ = rotation;

    //    reEvaluateCollision();
    //    notifyRigidBody();
    //}

    //void NewtonCollisionShape::SetTriangleMesh(Model* model, unsigned lodLevel /*= 0*/, const Vector3& scale /*= Vector3::ONE*/, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    //{
    //    if (!model)
    //        return;

    //    shapeType_ = SHAPE_TRIANGLEMESH;
    //    size_ = scale;
    //    position_ = position;
    //    rotation_ = rotation;
    //    model_ = model;
    //    modelLodLevel_ = lodLevel;



    //    reEvaluateCollision();
    //    notifyRigidBody();


    //}

    //void NewtonCollisionShape::SetConvexHull(Model* model, unsigned lodLevel /*= 0*/, float tolerance /*= 0.0f*/, const Vector3& scale /*= Vector3::ONE*/, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    //{
    //    if (!model)
    //        return;

    //    shapeType_ = SHAPE_CONVEXHULL;
    //    size_ = scale;
    //    position_ = position;
    //    rotation_ = rotation;
    //    model_ = model;
    //    modelLodLevel_ = lodLevel;
    //    hullTolerance_ = tolerance;


    //    reEvaluateCollision();
    //    notifyRigidBody();

    //}

    //void NewtonCollisionShape::SetCompound(Model* model, unsigned lodLevel /*= 0*/, float tolerance /*= 0.0f*/, const Vector3& scale /*= Vector3::ONE*/, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    //{
    //    if (!model)
    //        return;

    //    shapeType_ = SHAPE_COMPOUND;
    //    size_ = scale;
    //    position_ = position;
    //    rotation_ = rotation;
    //    model_ = model;
    //    modelLodLevel_ = lodLevel;
    //    hullTolerance_ = tolerance;


    //    reEvaluateCollision();
    //    notifyRigidBody();
    //}

    void NewtonCollisionShape::reEvaluateCollision()
    {
        if (!shapeNeedsRebuilt_)
            return;


            // first free any reference to an existing collision.
            freeInternalCollision();

            //call the derived class createNewtonCollision function.
            createNewtonCollision();

            //compute volume.
            updateVolume();


        shapeNeedsRebuilt_ = false;
    }

    void NewtonCollisionShape::createNewtonCollision()
    {

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

    }

    void NewtonCollisionShape::MarkDirty(bool dirty /*= true*/)
    {
        shapeNeedsRebuilt_ = dirty;
        if (rigidBody_)
            rigidBody_->MarkDirty(true);
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

//    void NewtonCollisionShape::formTriangleMeshCollision()
//    {
//        NewtonWorld* world = physicsWorld_->GetNewtonWorld();
//
//        if (formTriangleMesh()) {
//            newtonCollision_ = NewtonCreateCompoundCollisionFromMesh(world, newtonMesh_->mesh, 0.0f, 0, 0);//not working yet..
//        }
//        else
//        {
//            URHO3D_LOGWARNING("Unable To Create Triangle Mesh For Model.");
//        }
//    }
//
//    bool NewtonCollisionShape::formConvexHullCollision()
//{
//        NewtonWorld* world = physicsWorld_->GetNewtonWorld();
//
//        const unsigned char* vertexData;
//        const unsigned char* indexData;
//        unsigned elementSize, indexSize;
//        const PODVector<VertexElement>* elements;
//        Geometry* geo = model_->GetGeometry(modelGeomIndx_, modelLodLevel_);
//        geo->GetRawData(vertexData, elementSize, indexData, indexSize, elements);
//
//        bool hasPosition = VertexBuffer::HasElement(*elements, TYPE_VECTOR3, SEM_POSITION);
//
//        if (vertexData && indexData && hasPosition) {
//
//            unsigned vertexStart = geo->GetVertexStart();
//            unsigned vertexCount = geo->GetVertexCount();
//            unsigned indexStart = geo->GetIndexStart();
//            unsigned indexCount = geo->GetIndexCount();
//
//            unsigned positionOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);
//
//            newtonCollision_ = NewtonCreateConvexHull(world, vertexCount, (float*)vertexData, elementSize, hullTolerance_, 0, nullptr);
//            return true;
//        }
//        else
//        {
//            URHO3D_LOGWARNING("Unable To Create Convex Hull For Model: " + model_->GetName());
//            return false;
//        }
//    }
//
//    bool NewtonCollisionShape::formCompoundCollision()
//{
//        NewtonWorld* world = physicsWorld_->GetNewtonWorld();
//
//        if (formTriangleMesh())
//        {
//            newtonCollision_ = NewtonCreateCompoundCollisionFromMesh(world, newtonMesh_->mesh, hullTolerance_, 0, 0);
//            return true;
//        }
//        return false;
//    }
//
//
//
//
 
    void NewtonCollisionShape::OnNodeSet(Node* node)
    {

        if (node)
        {

            SubscribeToEvent(node, E_NODEADDED, URHO3D_HANDLER(NewtonCollisionShape, HandleNodeAdded));


            physicsWorld_ = WeakPtr<UrhoNewtonPhysicsWorld>(GetScene()->GetOrCreateComponent<UrhoNewtonPhysicsWorld>());

            //reEvaluateCollision();
            physicsWorld_->addCollisionShape(this);

        }
        else
        {
            freeInternalCollision();
            if (physicsWorld_)
                physicsWorld_->removeCollisionShape(this);
        }

    }



    void NewtonCollisionShape::HandleNodeAdded(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeAdded::P_NODE].GetPtr());

        //if the node has been added to a parent.
        if (node == node_)
        {
            //markt the rigid body dirty
            if (node->HasComponent<NewtonRigidBody>())
            {
                node->GetComponent<NewtonRigidBody>()->MarkDirty();
            }

            //mark its old parent dirty as well.
            if (oldNodeParent_)
            {

                //go up until rigid body is found.
                NewtonRigidBody* oldParentRigidBody = oldNodeParent_->GetComponent<NewtonRigidBody>();
                if (!oldParentRigidBody)
                    oldParentRigidBody = oldNodeParent_->GetParentComponent<NewtonRigidBody>(true);

                if (oldParentRigidBody)
                    oldParentRigidBody->MarkDirty();
            }
            oldNodeParent_ = nullptr;
        }
    }

    void NewtonCollisionShape::HandleNodeRemoved(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeRemoved::P_NODE].GetPtr());
        if (node == node_)
        {
            Node* oldParent = static_cast<Node*>(eventData[NodeRemoved::P_PARENT].GetPtr());
            oldNodeParent_ = oldParent;
        }
    }












    NewtonCollisionShape_Box::NewtonCollisionShape_Box(Context* context) : NewtonCollisionShape(context)
    {

    }

    NewtonCollisionShape_Box::~NewtonCollisionShape_Box()
    {

    }

    void NewtonCollisionShape_Box::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_Box>(DEF_PHYSICS_CATEGORY.CString());


    }

    void NewtonCollisionShape_Box::createNewtonCollision()
    {
        // get a newton collision object (note: the same NewtonCollision could be shared between multiple component so this is not nessecarily a unique pointer)
        newtonCollision_ = NewtonCreateBox(physicsWorld_->GetNewtonWorld(), size_.x_,
            size_.y_,
            size_.z_, 0, nullptr);
    }







    NewtonCollisionShape_Sphere::NewtonCollisionShape_Sphere(Context* context) : NewtonCollisionShape(context)
    {

    }

    NewtonCollisionShape_Sphere::~NewtonCollisionShape_Sphere()
    {

    }

    void NewtonCollisionShape_Sphere::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_Sphere>(DEF_PHYSICS_CATEGORY.CString());
    }

    void NewtonCollisionShape_Sphere::createNewtonCollision()
    {
        // get a newton collision object (note: the same NewtonCollision could be shared between multiple component so this is not nessecarily a unique pointer)
        newtonCollision_ = NewtonCreateSphere(physicsWorld_->GetNewtonWorld(), radius_, 0, nullptr);
    }












    NewtonCollisionShape_Geometry::NewtonCollisionShape_Geometry(Context* context) : NewtonCollisionShape(context)
    {

    }

    NewtonCollisionShape_Geometry::~NewtonCollisionShape_Geometry()
    {

    }

    void NewtonCollisionShape_Geometry::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_Geometry>(DEF_PHYSICS_CATEGORY.CString());
    }

    bool NewtonCollisionShape_Geometry::resolveOrCreateTriangleMesh()
{
        if (!model_)
            return false;

        NewtonWorld* world = physicsWorld_->GetNewtonWorld();

        /// if the newton mesh is in cache already - return that.
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
            URHO3D_LOGWARNING("Unable To Create NewtonMesh For Model: " + model_->GetName());
            return false;
        }
    }

    void NewtonCollisionShape_Geometry::OnNodeSet(Node* node)
    {
        NewtonCollisionShape::OnNodeSet(node);

        if (node)
            autoSetModel();
    }

    void NewtonCollisionShape_Geometry::createNewtonCollision()
    {

    }


    void NewtonCollisionShape_Geometry::autoSetModel()
    {
        StaticModel* stMdl = node_->GetComponent<StaticModel>();

        if (stMdl)
        {
            SetModel(stMdl->GetModel());
        }
    }













    NewtonCollisionShape_ConvexHullCompound::NewtonCollisionShape_ConvexHullCompound(Context* context) : NewtonCollisionShape_Geometry(context)
    {
    }

    NewtonCollisionShape_ConvexHullCompound::~NewtonCollisionShape_ConvexHullCompound()
    {
    }

    void NewtonCollisionShape_ConvexHullCompound::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_ConvexHullCompound>(DEF_PHYSICS_CATEGORY.CString());
    }


    void NewtonCollisionShape_ConvexHullCompound::createNewtonCollision()
    {
        NewtonWorld* world = physicsWorld_->GetNewtonWorld();

        resolveOrCreateTriangleMesh();
        newtonCollision_ = NewtonCreateCompoundCollisionFromMesh(world, newtonMesh_->mesh, hullTolerance_, 0, 0);
    }




    NewtonCollisionShape_ConvexDecompositionCompound::NewtonCollisionShape_ConvexDecompositionCompound(Context* context) : NewtonCollisionShape_Geometry(context)
    {

    }



    NewtonCollisionShape_ConvexDecompositionCompound::~NewtonCollisionShape_ConvexDecompositionCompound()
    {

    }




    void NewtonCollisionShape_ConvexDecompositionCompound::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_ConvexDecompositionCompound>(DEF_PHYSICS_CATEGORY.CString());
    }

    void NewtonCollisionShape_ConvexDecompositionCompound::createNewtonCollision()
    {
        NewtonWorld* world = physicsWorld_->GetNewtonWorld();

        resolveOrCreateTriangleMesh();

        //decompose
        newtonMesh_->mesh = NewtonMeshApproximateConvexDecomposition(newtonMesh_->mesh, 0.01f, 0.2f, 256, 100, nullptr, nullptr);

        newtonCollision_ = NewtonCreateCompoundCollisionFromMesh(world, newtonMesh_->mesh, hullTolerance_, 0, 0);
    }

    NewtonCollisionShape_ConvexHull::NewtonCollisionShape_ConvexHull(Context* context) : NewtonCollisionShape_Geometry(context)
    {

    }

    NewtonCollisionShape_ConvexHull::~NewtonCollisionShape_ConvexHull()
    {

    }

    void NewtonCollisionShape_ConvexHull::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_ConvexHull>(DEF_PHYSICS_CATEGORY.CString());
    }

    void NewtonCollisionShape_ConvexHull::createNewtonCollision()
    {
        NewtonWorld* world = physicsWorld_->GetNewtonWorld();

        resolveOrCreateTriangleMesh();
        newtonCollision_ = NewtonCreateConvexHullFromMesh(world, newtonMesh_->mesh, hullTolerance_, 0);
    }






    NewtonCollisionShape_Cylinder::NewtonCollisionShape_Cylinder(Context* context) : NewtonCollisionShape(context)
    {

    }

    NewtonCollisionShape_Cylinder::~NewtonCollisionShape_Cylinder()
    {

    }

    void NewtonCollisionShape_Cylinder::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_Cylinder>(DEF_PHYSICS_CATEGORY.CString());
    }

    void NewtonCollisionShape_Cylinder::createNewtonCollision()
    {
        // get a newton collision object (note: the same NewtonCollision could be shared between multiple component so this is not nessecarily a unique pointer)
        newtonCollision_ = NewtonCreateCylinder(physicsWorld_->GetNewtonWorld(), radius1_, radius2_, length_, 0, nullptr);
    }

    NewtonCollisionShape_Capsule::NewtonCollisionShape_Capsule(Context* context) : NewtonCollisionShape(context)
    {
    }

    NewtonCollisionShape_Capsule::~NewtonCollisionShape_Capsule()
    {
    }

    void NewtonCollisionShape_Capsule::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_Cylinder>(DEF_PHYSICS_CATEGORY.CString());
    }

    void NewtonCollisionShape_Capsule::createNewtonCollision()
    {
        newtonCollision_ = NewtonCreateCapsule(physicsWorld_->GetNewtonWorld(), radius1_, radius2_, length_, 0, nullptr);
    }

    NewtonCollisionShape_Cone::NewtonCollisionShape_Cone(Context* context) : NewtonCollisionShape(context)
    {
    }

    NewtonCollisionShape_Cone::~NewtonCollisionShape_Cone()
    {
    }

    void NewtonCollisionShape_Cone::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_Cone>(DEF_PHYSICS_CATEGORY.CString());
    }

    void NewtonCollisionShape_Cone::createNewtonCollision()
    {
        newtonCollision_ = NewtonCreateCone(physicsWorld_->GetNewtonWorld(), radius_, length_, 0, nullptr);
    }


}
