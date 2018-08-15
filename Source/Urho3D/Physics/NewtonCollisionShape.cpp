
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
#include "NewtonNodePhysicsGlue.h"


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


    void NewtonCollisionShape::reEvaluateCollision()
    {
            // first free any reference to an existing collision.
            freeInternalCollision();

            //call the derived class createNewtonCollision function.
            createNewtonCollision();


            //compute volume.
            updateVolume();
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

    void NewtonCollisionShape::MarkDirty(bool dirty /*= true*/)
    {
        shapeNeedsRebuilt_ = dirty;
        //nodeGlue_->MarkDirty();
    }

    NewtonCollision* NewtonCollisionShape::GetNewtonCollision()
    {
        if (newtonCollision_ == nullptr)
        {
            reEvaluateCollision();
        }
        return newtonCollision_;
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

 
    void NewtonCollisionShape::OnNodeSet(Node* node)
    {

        if (node)
        {
            ///auto create physics world
            physicsWorld_ = WeakPtr<UrhoNewtonPhysicsWorld>(GetScene()->GetOrCreateComponent<UrhoNewtonPhysicsWorld>());

            ///auto create node physics glue component
            node->GetOrCreateComponent<NewtonNodePhysicsGlue>();



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
        StringHash meshKey = UrhoNewtonPhysicsWorld::NewtonMeshKey(model_->GetName(), modelLodLevel_, "");
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
        resolveOrCreateTriangleMesh();
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
        NewtonCollisionShape_Geometry::createNewtonCollision();

        //NewtonWorld* world = physicsWorld_->GetNewtonWorld();


        //String keyData = String(maxConcavity_) + String(backFaceDistanceFactor_) + String(maxCompounds_) + String(maxVertexPerHull_);

        ///// if the newton mesh is in cache already - return that.
        //StringHash meshKey = UrhoNewtonPhysicsWorld::NewtonMeshKey(model_->GetName(), modelLodLevel_, keyData);
        //NewtonMeshObject* cachedMesh = physicsWorld_->GetNewtonMesh(meshKey);
        //if (cachedMesh)
        //{
        //    meshDecomposition_ = cachedMesh;
        //}
        //else
        //{
        //    meshDecomposition_ = physicsWorld_->GetCreateNewtonMesh(meshKey);
        //    meshDecomposition_->mesh = NewtonMeshApproximateConvexDecomposition(newtonMesh_->mesh, maxConcavity_, backFaceDistanceFactor_, maxCompounds_, maxVertexPerHull_, nullptr, nullptr);

        //}

        //newtonCollision_ = NewtonCreateCompoundCollisionFromMesh(world, meshDecomposition_->mesh, hullTolerance_, 0, 0);
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
        NewtonCollisionShape_Geometry::createNewtonCollision();

        NewtonWorld* world = physicsWorld_->GetNewtonWorld();

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
