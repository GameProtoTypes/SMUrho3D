#include "NewtonCollisionShapesDerived.h"
#include "UrhoNewtonPhysicsWorld.h"
#include "NewtonRigidBody.h"
#include "NewtonMeshObject.h"
#include "NewtonPhysicsMaterial.h"

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

