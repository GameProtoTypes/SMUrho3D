#include "CollisionShapesDerived.h"
#include "PhysicsWorld.h"
#include "RigidBody.h"
#include "NewtonMeshObject.h"
#include "PhysicsMaterial.h"

#include "../Core/Context.h"
#include "../Scene/Component.h"
#include "../Scene/Node.h"
#include "../Scene/Scene.h"
#include "../Graphics/Model.h"
#include "../Resource/Image.h"
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
#include "CollisionShape.h"
#include "Graphics/HeightmapTerrain.h"
#include "Container/ArrayPtr.h"


namespace Urho3D {

    CollisionShape_Box::CollisionShape_Box(Context* context) : CollisionShape(context)
    {

    }

    CollisionShape_Box::~CollisionShape_Box()
    {

    }

    void CollisionShape_Box::RegisterObject(Context* context)
    {
        context->RegisterFactory<CollisionShape_Box>(DEF_PHYSICS_CATEGORY.CString());


    }

    void CollisionShape_Box::buildNewtonCollision()
    {
        // get a newton collision object (note: the same NewtonCollision could be shared between multiple component so this is not nessecarily a unique pointer)
        newtonCollision_ = NewtonCreateBox(physicsWorld_->GetNewtonWorld(), size_.x_,
            size_.y_,
            size_.z_, 0, nullptr);
    }







    CollisionShape_Sphere::CollisionShape_Sphere(Context* context) : CollisionShape(context)
    {

    }

    CollisionShape_Sphere::~CollisionShape_Sphere()
    {

    }

    void CollisionShape_Sphere::RegisterObject(Context* context)
    {
        context->RegisterFactory<CollisionShape_Sphere>(DEF_PHYSICS_CATEGORY.CString());
    }

    void CollisionShape_Sphere::buildNewtonCollision()
    {
        // get a newton collision object (note: the same NewtonCollision could be shared between multiple component so this is not nessecarily a unique pointer)
        newtonCollision_ = NewtonCreateSphere(physicsWorld_->GetNewtonWorld(), radius_, 0, nullptr);
    }












    CollisionShape_Geometry::CollisionShape_Geometry(Context* context) : CollisionShape(context)
    {

    }

    CollisionShape_Geometry::~CollisionShape_Geometry()
    {

    }

    void CollisionShape_Geometry::RegisterObject(Context* context)
    {
        context->RegisterFactory<CollisionShape_Geometry>(DEF_PHYSICS_CATEGORY.CString());
    }

    bool CollisionShape_Geometry::resolveOrCreateTriangleMesh()
    {
        if (!model_)
            return false;

        NewtonWorld* world = physicsWorld_->GetNewtonWorld();

        /// if the newton mesh is in cache already - return that.
        StringHash meshKey = PhysicsWorld::NewtonMeshKey(model_->GetName(), modelLodLevel_, "");
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

    void CollisionShape_Geometry::OnNodeSet(Node* node)
    {
        CollisionShape::OnNodeSet(node);

        if (node)
            autoSetModel();
    }

    void CollisionShape_Geometry::buildNewtonCollision()
    {
        resolveOrCreateTriangleMesh();
    }


    void CollisionShape_Geometry::autoSetModel()
    {
        StaticModel* stMdl = node_->GetComponent<StaticModel>();

        if (stMdl)
        {
            SetModel(stMdl->GetModel());
        }
    }













    CollisionShape_ConvexHullCompound::CollisionShape_ConvexHullCompound(Context* context) : CollisionShape_Geometry(context)
    {
    }

    CollisionShape_ConvexHullCompound::~CollisionShape_ConvexHullCompound()
    {
    }

    void CollisionShape_ConvexHullCompound::RegisterObject(Context* context)
    {
        context->RegisterFactory<CollisionShape_ConvexHullCompound>(DEF_PHYSICS_CATEGORY.CString());
    }


    void CollisionShape_ConvexHullCompound::buildNewtonCollision()
    {
        NewtonWorld* world = physicsWorld_->GetNewtonWorld();

        resolveOrCreateTriangleMesh();
        newtonCollision_ = NewtonCreateCompoundCollisionFromMesh(world, newtonMesh_->mesh, hullTolerance_, 0, 0);
    }




    CollisionShape_ConvexDecompositionCompound::CollisionShape_ConvexDecompositionCompound(Context* context) : CollisionShape_Geometry(context)
    {

    }



    CollisionShape_ConvexDecompositionCompound::~CollisionShape_ConvexDecompositionCompound()
    {

    }




    void CollisionShape_ConvexDecompositionCompound::RegisterObject(Context* context)
    {
        context->RegisterFactory<CollisionShape_ConvexDecompositionCompound>(DEF_PHYSICS_CATEGORY.CString());
    }

    void CollisionShape_ConvexDecompositionCompound::buildNewtonCollision()
    {
        CollisionShape_Geometry::buildNewtonCollision();

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

    CollisionShape_ConvexHull::CollisionShape_ConvexHull(Context* context) : CollisionShape_Geometry(context)
    {

    }

    CollisionShape_ConvexHull::~CollisionShape_ConvexHull()
    {

    }

    void CollisionShape_ConvexHull::RegisterObject(Context* context)
    {
        context->RegisterFactory<CollisionShape_ConvexHull>(DEF_PHYSICS_CATEGORY.CString());
    }

    void CollisionShape_ConvexHull::buildNewtonCollision()
    {
        CollisionShape_Geometry::buildNewtonCollision();

        NewtonWorld* world = physicsWorld_->GetNewtonWorld();

        newtonCollision_ = NewtonCreateConvexHullFromMesh(world, newtonMesh_->mesh, hullTolerance_, 0);
    }






    CollisionShape_Cylinder::CollisionShape_Cylinder(Context* context) : CollisionShape(context)
    {
    }

    CollisionShape_Cylinder::~CollisionShape_Cylinder()
    {

    }

    void CollisionShape_Cylinder::RegisterObject(Context* context)
    {
        context->RegisterFactory<CollisionShape_Cylinder>(DEF_PHYSICS_CATEGORY.CString());
    }

    void CollisionShape_Cylinder::buildNewtonCollision()
    {
        // get a newton collision object (note: the same NewtonCollision could be shared between multiple component so this is not nessecarily a unique pointer)
        newtonCollision_ = NewtonCreateCylinder(physicsWorld_->GetNewtonWorld(), radius1_, radius2_, length_, 0, nullptr);

        //set the internal offset correction
        internalRotOffset_ = Quaternion(0, 0, 90);


    }

    CollisionShape_Capsule::CollisionShape_Capsule(Context* context) : CollisionShape(context)
    {
    }

    CollisionShape_Capsule::~CollisionShape_Capsule()
    {
    }

    void CollisionShape_Capsule::RegisterObject(Context* context)
    {
        context->RegisterFactory<CollisionShape_Cylinder>(DEF_PHYSICS_CATEGORY.CString());
    }

    void CollisionShape_Capsule::buildNewtonCollision()
    {
        newtonCollision_ = NewtonCreateCapsule(physicsWorld_->GetNewtonWorld(), radius1_, radius2_, length_, 0, nullptr);
    }

    CollisionShape_Cone::CollisionShape_Cone(Context* context) : CollisionShape(context)
    {
    }

    CollisionShape_Cone::~CollisionShape_Cone()
    {
    }

    void CollisionShape_Cone::RegisterObject(Context* context)
    {
        context->RegisterFactory<CollisionShape_Cone>(DEF_PHYSICS_CATEGORY.CString());
    }

    void CollisionShape_Cone::buildNewtonCollision()
    {
        newtonCollision_ = NewtonCreateCone(physicsWorld_->GetNewtonWorld(), radius_, length_, 0, nullptr);
    }















    NewtonCollisionShape_HeightmapTerrain::NewtonCollisionShape_HeightmapTerrain(Context* context) : CollisionShape(context)
    {

    }

    NewtonCollisionShape_HeightmapTerrain::~NewtonCollisionShape_HeightmapTerrain()
    {

    }

    void NewtonCollisionShape_HeightmapTerrain::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape_HeightmapTerrain>(DEF_PHYSICS_CATEGORY.CString());
    }

    void NewtonCollisionShape_HeightmapTerrain::buildNewtonCollision()
    {



        //find a terrain component
        HeightmapTerrain* terrainComponent = node_->GetComponent<HeightmapTerrain>();
        if (terrainComponent)
        {

            int size = terrainComponent->GetHeightMap()->GetHeight();

            SharedArrayPtr<float> heightData = terrainComponent->GetHeightData();


            char* const attibutes = new char[size * size];
            memset(attibutes, 0, size * size * sizeof(char));

            Vector3 spacing = terrainComponent->GetSpacing();
            newtonCollision_ = NewtonCreateHeightFieldCollision(physicsWorld_->GetNewtonWorld(), size, size, 0, 0, heightData, attibutes, 1.0f, spacing.x_, spacing.z_, 0);

            delete[] attibutes;
            

            //set the internal offset correction to match where HeightmapTerrain renders
            internalPosOffset_ = -Vector3(float(size*spacing.x_)*0.5f - spacing.x_*0.5f, 0, float(size*spacing.z_)*0.5f - spacing.z_*0.5f);

        }

    }

}

