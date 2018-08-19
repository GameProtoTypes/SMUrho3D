
#include "NewtonCollisionShape.h"
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
#include "NewtonDebugDrawing.h"

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


    void NewtonCollisionShape::SetPhysicsMaterial(NewtonPhysicsMaterial* material)
    {
        physicsMaterial_ = material;

        //add the physics material to the newton world 
        physicsWorld_->addPhysicsMaterial(material);
    }

    void NewtonCollisionShape::updateBuild()
    {
            // first free any reference to an existing collision.
            freeInternalCollision();

            //call the derived class createNewtonCollision function.
            buildNewtonCollision();

            //apply material
            applyMaterial();

            //compute volume.
            updateVolume();
    }

    void NewtonCollisionShape::buildNewtonCollision()
{

    }

    void NewtonCollisionShape::freeInternalCollision()
    {
        if (newtonCollision_) {
            NewtonDestroyCollision(newtonCollision_);//decrement the reference count of the collision.
            newtonCollision_ = nullptr;
        }
    }

    void NewtonCollisionShape::applyMaterial()
    {
        if (physicsMaterial_)
        {

        }

    }

    void NewtonCollisionShape::MarkDirty(bool dirty /*= true*/)
    {
        shapeNeedsRebuilt_ = dirty;
    }

    NewtonCollision* NewtonCollisionShape::GetNewtonCollision()
    {
        if (newtonCollision_ == nullptr)
        {
            updateBuild();
        }
        return newtonCollision_;
    }


    void NewtonCollisionShape::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        Component::DrawDebugGeometry(debug, depthTest);
        if (newtonCollision_) {
            NewtonDebug_DrawCollision(newtonCollision_, node_->LocalToWorld(GetOffsetMatrix()), Color::GREEN, debug, depthTest);
        }
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










}
