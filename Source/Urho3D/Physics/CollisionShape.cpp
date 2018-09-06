
#include "CollisionShape.h"
#include "PhysicsWorld.h"
#include "RigidBody.h"
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



    CollisionShape::CollisionShape(Context* context) : Component(context)
    {
        SubscribeToEvent(E_NODEADDED, URHO3D_HANDLER(CollisionShape, HandleNodeAdded));
        SubscribeToEvent(E_NODEREMOVED, URHO3D_HANDLER(CollisionShape, HandleNodeRemoved));
    }

    CollisionShape::~CollisionShape()
    {
        freeInternalCollision();
    }

    void CollisionShape::RegisterObject(Context* context)
    {
        context->RegisterFactory<CollisionShape>(DEF_PHYSICS_CATEGORY.CString());
        URHO3D_COPY_BASE_ATTRIBUTES(Component);

        URHO3D_ACCESSOR_ATTRIBUTE("Position Offset", GetPositionOffset, SetPositionOffset, Vector3, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Rotational Offset", GetRotationOffset, SetRotationOffset, Quaternion, Quaternion::IDENTITY, AM_DEFAULT);
    }


    void CollisionShape::SetPhysicsMaterial(NewtonPhysicsMaterial* material)
    {
        physicsMaterial_ = material;

        //add the physics material to the newton world if it is not there already.
        physicsWorld_->addPhysicsMaterial(material);
    }

    void CollisionShape::updateBuild()
    {
            // first free any reference to an existing collision.
            freeInternalCollision();

            //call the derived class createNewtonCollision function.
            buildNewtonCollision();
    }

    void CollisionShape::buildNewtonCollision()
    {

    }

    void CollisionShape::freeInternalCollision()
    {
        if (newtonCollision_) {
            physicsWorld_->addToFreeQueue(newtonCollision_);
            newtonCollision_ = nullptr;
        }
    }


    void CollisionShape::MarkDirty(bool dirty /*= true*/)
    {
        if (shapeNeedsRebuilt_ != dirty) {
            shapeNeedsRebuilt_ = dirty;
            if(dirty)
                MarkRigidBodyDirty();
        }
    }

    NewtonCollision* CollisionShape::GetNewtonCollision()
    {
        return newtonCollision_;
    }


    void CollisionShape::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        Component::DrawDebugGeometry(debug, depthTest);
        if (newtonCollision_) {
            NewtonDebug_DrawCollision(newtonCollision_, node_->LocalToWorld(GetOffsetMatrix()), Color::GREEN, debug, depthTest);
        }
    }

    void CollisionShape::OnSetEnabled()
    {
        MarkRigidBodyDirty();
    }


    void CollisionShape::MarkRigidBodyDirty()
    {
        PODVector<RigidBody*> rootRigidBodies;
        GetRootRigidBodies(rootRigidBodies, node_, true);
        if (rootRigidBodies.Size() > 1)
            rootRigidBodies[rootRigidBodies.Size() - 2]->MarkDirty(true);
        else if (rootRigidBodies.Size() == 1)
        {
            rootRigidBodies.Back()->MarkDirty(true);//mark scene rig body dirty
        }
    }

    void CollisionShape::OnNodeSet(Node* node)
    {

        if (node)
        {
            ///auto create physics world
            physicsWorld_ = WeakPtr<PhysicsWorld>(GetScene()->GetOrCreateComponent<PhysicsWorld>());


            physicsWorld_->addCollisionShape(this);
            node->AddListener(this);

            SubscribeToEvent(node, E_NODETRANSFORMCHANGE, URHO3D_HANDLER(CollisionShape, HandleNodeTransformChange));

        }
        else
        {
            freeInternalCollision();
            if (physicsWorld_)
                physicsWorld_->removeCollisionShape(this);

            UnsubscribeFromEvent(E_NODETRANSFORMCHANGE);
        }

    }



    void CollisionShape::OnNodeSetEnabled(Node* node)
    {
        MarkRigidBodyDirty();
    }

    void CollisionShape::HandleNodeAdded(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeAdded::P_NODE].GetPtr());
        Node* newParent = static_cast<Node*>(eventData[NodeRemoved::P_PARENT].GetPtr());

        if (node == node_)
        {
            OnPhysicsNodeAdded(node);
        }
    }

    void CollisionShape::HandleNodeRemoved(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeRemoved::P_NODE].GetPtr());
        if (node == node_)
        {
            Node* oldParent = static_cast<Node*>(eventData[NodeRemoved::P_PARENT].GetPtr());

            if (oldParent)
            {
                OnPhysicsNodeRemoved(oldParent);
            }
            else
            {
                URHO3D_LOGINFO("should not happen");
            }
        }
    }


    void CollisionShape::HandleNodeTransformChange(StringHash event, VariantMap& eventData)
    {
        MarkRigidBodyDirty();
    }

}
