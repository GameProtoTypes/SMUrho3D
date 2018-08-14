#include "NewtonNodePhysicsGlue.h"
#include "UrhoNewtonPhysicsWorld.h"
#include "Core/Context.h"
#include "NewtonRigidBody.h"
#include "Scene/Node.h"
#include "Scene/SceneEvents.h"
#include "IO/Log.h"

namespace Urho3D {

    NewtonNodePhysicsGlue::NewtonNodePhysicsGlue(Context* context) : Component(context)
    {
        SubscribeToEvent(E_NODEADDED, URHO3D_HANDLER(NewtonNodePhysicsGlue, HandleNodeAdded));
        SubscribeToEvent(E_NODEREMOVED, URHO3D_HANDLER(NewtonNodePhysicsGlue, HandleNodeRemoved));
    }

    NewtonNodePhysicsGlue::~NewtonNodePhysicsGlue()
    {

    }

    void NewtonNodePhysicsGlue::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonNodePhysicsGlue>(DEF_PHYSICS_CATEGORY.CString());
    }


    void NewtonNodePhysicsGlue::MarkDirty()
    {
        if (node_->HasComponent<NewtonRigidBody>())
        {
            node_->GetComponent<NewtonRigidBody>()->MarkDirty(true);
        }


    }

    void NewtonNodePhysicsGlue::OnNodeSet(Node* node)
    {
        if (node)
        {
            PODVector<NewtonNodePhysicsGlue*> dest;
            node->GetComponents<NewtonNodePhysicsGlue>(dest);
            if (dest.Size() > 1)
            {
                URHO3D_LOGWARNING("More than one " + NewtonNodePhysicsGlue::GetTypeNameStatic() + " component should not be added to a single node!");
            }
            if (node == (Node*)node->GetScene())
            {
                URHO3D_LOGWARNING(NewtonNodePhysicsGlue::GetTypeNameStatic() + " should not be added to the scene node!");
            }
        }
        else
        {

        }
    }


    void NewtonNodePhysicsGlue::HandleNodeAdded(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeAdded::P_NODE].GetPtr());
        Node* newParent = static_cast<Node*>(eventData[NodeRemoved::P_PARENT].GetPtr());

        if (node == node_)
        {
            //trigger a rebuild on the root of the new tree.
            NewtonRigidBody* mostRootRigBody = GetMostRootRigidBody(node);
            if (mostRootRigBody)
                mostRootRigBody->MarkDirty(true);
        }
    }

    void NewtonNodePhysicsGlue::HandleNodeRemoved(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeRemoved::P_NODE].GetPtr());
        if (node == node_)
        {
            Node* oldParent = static_cast<Node*>(eventData[NodeRemoved::P_PARENT].GetPtr());


            if (oldParent)
            {
                
                //trigger a rebuild on the root of the old parents tree.
                NewtonRigidBody* mostRootRigBody = GetMostRootRigidBody(oldParent);
                if(mostRootRigBody)
                    mostRootRigBody->MarkDirty(true);

            }
            else
            {
                URHO3D_LOGINFO("should not happen");
            }
        }
    }

    Urho3D::NewtonRigidBody* GetMostRootRigidBody(Node* node)
    {
        NewtonRigidBody* mostRootRigidBody = node->GetComponent<NewtonRigidBody>();
        if (!mostRootRigidBody)
            mostRootRigidBody = node->GetParentComponent<NewtonRigidBody>(true);

        while (mostRootRigidBody)//keep going up.
        {
            NewtonRigidBody* rig = mostRootRigidBody->GetNode()->GetParentComponent<NewtonRigidBody>(true);
            if (rig)
                mostRootRigidBody = rig;
            else
                break;
        }


        return mostRootRigidBody;
    }


}
