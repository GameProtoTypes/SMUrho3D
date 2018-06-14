#include "../Physics/NewtonPhysicsWorld.h"
#include "../Physics/NewtonCollisionShape.h"
#include "../Physics/NewtonRigidBody.h"
#include "../Core/Context.h"
#include "IO/Log.h"
#include "Scene/Scene.h"
#include "Scene/Node.h"
#include "dMatrix.h"
#include "Newton.h"

namespace Urho3D {
    NewtonRigidBody::NewtonRigidBody(Context* context) : Component(context)
    {

    }

    NewtonRigidBody::~NewtonRigidBody()
    {

    }

    void NewtonRigidBody::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonRigidBody>();
    }



    void NewtonRigidBody::SetMass(float mass)
    {
    }


    void NewtonRigidBody::SetFriction(float friction)
    {
    }
    void NewtonRigidBody::SetLinearVelocity(const Vector3& velocity)
    {
    }

    void NewtonRigidBody::addToPhysicsWorld()
    {
        if(physicsWorld_)
            physicsWorld_->addRigidBody(this);
    }

    void NewtonRigidBody::removeFromPhysicsWorld()
    {
        if (physicsWorld_)
            physicsWorld_->removeRigidBody(this);
    }

    void NewtonRigidBody::createBody()
    {

        dMatrix mat = UrhoToNewton(node_->GetWorldTransform());


        NewtonWorld* newtWorld = GetScene()->GetComponent<NewtonPhysicsWorld>()->GetNewtonWorld();
        newtonBody_ = NewtonCreateDynamicBody(newtWorld, colShape_->GetNewtonCollision(), &mat[0][0]);
    }


    void NewtonRigidBody::freeBody()
    {
        if (newtonBody_)
        {
            NewtonDestroyBody(newtonBody_);
            newtonBody_ = nullptr;
        }
    }

    void NewtonRigidBody::OnNodeSet(Node* node)
    {
       // throw std::logic_error("The method or operation is not implemented.");
    }

    void NewtonRigidBody::OnSceneSet(Scene* scene)
    {
        if (scene)
        {
            if (scene == node_)
                URHO3D_LOGWARNING(GetTypeName() + " should not be created to the root scene node");

            //Auto-create a physics world on the scene if it does not yet exist.
            physicsWorld_ = WeakPtr<NewtonPhysicsWorld>(scene->GetOrCreateComponent<NewtonPhysicsWorld>());

            addToPhysicsWorld();

            // Create shape now if necessary (attributes modified before adding to scene)
            //if (retryCreation_)
            //{
            //    UpdateShape();
            //    NotifyRigidBody();
            //}
        }
        else
        {

            if (physicsWorld_)
                removeFromPhysicsWorld();

           

            // Recreate when moved to a scene again
            //retryCreation_ = true;
        }
    }

}
