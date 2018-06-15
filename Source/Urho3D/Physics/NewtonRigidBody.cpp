#include "../Physics/NewtonPhysicsWorld.h"
#include "../Physics/NewtonCollisionShape.h"
#include "../Physics/NewtonRigidBody.h"
#include "../Core/Context.h"
#include "IO/Log.h"
#include "Scene/Scene.h"
#include "Scene/Node.h"
#include "dMatrix.h"
#include "Newton.h"
#include "NewtonDebugDrawing.h"
#include "UrhoNewtonConversions.h"


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
        mass_ = mass;
    }

    void NewtonRigidBody::SetFriction(float friction)
    {
    }

    void NewtonRigidBody::SetLinearVelocity(const Vector3& velocity)
    {
    }

    void NewtonRigidBody::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        if (newtonBody_) {
            NewtonBodyDebugDrawAABB(newtonBody_, debug, depthTest);
            NewtonBodyDebugShowCollision(newtonBody_, debug, depthTest);
        }
    }




    void NewtonRigidBody::freeBody()
    {
        if (newtonBody_ != nullptr) {
            NewtonDestroyBody(newtonBody_);
            newtonBody_ = nullptr;
        }
    }

    void NewtonRigidBody::rebuildBody()
    {
        freeBody();

        colShape_ = node_->GetComponent<NewtonCollisionShape>();
        if (colShape_ && (colShape_->GetNewtonCollision() != nullptr)) {

            Matrix4 transform;
            transform.SetTranslation(node_->GetWorldPosition());
            transform.SetRotation(node_->GetWorldRotation().RotationMatrix());
           
            dMatrix mat = UrhoToNewton(transform);

            NewtonWorld* newtWorld = GetScene()->GetComponent<NewtonPhysicsWorld>()->GetNewtonWorld();
            newtonBody_ = NewtonCreateDynamicBody(newtWorld, colShape_->GetNewtonCollision(), &mat[0][0]);
            NewtonBodySetMassProperties(newtonBody_, mass_, colShape_->GetNewtonCollision());
            NewtonBodySetUserData(newtonBody_, (void*)this);

            //assign callbacks
            NewtonBodySetForceAndTorqueCallback(newtonBody_, Newton_ApplyForceAndTorqueCallback);
            NewtonBodySetTransformCallback(newtonBody_, Newton_SetTransformCallback);
            NewtonBodySetDestructorCallback(newtonBody_, Newton_DestroyBodyCallback);

        }
    }





    void NewtonRigidBody::OnNodeSet(Node* node)
    {
       
    }

    void NewtonRigidBody::OnSceneSet(Scene* scene)
    {
        if (scene)
        {
            if (scene == node_)
                URHO3D_LOGWARNING(GetTypeName() + " should not be created to the root scene node");

            //Auto-create a physics world on the scene if it does not yet exist.
            physicsWorld_ = WeakPtr<NewtonPhysicsWorld>(scene->GetOrCreateComponent<NewtonPhysicsWorld>());

            rebuildBody();

            physicsWorld_->addRigidBody(this);
           
        }
        else
        {

            freeBody();

            if (physicsWorld_)
                physicsWorld_->removeRigidBody(this);


            colShape_->updateReferenceToRigidBody();

        }
    }



}
