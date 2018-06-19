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
#include "dgQuaternion.h"


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

        rebuildBody();
    }

    void NewtonRigidBody::SetFriction(float friction)
    {
    }

    void NewtonRigidBody::SetLinearVelocity(const Vector3& velocity)
    {
        NewtonBodySetVelocity(newtonBody_, &UrhoToNewton(velocity)[0]);
    }

    void NewtonRigidBody::SetContinuousCollision(bool sweptCollision)
    {
        continuousCollision_ = sweptCollision;

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

            NewtonBodySetContinuousCollisionMode(newtonBody_, continuousCollision_);


            //assign callbacks
            NewtonBodySetForceAndTorqueCallback(newtonBody_, Newton_ApplyForceAndTorqueCallback);
            //NewtonBodySetTransformCallback(newtonBody_, Newton_SetTransformCallback); //not really needed since we pole for the transform at a specific time.
            NewtonBodySetDestructorCallback(newtonBody_, Newton_DestroyBodyCallback);






            bakeForceAndTorque();
        }
    }





    void NewtonRigidBody::bakeForceAndTorque()
    {
        Vector3 gravityForce = GetScene()->GetComponent<NewtonPhysicsWorld>()->GetGravity() * mass_;

        Quaternion worldOrientation = GetNode()->GetWorldRotation();
        Vector3 netForceWrldSpc = worldOrientation * netForce_;
        Vector3 netTorqueWrldSpc = worldOrientation * netTorque_;

        netForceNewton_ = UrhoToNewton((gravityForce + netForceWrldSpc));
        netTorqueNewton_ = UrhoToNewton(netTorqueWrldSpc);
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

    void NewtonRigidBody::AddForce(const Vector3& force)
    {
        AddForce(force, Vector3(0, 0, 0));
    }

    void NewtonRigidBody::AddForce(const Vector3& force, const Vector3& position)
    {
        netForce_ += force;
        netTorque_ += position.CrossProduct(force);
        bakeForceAndTorque();
    }

    void NewtonRigidBody::ResetForces()
    {
        netForce_ = Vector3(0, 0, 0);
        netTorque_ = Vector3(0, 0, 0);
        bakeForceAndTorque();
    }

    Vector3 NewtonRigidBody::GetNetForce()
    {
        return netForce_;
    }


    Urho3D::Vector3 NewtonRigidBody::GetNetTorque()
    {
        return netTorque_;
    }

    void NewtonRigidBody::ApplyTransform()
    {
        dVector pos;
        dgQuaternion quat;
        NewtonBodyGetPosition(newtonBody_, &pos[0]);
        NewtonBodyGetRotation(newtonBody_, &quat[0]);


        //node_->SetWorldTransform(NewtonToUrhoVec3(pos), Quaternion());
        node_->SetWorldTransform(NewtonToUrhoVec3(pos), NewtonToUrhoQuat(quat));
    }


    void NewtonRigidBody::GetBakedForceAndTorque(dVector& force, dVector& torque)
    {
        force = netForceNewton_;
        torque = netTorqueNewton_;
    }

}
