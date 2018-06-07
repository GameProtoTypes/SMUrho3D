#include "../Physics/NewtonRigidBody.h"
#include "../Core/Context.h"

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

}
