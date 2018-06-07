#pragma once
#include "../Scene/Component.h"
namespace Urho3D
{

    class Component;

    class URHO3D_API NewtonRigidBody : public Component
    {
        URHO3D_OBJECT(NewtonRigidBody, Component);
    public:
        /// Construct.
        NewtonRigidBody(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~NewtonRigidBody() override;
        /// Register object factory.
        static void RegisterObject(Context* context);


        /// Set mass. Zero mass makes the body static.
        void SetMass(float mass);


        /// Set friction coefficient.
        void SetFriction(float friction);

        /// Set linear velocity.
        void SetLinearVelocity(const Vector3& velocity);
    };

}
