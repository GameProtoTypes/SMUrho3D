#pragma once
#include "../Scene/Component.h"



namespace Urho3D {

    class Context;
    class NewtonRigidBody;
    class UrhoNewtonPhysicsWorld;

    ///Base class for newton constraints.
    class URHO3D_API NewtonConstraint : public Component
    {
        URHO3D_OBJECT(NewtonConstraint, Component);


    public:
        /// Construct.
        NewtonConstraint(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~NewtonConstraint() override;

        static void RegisterObject(Context* context);

        /// Visualize the component as debug geometry.
        void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;


        /// Set whether to disable collisions between connected bodies.
        void SetDisableCollision(bool disable);


        /// Return physics world.
        UrhoNewtonPhysicsWorld* GetPhysicsWorld() const { return physicsWorld_; }

        /// Return rigid body in own scene node.
        NewtonRigidBody* GetOwnBody() const { return ownBody_; }

        /// Return the other rigid body. May be null if connected to the static world.
        NewtonRigidBody* GetOtherBody() const { return otherBody_; }

    protected:
        /// Physics world.
        WeakPtr<UrhoNewtonPhysicsWorld> physicsWorld_;
        /// Own rigid body.
        WeakPtr<NewtonRigidBody> ownBody_;
        /// Other rigid body.
        WeakPtr<NewtonRigidBody> otherBody_;

    };
}

