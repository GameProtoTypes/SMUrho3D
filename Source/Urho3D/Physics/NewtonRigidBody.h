#pragma once
#include "../Scene/Component.h"
#include "Newton.h"

class NewtonBody;

namespace Urho3D
{
    class Component;
    class NewtonPhysicsWorld;
    class NewtonCollisionShape;
    class URHO3D_API NewtonRigidBody : public Component
    {
        URHO3D_OBJECT(NewtonRigidBody, Component);
    public:

        friend class NewtonCollisionShape;
        friend class NewtonPhysicsWorld;
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


        ///Apply the current newton body transform to the node.
        void ApplyTransform();


        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;


    protected:

        /// Internal newton body
        NewtonBody * newtonBody_ = nullptr;
        /// Physics world.
        WeakPtr<NewtonPhysicsWorld> physicsWorld_;
        /// Rigid body.
        WeakPtr<NewtonCollisionShape> colShape_;
        /// Mass.
        float mass_ = 0.0f;


        void freeBody();
        /// rebuilds the internal body
        void rebuildBody();
       



        virtual void OnNodeSet(Node* node) override;

        virtual void OnSceneSet(Scene* scene) override;




    public:
        
    };



}
