#pragma once
#include "../Scene/Component.h"
#include "Newton.h"
#include "dVector.h"

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


        /// Apply force to center of mass.
        void ApplyForce(const Vector3& force);
        /// Apply force at local position.
        void ApplyForce(const Vector3& force, const Vector3& position);

        /// Reset accumulated forces.
        void ResetForces();

        /// Return the net force acting on the body.
        Vector3 GetNetForce();

        /// Return the net force acting on the body.
        Vector3 GetNetTorque();


        ///Apply the current newton body transform to the node.
        void ApplyTransform();

        void GetBakedForceAndTorque(dVector& force, dVector& torque);

        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;


    protected:

        struct LocalForce {
            Vector3 force_;
            Vector3 localOffset;
        };

        /// Internal newton body
        NewtonBody * newtonBody_ = nullptr;
        /// Physics world.
        WeakPtr<NewtonPhysicsWorld> physicsWorld_;
        /// Rigid body.
        WeakPtr<NewtonCollisionShape> colShape_;
        /// Mass.
        float mass_ = 0.0f;

        ///Net Force in local cordinates
        Vector3 netForce_;
        ///Net Torque in local cordinates
        Vector3 netTorque_;

        dVector netForceNewton_;
        dVector netTorqueNewton_;





        void freeBody();
        /// rebuilds the internal body
        void rebuildBody();
       
        ///precomputes force and torque for quick pass to newton callback
        void bakeForceAndTorque();



        virtual void OnNodeSet(Node* node) override;

        virtual void OnSceneSet(Scene* scene) override;



        
    };



}
