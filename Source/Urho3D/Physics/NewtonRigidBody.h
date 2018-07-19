#pragma once
#include "../Scene/Component.h"
#include "Newton.h"
#include "dVector.h"

class NewtonBody;

namespace Urho3D
{
    class Component;
    class UrhoNewtonPhysicsWorld;
    class NewtonCollisionShape;
    class URHO3D_API NewtonRigidBody : public Component
    {
        URHO3D_OBJECT(NewtonRigidBody, Component);
    public:

        friend class NewtonCollisionShape;
        friend class UrhoNewtonPhysicsWorld;
        /// Construct.
        NewtonRigidBody(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~NewtonRigidBody() override;
        /// Register object factory.
        static void RegisterObject(Context* context);

        ///Set a scaler on the mass of the rigid body - (scale is applied after collision shape densities)
        void SetMassScale(float massDensityScale);

        /// Set friction coefficient.
        void SetFriction(float friction);

        /// Set linear velocity.
        void SetLinearVelocity(const Vector3& velocity);

        void SetContinuousCollision(bool sweptCollision);


        /// Apply force to center of mass.
        void AddForce(const Vector3& force);
        /// Apply force at local position.
        void AddForce(const Vector3& force, const Vector3& position);

        /// Reset accumulated forces.
        void ResetForces();

        /// Return the net force acting on the body.
        Vector3 GetNetForce();

        /// Return the net force acting on the body.
        Vector3 GetNetTorque();

        /// Return the currently used newton collision
        NewtonCollision* GetEffectiveNewtonCollision();

        ///Apply the current newton body transform to the node.
        void ApplyTransform();

        ///Return the net force and torque for newton.
        void GetBakedForceAndTorque(dVector& force, dVector& torque);

        /// Draw Debug geometry
        void DrawDebugGeometry(DebugRenderer* debug, bool depthTest, bool showAABB = true, bool showCollisionMesh = true, bool showCenterOfMass = true, bool showContactForces = true);

        /// rebuilds the internal body
        void reEvaluateBody();
    protected:


        /// Internal newton body
        NewtonBody * newtonBody_ = nullptr;
        /// compound collision if needed.
        NewtonCollision* compoundCollision_ = nullptr;


        /// Physics world.
        WeakPtr<UrhoNewtonPhysicsWorld> physicsWorld_;
        /// Rigid body.
        WeakPtr<NewtonCollisionShape> colShape_;


        ///Continuous Collision
        bool continuousCollision_ = false;

        ///Net Force in local cordinates
        Vector3 netForce_;
        ///Net Torque in local cordinates
        Vector3 netTorque_;

        dVector netForceNewton_;
        dVector netTorqueNewton_;

        ///effective mass
        float mass_;
        ///mass scale
        float massScale_;


        void freeBody();

       
        ///precomputes force and torque for quick pass to newton callback
        void bakeForceAndTorque();



        virtual void OnNodeSet(Node* node) override;

        virtual void OnSceneSet(Scene* scene) override;



        void HandleNodeAdded(StringHash event, VariantMap& eventData);
        void HandleNodeRemoved(StringHash event, VariantMap& eventData);
        void HandleNodeParentChange(StringHash event, VariantMap& eventData);
    };



}
