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
    class NewtonNodePhysicsGlue;
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
        ///Get the mass scale of the rigid body
        float GetMassScale() const { return massScale_; }

        /// get the mass of the rigid body 
        float GetEffectiveMass() { return mass_; }

        /// Set friction coefficient.
        void SetFriction(float friction);

        /// Set linear velocity.
        void SetLinearVelocity(const Vector3& velocity);

        /// Set whether the collision size should be effected by the node scale.
        void SetInheritNodeScale(bool enable = true);

        bool GetInheritNodeScale() {
            return inheritNodeScale_;
        }

        /// Set continuous collision so that the body will not pass through walls.
        void SetContinuousCollision(bool sweptCollision);


        /// Add a force to the body in world cordinates on the body's center of mass.
        void AddWorldForce(const Vector3& force);
        /// Add a force to the body in world cordinates localPosition from the body's center of mass.
        void AddWorldForce(const Vector3& force, const Vector3& localPosition);
        /// Add a torque to the body in world space
        void AddWorldTorque(const Vector3& torque);

        /// Add a force to the body in local cordinates on the body's center of mass.
        void AddLocalForce(const Vector3& force);
        /// Add a force to the body in local cordinates localPosition from the body's center of mass.
        void AddLocalForce(const Vector3& force, const Vector3& localPosition);
        /// Add a torque to the body in local space
        void AddLocalTorque(const Vector3& torque);

        /// Reset accumulated forces.
        void ResetForces();

        /// apply an impulse to the body at the localPosition to aquire the target velocity next physics update.
        void AddImpulse(const Vector3& localPosition, const Vector3& targetVelocity);




        /// Return the net force acting on the body.
        Vector3 GetNetForce();

        /// Return the net torque acting on the body.
        Vector3 GetNetTorque();

        ///Get the currently used newton body.
        NewtonBody* GetNewtonBody() { return newtonBody_; }
        /// Return the currently used newton collision
        NewtonCollision* GetEffectiveNewtonCollision();


        /// Return the world position of the center of mass.
        Vector3 GetCenterOfMassPosition();
        /// Return the world orientation of the center of mass.
        Quaternion GetCenterOfMassRotation();



        ///Apply the current newton body transform to the node.
        void ApplyTransform();

        ///Return the net force and torque for newton.
        void GetBakedForceAndTorque(dVector& force, dVector& torque);

        /// Draw Debug geometry
        void DrawDebugGeometry(DebugRenderer* debug, bool depthTest, bool showAABB = true, bool showCollisionMesh = true, bool showCenterOfMass = true, bool showContactForces = true);

        /// mark the rigid body as dirty causing the newton rigid body to be rebuilt by the physics world
        void MarkDirty(bool dirty = true);

        bool GetDirty() const { return needsRebuilt_; }


        /// mark the internal newton transform as dirty indicating the transform needs to be copied to the node.
        void MarkInternalTransformDirty(bool dirty = true);

        bool GetInternalTransformDirty();


        /// rebuilds the internal body
        void reBuildBody();

        /// rebuild the internal body away of the scene herarchy (forms compound collision)
        void reBuildBodyParent();
    protected:


        /// Internal newton body
        NewtonBody * newtonBody_ = nullptr;
        /// compound collision if needed.
        NewtonCollision* compoundCollision_ = nullptr;


        /// Physics world.
        WeakPtr<UrhoNewtonPhysicsWorld> physicsWorld_;

        WeakPtr<NewtonNodePhysicsGlue> nodeGlue_;



        ///Continuous Collision
        bool continuousCollision_ = false;
        /// flag indicating collision shape should be additionally sized based on node scale.
        bool inheritNodeScale_ = true;

        ///Net Force in local cordinates
        Vector3 netForce_;
        ///Net Torque in local cordinates
        Vector3 netTorque_;
        ///angular dampending
        Vector3 angularDampening_;
        ///linera dampening
        float linearDampening_ = 0.0f;

        dVector netForceNewton_;
        dVector netTorqueNewton_;

        ///effective mass
        float mass_ = 0.0f;
        ///mass scale
        float massScale_ = 1.0f;

        ///dirty flag
        bool needsRebuilt_ = true;
        /// flag indicating the newton body has changed transforms and needs to update the node.
        bool transformDirty_ = true;

        void freeBody();

        ///precomputes force and torque for quick pass to newton callback
        void bakeForceAndTorque();



        virtual void OnNodeSet(Node* node) override;

        virtual void OnSceneSet(Scene* scene) override;


        void HandleNodeTransformChange(StringHash event, VariantMap& eventData);


        void applyDefferedActions();

        //temp variable for deferered actions on the newtonbody in case it has not been created yet.
        bool nextLinearVelocityNeeded_ = false;
        Vector3 nextLinearVelocity_;




    };



}
