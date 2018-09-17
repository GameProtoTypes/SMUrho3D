#pragma once
#include "../Scene/Component.h"
#include "Newton.h"
#include "dVector.h"
#include "../Scene/Node.h"

class NewtonBody;

namespace Urho3D
{
    class Component;
    class PhysicsWorld;
    class CollisionShape;
    class NewtonNodePhysicsGlue;
    class PhysicsMaterial;
    class URHO3D_API RigidBody : public Component
    {
        URHO3D_OBJECT(RigidBody, Component);
    public:

        friend class CollisionShape;
        friend class Constraint;
        friend class PhysicsWorld;

        /// Construct.
        RigidBody(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~RigidBody() override;
        /// Register object factory.
        static void RegisterObject(Context* context);

        ///Set a scaler on the mass of the rigid body - (scalar is applied to collision shape densities)
        void SetMassScale(float massDensityScale);


        /// set the physics material resource.
        void SetPhysicsMaterial(PhysicsMaterial* material);

        /// Set materials attribute.
        void SetPhysMaterialAttr(const ResourceRef& value);

        /// Return materials attribute.
        ResourceRef GetPhysMaterialAttr() const;

        PhysicsWorld* GetPhysicsWorld() const { return physicsWorld_; }


        Matrix3x4 GetPhysicsTransform() { return Matrix3x4(targetPos_, targetRotation_, 1.0f); }

        Vector3 GetPhysicsPosition() { return targetPos_; }

        Quaternion GetPhysicsRotation() { return targetRotation_; }

        ///Get the mass scale of the rigid body
        float GetMassScale() const { return massScale_; }

        /// get the mass of the rigid body 
        float GetEffectiveMass() { return mass_; }

        void SetCollisionLayer(unsigned layer)
        {
            collisionLayer_ = layer;
        }
        unsigned GetCollisionLayer() const { return collisionLayer_; }


        /// Set linear velocity in world cordinates.
        void SetLinearVelocity(const Vector3& velocity);

        /// Set the Angular velocity in world cordinates
        void SetAngularVelocity(const Vector3& angularVelocity);


        /// Set linear damping factor (0.0 to 1.0) default is 0
        void SetLinearDamping(float dampingFactor);

        float GetLinearDamping() const { return linearDampening_; }

        /// Set Angular Damping factor (0.0 to 1.0) for angle component. default is 0
        void SetAngularDamping(float angularDamping);

        float GetAngularDamping() const { return angularDampening_; }

        /// Set the internal linear damping - this is used internally by the newton solver to bring bodies to sleep more effectively.
        /// Be Aware that this parameter will show different behalvior with different solver iteration rates. (0.0 to 1.0) default is zero
        void SetInternalLinearDamping(float damping);

        /// Set the internal angular damping - this is used internally by the newton solver to bring bodies to sleep more effectively.
        /// Be Aware that this parameter will show different behalvior with different solver iteration rates. (0.0 to 1.0) default is zero
        void SetInternalAngularDamping(float angularDamping);

        /// Set the interpolation duration for applying transform to the scene node. 1.0f is no interpolation, 0.0f is inifinitely slow interpolation.
        void SetInterpolationFactor(float factor = 0.0f);

        float GetInterpolationFactor() const { return interpolationFactor_; }

        /// returns true if the interpolation is within tolerance of target rigidbody value.
        bool InterpolationWithinRestTolerance();

        /// snap current interpolated values directly to target values.
        void SnapInterpolation();

        /// Set whether the collision size should be effected by the node scale.
        void SetInheritNodeScale(bool enable = true);


        bool GetInheritNodeScale() const {
            return inheritCollisionNodeScales_;
        }

        /// Set continuous collision so that the body will not pass through walls.
        void SetContinuousCollision(bool sweptCollision);

        bool GetContinuousCollision() const { return continuousCollision_; }

        void SetAutoSleep(bool enableAutoSleep);

        bool GetAutoSleep() const { return autoSleep_; }

        /// force the body to be awake
        void Activate();
        /// force the body to sleep
        void DeActivate();

        /// Setting this to true will make the rigid body act as a root scene body with Inifite mass.
        void SetIsSceneRootBody(bool enable);


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
        NewtonBody* GetNewtonBody() const { return newtonBody_; }
        /// Return the currently used newton collision
        NewtonCollision* GetEffectiveNewtonCollision() const;


        Vector3 GetLinearVelocity(TransformSpace space = TS_WORLD) const;


        Vector3 GetAngularVelocity(TransformSpace space = TS_WORLD) const;


        Vector3 GetAcceleration();



        /// Return the world position of the center of mass.
        Vector3 GetCenterOfMassPosition();
        /// Return the world orientation of the center of mass.
        Quaternion GetCenterOfMassRotation();

        /// Get Immediately connected contraints to this rigid body.
        void GetConnectedContraints(PODVector<Constraint*>& contraints);
        PODVector<Constraint*> GetConnectedContraints();


        ///Apply the current newton body transform to the node.
        void ApplyTransform();

        ///Return the net force and torque for newton.
        void GetForceAndTorque(Vector3& force, Vector3& torque);

        /// Draw Debug geometry
        void DrawDebugGeometry(DebugRenderer* debug, bool depthTest, bool showAABB = true, bool showCollisionMesh = true, bool showCenterOfMass = true, bool showContactForces = true);

        /// mark the rigid body as dirty causing the newton rigid body to be rebuilt by the physics world
        void MarkDirty(bool dirty = true);

        bool GetDirty() const { return needsRebuilt_; }


        /// mark the internal newton transform as dirty indicating the transform needs to be copied to the node.
        void MarkInternalTransformDirty(bool dirty = true);

        bool GetInternalTransformDirty();

        





        virtual void OnSetEnabled() override;

    protected:


        /// Internal newton body
        NewtonBody * newtonBody_ = nullptr;
        /// compound collision if needed.
        NewtonCollision* compoundCollision_ = nullptr;
        /// Physics world.
        WeakPtr<PhysicsWorld> physicsWorld_;

        ///reference to physics material resource;
        WeakPtr<PhysicsMaterial> physicsMaterial_ = nullptr;

        bool sceneRootBodyMode_ = false;
        ///Continuous Collision
        bool continuousCollision_ = false;
        /// flag indicating collision shape should be additionally sized based on node scale.
        bool inheritCollisionNodeScales_ = true;
        /// flag indicating debug geometry for the collision should be shown in the debug renderer
        bool drawPhysicsDebugCollisionGeometry_ = true;


        Node* prevNode_ = nullptr;

        ///Net Force in local cordinates
        Vector3 netForce_;
        ///Net Torque in local cordinates
        Vector3 netTorque_;
        ///angular dampending
        float angularDampening_ = 0.0f;
        ///linera dampening
        float linearDampening_ = 0.0f;
        ///angular dampending
        Vector3 angularDampeningInternal_;
        ///linera dampening
        float linearDampeningInternal_ = 0.0f;

        ///currently connected constraints.
        HashSet<Constraint*> connectedConstraints_;



        dVector netForceNewton_;
        dVector netTorqueNewton_;

        ///effective mass
        float mass_ = 0.0f;
        ///mass scale
        float massScale_ = 1.0f;

        bool autoSleep_ = true;

        unsigned collisionLayer_ = 0;

        ///dirty flag
        bool needsRebuilt_ = true;
        /// flag indicating the newton body has changed transforms and needs to update the node.
        bool transformDirty_ = true;

        void freeBody();

        /// rebuilds the internal body based on the current status of collision shapes on this node and child nodes. (be sure to update the children first!)
        void reBuildBody();

        ///precomputes force and torque for quick pass to newton callback
        void bakeForceAndTorque();

        virtual void OnNodeSet(Node* node) override;

        virtual void OnSceneSet(Scene* scene) override;


        void HandleNodeAdded(StringHash event, VariantMap& eventData);
        void HandleNodeRemoved(StringHash event, VariantMap& eventData);
        void HandleNodeTransformChange(StringHash event, VariantMap& eventData);


       

        //variables for deferered singular actions on the newtonbody in case it has not been created yet.
        void applyDefferedActions();
        bool nextLinearVelocityNeeded_ = false;
        Vector3 nextLinearVelocity_;
        bool nextAngularVelocityNeeded_ = false;
        Vector3 nextAngularVelocity_;
        bool nextSleepStateNeeded_ = false;
        bool nextSleepState_ = false;


        //interpolation

        void updateInterpolatedTransform();
        Vector3 targetPos_;
        Quaternion targetRotation_;
        Vector3 interpolatedPos_;
        Quaternion interpolatedRotation_;
        float interpolationFactor_ = 1.0f;


        virtual void OnNodeSetEnabled(Node* node) override;

    };



}
