#pragma once
#include "../Scene/Component.h"


class dCustomJoint;
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
        /// Set other body to connect to. Set to null to connect to the static world.
        void SetOtherBody(NewtonRigidBody* body);

        /// Set constraint position relative to own body.
        void SetPosition(const Vector3& position);

        /// Set constraint position relative to the other body. If connected to the static world, is a world space position.
        void SetOtherPosition(const Vector3& position);


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
        /// Internal newtonJoint.
        dCustomJoint* newtonJoint_ = nullptr;
        /// Constraint other body position.
        Vector3 otherPosition_;
        /// Constraint position.
        Vector3 position_;

        /// Upper level re-evaulation.
        void reEvalConstraint();
        /// build the newton constraint.
        virtual void buildConstraint();
        /// frees and deletes the internal joint.
        void freeConstraint();
        /// Called right before rebuildConstraint, checks both bodies and destroys old joints.
        bool preRebuildCheckAndClean();

        virtual void OnNodeSet(Node* node) override;

    };
}

