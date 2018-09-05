#pragma once
#include "../Scene/Component.h"


class dCustomJoint;
namespace Urho3D {

    class Context;
    class RigidBody;
    class UrhoNewtonPhysicsWorld;

    ///Base class for newton constraints.
    class URHO3D_API NewtonConstraint : public Component
    {
        URHO3D_OBJECT(NewtonConstraint, Component);


    public:

        friend class UrhoNewtonPhysicsWorld;
        friend class RigidBody;
        /// Construct.
        NewtonConstraint(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~NewtonConstraint() override;

        static void RegisterObject(Context* context);

        /// Visualize the component as debug geometry.
        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

        void MarkDirty(bool dirty = true) { needsRebuilt_ = dirty; }

        /// Set whether to disable collisions between connected bodies.
        void SetDisableCollision(bool disable);

        /// Set other body to connect to. Set to null to connect to the static world.
        void SetOtherBody(RigidBody* body);

        void SetOtherBodyById(unsigned bodyId);



        /// Set constraint position in local cordinates to body.
        void SetPosition(const Vector3& position);
        /// set the rotational frame to use on own body 
        void SetRotation(const Quaternion& rotation);



        /// Set constraint position in local cordinates relative to the other body. If connected to the static world, is a world space position.
        void SetOtherPosition(const Vector3& position);
        /// set the rotational frame to use on own body. If connected to the static world, is a world space position.
        void SetOtherRotation(const Quaternion& rotation);

        /// Return physics world.
        UrhoNewtonPhysicsWorld* GetPhysicsWorld() const { return physicsWorld_; }

        /// Return rigid body in own scene node.
        RigidBody* GetOwnBody() const { return ownBody_; }

        /// Return the other rigid body. May be null if connected to the static world.
        RigidBody* GetOtherBody() const { return otherBody_; }

        unsigned GetOtherBodyId() const { return otherBodyId_; }

        Vector3 GetOtherPosition() const { return otherPosition_; }

        Quaternion GetOtherRotation() const { return otherRotation_; }




        dCustomJoint* GetNewtonJoint() const {
            return  newtonJoint_;
        }

        virtual void OnSetEnabled() override;

    protected:
        /// Physics world.
        WeakPtr<UrhoNewtonPhysicsWorld> physicsWorld_;
        /// Own rigid body.
        WeakPtr<RigidBody> ownBody_;
        unsigned ownBodyId_ = 0;
        /// Other rigid body.
        WeakPtr<RigidBody> otherBody_;
        unsigned otherBodyId_ = 0;
        /// Internal newtonJoint.
        dCustomJoint* newtonJoint_ = nullptr;
        /// Flag indicating the two bodies should collide with each other.
        bool enableBodyCollision_ = false;
        /// Constraint other body position.
        Vector3 otherPosition_;
        Quaternion otherRotation_;

        /// Constraint position.
        Vector3 position_;
        Quaternion rotation_;

        ///dirty flag.
        bool needsRebuilt_ = true;

        /// Upper level re-evaulation.
        void reEvalConstraint();
        
        /// build the newton constraint.
        virtual void buildConstraint();
        
        /// frees and deletes the internal joint.
        void freeInternal();

        void AddJointReferenceToBody(RigidBody* rigBody);
        void RemoveJointReferenceFromBody(RigidBody* rigBody);



        virtual void OnNodeSet(Node* node) override;
        virtual void OnNodeSetEnabled(Node* node) override;

    };
}

