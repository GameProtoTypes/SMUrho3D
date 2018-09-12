#pragma once
#include "../Scene/Component.h"


class dCustomJoint;
class NewtonBody;
namespace Urho3D {

    class Context;
    class RigidBody;
    class PhysicsWorld;

    ///Base class for newton constraints.
    class URHO3D_API Constraint : public Component
    {
        URHO3D_OBJECT(Constraint, Component);


    public:

        friend class PhysicsWorld;
        friend class RigidBody;
        /// Construct.
        Constraint(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~Constraint() override;

        static void RegisterObject(Context* context);

        /// Visualize the component as debug geometry.
        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

        void MarkDirty(bool dirty = true) { needsRebuilt_ = dirty; }

        /// Set whether to disable collisions between connected bodies.
        void SetDisableCollision(bool disable);

        /// Set other body to connect to. Set to null to connect to the static world.
        virtual void SetOtherBody(RigidBody* body);

        void SetOtherBodyById(unsigned bodyId);


        ///set the world position of both frames on both bodies
        void SetWorldPosition(const Vector3& position);
        ///set the world rotation of both frames on both bodies
        void SetWorldRotation(const Quaternion& rotation);



        /// Set constraint position in local cordinates to node.
        void SetOwnPosition(const Vector3& position);
        /// set the rotational frame to use on own node 
        void SetOwnRotation(const Quaternion& rotation);

        void SetOwnWorldPosition(const Vector3& worldPosition);

        void SetOwnWorldRotation(const Quaternion& worldRotation);


        Vector3 GetOwnPosition() const { return position_; }

        Quaternion GetOwnRotation() const { return rotation_; }



        /// Set constraint position in local cordinates relative to the other body. If connected to the static world, is a world space position.
        virtual void SetOtherPosition(const Vector3& position);
        /// set the rotational frame to use on other body. If connected to the static world, is a world space position.
        virtual void SetOtherRotation(const Quaternion& rotation);

        /// Set constraint position in local cordinates relative to the other body. If connected to the static world, is a world space position.
        virtual void SetOtherWorldPosition(const Vector3& position);
        /// set the rotational frame to use on other body. If connected to the static world, is a world space position.
        virtual void SetOtherWorldRotation(const Quaternion& rotation);


        /// Return physics world.
        PhysicsWorld* GetPhysicsWorld() const { return physicsWorld_; }

        /// Return rigid body in own scene node.
        RigidBody* GetOwnBody() const { return ownBody_; }

        NewtonBody* GetOwnNewtonBody() const;

        /// Return the other rigid body. May be null if connected to the static world.
        RigidBody* GetOtherBody() const { return otherBody_; }

        NewtonBody* GetOtherNewtonBody() const;

        unsigned GetOtherBodyId() const { return otherBodyId_; }


        Vector3 GetOtherPosition() const { return otherPosition_; }

        Quaternion GetOtherRotation() const { return otherRotation_; }


        Matrix3x4 GetOwnWorldFrame() const;

        Matrix3x4 GetOtherWorldFrame() const;


        dCustomJoint* GetNewtonJoint() const {
            return  newtonJoint_;
        }

        virtual void OnSetEnabled() override;

    protected:
        /// Physics world.
        WeakPtr<PhysicsWorld> physicsWorld_;
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


        float stiffness_ = 0.7f;

        int solverIterations_ = 16;

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

        void LogNodeScaleWarning(Node* node);
    };
}

