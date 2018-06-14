#pragma once


#include "../Scene/Component.h"

class NewtonCollision;

namespace Urho3D
{
    class NewtonPhysicsWorld;
    class NewtonRigidBody;
    class Component;
    
    class URHO3D_API NewtonCollisionShape : public Component
    {
        URHO3D_OBJECT(NewtonCollisionShape, Component);
    public:

        friend class NewtonPhysicsWorld;

        NewtonCollisionShape(Context* context);

        ~NewtonCollisionShape() override;

        static void RegisterObject(Context* context);



        /// Set as a box.
        void SetBox(const Vector3& size, const Vector3& position = Vector3::ZERO, const Quaternion& rotation = Quaternion::IDENTITY);


        /// Returns the internal newton collision
        NewtonCollision* GetNewtonCollision();






    protected:

        /// Physics world.
        WeakPtr<NewtonPhysicsWorld> physicsWorld_;
        /// Rigid body.
        WeakPtr<NewtonRigidBody> rigidBody_;
        /// Internal Newton Collision
        NewtonCollision* newtonCollision_ = nullptr;

        /// Frees the internal collision shape;
        void freeInternalCollision();

        void addToPhysicsWorld();

        void removeFromPhysicsWorld();

        virtual void OnNodeSet(Node* node) override;


        virtual void OnSceneSet(Scene* scene) override;


        virtual void OnMarkedDirty(Node* node) override;

    };



}
