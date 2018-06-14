#pragma once



#include "../Scene/Component.h"
class NewtonWorld;
class dMatrix;
namespace Urho3D
{
    class Component;
    class NewtonCollisionShape;
    class NewtonRigidBody;

    class URHO3D_API NewtonPhysicsWorld : public Component
    {
        URHO3D_OBJECT(NewtonPhysicsWorld, Component);
    public:

        friend class NewtonCollisionShape;
        friend class NewtonRigidBody;

        /// Construct.
        NewtonPhysicsWorld(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~NewtonPhysicsWorld() override;
        /// Register object factory.
        static void RegisterObject(Context* context);

        /// Return the internal Newton world.
        NewtonWorld* GetNewtonWorld() { return newtonWorld_; }


        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

    protected:

        NewtonWorld* newtonWorld_ = nullptr;

        virtual void OnSceneSet(Scene* scene) override;

        void addCollisionShape(NewtonCollisionShape* collision);
        void removeCollisionShape(NewtonCollisionShape* collision);

        void addRigidBody(NewtonRigidBody* body);
        void removeRigidBody(NewtonRigidBody* body);


        void freeWorld();


        Vector<WeakPtr<NewtonCollisionShape>> collisionComponentList;
        Vector<WeakPtr<NewtonRigidBody>> rigidBodyComponentList;



        /// Step the simulation forward.
        void HandleUpdate(StringHash eventType, VariantMap& eventData);

    };





    /// Register Physics library objects.
    void URHO3D_API RegisterPhysicsLibrary(Context* context);
}
