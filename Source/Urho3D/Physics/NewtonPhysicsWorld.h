#pragma once



#include "../Scene/Component.h"
#include "Newton.h"
class NewtonWorld;
class dMatrix;
namespace Urho3D
{
    class Component;
    class NewtonCollisionShape;
    class NewtonRigidBody;

    static const Vector3 DEF_GRAVITY = Vector3(0, -9.81, 0);

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

        /// Saves the NewtonWorld to a serializable newton file.
        void SerializeNewtonWorld(String fileName);

        ///set the global force acting on all rigid bodies in the world
        void SetGravity(const Vector3& force);
        ///return global force acting on all rigid bodies
        Vector3 GetGravity();

        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

    protected:

        /// Internal newton world
        NewtonWorld* newtonWorld_ = nullptr;

        ///Global force
        Vector3 gravity_ = DEF_GRAVITY;



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

    /// netwon callbacks
    void Newton_ApplyForceAndTorqueCallback(const NewtonBody* body, dFloat timestep, int threadIndex);
    void Newton_SetTransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex);
    void Newton_DestroyBodyCallback(const NewtonBody* body);







    /// Register Physics library objects.
    void URHO3D_API RegisterPhysicsLibrary(Context* context);
}
