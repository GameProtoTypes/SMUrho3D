#pragma once



#include "../Scene/Component.h"
class NewtonWorld;
class dMatrix;
namespace Urho3D
{
    class Component;

    class URHO3D_API NewtonPhysicsWorld : public Component
    {
        URHO3D_OBJECT(NewtonPhysicsWorld, Component);
    public:
        /// Construct.
        NewtonPhysicsWorld(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~NewtonPhysicsWorld() override;
        /// Register object factory.
        static void RegisterObject(Context* context);

        /// Step the simulation forward.
        void Update(float timeStep);

        /// Return the internal Newton world.
        NewtonWorld* GetNewtonWorld() { return newtonWorld_; }

    protected:

        NewtonWorld* newtonWorld_ = nullptr;

        virtual void OnSceneSet(Scene* scene) override;

    };

    dMatrix UrhoToNewton(Matrix4 mat);


    /// Register Physics library objects.
    void URHO3D_API RegisterPhysicsLibrary(Context* context);
}
