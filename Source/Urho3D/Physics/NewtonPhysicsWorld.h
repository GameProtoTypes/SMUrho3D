#pragma once



#include "../Scene/Component.h"
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
    };

    /// Register Physics library objects.
    void URHO3D_API RegisterPhysicsLibrary(Context* context);
}
