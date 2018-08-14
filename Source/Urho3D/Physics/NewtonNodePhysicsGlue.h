#pragma once
#include "../Scene/Component.h"

namespace Urho3D
{
    class Component;
    class Context;
    class NewtonRigidBody;
    /// base class for newton collision shapes
    class URHO3D_API NewtonNodePhysicsGlue : public Component
    {
        URHO3D_OBJECT(NewtonNodePhysicsGlue, Component);
    public:


        NewtonNodePhysicsGlue(Context* context);

        virtual ~NewtonNodePhysicsGlue();

        static void RegisterObject(Context* context);

        void MarkDirty();



    protected:

        virtual void OnNodeSet(Node* node) override;

        void HandleNodeAdded(StringHash event, VariantMap& eventData);
        void HandleNodeRemoved(StringHash event, VariantMap& eventData);
    };

    NewtonRigidBody* GetMostRootRigidBody(Node* node);
}
