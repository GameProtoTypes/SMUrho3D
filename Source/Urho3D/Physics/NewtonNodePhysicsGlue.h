#pragma once
#include "Scene/Component.h"

namespace Urho3D
{
    class Context;
    /// base class for newton collision shapes
    class URHO3D_API NewtonNodePhysicsGlue : public Component
    {
        URHO3D_OBJECT(NewtonNodePhysicsGlue, Component);
    public:


        NewtonNodePhysicsGlue(Context* context);

        virtual ~NewtonNodePhysicsGlue();

        static void RegisterObject(Context* context);

    protected:

        virtual void OnNodeSet(Node* node) override;

        void HandleNodeAdded(StringHash event, VariantMap& eventData);
        void HandleNodeRemoved(StringHash event, VariantMap& eventData);


        WeakPtr<Node> oldNodeParent_;
    };

}
