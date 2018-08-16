#pragma once
#include "Resource/Resource.h"
#include "Core/Context.h"
#include "Core/Object.h"

namespace Urho3D {

    ///resource providing physics model data 
    class URHO3D_API NewtonPhysicsModel : public ResourceWithMetadata
    {
        URHO3D_OBJECT(NewtonPhysicsModel, ResourceWithMetadata);

    public:
        NewtonPhysicsModel(Context* context) : ResourceWithMetadata(context)
        {

        }
        virtual ~NewtonPhysicsModel() {}

        static void RegisterObject(Context* context) {
            context->RegisterFactory< NewtonPhysicsModel>();
        }

    protected:

        //friction, restitution, etc..
    };

}
