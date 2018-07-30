#include "NewtonFixedDistanceConstraint.h"
#include "UrhoNewtonPhysicsWorld.h"
#include "Core/Context.h"




namespace Urho3D {




    NewtonFixedDistanceConstraint::NewtonFixedDistanceConstraint(Context* context) : NewtonConstraint(context)
    {
    }

    NewtonFixedDistanceConstraint::~NewtonFixedDistanceConstraint()
    {
    }

    void NewtonFixedDistanceConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonFixedDistanceConstraint>(DEF_PHYSICS_CATEGORY.CString());
    }

    void NewtonFixedDistanceConstraint::SetDistance(float distance)
    {
        distance_ = distance;
    }

}
