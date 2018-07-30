#pragma once
#include "NewtonConstraint.h"


namespace Urho3D {
    class Context;
    class URHO3D_API NewtonFixedDistanceConstraint : public NewtonConstraint
    {
        URHO3D_OBJECT(NewtonFixedDistanceConstraint, NewtonConstraint);

    public:

        NewtonFixedDistanceConstraint(Context* context);
        ~NewtonFixedDistanceConstraint();

        static void RegisterObject(Context* context);


        void SetDistance(float distance);


    protected:

        float distance_ = 1.0f;

    };


}
