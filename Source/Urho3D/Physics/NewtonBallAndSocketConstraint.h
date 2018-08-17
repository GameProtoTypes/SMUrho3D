#pragma once



#pragma once
#include "NewtonConstraint.h"


namespace Urho3D {
    class Context;
    class URHO3D_API NewtonBallAndSocketConstraint : public NewtonConstraint
    {
        URHO3D_OBJECT(NewtonBallAndSocketConstraint, NewtonConstraint);

    public:

        NewtonBallAndSocketConstraint(Context* context);
        ~NewtonBallAndSocketConstraint();

        static void RegisterObject(Context* context);


    protected:

        virtual void buildConstraint() override;
    };


}
