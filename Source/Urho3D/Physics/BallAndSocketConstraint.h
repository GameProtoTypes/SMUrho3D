#pragma once



#pragma once
#include "Constraint.h"


namespace Urho3D {
    class Context;
    class URHO3D_API BallAndSocketConstraint : public Constraint
    {
        URHO3D_OBJECT(BallAndSocketConstraint, Constraint);

    public:

        BallAndSocketConstraint(Context* context);
        ~BallAndSocketConstraint();

        static void RegisterObject(Context* context);


    protected:

        virtual void buildConstraint() override;
    };


}
