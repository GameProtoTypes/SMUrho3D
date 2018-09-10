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



        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

    protected:

        virtual void buildConstraint() override;
    };


}
