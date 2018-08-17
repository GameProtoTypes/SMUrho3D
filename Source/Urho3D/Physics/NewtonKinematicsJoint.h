#pragma once
#include "NewtonConstraint.h"




namespace Urho3D {
    class Context;

    ///not really working yet.
    class URHO3D_API NewtonKinematicsConstraint : public NewtonConstraint
    {
        URHO3D_OBJECT(NewtonKinematicsConstraint, NewtonConstraint);

    public:

        NewtonKinematicsConstraint(Context* context);
        ~NewtonKinematicsConstraint();

        static void RegisterObject(Context* context);

        /// Visualize the component as debug geometry.
        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

        //void SetTargetTransform(Matrix3x4 matrix);
        void SetTargetPosition(Vector3 worldPos);

    protected:

        virtual void buildConstraint() override;

        void updateTarget();

        Vector3 currentTargetPos_;
    };
}
