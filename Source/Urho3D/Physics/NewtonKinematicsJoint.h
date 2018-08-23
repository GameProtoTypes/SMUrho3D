#pragma once
#include "NewtonConstraint.h"




namespace Urho3D {
    class Context;

    ///not really working yet.
    class URHO3D_API NewtonKinematicsControllerConstraint : public NewtonConstraint
    {
        URHO3D_OBJECT(NewtonKinematicsControllerConstraint, NewtonConstraint);

    public:

        NewtonKinematicsControllerConstraint(Context* context);
        ~NewtonKinematicsControllerConstraint();

        static void RegisterObject(Context* context);

        /// Visualize the component as debug geometry.
        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

        void SetMaxLinearFriction(float friction);

        void SetMaxAngularFriction(float friction);

        void SetConstrainRotation(bool enable);


        void SetTargetPosition(Vector3 worldPos);

        void SetTargetRotation(Quaternion worldOrientation);

        

    protected:

        virtual void buildConstraint() override;

        void updateTarget();

        ///Target Position the joint will pull the body towards.
        Vector3 currentTargetPos_;
        ///Target orientation the joint will torque the body towards.
        Quaternion currentTargetRotation_;

        bool constrainRotation_ = true;

        float linearFriction_ = 10000000.0f;
        float angularFriction_ = 10000000.0f;
    };
}
