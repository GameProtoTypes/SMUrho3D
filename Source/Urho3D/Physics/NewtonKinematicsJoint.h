#pragma once
#include "NewtonConstraint.h"




namespace Urho3D {
    class Context;

    ///Contraint for moving rigid bodies to a target position and orientation.
    class URHO3D_API NewtonKinematicsControllerConstraint : public NewtonConstraint
    {
        URHO3D_OBJECT(NewtonKinematicsControllerConstraint, NewtonConstraint);

    public:

        NewtonKinematicsControllerConstraint(Context* context);
        ~NewtonKinematicsControllerConstraint();

        static void RegisterObject(Context* context);

        /// Visualize the component as debug geometry.
        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

        ///Set Max Linear Friction - The higher this is the more powerful the joint will be but may exert too much force on other bodies.
        void SetMaxLinearFriction(float friction);
        ///Set Max Angular Friction - The higher this is the more powerful the joint will be but may exert too much force on other bodies.
        void SetMaxAngularFriction(float friction);

        ///Enforce rotational target. if disable only position will be constrained and the body will be free to rotate.
        void SetConstrainRotation(bool enable);

        /// Limit the rotation velocity to minimize instability. default true.
        void SetLimitRotationalVelocity(bool enable);

        /// Set the target position the contraint will move the reference frame on the own body to.
        void SetTargetPosition(Vector3 worldPos);
        /// Set the target rotation the constraint will move the reference frame of the own body to.
        void SetTargetRotation(Quaternion worldOrientation);

        

    protected:

        virtual void buildConstraint() override;

        void updateTarget();

        ///Target Position the joint will pull the body towards.
        Vector3 currentTargetPos_;
        ///Target orientation the joint will torque the body towards.
        Quaternion currentTargetRotation_;

        ///If enabled the constraint will force orientation to the current target orientation.
        bool constrainRotation_ = true;

        ///If enabled the constraint will limit the rotational velocity. if false the joint may become unstable.
        bool limitRotationalVelocity_ = true;

        float maxLinearFriction_ = 1000.0f;
        float maxAngularFriction_ = 1000.0f;
    };
}
