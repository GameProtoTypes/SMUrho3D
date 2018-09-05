#include "NewtonKinematicsJoint.h"
#include "UrhoNewtonPhysicsWorld.h"
#include "Core/Context.h"
#include "NewtonConstraint.h"
#include "dCustomKinematicController.h"
#include "RigidBody.h"
#include "UrhoNewtonConversions.h"
#include "Graphics/DebugRenderer.h"



namespace Urho3D {




    NewtonKinematicsControllerConstraint::NewtonKinematicsControllerConstraint(Context* context) : NewtonConstraint(context)
    {

    }

    NewtonKinematicsControllerConstraint::~NewtonKinematicsControllerConstraint()
    {

    }

    void NewtonKinematicsControllerConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonKinematicsControllerConstraint>(DEF_PHYSICS_CATEGORY.CString());
    }

    void NewtonKinematicsControllerConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        if (!ownBody_)
            return;

        if (!ownBody_->GetNewtonBody())
            return;

        debug->AddLine(ownBody_->GetCenterOfMassPosition(), currentTargetPos_, Color::GRAY, false);
    }

    void NewtonKinematicsControllerConstraint::SetLinearFrictionalAcceleration(float friction)
    {
        if (linearFrictionalAcceleration != friction) {
            linearFrictionalAcceleration = friction;
            if(newtonJoint_)
                updateFrictions();
        }
    }

    void NewtonKinematicsControllerConstraint::SetAngularFrictionalAcceleration(float friction)
    {
        if (angularFrictionalAcceleration != friction) {
            angularFrictionalAcceleration = friction;
            if (newtonJoint_)
                updateFrictions();
        }
    }

    void NewtonKinematicsControllerConstraint::SetConstrainRotation(bool enable)
    {
        if (constrainRotation_ != enable)
        {
            constrainRotation_ = enable;
            if (newtonJoint_)
                static_cast<dCustomKinematicController*>(newtonJoint_)->SetPickMode(!constrainRotation_);
        }
    }

    void NewtonKinematicsControllerConstraint::SetLimitRotationalVelocity(bool enable)
    {
        if (limitRotationalVelocity_ != enable)
        {
            limitRotationalVelocity_ = enable;
            if(newtonJoint_)
                static_cast<dCustomKinematicController*>(newtonJoint_)->SetLimitRotationVelocity(limitRotationalVelocity_);
        }
    }

    void NewtonKinematicsControllerConstraint::SetTargetPosition(Vector3 worldPos)
    {
        currentTargetPos_ = worldPos;
        updateTarget();
    }

    void NewtonKinematicsControllerConstraint::SetTargetRotation(Quaternion worldOrientation)
    {
        currentTargetRotation_ = worldOrientation;
        updateTarget();
    }

    void NewtonKinematicsControllerConstraint::buildConstraint()
    {
        //get own body transform.
        dMatrix matrix0;
        NewtonBodyGetMatrix(ownBody_->GetNewtonBody(), &matrix0[0][0]);


        newtonJoint_ = new dCustomKinematicController(ownBody_->GetNewtonBody(), UrhoToNewton(ownBody_->GetNode()->LocalToWorld(Matrix3x4(position_, rotation_, 1.0f))));
        static_cast<dCustomKinematicController*>(newtonJoint_)->SetPickMode(constrainRotation_);
        updateFrictions();
        static_cast<dCustomKinematicController*>(newtonJoint_)->SetLimitRotationVelocity(limitRotationalVelocity_);


        updateTarget();
    }

    void NewtonKinematicsControllerConstraint::updateTarget()
    {
        if (newtonJoint_) {

            static_cast<dCustomKinematicController*>(newtonJoint_)->SetTargetPosit(UrhoToNewton(currentTargetPos_));
            static_cast<dCustomKinematicController*>(newtonJoint_)->SetTargetRotation(UrhoToNewton(currentTargetRotation_));
        }
    }

    void NewtonKinematicsControllerConstraint::updateFrictions()
    {

        dFloat Ixx;
        dFloat Iyy;
        dFloat Izz;
        dFloat mass;
        NewtonBodyGetMass(ownBody_->GetNewtonBody(), &mass, &Ixx, &Iyy, &Izz);


        const dFloat inertia = dMax(Izz, dMax(Ixx, Iyy));


        static_cast<dCustomKinematicController*>(newtonJoint_)->SetMaxLinearFriction(mass * linearFrictionalAcceleration);
        static_cast<dCustomKinematicController*>(newtonJoint_)->SetMaxAngularFriction(inertia * angularFrictionalAcceleration);
    }

}
