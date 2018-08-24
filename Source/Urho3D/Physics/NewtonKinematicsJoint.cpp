#include "NewtonKinematicsJoint.h"
#include "UrhoNewtonPhysicsWorld.h"
#include "Core/Context.h"
#include "NewtonConstraint.h"
#include "dCustomKinematicController.h"
#include "NewtonRigidBody.h"
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

        debug->AddLine(ownBody_->GetCenterOfMassPosition(), currentTargetPos_, Color::GRAY, false);
    }

    void NewtonKinematicsControllerConstraint::SetMaxLinearFriction(float friction)
    {
        if (maxLinearFriction_ != friction) {
            maxLinearFriction_ = friction;
            if(newtonJoint_)
                static_cast<dCustomKinematicController*>(newtonJoint_)->SetMaxLinearFriction(maxLinearFriction_);
        }
    }

    void NewtonKinematicsControllerConstraint::SetMaxAngularFriction(float friction)
    {
        if (maxAngularFriction_ != friction) {
            maxAngularFriction_ = friction;
            if (newtonJoint_)
                static_cast<dCustomKinematicController*>(newtonJoint_)->SetMaxAngularFriction(maxAngularFriction_);
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
        static_cast<dCustomKinematicController*>(newtonJoint_)->SetMaxLinearFriction(maxLinearFriction_);
        static_cast<dCustomKinematicController*>(newtonJoint_)->SetMaxAngularFriction(maxAngularFriction_);
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

}
