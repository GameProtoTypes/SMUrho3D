#include "NewtonKinematicsJoint.h"
#include "PhysicsWorld.h"
#include "Core/Context.h"
#include "Constraint.h"
#include "dCustomKinematicController.h"
#include "RigidBody.h"
#include "UrhoNewtonConversions.h"
#include "Graphics/DebugRenderer.h"
#include "IO/Log.h"



namespace Urho3D {




    KinematicsControllerConstraint::KinematicsControllerConstraint(Context* context) : Constraint(context)
    {

    }

    KinematicsControllerConstraint::~KinematicsControllerConstraint()
    {

    }

    void KinematicsControllerConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<KinematicsControllerConstraint>(DEF_PHYSICS_CATEGORY.CString());
    }

    void KinematicsControllerConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        Constraint::DrawDebugGeometry(debug, depthTest);
    }

    void KinematicsControllerConstraint::SetLinearFrictionalAcceleration(float friction)
    {
        if (linearFrictionalAcceleration != friction) {
            linearFrictionalAcceleration = friction;
            if(newtonJoint_)
                updateFrictions();
        }
    }

    void KinematicsControllerConstraint::SetAngularFrictionalAcceleration(float friction)
    {
        if (angularFrictionalAcceleration != friction) {
            angularFrictionalAcceleration = friction;
            if (newtonJoint_)
                updateFrictions();
        }
    }

    void KinematicsControllerConstraint::SetConstrainRotation(bool enable)
    {
        if (constrainRotation_ != enable)
        {
            constrainRotation_ = enable;
            if (newtonJoint_)
                static_cast<dCustomKinematicController*>(newtonJoint_)->SetPickMode(!constrainRotation_);
        }
    }

    void KinematicsControllerConstraint::SetLimitRotationalVelocity(bool enable)
    {
        if (limitRotationalVelocity_ != enable)
        {
            limitRotationalVelocity_ = enable;
            if(newtonJoint_)
                static_cast<dCustomKinematicController*>(newtonJoint_)->SetLimitRotationVelocity(limitRotationalVelocity_);
        }
    }



    void KinematicsControllerConstraint::SetOtherBody(RigidBody* body)
    {
        URHO3D_LOGWARNING("KinematicsControllerConstraint Does not support an other body.");
        return;
    }

    void KinematicsControllerConstraint::SetOtherPosition(const Vector3& position)
    {
        otherPosition_ = position;
        updateTarget();
        //dont dirty because otherPosition_ is used for target frame
    }

    void KinematicsControllerConstraint::SetOtherRotation(const Quaternion& rotation)
    {
        otherRotation_ = rotation;
        updateTarget();
        //dont dirty because otherRotation_ is used for target frame
    }

    void KinematicsControllerConstraint::SetOtherWorldPosition(const Vector3& position)
    {
        SetOtherPosition(position);
    }

    void KinematicsControllerConstraint::SetOtherWorldRotation(const Quaternion& rotation)
    {
        SetOtherRotation(rotation);
    }

    void KinematicsControllerConstraint::buildConstraint()
    {


        newtonJoint_ = new dCustomKinematicController(GetOwnNewtonBody(), UrhoToNewton(GetOwnWorldFrame()));
        static_cast<dCustomKinematicController*>(newtonJoint_)->SetPickMode(constrainRotation_);
        updateFrictions();
        static_cast<dCustomKinematicController*>(newtonJoint_)->SetLimitRotationVelocity(limitRotationalVelocity_);


        updateTarget();
    }

    void KinematicsControllerConstraint::updateTarget()
    {
        if (newtonJoint_) {
            static_cast<dCustomKinematicController*>(newtonJoint_)->SetTargetMatrix(UrhoToNewton(GetOtherWorldFrame()));
        }
    }

    void KinematicsControllerConstraint::updateFrictions()
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
