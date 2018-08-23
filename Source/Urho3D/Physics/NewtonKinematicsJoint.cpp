#include "NewtonKinematicsJoint.h"
#include "UrhoNewtonPhysicsWorld.h"
#include "Core/Context.h"
#include "NewtonConstraint.h"
#include "dCustomKinematicController.h"
#include "NewtonRigidBody.h"
#include "UrhoNewtonConversions.h"
#include "Graphics/DebugRenderer.h"



namespace Urho3D {




    NewtonKinematicsConstraint::NewtonKinematicsConstraint(Context* context) : NewtonConstraint(context)
    {

    }

    NewtonKinematicsConstraint::~NewtonKinematicsConstraint()
    {

    }

    void NewtonKinematicsConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonKinematicsConstraint>(DEF_PHYSICS_CATEGORY.CString());
    }

    void NewtonKinematicsConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        if (!ownBody_)
            return;

        debug->AddLine(ownBody_->GetCenterOfMassPosition(), currentTargetPos_, Color::GRAY, false);
    }

    void NewtonKinematicsConstraint::SetMaxLinearFriction(float friction)
    {
        if (linearFriction_ != friction) {
            linearFriction_ = friction;
            static_cast<dCustomKinematicController*>(newtonJoint_)->SetMaxLinearFriction(linearFriction_);
        }
    }

    void NewtonKinematicsConstraint::SetMaxAngularFriction(float friction)
    {
        if (angularFriction_ != friction) {
            angularFriction_ = friction;
            static_cast<dCustomKinematicController*>(newtonJoint_)->SetMaxAngularFriction(angularFriction_);
        }
    }

    //void NewtonKinematicsConstraint::SetTargetTransform(Matrix3x4 matrix)
    //{
    //    currentTargetTransform_ = matrix;
    //    updateTarget();

    //}

    void NewtonKinematicsConstraint::SetTargetPosition(Vector3 worldPos)
    {
        currentTargetPos_ = worldPos;
        updateTarget();
    }

    void NewtonKinematicsConstraint::SetTargetRotation(Quaternion worldOrientation)
    {
        currentTargetRotation_ = worldOrientation;
        updateTarget();
    }

    void NewtonKinematicsConstraint::buildConstraint()
    {
        //get own body transform.
        dMatrix matrix0;
        NewtonBodyGetMatrix(ownBody_->GetNewtonBody(), &matrix0[0][0]);


        newtonJoint_ = new dCustomKinematicController(ownBody_->GetNewtonBody(), UrhoToNewton(ownBody_->GetNode()->GetWorldPosition()));
        //static_cast<dCustomKinematicController*>(newtonJoint_)->SetPickMode(0);
        static_cast<dCustomKinematicController*>(newtonJoint_)->SetMaxLinearFriction(linearFriction_);
        static_cast<dCustomKinematicController*>(newtonJoint_)->SetMaxAngularFriction(angularFriction_);


        updateTarget();
    }

    void NewtonKinematicsConstraint::updateTarget()
    {
        if (newtonJoint_) {

            static_cast<dCustomKinematicController*>(newtonJoint_)->SetTargetPosit(UrhoToNewton(currentTargetPos_));
            static_cast<dCustomKinematicController*>(newtonJoint_)->SetTargetRotation(UrhoToNewton(currentTargetRotation_));
        }
    }

}
