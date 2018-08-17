#include "NewtonBallAndSocketConstraint.h"
#include "UrhoNewtonPhysicsWorld.h"
#include "Core/Context.h"
#include "Newton.h"
#include "dMatrix.h"
#include "../Physics/NewtonRigidBody.h"
#include "UrhoNewtonConversions.h"
#include "Scene/Component.h"
#include "Scene/Scene.h"
#include "dCustomBallAndSocket.h"
#include "dCustomJoint.h"
#include "dCustom6dof.h"

namespace Urho3D {

    NewtonBallAndSocketConstraint::NewtonBallAndSocketConstraint(Context* context) : NewtonConstraint(context)
    {

    }

    Urho3D::NewtonBallAndSocketConstraint::~NewtonBallAndSocketConstraint()
    {

    }

    void Urho3D::NewtonBallAndSocketConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonBallAndSocketConstraint>(DEF_PHYSICS_CATEGORY.CString());
    }

    void Urho3D::NewtonBallAndSocketConstraint::buildConstraint()
    {
        //get own body transform.
        dMatrix ownBodyWorldMatrix;
        NewtonBodyGetMatrix(ownBody_->GetNewtonBody(), &ownBodyWorldMatrix[0][0]);
        //pinMatrix.m_posit += UrhoToNewton(position_) + dVector(0,-1,0);



        dMatrix pinAndPivotMatrix = ownBodyWorldMatrix;
        pinAndPivotMatrix.m_posit = ownBodyWorldMatrix.m_posit + dVector(0, -0.5f, 0);

        Quaternion quat(ownBody_->GetNode()->GetDirection(), Vector3(0, -1.0f, 0));
        Vector3 eulerAngles = quat.EulerAngles();
        Matrix3x4 pinMat(ownBody_->GetNode()->GetWorldPosition() + Vector3(0, -0.5f, 0), quat, 1.0f);

        // specify the limits for defining a Hinge around the x axis
        dVector minLinearLimits(0.0f, 0.0f, 0.0f, 0.0f);
        dVector maxLinearLimits(0.0f, 0.0f, 0.0f, 0.0f);

        dVector minAngulaLimits(-90 * dDegreeToRad, -90 * dDegreeToRad, -180 * dDegreeToRad);
        dVector maxAngulaLimits(90 * dDegreeToRad, 90 * dDegreeToRad, 180 * dDegreeToRad);

        
        // Create a 6DOF joint 
        newtonJoint_ = new dCustom6dof(UrhoToNewton(pinMat), otherBody_->GetNewtonBody(), ownBody_->GetNewtonBody());

        // set the hinge Limits
        static_cast<dCustom6dof*>(newtonJoint_)->SetLinearLimits(&minLinearLimits[0], &maxLinearLimits[0]);
        static_cast<dCustom6dof*>(newtonJoint_)->SetPitchLimits(minAngulaLimits.m_x, maxLinearLimits.m_x);
        static_cast<dCustom6dof*>(newtonJoint_)->SetRollLimits(minAngulaLimits.m_y, maxLinearLimits.m_y);
        static_cast<dCustom6dof*>(newtonJoint_)->SetYawLimits(minAngulaLimits.m_z, maxLinearLimits.m_z);




        //dMatrix matrix1;
        //NewtonBodyGetMatrix(otherBody_->GetNewtonBody(), &matrix1[0][0]);
        //dVector pivot1(matrix1.m_posit + UrhoToNewton(otherPosition_));

       // dMatrix tiltConeMatrix(dYawMatrix(-30.0f * dDegreeToRad) * pinMatrix);



        //newtonJoint_ = new dCustomBallAndSocket(pinMatrix,  otherBody_->GetNewtonBody(), ownBody_->GetNewtonBody());
        //static_cast<dCustomBallAndSocket*>(newtonJoint_)->EnableCone(true);
        //static_cast<dCustomBallAndSocket*>(newtonJoint_)->EnableTwist(true);
        //static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetConeLimits(60.0f * dDegreeToRad);
        //static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetTwistLimits(-1000.0f * dDegreeToRad, 1000.0f * dDegreeToRad);

    }


}
