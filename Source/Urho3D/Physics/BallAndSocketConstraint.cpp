#include "BallAndSocketConstraint.h"
#include "PhysicsWorld.h"
#include "Core/Context.h"
#include "Newton.h"
#include "dMatrix.h"
#include "../Physics/RigidBody.h"
#include "UrhoNewtonConversions.h"
#include "Scene/Component.h"
#include "Scene/Scene.h"
#include "dCustomBallAndSocket.h"
#include "dCustomJoint.h"
#include "dCustom6dof.h"
#include "NewtonDebugDrawing.h"
#include "Graphics/DebugRenderer.h"

namespace Urho3D {

    BallAndSocketConstraint::BallAndSocketConstraint(Context* context) : Constraint(context)
    {

    }

    void BallAndSocketConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        Constraint::DrawDebugGeometry(debug, depthTest);
    }

    Urho3D::BallAndSocketConstraint::~BallAndSocketConstraint()
    {

    }

    void Urho3D::BallAndSocketConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<BallAndSocketConstraint>(DEF_PHYSICS_CATEGORY.CString());


        URHO3D_COPY_BASE_ATTRIBUTES(Constraint);

        URHO3D_ACCESSOR_ATTRIBUTE("Cone Enabled", GetConeEnabled, SetConeEnabled, bool, true, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Cone Angle", GetConeAngle, SetConeAngle, float, 20.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Cone Friction", GetConeFriction, SetConeFriction, float, 0.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Twist Limits Enabled", GetTwistLimitsEnabled, SetTwistLimitsEnabled, bool, false, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Twist Angle Min", GetTwistLimitMin, SetTwistLimitMin, float, -45.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Twist Angle Max", GetTwistLimitMax, SetTwistLimitMax, float, 45.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Twist Friction", GetTwistFriction, SetTwistFriction, float, 0.0f, AM_DEFAULT);

    }

    void Urho3D::BallAndSocketConstraint::buildConstraint()
    {
        // Create a dCustomBallAndSocket
        newtonJoint_ = new dCustomBallAndSocket(UrhoToNewton(GetOwnWorldFrame()), UrhoToNewton(GetOtherWorldFrame()), GetOwnNewtonBody(), GetOtherNewtonBody());


        static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetConeLimits(coneAngle_ * dDegreeToRad);
        static_cast<dCustomBallAndSocket*>(newtonJoint_)->EnableCone(coneEnabled_);
        static_cast<dCustomBallAndSocket*>(newtonJoint_)->EnableTwist(twistLimitsEnabled_);
        static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetTwistLimits(twistMinAngle_* dDegreeToRad, twistMaxAngle_ * dDegreeToRad);
        static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetConeFriction(coneFriction_);
        static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetTwistFriction(twistFriction_);
    }


}
