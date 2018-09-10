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

namespace Urho3D {

    BallAndSocketConstraint::BallAndSocketConstraint(Context* context) : Constraint(context)
    {

    }

    Urho3D::BallAndSocketConstraint::~BallAndSocketConstraint()
    {

    }

    void Urho3D::BallAndSocketConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<BallAndSocketConstraint>(DEF_PHYSICS_CATEGORY.CString());
    }

    void Urho3D::BallAndSocketConstraint::buildConstraint()
    {
        // Create a dCustomBallAndSocket
        newtonJoint_ = new dCustomBallAndSocket(UrhoToNewton(GetOwnWorldFrame()), UrhoToNewton(GetOtherWorldFrame()), GetOwnNewtonBody(), GetOtherNewtonBody());

        static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetConeLimits(10.0f * dDegreeToRad);
        static_cast<dCustomBallAndSocket*>(newtonJoint_)->EnableCone(true);


    }


}
