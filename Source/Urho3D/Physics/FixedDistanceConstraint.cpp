#include "FixedDistanceConstraint.h"
#include "PhysicsWorld.h"
#include "Core/Context.h"
#include "Newton.h"
#include "dMatrix.h"
#include "../Physics/RigidBody.h"
#include "UrhoNewtonConversions.h"
#include "dCustomFixDistance.h"
#include "Scene/Component.h"
#include "Scene/Scene.h"

namespace Urho3D {




    FixedDistanceConstraint::FixedDistanceConstraint(Context* context) : Constraint(context)
    {
    }

    FixedDistanceConstraint::~FixedDistanceConstraint()
    {
    }

    void FixedDistanceConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<FixedDistanceConstraint>(DEF_PHYSICS_CATEGORY.CString());


        URHO3D_COPY_BASE_ATTRIBUTES(Constraint);



    }


    void FixedDistanceConstraint::buildConstraint()
{
        //get own body transform.
        dVector pivot0(UrhoToNewton(ownBody_->GetNode()->LocalToWorld(position_)));

        if (otherBody_) {

            dVector pivot1(UrhoToNewton(otherBody_->GetNode()->LocalToWorld(otherPosition_)));

            newtonJoint_ = new dCustomFixDistance(pivot0, pivot1, ownBody_->GetNewtonBody(), otherBody_->GetNewtonBody());

        }
        else
        {
            dVector pivot1(UrhoToNewton(otherPosition_));
            newtonJoint_ = new dCustomFixDistance(pivot0, pivot1, ownBody_->GetNewtonBody(), nullptr );
        }



    }

}
