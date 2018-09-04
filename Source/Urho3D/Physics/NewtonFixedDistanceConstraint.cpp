#include "NewtonFixedDistanceConstraint.h"
#include "UrhoNewtonPhysicsWorld.h"
#include "Core/Context.h"
#include "Newton.h"
#include "dMatrix.h"
#include "../Physics/NewtonRigidBody.h"
#include "UrhoNewtonConversions.h"
#include "dCustomFixDistance.h"
#include "Scene/Component.h"
#include "Scene/Scene.h"

namespace Urho3D {




    NewtonFixedDistanceConstraint::NewtonFixedDistanceConstraint(Context* context) : NewtonConstraint(context)
    {
    }

    NewtonFixedDistanceConstraint::~NewtonFixedDistanceConstraint()
    {
    }

    void NewtonFixedDistanceConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonFixedDistanceConstraint>(DEF_PHYSICS_CATEGORY.CString());


        URHO3D_COPY_BASE_ATTRIBUTES(NewtonConstraint);



    }


    void NewtonFixedDistanceConstraint::buildConstraint()
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
