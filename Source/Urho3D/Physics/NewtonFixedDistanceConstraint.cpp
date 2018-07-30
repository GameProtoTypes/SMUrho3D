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
    }


    void NewtonFixedDistanceConstraint::buildConstraint()
{
        //get own body transform.
        dMatrix matrix0;
        NewtonBodyGetMatrix(ownBody_->GetNewtonBody(), &matrix0[0][0]);
        dVector pivot0(matrix0.m_posit + UrhoToNewton(position_));

        dMatrix matrix1;
        NewtonBodyGetMatrix(otherBody_->GetNewtonBody(), &matrix1[0][0]);
        dVector pivot1(matrix1.m_posit + UrhoToNewton(otherPosition_));

        newtonJoint_ = new dCustomFixDistance(pivot1, pivot0, otherBody_->GetNewtonBody(), ownBody_->GetNewtonBody());
    }

}
