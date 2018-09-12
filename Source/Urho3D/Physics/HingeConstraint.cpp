#include "HingeConstraint.h"
#include "PhysicsWorld.h"
#include "Core/Context.h"
#include "Newton.h"
#include "dMatrix.h"
#include "../Physics/RigidBody.h"
#include "UrhoNewtonConversions.h"
#include "Scene/Component.h"
#include "Scene/Scene.h"
#include "dCustomJoint.h"
#include "NewtonDebugDrawing.h"
#include "Graphics/DebugRenderer.h"
#include "dCustomHinge.h"


Urho3D::HingeConstraint::HingeConstraint(Context* context) : Constraint(context)
{

}

Urho3D::HingeConstraint::~HingeConstraint()
{

}

void Urho3D::HingeConstraint::RegisterObject(Context* context)
{
    context->RegisterFactory<HingeConstraint>(DEF_PHYSICS_CATEGORY.CString());


    URHO3D_COPY_BASE_ATTRIBUTES(Constraint);

    URHO3D_ACCESSOR_ATTRIBUTE("Angle Min", GetMinAngle, SetMinAngle, float, -45.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Angle Max", GetMaxAngle, SetMaxAngle, float,  45.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Friction", GetFriction, SetFriction, float, 0.0f, AM_DEFAULT);

}

void Urho3D::HingeConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
{

}

void Urho3D::HingeConstraint::buildConstraint()
{
    // Create a dCustomHinge
    newtonJoint_ = new dCustomHinge(UrhoToNewton(GetOwnWorldFrame()), UrhoToNewton(GetOtherWorldFrame()), GetOwnNewtonBody(), GetOtherNewtonBody());

    static_cast<dCustomHinge*>(newtonJoint_)->SetLimits(minAngle_ * dDegreeToRad, maxAngle_ * dDegreeToRad);
    static_cast<dCustomHinge*>(newtonJoint_)->SetFriction(friction_);
}
