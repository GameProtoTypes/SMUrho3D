#include "NewtonConstraint.h"
#include "NewtonRigidBody.h"
#include "UrhoNewtonPhysicsWorld.h"
#include "Core/Context.h"
#include "Scene/Component.h"
#include "Graphics/DebugRenderer.h"

namespace Urho3D {
    NewtonConstraint::NewtonConstraint(Context* context) : Component(context)
    {

    }

    NewtonConstraint::~NewtonConstraint()
    {

    }

    void NewtonConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonConstraint>(DEF_PHYSICS_CATEGORY.CString());
    }

    void NewtonConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        if (!ownBody_ || !otherBody_)
            return;

        debug->AddLine(ownBody_->GetCenterOfMassPosition(), otherBody_->GetCenterOfMassPosition(), Color::BLUE, false);
    }

    void NewtonConstraint::SetDisableCollision(bool disable)
    {
        //#todo
    }

}
