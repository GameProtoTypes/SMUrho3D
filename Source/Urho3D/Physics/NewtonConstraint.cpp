#include "NewtonConstraint.h"
#include "NewtonRigidBody.h"
#include "UrhoNewtonPhysicsWorld.h"
#include "Core/Context.h"
#include "Scene/Component.h"
#include "Graphics/DebugRenderer.h"

#include "Scene/Scene.h"
#include "dCustomFixDistance.h"
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

    void NewtonConstraint::SetOtherBody(NewtonRigidBody* body)
    {
        otherBody_ = body;
        reEvalConstraint();
    }


    void NewtonConstraint::SetPosition(const Vector3& position)
    {
        position_ = position;
        reEvalConstraint();
    }


    void NewtonConstraint::SetOtherPosition(const Vector3& position)
    {
        otherPosition_ = position_;
        reEvalConstraint();
    }


    void NewtonConstraint::reEvalConstraint()
    {
        if (preRebuildCheckAndClean())
            buildConstraint();
    }

    void NewtonConstraint::buildConstraint()
    {
        /// ovverride in derived classes.
    }


    void NewtonConstraint::freeConstraint()
    {
        if (newtonJoint_ != nullptr) {
            delete newtonJoint_;
            newtonJoint_ = nullptr;
        }
    }

    bool NewtonConstraint::preRebuildCheckAndClean()
    {
        if (!ownBody_ || !otherBody_)
            return false;

        if (!ownBody_->GetNewtonBody() || !ownBody_->GetNewtonBody())
            return false;

        freeConstraint();
        
        return true;
    }

    void NewtonConstraint::OnNodeSet(Node* node)
    {
        if (node)
        {
            NewtonRigidBody* rigBody = node->GetComponent<NewtonRigidBody>();
            if (rigBody) {
                ownBody_ = rigBody;
                reEvalConstraint();
            }
        }
        else
        {
            ownBody_ = nullptr;
            freeConstraint();
        }
    }
}
