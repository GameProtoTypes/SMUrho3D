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
        enableBodyCollision_ = !disable;
        
    }

    void NewtonConstraint::SetOtherBody(NewtonRigidBody* body)
    {
        otherBody_ = body;
        MarkDirty();
    }


    void NewtonConstraint::SetPosition(const Vector3& position)
    {
        position_ = position;
        MarkDirty();
    }


    void NewtonConstraint::SetOtherPosition(const Vector3& position)
    {
        otherPosition_ = position_;
        MarkDirty();
    }


    void NewtonConstraint::reEvalConstraint()
    {
        if (preRebuildCheckAndClean())
        {
            buildConstraint();
            NewtonJointSetCollisionState((NewtonJoint*)newtonJoint_, enableBodyCollision_);
            MarkDirty(false);
        }
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
            //auto create physics world similar to rigid body.
            physicsWorld_ = node->GetScene()->GetOrCreateComponent<UrhoNewtonPhysicsWorld>();

            NewtonRigidBody* rigBody = node->GetComponent<NewtonRigidBody>();
            if (rigBody) {
                ownBody_ = rigBody;
            }
           
            if(physicsWorld_)
                physicsWorld_->addConstraint(this);

        }
        else
        {
            ownBody_ = nullptr;
            if (physicsWorld_)
                physicsWorld_->removeConstraint(this);

        }
    }
}
